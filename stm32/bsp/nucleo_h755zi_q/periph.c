/* Peripheral bring-up, hand-written (no CubeMX). One function per peripheral:
 * clocks (LL) → pins (LL) → HAL handle init → NVIC. Values mirror the
 * bench-validated h755zi-q firmware exactly. */
#include "board.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
FDCAN_HandleTypeDef hfdcan1;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart3;

/* Alternate-function pin, low speed (all AF pins on this board are low speed) */
static void pin_af(GPIO_TypeDef *port, uint32_t pin, uint32_t af,
                   uint32_t otype, uint32_t pull)
{
  LL_GPIO_SetPinSpeed(port, pin, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(port, pin, otype);
  LL_GPIO_SetPinPull(port, pin, pull);
  if (pin <= LL_GPIO_PIN_7)
    LL_GPIO_SetAFPin_0_7(port, pin, af);
  else
    LL_GPIO_SetAFPin_8_15(port, pin, af);
  LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_ALTERNATE);
}

void bsp_gpio_init(void)
{
  GPIO_InitTypeDef g = {0};

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA | LL_AHB4_GRP1_PERIPH_GPIOB
                         | LL_AHB4_GRP1_PERIPH_GPIOC | LL_AHB4_GRP1_PERIPH_GPIOD
                         | LL_AHB4_GRP1_PERIPH_GPIOE | LL_AHB4_GRP1_PERIPH_GPIOF
                         | LL_AHB4_GRP1_PERIPH_GPIOG);
  LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SYSCFG);  /* EXTI pin mux */

  /* PC8 nSCS: DRV8316 chip select, manual GPIO (drv8316.c toggles it) */
  LL_GPIO_ResetOutputPin(nSCS_GPIO_Port, LL_GPIO_PIN_8);
  LL_GPIO_SetPinSpeed(nSCS_GPIO_Port, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(nSCS_GPIO_Port, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(nSCS_GPIO_Port, LL_GPIO_PIN_8, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinMode(nSCS_GPIO_Port, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);

  /* PC6 nFAULT: HAL so the EXTI falling-edge mux + HAL_GPIO_EXTI_Callback work */
  g.Pin = nFault_Pin;
  g.Mode = GPIO_MODE_IT_FALLING;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(nFault_GPIO_Port, &g);
  HAL_NVIC_SetPriority(nFault_EXTI_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(nFault_EXTI_IRQn);
}

static void adc_common(ADC_HandleTypeDef *h, ADC_TypeDef *inst)
{
  h->Instance = inst;
  h->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;   /* PLL2 200 MHz / 4 */
  h->Init.Resolution = ADC_RESOLUTION_16B;
  h->Init.ScanConvMode = ADC_SCAN_DISABLE;
  h->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  h->Init.LowPowerAutoWait = DISABLE;
  h->Init.ContinuousConvMode = DISABLE;
  h->Init.NbrOfConversion = 1;
  h->Init.DiscontinuousConvMode = DISABLE;
  h->Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  h->Init.Overrun = ADC_OVR_DATA_PRESERVED;
  h->Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  h->Init.OversamplingMode = ENABLE;
  h->Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(h) != HAL_OK)
    Error_Handler();
  HAL_ADCEx_DisableInjectedQueue(h);
}

static void adc_injected(ADC_HandleTypeDef *h, uint32_t channel, uint32_t trig)
{
  ADC_InjectionConfTypeDef c = {0};

  c.InjectedChannel = channel;
  c.InjectedRank = ADC_INJECTED_RANK_1;
  c.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  c.InjectedSingleDiff = ADC_SINGLE_ENDED;
  c.InjectedOffsetNumber = ADC_OFFSET_NONE;
  c.InjectedNbrOfConversion = 1;
  c.ExternalTrigInjecConv = trig;
  c.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  c.InjecOversamplingMode = ENABLE;
  c.InjecOversampling.Ratio = 8;                   /* 8x oversample >>3 */
  c.InjecOversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  if (HAL_ADCEx_InjectedConfigChannel(h, &c) != HAL_OK)
    Error_Handler();
}

/* ADC1 (master, PF11 phase A) + ADC2 (slave, PA6 phase B), injected simultaneous,
 * triggered by TIM1 TRGO near the PWM counter peak. The ADC1 JEOC interrupt is
 * the 40 kHz current-loop tick. */
void bsp_adc_init(void)
{
  ADC_MultiModeTypeDef mm = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);
  LL_GPIO_SetPinMode(GPIOF, LL_GPIO_PIN_11, LL_GPIO_MODE_ANALOG);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ANALOG);

  adc_common(&hadc1, ADC1);
  mm.Mode = ADC_DUALMODE_INJECSIMULT;
  mm.DualModeData = ADC_DUALMODEDATAFORMAT_DISABLED;
  mm.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_9CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &mm) != HAL_OK)
    Error_Handler();
  adc_injected(&hadc1, ADC_CHANNEL_2, ADC_EXTERNALTRIGINJEC_T1_TRGO);

  adc_common(&hadc2, ADC2);
  adc_injected(&hadc2, ADC_CHANNEL_3, ADC_INJECTED_SOFTWARE_START); /* slave follows master */

  HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
}

/* SPI1: DRV8316 register access, 16-bit mode 1, ~3.1 MHz, manual CS on PC8 */
void bsp_spi1_init(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
  pin_af(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_5, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_NO); /* SCK  */
  pin_af(GPIOD, LL_GPIO_PIN_7, LL_GPIO_AF_5, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_NO); /* MOSI */
  pin_af(GPIOG, LL_GPIO_PIN_9, LL_GPIO_AF_5, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_NO); /* MISO */

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
    Error_Handler();
}

/* TIM1: 40 kHz center-aligned complementary 3-phase PWM (200 MHz / (2*2500)),
 * CH4 @ 2450 generates the ADC injected trigger via TRGO */
void bsp_tim1_init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 2500;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    Error_Handler();
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    Error_Handler();
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    Error_Handler();
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    Error_Handler();
  sConfigOC.Pulse = 2450;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    Error_Handler();
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 30;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    Error_Handler();

  /* PWM pins last (outputs stay inert until CCER/MOE are enabled by foc) */
  pin_af(GPIOE, LL_GPIO_PIN_9, LL_GPIO_AF_1, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_NO);  /* CH1  */
  pin_af(GPIOE, LL_GPIO_PIN_11, LL_GPIO_AF_1, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_NO); /* CH2  */
  pin_af(GPIOE, LL_GPIO_PIN_13, LL_GPIO_AF_1, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_NO); /* CH3  */
  pin_af(GPIOE, LL_GPIO_PIN_8, LL_GPIO_AF_1, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_NO);  /* CH1N */
  pin_af(GPIOB, LL_GPIO_PIN_0, LL_GPIO_AF_1, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_NO);  /* CH2N */
  pin_af(GPIOB, LL_GPIO_PIN_1, LL_GPIO_AF_1, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_NO);  /* CH3N */
}

/* TIM4: Hall sensor interface, 100 kHz tick (200 MHz / 2000), XOR capture on IC1 */
void bsp_tim4_init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_HallSensor_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
  pin_af(GPIOD, LL_GPIO_PIN_12, LL_GPIO_AF_2, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_UP);
  pin_af(GPIOD, LL_GPIO_PIN_13, LL_GPIO_AF_2, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_UP);
  pin_af(GPIOD, LL_GPIO_PIN_14, LL_GPIO_AF_2, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_UP);

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    Error_Handler();
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
    Error_Handler();
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.Commutation_Delay = 0;
  if (HAL_TIMEx_HallSensor_Init(&htim4, &sConfig) != HAL_OK)
    Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    Error_Handler();

  HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/* USART3: CLI console, 115200 8N1, single-byte RX interrupt */
void bsp_usart3_init(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
  pin_af(GPIOD, LL_GPIO_PIN_8, LL_GPIO_AF_7, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_NO); /* TX */
  pin_af(GPIOD, LL_GPIO_PIN_9, LL_GPIO_AF_7, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_NO); /* RX */

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
    Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
    Error_Handler();

  HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* FDCAN1: classic CAN 1 Mbit/s (200 MHz / 10 / 20 tq), polled RX FIFO0 */
void bsp_fdcan1_init(void)
{
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_FDCAN);
  pin_af(GPIOD, LL_GPIO_PIN_0, LL_GPIO_AF_9, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_NO); /* RX */
  pin_af(GPIOD, LL_GPIO_PIN_1, LL_GPIO_AF_9, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_PULL_NO); /* TX */

  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 4;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 8;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    Error_Handler();
}

/* I2C1: TMAG5273 encoder, ~400 kHz fast mode (kernel = APB1 100 MHz), blocking */
void bsp_i2c1_init(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    Error_Handler();

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
  pin_af(GPIOB, LL_GPIO_PIN_8, LL_GPIO_AF_4, LL_GPIO_OUTPUT_OPENDRAIN, LL_GPIO_PULL_NO); /* SCL */
  pin_af(GPIOB, LL_GPIO_PIN_9, LL_GPIO_AF_4, LL_GPIO_OUTPUT_OPENDRAIN, LL_GPIO_PULL_NO); /* SDA */

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x009034B6;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    Error_Handler();
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    Error_Handler();
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    Error_Handler();
}

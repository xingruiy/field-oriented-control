# ============================================================================
# foc-base Makefile - thin wrapper over scripts/ and tcl/.
#
# The shell/Tcl scripts remain the source of truth; this just dispatches.
# Vivado tools (xvlog/xelab/xsim/vivado) must already be on PATH, e.g.:
#     source ~/amd/2025.2/Vivado/settings64.sh
#
# Run from this directory (foc-base/). `make` with no target prints help.
# ============================================================================

TOP     ?= tb_foc_pkg          # default testbench for `sim` / `gui`
SCRIPTS := scripts
TCL     := tcl
VIVADO_WORK := build/vivado

.DEFAULT_GOAL := help
.PHONY: help sim gui regress test lut build bitstream program flash clean distclean

help: ## Show this help
	@echo "foc-base make targets (override TOP=tb_<module> where relevant):"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
	  | awk 'BEGIN {FS = ":.*?## "} {printf "  \033[36m%-12s\033[0m %s\n", $$1, $$2}'
	@echo "  \033[36mtb_<name>\033[0m    Run that testbench (e.g. make tb_foc_top)"

sim: ## Compile + run one testbench (TOP=tb_foc_pkg)
	$(SCRIPTS)/simulate.sh $(TOP)

gui: ## Run one testbench in the xsim GUI (TOP=...)
	$(SCRIPTS)/simulate.sh $(TOP) --gui

# Convenience: `make tb_foo` == `make sim TOP=tb_foo`
tb_%:
	$(SCRIPTS)/simulate.sh $@

regress: ## Run every testbench under sim/ and print a verdict
	$(SCRIPTS)/regress.sh

test: regress ## Alias for regress

lut: ## Regenerate rtl/math/sincos_lut.mem
	python3 $(SCRIPTS)/gen_sincos_lut.py

build: ## Synthesize + implement -> build/impl/foc_top.bit
	mkdir -p $(VIVADO_WORK)
	cd $(VIVADO_WORK) && vivado -mode batch -source $(CURDIR)/$(TCL)/build.tcl

bitstream: build ## Alias for build

program: ## Program the Arty S7-50 FPGA SRAM over USB-JTAG
	mkdir -p $(VIVADO_WORK)
	cd $(VIVADO_WORK) && vivado -mode batch -source $(CURDIR)/$(TCL)/program.tcl

flash: build ## Build bitstream, then program the Arty S7-50 QSPI flash
	mkdir -p $(VIVADO_WORK)
	cd $(VIVADO_WORK) && vivado -mode batch -source $(CURDIR)/$(TCL)/program_flash.tcl

clean: ## Remove sim artifacts (keeps build/impl bitstream)
	rm -rf build/sim build/vivado xsim.dir xvlog.log xvlog.pb *.jou *.log *.pb .Xil

distclean: clean ## clean + remove synth/impl output (build/impl)
	rm -rf build/impl

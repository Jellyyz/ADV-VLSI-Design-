
#Collect All Source Files
# SIM_VER := $(shell find $(PWD)/hdl/sim_ver -name '*.inc')
# SYNOPSYS := $(shell find /software/Synopsys-2021_x86_64/syn/R-2020.09-SP4/dw/sim_ver/ -name '*.inc' -o -name '*.v')
HDL_SRCS := $(shell find $(PWD)/hdl -name '*.sv' -o -name '*.v')
HVL_SRCS := $(shell find $(PWD)/hvl -name '*.sv' -o -name '*.v')

SRCS := $(SIM_VER) $(HDL_SRCS) $(HVL_SRCS)

VCS_FLAGS= -full64 -sv=2012 +lint=all,noSVA-UA,noNS,noSVA-AECASR,noSVA-FINUA -timescale=1ns/10ps -debug_acc+all -kdb -fsdb +v2k +incdir+/home/ghuang23/Desktop/ADV-VLSI-Design-/hdl/sim_ver
.PHONY: clean
.PHONY: run

sim/simv: $(SRCS) 
	mkdir -p sim
	cd sim && vcs -R $(SRCS) $(VCS_FLAGS)

run: sim/simv
	cd sim && ./simv

clean: 
	rm -rf sim

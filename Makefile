
#Collect All Source Files
HDL_SRCS := $(shell find $(PWD)/hdl -name '*.sv')
HVL_SRCS := $(shell find $(PWD)/hvl -name '*.sv' -o -name '*.v')
SRCS := $(HDL_SRCS) $(HVL_SRCS)

VCS_FLAGS= -full64 -sv=2012 +lint=all,noSVA-UA,noNS,noSVA-AECASR,noSVA-FINUA -timescale=1ns/10ps -debug_acc+all -kdb -fsdb +v2k

.PHONY: clean
.PHONY: run

sim/simv: $(SRCS) 
	mkdir -p sim
	cd sim && vcs -R $(SRCS) $(VCS_FLAGS)

run: sim/simv
	cd sim && ./simv

clean: 
	rm -rf sim

VCS=vcs
VCSFLAGS=-sverilog -timescale=1ns/100ps -full64 -debug_access+all +v2k -nc -lca -licqueue -v -L /software/quartus-std-18.1/modelsim_ase/altera/verilog/src/altera_mf.v
VCSGUIFLAGS=-gui -kdb
.DEFAULT_GOAL:= mp2

FPU_SRC = ../FPU


HVL_DEPS = $(addprefix $(HDL_SRC)/, fp_types.sv)
HVL_TOP = FPU/rand_float_gen.sv

.PHONY: rand_fp
rand_fp:
	$(VCS) $(VCSFLAGS) -top random_float_gen $(HVL_DEPS) $(HVL_TOP)

.PHONY: clean
clean:
	rm -f *.vcd simv ucli.key
	rm -rf csrc/ simv.daidir/
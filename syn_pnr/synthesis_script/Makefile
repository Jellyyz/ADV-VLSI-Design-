synthesis = dc_shell
core_synth_script = core_synth_dc.tcl

core: $(core_synth_script)
	$(synthesis) -f $<


clean:
	rm -rf *.rpt *.out *.log *.svf *.pvl *.syn *.mr alib-52 *.pvk results reports

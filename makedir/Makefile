all: chiptop-tb aes-ks-tb full_sim_tb

full_sim_tb: ../hvl/magic_mem_sim/top.sv full_sim_tb.scr  
	vcs -gui -kdb -timescale=10ns/1ns -full64 -top swamy_tb -f full_sim_tb.scr -o $@ \
		-xlrm uniq_prior_final -debug_access+r -kdb -sverilog \
		+lint=ALL +lint=TFIPC-L -debug_access+all  -fsdb +memcbk

aes-ks-tb: aes-ks-tb.scr ../hvl/aes_ks_tb.sv
	vcs -full64 -top aes_ks_tb -f aes-ks-tb.scr -o $@ \
		-xlrm uniq_prior_final -debug_access+r -kdb -sverilog \
		+lint=ALL +lint=TFIPC-L -debug_access+all  -fsdb +memcbk

chiptop-tb: chiptop-tb.scr ../hvl/chiptop-tb.sv
	vcs -full64 -top chiptop_tb -f chiptop-tb.scr -o $@ \
		-xlrm uniq_prior_final -debug_access+r -kdb -sverilog \
		+lint=ALL +lint=TFIPC-L -debug_access+all  -fsdb

chiptop-tb-postsynth: chiptop-tb-postsynth.scr 
	vcs -full64 $(TSMC65RF_PDK_IC6)/stdcell_dig/fb_tsmc065gp_rvt_lvt/aci/sc-ad10/verilog/tsmc65_rvt_sc_adv10.v \
		-top chiptop_tb -f chiptop-tb-postsynth.scr -o $@ \
		-xlrm uniq_prior_final -debug_access+r -kdb -sverilog \
		+lint=ALL +lint=TFIPC-L -debug_access+all -fsdb +delay_mode_zero

chiptop-jtag-tb-postsynth: chiptop-jtag-tb-postpnr.scr
	vcs -full64 $(TSMC65RF_PDK_IC6)/stdcell_dig/fb_tsmc065gp_rvt_lvt/aci/sc-ad10/verilog/tsmc65_rvt_sc_adv10.v \
		-top chiptop_jtag_tb -f chiptop-jtag-tb-postsynth.scr -o $@ \
		-xlrm uniq_prior_final -debug_access+r -kdb -sverilog \
		+lint=ALL +lint=TFIPC-L -debug_access+all -fsdb +delay_mode_zero

chiptop-tb-postpnr: chiptop-tb-postpnr.scr 
	vcs -full64 $(TSMC65RF_PDK_IC6)/stdcell_dig/fb_tsmc065gp_rvt_lvt/aci/sc-ad10/verilog/tsmc65_rvt_sc_adv10.v \
		-top chiptop_tb -f chiptop-tb-postpnr.scr -o $@ \
		-xlrm uniq_prior_final -debug_access+r -kdb -sverilog \
		+lint=ALL +lint=TFIPC-L -debug_access+all -fsdb +neg_tchk +sdfverbose \
    	-sdf max:chiptop_tb.dut:../syn_pnr/sdfout/chiptop.pnr.sdf 

chiptop-jtag-tb-postpnr: chiptop-jtag-tb-postpnr.scr 
	vcs -full64 $(TSMC65RF_PDK_IC6)/stdcell_dig/fb_tsmc065gp_rvt_lvt/aci/sc-ad10/verilog/tsmc65_rvt_sc_adv10.v \
		-top chiptop_jtag_tb -f chiptop-jtag-tb-postpnr.scr -o $@ \
		-xlrm uniq_prior_final -debug_access+r -kdb -sverilog \
		+lint=ALL +lint=TFIPC-L -debug_access+all -fsdb +neg_tchk +sdfverbose \
    	-sdf max:chiptop_jtag_tb.dut:../syn_pnr/sdfout/chiptop.pnr.sdf 

full-app-tb: full_app_tb.scr ../hvl/full_app_tb.sv
	vcs -full64 -top full_app_tb -f full_app_tb.scr -o $@ \
		-xlrm uniq_prior_final -debug_access+r -kdb -sverilog \
		+lint=ALL +lint=TFIPC-L -debug_access+all  -fsdb

full-app-tb-postsynth: full_app_tb_postsynth.scr 
	vcs -full64 $(TSMC65RF_PDK_IC6)/stdcell_dig/fb_tsmc065gp_rvt_lvt/aci/sc-ad10/verilog/tsmc65_rvt_sc_adv10.v \
		-top full_app_tb -f full_app_tb_postsynth.scr -o $@ \
		-xlrm uniq_prior_final -debug_access+r -kdb -sverilog \
		+lint=ALL +lint=TFIPC-L -debug_access+all -fsdb +delay_mode_zero
		
full-app-tb-postpnr: full_app_tb_postpnr.scr 
	vcs -full64 $(TSMC65RF_PDK_IC6)/stdcell_dig/fb_tsmc065gp_rvt_lvt/aci/sc-ad10/verilog/tsmc65_rvt_sc_adv10.v \
		-top full_app_tb -f full_app_tb_postpnr.scr -o $@ \
		-xlrm uniq_prior_final -debug_access+r -kdb -sverilog \
		+lint=ALL +lint=TFIPC-L -debug_access+all -fsdb +neg_tchk +sdfverbose \
    	-sdf max:full_app_tb.dut:../syn_pnr/sdfout/chiptop.pnr.sdf
		
run_postsynth:
	./chiptop-tb-postsynth -gui -cm assert +MemInitFile=../sw/basic-add/basic-add.vmem
	
chiptop-jtag-tb: chiptop-jtag-tb.scr ../hvl/chiptop-jtag-tb.sv
	vcs -full64 -cm assert -top chiptop_jtag_tb -f chiptop-jtag-tb.scr -o $@ \
		-xlrm uniq_prior_final -debug_access+r -kdb -sverilog \
		+lint=ALL +lint=TFIPC-L -debug_access+all  -fsdb 
run: chiptop-tb
	./chiptop-tb -l vcs.log 
clean:
	$(RM) chiptop-tb
	$(RM) -rf csrc
	$(RM) chiptop-jtag-tb
	$(RM) -rf chiptop-tb.daidir
	$(RM) -rf *.daidir
	$(RM) inter.fsdb
	$(RM) .inter.*
	$(RM) -rf verdi*
	$(RM) novas_dump.log
	$(RM) novas.*
	$(RM) ucli.key
	$(RM) vc_hdrs.h 
	$(RM) -rf verdi*
	$(RM) aes-ks-tb
	$(RM) *.log
	$(RM) .vpd
	$(RM) -rf *.vdb
	$(RM) chiptop-tb-postsynth
	$(RM) chiptop-tb-postpnr
	$(RM) chiptop-jtag-tb-postsynth
	$(RM) -rf chiptop-tb-postsynth.daidir
	$(RM) -rf csrc
	$(RM) -rf DVEfiles
	$(RM) chiptop-jtag-tb-postpnr
	$(RM) chiptop_jtag_tb-postpnr.daidir
	$(RM) chiptop-tb-postpnr.daidir

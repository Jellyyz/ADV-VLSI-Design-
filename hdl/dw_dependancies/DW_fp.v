
// this file is all the dependancies needed by our modules. do not remove

module DW_fp_sqrt_inst( inst_a, inst_rnd, z_inst, status_inst );
parameter inst_sig_width = 23;
parameter inst_exp_width = 8;
parameter inst_ieee_compliance = 0;
input [inst_sig_width+inst_exp_width : 0] inst_a;
input [2 : 0] inst_rnd;
output [inst_sig_width+inst_exp_width : 0] z_inst;
output [7 : 0] status_inst;
 // Instance of DW_fp_sqrt
 DW_fp_sqrt #(inst_sig_width, inst_exp_width, inst_ieee_compliance) U1 (
 .a(inst_a),
 .rnd(inst_rnd),
 .z(z_inst),
 .status(status_inst) );
endmodule


module DW_fp_addsub_inst( inst_a, inst_b, inst_rnd, inst_op, z_inst,
status_inst );
parameter sig_width = 23;
parameter exp_width = 8;
parameter ieee_compliance = 0;
input [sig_width+exp_width : 0] inst_a;
input [sig_width+exp_width : 0] inst_b;
input [2 : 0] inst_rnd;
input inst_op;
output [sig_width+exp_width : 0] z_inst;
output [7 : 0] status_inst;
 // Instance of DW_fp_addsub
 DW_fp_addsub #(sig_width, exp_width, ieee_compliance)
 U1 ( .a(inst_a), .b(inst_b), .rnd(inst_rnd),
 .op(inst_op), .z(z_inst), .status(status_inst) );
endmodule

module DW_fp_cmp_inst( inst_a, inst_b, inst_zctr, aeqb_inst, altb_inst,
agtb_inst, unordered_inst, z0_inst, z1_inst, status0_inst,
status1_inst );
parameter sig_width = 23;
parameter exp_width = 8;
parameter ieee_compliance = 0;
input [sig_width+exp_width : 0] inst_a;
input [sig_width+exp_width : 0] inst_b;
input inst_zctr;
output aeqb_inst;
output altb_inst;
output agtb_inst;
output unordered_inst;
output [sig_width+exp_width : 0] z0_inst;
output [sig_width+exp_width : 0] z1_inst;
output [7 : 0] status0_inst;
output [7 : 0] status1_inst;
 // Instance of DW_fp_cmp
 DW_fp_cmp #(sig_width, exp_width, ieee_compliance)
 U1 ( .a(inst_a), .b(inst_b), .zctr(inst_zctr), .aeqb(aeqb_inst),
.altb(altb_inst), .agtb(agtb_inst), .unordered(unordered_inst),
.z0(z0_inst), .z1(z1_inst), .status0(status0_inst),
.status1(status1_inst) );
endmodule

module DW_fp_div_inst( inst_a, inst_b, inst_rnd, z_inst, status_inst );
parameter sig_width = 23;
parameter exp_width = 8;
parameter ieee_compliance = 0;
parameter faithful_round = 0;
input [sig_width+exp_width : 0] inst_a;
input [sig_width+exp_width : 0] inst_b;
input [2 : 0] inst_rnd;
output [sig_width+exp_width : 0] z_inst;
output [7 : 0] status_inst;
 // Instance of DW_fp_div
DW_fp_div #(sig_width, exp_width, ieee_compliance, faithful_round) U1
( .a(inst_a), .b(inst_b), .rnd(inst_rnd), .z(z_inst), .status(status_inst)
);
endmodule

module DW_fp_mult_inst( inst_a, inst_b, inst_rnd, z_inst, status_inst );
parameter sig_width = 23;
parameter exp_width = 8;
parameter ieee_compliance = 1;
input [sig_width+exp_width : 0] inst_a;
input [sig_width+exp_width : 0] inst_b;
input [2 : 0] inst_rnd;
output [sig_width+exp_width : 0] z_inst;
output [7 : 0] status_inst;
 // Instance of DW_fp_mult
 DW_fp_mult #(sig_width, exp_width, ieee_compliance)
 U1 ( .a(inst_a), .b(inst_b), .rnd(inst_rnd), .z(z_inst), .status(status_inst) );
endmodule

module DW_fp_flt2i_inst( inst_a, inst_rnd, z_inst, status_inst );
parameter inst_sig_width = 23;
parameter inst_exp_width = 8;
parameter inst_isize = 32;
parameter inst_ieee_compliance = 0;
input [inst_sig_width+inst_exp_width : 0] inst_a;
input [2 : 0] inst_rnd;
output [inst_isize-1 : 0] z_inst;
output [7 : 0] status_inst;
 // Instance of DW_fp_flt2i
 DW_fp_flt2i #(inst_sig_width, inst_exp_width, inst_isize, inst_ieee_compliance) U1
(
.a(inst_a),
.rnd(inst_rnd),
.z(z_inst),
.status(status_inst) );
endmodule

module DW_fp_i2flt_inst( inst_a, inst_rnd, z_inst, status_inst );
parameter sig_width = 23;
parameter exp_width = 8;
parameter isize = 32;
parameter isign = 1;
input [isize-1 : 0] inst_a;
input [2 : 0] inst_rnd;
output [sig_width+exp_width : 0] z_inst;
output [7 : 0] status_inst;
 // Instance of DW_fp_i2flt
 DW_fp_i2flt #(sig_width, exp_width, isize, isign)
 U1 ( .a(inst_a), .rnd(inst_rnd), .z(z_inst), .status(status_inst) );
endmodule

module DW_fp_mac_inst( inst_a, inst_b, inst_c, inst_rnd, z_inst, status_inst );
parameter inst_sig_width = 23;
parameter inst_exp_width = 8;
parameter inst_ieee_compliance = 0;
input [inst_sig_width+inst_exp_width : 0] inst_a;
input [inst_sig_width+inst_exp_width : 0] inst_b;
input [inst_sig_width+inst_exp_width : 0] inst_c;
input [2 : 0] inst_rnd;
output [inst_sig_width+inst_exp_width : 0] z_inst;
output [7 : 0] status_inst;
 // Instance of DW_fp_mac
 DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) U1 (
.a(inst_a),
.b(inst_b),
.c(inst_c),
.rnd(inst_rnd),
.z(z_inst),
.status(status_inst) );
endmodule
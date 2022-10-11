// module FPU_top(
//     input logic Clk, rst, 
//     input logic [31:0] rs1, rs2, 
//     input logic FPU_sel, 
//     output logic [31:0] alu_out 

// ); 


// logic [7:0] status_inst; 

// DW_fp_sqrt_inst DW_fp_sqrt_inst(

// .inst_a(rs1), 
// .inst_rnd(3'b111), 
// .z_inst(alu_out), 
// .status_inst(status_inst)
// ); 







// // logic [31:0] rs1_class, rs2_class; 
// // logic [31:0] f_compare; 
// // enum [7:0] {ADD, MULT, SUB, DIV} curr_state, next_state; 
// // can use the classifer for helping other modules 
// // FPU_classifer FPU_classifer_rs1(
// //     //input 
// //     .f_num(rs1), 
    
// //     //output 
// //     .f_num_class(rs1_class)
// // ); 
// // FPU_classifer FPU_classifer_rs2(
// //     //input 
// //     .f_num(rs2), 
    
// //     //output 
// //     .f_num_class(rs2_class)
// // ); 
// // FPU_compare FPU_compare(
// //     //input 
// //     .f_num_a(rs1), .f_num_b(rs2),
// //     .f_class_a(rs2_class), .f_class_b(rs2_class), 
// //     //output 
// //     .f_compare(f_compare)
// // );

// // always_comb begin 

// //     unique case(FPU_sel)
// //     ADD:begin 
// //     end
// //     MULT:begin 
// //     end 
// //     SUB:begin 
// //     end 
// //     DIV:begin 
// //     end  



// //     endcase  


// // end 






// endmodule 


// module DW_fp_sqrt_inst( inst_a, inst_rnd, z_inst, status_inst );
// parameter inst_sig_width = 23;
// parameter inst_exp_width = 8;
// parameter inst_ieee_compliance = 0;
// input [inst_sig_width+inst_exp_width : 0] inst_a;
// input [2 : 0] inst_rnd;
// output [inst_sig_width+inst_exp_width : 0] z_inst;
// output [7 : 0] status_inst;
//  // Instance of DW_fp_sqrt
//  DW_fp_sqrt #(inst_sig_width, inst_exp_width, inst_ieee_compliance) U1 (
//  .a(inst_a),
//  .rnd(inst_rnd),
//  .z(z_inst),
//  .status(status_inst) );
// endmodule
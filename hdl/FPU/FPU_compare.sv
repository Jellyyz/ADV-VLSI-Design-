// // University of Illinois @ Urbana-Champaign ECE498HK
// // BLOCK NAME : FPU_compare 
// // This block allows for a two 32 bit inputs to be compared with one another, 
// // outputting an encoded of the status of the compare 
// // Key for f_num_class :
// // equal to 
// // greater than 
// // less than 
// // cmp_op 
// // 000 less than or equal to 
// // 001 less than 
// // 010 equal to 

// // set result to be 0 is either numbers are NaN
// // exception is raised if either numbers are NaN
// // and the operation being done is 001 | 000 
// import FPU_class_types::*;

// module FPU_compare(
//     input logic [31:0] f_num_a, f_num_b,
//     input logic [31:0] f_class_a, f_class_b, 
//     input logic [2:0] compare_op,  
//     output logic [4:0] f_compare 
// ); 


// always_comb begin 
//     unique case(cmp_op)
//     3'b000:begin 
//         f_compare = (f_num_a < f_num_b || f_num_a == f_num_b) ? 1 : 0; 
//     end 
//     3'b001:begin 
//         f_compare = (f_num_a < f_num_b) ? 1 : 0; 
//     end 
//     3'b010:begin 
//         f_compare = (f_num_a == f_num_b) ? 1 : 0; 
//     end 
//     default begin 
//         $display("Inputted wrong compare_op")
//     end     
//     f_compare = (f_class_a == SNAN || f_class_a == QNAN || f_class_b == SNAN || f_class_b == QNAN ) ? 1 : 0; 

// end 



// endmodule 

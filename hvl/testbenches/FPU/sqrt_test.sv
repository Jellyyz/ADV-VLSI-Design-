// module testbench();

// // taken from FPU/FPU_sqrt.sv
// parameter inst_sig_width = 23;
// parameter inst_exp_width = 8;
// parameter inst_ieee_compliance = 0;
// input [inst_sig_width+inst_exp_width : 0] inst_a;
// input [2 : 0] inst_rnd;
// output [inst_sig_width+inst_exp_width : 0] z_inst;
// output [7 : 0] status_inst;


// DW_fp_sqrt_inst dut(
//     .inst_a(inst_a), 
//     .inst_rnd(inst_rnd), 
//     .z_inst(z_inst), 
//     .status_inst(status_inst)
// ); 

// // inst_a = data to be taken sqrt of
// // inst_rnd = how well should the answer be rounded 
// // z_inst = square root answer of a 
// // state flags 
//     // 0 - zero 
//     // 1 - infinity 
//     // 2 - invalid operation 
//     // 3 - tiny number, magnitude smaller than the minimum normalized number 
//     // 4 - huge number, magnitude larger than the max normalized number 
//     // 5 - inexact, the number outputted is not equal to the infinitely precise result 
//     // 6 - hugeint, the number is greater than the largest 2s complement integer with the same sign 
//     // 7 - comp, this is some operation specific flag 

// logic sqrt_ans; 
// real i; 
// task pos_num();
//     for(real i; i < 2 ** inst_sig_width; i++)begin
//         inst_a <= i;
//         sqrt_ans <= z_inst; 
//         check_result(i, sqrt_ans); 
//     end 
// endtask : pos_num

// task check_result(real i, real sqrt_ans); 
    
//     if(sqrt(i) != sqrt_ans)begin
//         $display("Incorrect ans detected at time: ", $time); 
//         $display("Inputted: %f, Outputted: %f, expected: %f", i, sqrt_ans, sqrt(i)); 
//         $finish; 
//     end 

// endtask : check_result 

// initial begin 
//     pos_num(); 
//     $display("Successfully passed all test cases."); 
//     $finish; 

// end 


// endmodule : testbench 

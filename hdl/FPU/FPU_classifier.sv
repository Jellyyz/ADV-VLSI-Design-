// University of Illinois @ Urbana-Champaign ECE498HK
// BLOCK NAME : FPU_Classifer 
// This block allows for a 32 bit input to be classified into different 
// classes based on the type of the 32 bit input 
// Key for f_num_class :
// 0 - negative infinity 
// 1 - negative normal 
// 2 - negative subnormal 
// 3 - negative zero 
// 4 - positive zero
// 5 - positive subnormal 
// 6 - positive normal 
// 7 - positive infinity 
// 8 - sNaN
// 9 - qNaN

package FPU_class_types; 
typedef enum logic [9:0]{
    N_INF = 10'b0000000001, 
    N_NORMAL= 10'b0000000010, 
    N_SUBNORMAL = 10'b0000000100,
    N_ZERO = 10'b0000001000,
    P_ZERO = 10'b0000010000, 
    P_SUBNORMAL = 10'b0000100000,
    P_NORMAL = 10'b0001000000, 
    P_INF = 10'b0010000000, 
    SNAN = 10'b0100000000,
    QNAN = 10'b1000000000
} fclass_enum; 
endpackage

module FPU_classifer(
    input logic [31:0] f_num,
    output logic [31:0] f_num_class
); 



logic sign; 
logic [7:0] exponent; 
logic [24:0] mantissa; 
logic expAllOne, expAllZero, mantissaZero, d1;

// fclass_enum debug_out;

// spread out the input number to start classifying them 
assign sign = f_num[31]; 
assign exponent = f_num[30:23]; 
assign mantissa = f_num[22:0];
assign expAllOne = (exponent == '1) ? 1'b1: 1'b0;
assign expAllZero = (exponent == '0) ? 1'b1: 1'b0;
assign mantissaZero = (mantissa == '0) ? 1'b1: 1'b0;
assign d1 = mantissa[22];

assign f_num_class[0] = sign  & expAllOne                 & mantissaZero;
assign f_num_class[1] = sign  & ~expAllOne & ~expAllZero;
assign f_num_class[2] = sign               & expAllZero   & ~mantissaZero;
assign f_num_class[3] = sign               & expAllZero   & mantissaZero;
assign f_num_class[4] = ~sign              & expAllZero   & mantissaZero;
assign f_num_class[5] = ~sign              & expAllZero   & ~mantissaZero;
assign f_num_class[6] = ~sign & ~expAllOne & ~expAllZero;
assign f_num_class[7] = ~sign & expAllOne                 & mantissaZero;
assign f_num_class[8] =         expAllOne                 & ~mantissaZero & ~d1;
assign f_num_class[9] =         expAllOne                 & ~mantissaZero & d1;
// bits [31:10] are hard coded to 0 
assign f_num_class[31:10] = 21'b0; 

// assign out_debug = fclass_enum'(f_num_class);

endmodule 

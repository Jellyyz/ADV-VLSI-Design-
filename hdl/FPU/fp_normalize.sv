import fp_types::*; 

module fp_normalize(
    input IEEE_fp x1,
    input IEEE_fp x2,
    input logic [7:0] shamt, // The amount that we need to shift by to normalize
    input fp_op_t op, 
    output IEEE_extended_fp normalized_x2

);

normalized_x2.mant = {1'b1, x2.mant}>>shamt;
normalized_x2.exp = x2.exp;


endmodule
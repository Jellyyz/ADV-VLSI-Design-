package fp_types;
localparam PRECISION = 3

struct packed{
    logic sign_bit;
    logic[7:0] exp;
    logic [22:0] mant;
}IEEE_fp;

struct packed{
    logic sign_bit;
    logic[7:0] exp;
    logic [22 + PRECISION:0] mant;
}IEEE_extended_fp; //more prceision so we can round later, can add more bits later if needed

typedef union packed{
    logic [31:0] raw_fp_num;
    IEEE_fp ieee_fp;
}IEEE_fp_union;

typedef logic[3:0]{
    fp_add = 3'b000,
    fp_sub = 3'b001,
    fp_mul = 3'b010,
    fp_div = 3'b011,
    fp_class = 3'b100,
    fp_comp = 3'b101,
    fp_sqrt = 3'b110,
    fp_conv = 3'b111
} fp_op_t;

endpackage : fp_types


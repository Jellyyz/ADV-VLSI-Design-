package fp_types;

struct packed{
    logic sign_bit;
    logic[7:0] exp;
    logic [22:0] mant;
}IEEE_fp;

typedef union packed{
    logic [31:0] raw_fp_num;
    IEEE_fp ieee_fp;
}IEEE_fp_union;

endpackage : fp_types


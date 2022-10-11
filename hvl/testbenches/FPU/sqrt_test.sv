module testbench();
timeunit 1ns;
timeprecision 1ns;

// taken from https://github.com/zacharymikel/ieee-convert/blob/master/float.v
`define MAX_DEC_LENGTH 10
`define REG_WIDTH 31
`define BASE_EXP_VAL 127
`define MANTISSA_MSB 22


// taken from FPU/FPU_sqrt.sv
parameter inst_sig_width = 23;
parameter inst_exp_width = 8;
parameter inst_ieee_compliance = 0;
logic clk; 
logic [inst_sig_width+inst_exp_width : 0] inst_a;
logic [2 : 0] inst_rnd;
logic [inst_sig_width+inst_exp_width : 0] z_inst;
logic [7 : 0] status_inst;
logic [31:0] global_sqrt_input; 
logic [31:0] global_sqrt_output; 
// Toggle the clock
// #1 means wait for a delay of 1 timeunit
always begin : CLOCK_GENERATION
#1 clk = ~clk;
end

initial begin: INITVALS
    clk = 0;
end 

// write the results into a fsdb file 
initial begin
    $fsdbDumpfile("dump.fsdb");
    $fsdbDumpvars();
end
DW_fp_sqrt_inst DW_fp_sqrt_inst(

.inst_a(inst_a), 
.inst_rnd(inst_rnd), 
.z_inst(z_inst), 
.status_inst(status_inst)
); 


// inst_a = data to be taken sqrt of
// inst_rnd = how well should the answer be rounded 
// z_inst = square root answer of a 
// state flags 
    // 0 - zero 
    // 1 - infinity 
    // 2 - invalid operation 
    // 3 - tiny number, magnitude smaller than the minimum normalized number 
    // 4 - huge number, magnitude larger than the max normalized number 
    // 5 - inexact, the number outputted is not equal to the infinitely precise result 
    // 6 - hugeint, the number is greater than the largest 2s complement integer with the same sign 
    // 7 - comp, this is some operation specific flag 
initial begin 
    forever begin
        clk = 0;
        #10 clk = ~clk;
    end 
end

task check_sqrt();
input [31:0] operand; 
input [2:0] round;
output [31:0] ans; 
output [7:0] status;  
    #5; 
    inst_rnd = round; 
    inst_a = operand;
    ans = z_inst; 
    status = status_inst;
endtask : check_sqrt



// grab sign bit 
function set_sign;
input [31:0] data;
begin
    set_sign = data[31];
end
endfunction

// set the exponent value 
function [7:0] set_msb_index;
input [31:0] data;
reg   [31:0] result; 
integer i;
begin
    result = -1;
    
    if(data[31] == 0) begin 
        // find the most significant 1 bit after the sign
        for(i = `REG_WIDTH; i >= 0 && result == -1; i--) begin
            if(data[i] == 1)
                result = i;
        end
    end

    else if(data[31] == 1) begin
        // find the most significant 1 bit after the sign
        for(i = 0; i <= `REG_WIDTH && result == -1; i++) begin
            if(data[i] == 1)
                result = i;
        end

    end

    if(result == -1) begin 
        result = 0; 
    end 
    
    set_msb_index = result; 
end
endfunction

// convert rhs argument to fractional binary value 
function [31:0] convert_rhs; 
input [31:0] data;
reg   [31:0] result; 
integer i; 
integer max; 
begin

    max = 0;

    // find base 10 that is larger than our rhs 
    for(i = 1; i < `MAX_DEC_LENGTH && max == 0; i++) begin
        if((10 ** i) > data)
            max = 10 ** i;
    end

    result = 32'b0;

    // use the multiple + push technique to generate a binary fractal number
    for(i = 0; i <= `REG_WIDTH; i++) begin

        // multiply the decimal num by 2 
        data = data * 2;

        // shift our binary fraction left each time
        result = result << 1;

        // if dec result was > than e.g. 100, we push a 1
        if(data >= max) begin
            data = data - max;
            result = result | 1'b1;
        end

        // else we push a 0 
        else begin
            result = result | 1'b0; 
        end

    end
    convert_rhs = result; 
end
endfunction

task convert;
// main program variables
input [31:0] lhs; // Left had side of the decimal number.
input [31:0] rhs; // Right hand side of the decimal number.
input hello; 
reg [31:0] res; // Resulting IEEE 754 value

integer rhs_decimal; 
integer left_msb_index;
integer right_msb_index;
integer lhs_mask; 
integer rhs_mask;
integer sign; 
integer i;
begin
    
    rhs_decimal = rhs;

    lhs_mask = 0;
    rhs_mask = 0; 
    sign = 0;

    if(lhs[31] == 1) begin 
        lhs = ~(lhs - 1);
        sign = 1'b1;
    end

    // find most sigificant 1-bit on lhs
    left_msb_index = set_msb_index(lhs);

    // convert rhs to binary fraction
    // find most significant 1-bit on rhs  
    rhs = convert_rhs(rhs);

    right_msb_index = set_msb_index(rhs);

    if(lhs != 0) begin 

        // set mask for lhs 
        for(i = 0; i < left_msb_index; i++)
            lhs_mask[i] = 1'b1;

        res[22:0] = (lhs & lhs_mask) << ((`MANTISSA_MSB - left_msb_index) + 1);
        res[22:0] = res[22:0] | (rhs >> (left_msb_index + 9));

        // set the last bit to 1 to round up 
        if(right_msb_index > `MANTISSA_MSB) begin 
            for(i = right_msb_index - `MANTISSA_MSB; i >= 0; i--)
                if(rhs[i] == 1)
                    res[0] = 1;
        end 

        if(sign == 0)
            sign = set_sign(lhs);
        res[31] = sign;

        // exponent
        res[30:23] = 127 + left_msb_index;
        //$display("Converted: %0d\.%0d = %b", lhs, rhs_decimal, res);
        if(hello)
            global_sqrt_input = res; 
    
        else 
            global_sqrt_output = res; 
    end

end
endtask


int random_negative_inputa; 
int random_negative_inputb; 
int random_positive_inputa; 
int random_positive_inputb; 

initial begin 

    logic [2:0] round; 
    logic [31:0] ans;
    logic [7:0] status = 69; 
    $display("This is a test for FP_sqrt functionality.");
    $display("FP_sqrt has input ROUND && OPERATOR. It has output ANS && STATUS.");
    check_sqrt(0, 0, ans, status);
    $display("Starting +0 tests... ");
    $display("For +0:");
    for(round = 3'b0; round <= (3'b11); round++)begin 
        check_sqrt(32'b00000000000000000000000000000000, round, ans, status);
        $display("32'b00000000000000000000000000000000; Round = %b, ANS = %b, Status = %b", round, ans, status); 
    end 

    $display("Starting -0 tests... ");
    $display("For -0:");
    for(round = 3'b0; round <= (3'b11); round++)begin 
        check_sqrt(32'b10000000000000000000000000000000, round, ans, status);
        $display("32'b10000000000000000000000000000000; Round = %b, ANS = %b, Status = %b", round, ans, status); 
    end 

    $display("Starting -Infinity tests... ");
    $display("For -0:");
    for(round = 3'b0; round <= (3'b11); round++)begin 
        check_sqrt(32'b11111111100000000000000000000000, round, ans, status);
        $display("32'b11111111100000000000000000000000; Round = %b, ANS = %b, Status = %b", round, ans, status); 
    end 

    $display("Starting +Infinity tests... ");
    $display("For -0:");
    for(round = 3'b0; round <= (3'b11); round++)begin 
        check_sqrt(32'b01111111100000000000000000000000, round, ans, status);
        $display("32'b01111111100000000000000000000000; Round = %b, ANS = %b, Status = %b", round, ans, status); 
    end 


    $display("Starting -Subnormal tests... ");
    $display("For -0:");
    for(round = 3'b0; round <= (3'b11); round++)begin 
        check_sqrt(32'b10000000010000000000000000000000, round, ans, status);
        $display("32'b10000000010000000000000000000000; Round = %b, ANS = %b, Status = %b", round, ans, status); 
    end 

    $display("Starting +Subnormal tests... ");
    $display("For -0:");
    for(round = 3'b0; round <= (3'b11); round++)begin 
        check_sqrt(32'b00000000010000000000000000000000, round, ans, status);
        $display("32'b00000000010000000000000000000000; Round = %b, ANS = %b, Status = %b", round, ans, status); 
    end 

    $display("Starting -sNaN tests... ");
    $display("For -0:");
    for(round = 3'b0; round <= (3'b11); round++)begin 
        check_sqrt(32'b11111111101000000000000000000000, round, ans, status);
        $display("32'b11111111101000000000000000000000; Round = %b, ANS = %b, Status = %b", round, ans, status); 
    end 

    $display("Starting +sNaN tests... ");
    $display("For -0:");
    for(round = 3'b0; round <= (3'b11); round++)begin 
        check_sqrt(32'b01111111101000000000000000000000, round, ans, status);
        $display("32'b01111111101000000000000000000000; Round = %b, ANS = %b, Status = %b", round, ans, status); 
    end 

    $display("Starting -qNaN tests... ");
    $display("For -0:");
    for(round = 3'b0; round <= (3'b11); round++)begin 
        check_sqrt(32'b11111111110000000000000000000000, round, ans, status);
        $display("32'b11111111110000000000000000000000; Round = %b, ANS = %b, Status = %b", round, ans, status); 
    end 

    $display("Starting +qNaN tests... ");
    $display("For -0:");
    for(round = 3'b0; round <= (3'b11); round++)begin 
        check_sqrt(32'b01111111110000000000000000000000, round, ans, status);
        $display("32'b01111111110000000000000000000000; Round = %b, ANS = %b, Status = %b", round, ans, status); 
    end 

    $display("Starting random negative number test... ");
    $display("For +0:");
    random_negative_inputa =  ($urandom % 10000000);
    $display("LHS = %d", random_negative_inputa); 
    random_negative_inputb =  ($urandom % 10000000);
    $display("RHS = %d", random_negative_inputb); 
    convert(-random_negative_inputa, random_negative_inputb , 1); 

    for(round = 3'b0; round <= (3'b11); round++)begin 
        check_sqrt(global_sqrt_input, round, ans, status);
        $display("Binary: %b; Round = %b, ANS = %b, Status = %b", global_sqrt_input, round, ans, status); 
    end 

    $display("Starting random positive numbertest... ");
    $display("For +0:");
    random_negative_inputa =  ($urandom % 10000000);
    $display("LHS = %d", random_negative_inputa); 
    random_negative_inputb =  ($urandom % 10000000);
    $display("RHS = %d", random_negative_inputb); 
    convert(random_negative_inputa, random_negative_inputb , 1); 

    for(round = 3'b0; round <= (3'b11); round++)begin 
        check_sqrt(global_sqrt_input, round, ans, status);
        $display("Binary: %b; Round = %b, ANS = %b, Status = %b", global_sqrt_input, round, ans, status); 
    end 

    $display("state flags:");
    $display("0 - zero ");
    $display("1 - infinity ");
    $display("2 - invalid operation ");
    $display("3 - tiny number, magnitude smaller than the minimum normalized number ");
    $display("4 - huge number, magnitude larger than the max normalized number ");
    $display("5 - inexact, the number outputted is not equal to the infinitely precise result ");
    $display("6 - hugeint, the number is greater than the largest 2s complement integer with the same sign ");
    $display("7 - comp, this is some operation specific flag ");
    $display("Finished tests..."); 
    $finish;
end 

endmodule : testbench 

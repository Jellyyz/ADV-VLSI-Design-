import fp_types::*

module random_float_gen()

class Rndom_float;

    function IEEE_fp subnormal();
        IEEE_fp rand_fp;
        rand_fp.sign_bit = $urandom();
        rand_fp.exp = '0;
        rand_fp.mant = $urandom();
        if(rand_fp.mant == '0) begin
            rand_fp.mant = 1'b1;
        end
        
        return rand_fp;
    endfunction

    function IEEE_fp normal();
        IEEE_fp rand_fp;
        rand_fp.sign_bit = $urandom();
        rand_fp.exp = $urandom();
        if(rand_fp.exp == '1) begin
            rand_fp.mant = 8'b11111110;
        end
        if(rand_fp.exp == '0) begin
            rand_fp.mant = 8'b00000001;
        end
        rand_fp.mant = $urandom();

        return rand_fp;
    endfunction

    function IEEE_fp zero();
        IEEE_fp rand_fp;
        rand_fp.sign_bit = $urandom();
        rand_fp.exp = '0;
        rand_fp.mant = '0;
        return rand_fp;
    endfunction

    function IEEE_fp inf();
        IEEE_fp rand_fp;
        rand_fp.sign_bit = $urandom();
        rand_fp.exp = '1;
        rand_fp.mant = '0;
        return rand_fp;
    endfunction

    function IEEE_fp qNaN();
        IEEE_fp rand_fp;
        rand_fp.sign_bit = $urandom();
        rand_fp.exp = '1;
        rand_fp.mant = '0;
        rand_fp.mant[22] = 1'b1;

        return rand_fp;
    endfunction


    function shortreal fp_to_dec(IEEE_fp fp_no);
        shortreal ans = 2^(fp_no.exp-128)
        ans = ans*int'(fp_no.mant);
        ans = ans*2^(-23)
        ans = sign_bit == 1'b1 ? ans*(-1) : ans;

        return ans;
    endfunction
endclass


Rndom_float rand_flt = new();

task random_float();
    logic[2:0] rand_seed = $urandom();
    IEEE_fp rand_no;
    case (rand_seed%5):
        3'h0: begin
            rand_no = rand_flt.normal();
        end
        3'h1: begin
            rand_no = rand_flt.subnormal();
        end
        3'h2: begin
            rand_no = rand_flt.zero();
        end
        3'h3: begin
            rand_no = rand_flt.inf();
        end
        3'h4: begin
            rand_no = rand_flt.qNaN();
        end
        default: begin
            rand_no = rand_flt.zero();
        end
    endcase
    $display("\n");
    $display("Rand no- IEEE %b %b %b ", rand_no.sign_bit, rand_no.exp, rand_no.mant);
    $display("Rand no- FP %f: ", rand_flt.fp_to_dec(rand_no))
    $display("\n");
endtask

initial begin
    repeat (80) begin
        random_float();
    end
end

endmodule : random_float_gen
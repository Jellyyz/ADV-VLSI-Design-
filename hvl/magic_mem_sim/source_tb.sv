`ifndef SOURCE_TB
`define SOURCE_TB

`define MAGIC_MEM 0
`define PARAM_MEM 1
`define MEMORY `PARAM_MEM

// Set these to 1 to enable the feature
`define USE_SHADOW_MEMORY 1
`define USE_RVFI_MONITOR 1

// `include "tb_itf.sv"
`include "../hvl/magic_mem_sim/tb_itf.sv"
module source_tb(
    tb_itf.magic_mem magic_mem_itf,
    tb_itf.mem mem_itf,
    tb_itf.sm sm_itf,
    tb_itf.tb tb_itf
);

initial begin
    $display("Compilation Successful");
    tb_itf.path_mb.put("memory.lst");
    tb_itf.rst = 1'b1;
    repeat (5) @(posedge tb_itf.clk);
    tb_itf.rst = 1'b0;
end

/**************************** Halting Conditions *****************************/
int timeout = 10000;

always @(posedge tb_itf.clk) begin
    if (timeout == 0) begin
        $display("TOP: Timed out");
        $finish;
    end
    timeout <= timeout - 1;
end

/************************** End Halting Conditions ***************************/
`define PARAM_RESPONSE_NS 50 * 10
`define PARAM_RESPONSE_CYCLES $ceil(`PARAM_RESPONSE_NS / `PERIOD_NS)
`define PAGE_RESPONSE_CYCLES $ceil(`PARAM_RESPONSE_CYCLES / 2.0)

generate
    magic_memory_dp mem(magic_mem_itf);
endgenerate


endmodule

`endif

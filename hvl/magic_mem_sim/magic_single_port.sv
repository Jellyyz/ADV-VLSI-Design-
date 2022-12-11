/*
 * Dual-port magic memory
 *
 */


module magic_memory_sp
(
    tb_itf.magic_mem_single itf
);


logic [7:0] mem [logic [31:0]];

initial
begin
    string s;
    itf.path_mb.peek(s);
    $readmemh(s, mem);
end

always @(itf.mmscb)
begin : response
   
    //ONLY USe THE READ/WRITE - B ports
    // if (itf.mmscb.read_a) begin
    //     itf.mmscb.resp_a <= 1'b1;
    //     for (int i = 0; i < 4; i++) begin
    //         itf.mmscb.rdata_a[i*8 +: 8] <= mem[itf.mmscb.address_a+i];
    //     end
    // end else begin
    //     itf.mmscb.resp_a <= 1'b0;
    // end

    if (itf.mmscb.read) begin
        itf.mmscb.resp <= 1'b1;
        itf.mmscb.rdata[7:0] <= mem[itf.mmscb.addr];
    end else if (itf.mmscb.write) begin
        itf.mmscb.resp <= 1'b1;
        mem[itf.mmscb.addr] = itf.mmscb.wdata[7:0];
    end else begin
        itf.mmscb.resp <= 1'b0;
    end   
end : response

endmodule : magic_memory_sp

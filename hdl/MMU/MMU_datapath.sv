module MMU_datapath(
    // clk-rst interface
    input logic clk, rst, 

    
    input logic data_in_from_pmem, // inputs from pmem,  
    output logic counter,          // outputs to the control unit, can be used to determine if state needs to change 

  
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ~~~ PMEM/CPU interface signals ~~
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  


    // data coming from either the pmem or the on chip memory 
    output logic pmem_data_in,    // inputs from pmem data in, inputs into two places - data to icache and data to CPU

    // addresses to select between data addresses and instruction addr 
    input logic pmem_instr_addr_from_CPU, // outputs to pmem_addr_mux - contains instruction address for pmem
    input logic pmem_data_addr_from_CPU,  // outputs to pmem_addr_mux - contains data address for pmem 
    
    // physical memory interface 
    output logic addr_out_to_pmem,        // outputs to the pmem - contains the correct address for the pmem, comes from above 2 signals 

    // data outputs for icache / CPU from pmem 
    output logic pmem_data_to_icache, 
    output logic pmem_data_to_CPU, 


    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ~~~~~ to sram? lol idfk ~~~~~~~~
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    output logic sram_addr,               // outputs from mmu into sram, contains addr for sram 
    output logic sram_data,               // outputs from mmu into sram, contains data for sram 


    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ~~~~~ load and sel signals ~~~~~~
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    input logic ld_onchip_addr_reg,       // inputs from control, en for addr onchip reg 
    input logic ld_onchip_data_reg,       // inputs from control, allows for the onchip data reg to be loaded 
    input logic ld_pmem_addr_reg,         // inputs from control, allows for address register outputted to pmem to be chosen
    input logic ld_icache_data,           // inputs from control, allows for the icache_data_in to recieve a line of data 
    input logic ld_cpu_data,              // inputs from control, allows for the CPU to recieve some data 
    input logic counter_en,               // inputs from control, allows for counter to be enabled, else just output 0  
    input logic addr_reg_mux_sel,         // inputs from control, selects if address of pmem is inst_addr (0) or data_addr(1)
    input logic data_to_cpu_mux_sel,      // inputs from control, selects betw. pmem and onchip datas

    output logic counter_control_helper   // need to let the control unit the status of the counter 
);  

logic data_addr_reg_out;                   // contains the correct address to index into the Sram 
logic data_reg_out;                        // contains the correct write data into the sram, comes from the CPU 


logic [31:0] data_to_cpu_mux;              // contains the input into the data_to_cpu reg 
logic [31:0] addr_reg_mux_out;             // contains the input into the address for pmem 
logic [7:0] counter;                       // contains value of current counter 
logic counter_rst;                         // contains the reset for the counter 

logic [31:0] onchip_data_out_to_CPU;       // data from the data_mem from data_memory

assign addr_out_to_pmem = addr_reg_out + counter;  
assign counter_control_helper = counter; 


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~ MUX interfaces ~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
always_comb begin: ADDR_TO_PMEM_REG_MUX
    // 1 is data address, 0 is instruction address 
    addr_reg_mux_out = (addr_reg_mux_sel) ? pmem_instr_addr_from_CPU : pmem_data_addr_from_CPU; 
end 
always_comb begin: DATA_TO_CPU_MUX
    // 1 is data from the pmem- offchip, 0 is data from the onchip 
    data_to_cpu_mux = (data_to_cpu_mux_sel) ? pmem_data_in : onchip_data_out_to_CPU; 
end 


// @CHECK 
assign counter_rst = rst; // unsure ..? froom control?
always_ff @(posedge clk or posedge rst)begin: COUNTER   
    if(counter_rst)begin 
        counter <= 7'd0; 
    end     
    else if(counter_en) begin 
        counter <= counter + 1; 
    end 
    else begin 
        counter <= 8'h0; 
    end     
end 



register addr_reg(
    // inputs 
    .clk(clk), .rst(rst), 
    .load(ld_pmem_addr_reg), 
    .in(addr_reg_mux_out),

    // outputs 
    .out(addr_reg_out)
); 


register data_to_icache(
    // inputs 
    .clk(clk), .rst(rst), 
    .load(ld_icache_data),
    .in(pmem_data_in), 

    // outputs 
    .out(pmem_data_to_icache) 

); 
register data_to_cpu(
    // inputs 
    .clk(clk), .rst(rst), 
    .load(ld_cpu_data),
    .in(data_to_cpu_mux), 

    // outputs 
    .out(pmem_data_to_CPU) 

); 


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~ ON CHIP MEM ~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

register onchip_data_register(
    // inputs 
    .clk(clk), .rst(rst), 
    .load(ld_onchip_data_reg),
    .in(pmem_data_to_CPU), 

    // outputs 
    .out(sram_data) 

); 

register onchip_data_addr_register(
    // inputs 
    .clk(clk), .rst(rst), 
    .load(ld_onchip_addr_reg),
    .in(addr_reg_mux_out), 

    // outputs 
    .out(sram_addr) 

); 

// @CHECK 
// potential SRAM logic needed for main on chip memory data 
// also assuming the mux of the addr reg will be the addr of the sram for now pls fix if not
// also is the address of the on chip memory using the address register on the mmu? if so this is a very big 2 cycle delay no?
// if that is that case a lot of thigns needs to be changed.. 

endmodule 


// from ece411

module register #(parameter width = 32)
(
    input clk,
    input rst,
    input load,
    input [width-1:0] in,
    output logic [width-1:0] out
);

logic [width-1:0] data;

always_ff @(posedge clk)
begin
    if (rst)
    begin
        data <= '0;
    end
    else if (load)
    begin
        data <= in;
    end
    else
    begin
        data <= data;
    end
end

always_comb
begin
    out = data;
end

endmodule : register

module MMU_control(
    // clk-rst interface 
    input logic clk, rst, 

    // recieved an instruction request (read only)
    input logic instruction_req, 

    // on-off chip r/w 
    input logic on_c_data_read, on_c_data_write, 
    input logic off_c_data_read, off_c_data_write, 
    // counter to compare to in order to check if entire data has come in 
    input logic width_of_instruction, 

    // on-off chip resp 
    input logic onc_resp, offc_resp, 

    // MMU response 
    output logic mmu_resp,                // outputs from control, inputs into reciever to let them know action is needed

    // load addr_reg
    output logic addr_reg,

    // counter stuff
    output logic counter_rst, 
    input logic counter_control_helper,   // input from datapath, curr state of counter 



    // on-chip memory loads 
    output logic ld_onchip_data_reg,       // outputs from control, allows for the on chip data reg to be loaded, to perform any writes to onc
    output logic ld_onchip_addr_reg,       // outputs from control, allows for the on chip addr reg ot be loaded 
    
    output logic ld_pmem_addr_reg,         // outputs from control, allows for address register outputted to pmem to be chosen
    output logic ld_icache_data,           // outputs from control, allows for the icache_data_in to recieve a line of data 
    output logic ld_cpu_data,              // outputs from control, allows for the CPU to recieve some data 
    output logic addr_reg_mux_sel,         // outputs from control, selects if address of pmem is inst_addr (0) or data_addr(1)
    output logic data_to_cpu_mux_sel       // outputs from control, selects betw. pmem and onchip datas


); 
enum logic [4:0] {

    RST, 
    IDLE,
    RESPOND, 
    OFF_C_R, OFF_C_W, 
    ON_C_R, ON_C_W, 
    REQ_INST

} curr_state, next_state; 
always_ff @ (posedge clk or posedge rst)begin

    if(rst)begin 
        curr_state <= RST; 
    end 
    else begin 
        curr_state <= next_state; 
    end 
    
end

always_comb begin : CONDITION_FOR_NEXT_STATE 
    next_state = curr_state; 

    unique case(curr_state)
        RST:begin 
            next_state = IDLE;
        end 
        IDLE:begin
            if(instruction_req)begin 
                next_state = REQ_INST; 
            end  
            else if(on_c_data_read)begin 
                next_state = ON_C_R; 
            end 
            else if(on_c_data_write)begin 
                next_state = ON_C_W; 
            end 
            else if(off_c_data_read)begin 
                next_state = OFF_C_R; 
            end 
            else if(off_c_data_write)begin 
                next_state = OFF_C_W; 
            end
        end 
        ON_C_R, ON_C_W:begin 
            if(onc_resp)begin 
                next_state = RESP; 
            end 
        end 
        OFF_C_R, OFF_C_W:begin
            if(offc_resp)begin 
                next_state = RESP; 
            end  
        end 
        REQ_INST:begin 
            if(counter == width_of_instruction)begin 
                next_state = RESP; 
            end 
        end 

        RESP: begin 
            next_state = IDLE; 
        end 
    endcase 

end 

always_comb begin: CONTROL_SIGNAL
    // default cases 
    counter_rst = 1'b0; 
    mmu_resp = 1'b0; 
    ld_onchip_addr_reg = 1'b0; 
    ld_onchip_data_reg = 1'b0; 
    ld_pmem_addr_reg = 1'b0;
    ld_icache_data = 1'b0; 
    ld_cpu_data = 1'b0; 
    data_to_cpu_mux_sel = 1'b0; // 1 is data address, 0 is instruction address 
    addr_reg_mux_sel = 1'b0;    // 1 is data from the pmem- offchip, 0 is data from the onchip 
    unique case
        RST:begin 
            counter_rst = 1'b1; 
        end 
        IDLE:begin
            ; // do nothing, wait for requests 
        end 
        RESPOND:begin 
            mmu_resp = 1'b1; 
        end 

// Important states :) 
// for all of these counter needs to be on since we need to know when MMU will stop transmitting data 
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ~~~~~~ on chip states ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // on chip memory read 
        // 1. to data cpu mux needs to accept on chip memory data - defaulted already 
        // 2. since onc is expecting data, should switch addr reg mux to be data - defaulted already
        // 3. ld data to cpu reg  
        // 4. ld onc addr reg = 1  
        ON_C_R:begin 
            ld_cpu_data = 1'b1; 
            ld_onchip_addr_reg = 1'b1; 
            counter_en = 1'b1; 
        end 
        // on chip memory write 
        // 1. ld_onchip_data_reg is high 
        // 2. since onc is expecting data, should switch addr reg mux to be data - defaulted
        // 3. ld_onchip_addr_reg 
        ON_C_W:begin 
            ld_onchip_data_reg = 1'b1; 
            ld_onchip_addr_reg = 1'b1; 
            counter_en = 1'b1; 
        end 
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ~~~~~ off chip states ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // performing an off chip memory read (2 cycles):
        // 1. switch the addr mux to be data_addr -defaulted 
        // 2. addr register ld on 
        // 3. allow counter to now be enabled 
        // 4. data to cpu mux comes from pmem 
        // 5. data to cpu reg ld on 
        OFF_C_R:begin 
            ld_pmem_addr_reg = 1'b1;
            counter_en = 1'b1; 
            data_to_cpu_mux_sel = 1'b1;
            ld_cpu_data = 1'b1; 
        end 
        // @CHECK
        // performing an off chip memory write 
        // 1. switch addr mux to be data_addr -0defaulted 
        // 2. addr reg on 
        // 3. turn on data reg from on chip memory? wtf is this datapath lul - no other sources for data in for pmem
        OFF_C_W:begin  
            ld_pmem_addr_reg = 1'b1;
            ld_onchip_data_reg = 1'b1; 
            counter_en = 1'b1; 
        end     

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ~~~~~~~~~~~~~~~ instruction req ~~~~~~~~~~~~~~~~~~~~~~
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // DOES THIS GO INTO DATA_TO_CPU OR WHAT WTF DATAPATH SCUFFED AF
        // where tf does data come from, pmem?
        // @CHECK 
        // requested an instruction - comes from pmem and outputs to data to CPU always?  
        // 1. instr addr in the addr mux 
        // 2. ld addr into addr reg
        // 3. ld data into cpu reg 
        // 4. choose pmem in data to cpu reg mux 

        REQ_INST:begin 
            addr_reg_mux_sel = 1'b1; 
            ld_pmem_addr_reg = 1'b1; 
            ld_cpu_data = 1'b1; 
            data_to_cpu_mux_sel = 1'b1; 
            counter_en = 1'b1;
        end 
        default:begin 
            $display("Waiting for rst"); 
        end 
    endcase 
end 
endmodule 
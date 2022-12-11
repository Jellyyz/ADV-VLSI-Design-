module mmu #(
    DATA_MEM_START = 16'h8000
)
(
    input clk, rst_ni,

    //instruction signals to/from CPU

    input instr_req_i, //request for instruction
    output logic instr_gnt_o,  //request ack (address buffered?)
    output logic instr_rvalid_o, //data is valid
    input logic [31:0] instr_addr_i,  // instruction address
    output logic [31:0] instr_rdata_o,  // instruction read data

    //data signals to/from CPU 
    //#TODO: Check the bit lengths of these and make sure they're correct

    input data_req_i,   
    output logic data_gnt_o,   
    output logic data_rvalid_o,
    input  data_we_i,    
    input logic [3:0] data_be_i,    
    input logic [31:0] data_addr_i,  
    input logic [31:0] data_wdata_i,
    output logic [31:0] data_rdata_o,

    //output signals to/from Ext. SRAM
    input logic [7:0] ext_sram_rdata_i,
    output logic [7:0] ext_sram_wdata_o,
    output logic [31:0] ext_sram_addr_o,
    output logic ext_sram_read_o,
    output logic ext_sram_write_o
     
);

enum logic [2:0] {
    IDLE, INSTR_R1, INSTR_R2, DATA_R1, DATA_R2, DATA_W1, DATA_W2 
}state, next_state;

//buffer the address_o signal and write_data signal
logic load_ext_sram_addr_o, load_ext_sram_wdata_o;

//buffer the rdata chunks we get from the Esram
logic [31:0] instr_rdata_buffer, data_rdata_buffer;

logic load_instr_rdata_buffer, load_data_rdata_buffer;
logic [1:0] instr_rdata_buffer_ld_idx, data_rdata_buffer_ld_idx, ext_sram_wdata_idx;

logic[2:0] counter; //offset from address that we will access next cycle. 
logic inc_counter, reset_counter;
logic currently_serving_instr;
logic currently_serving_data;


function void set_defaults();
    load_ext_sram_addr_o        = 1'b0;
    load_ext_sram_wdata_o       = 1'b0;
    load_instr_rdata_buffer     = 1'b0;
    load_data_rdata_buffer      = 2'b00;
    instr_rdata_buffer_ld_idx   = 2'b00;
    ext_sram_wdata_idx          = 2'b00;
    data_rdata_buffer_ld_idx    = 1'b0;
    instr_gnt_o                 = 1'b0;
    instr_rvalid_o              = 1'b0;
    data_gnt_o                  = 1'b0;
    data_rvalid_o               = 1'b0;
    ext_sram_read_o             = 1'b0;
    ext_sram_write_o            = 1'b0; 
    inc_counter                 = 1'b0;
    reset_counter               = 1'b0; 
endfunction



//===============DATAPATH ELEMENTS ===================

always_ff @(posedge clk) begin: COUNTER_LOGIC
    if(!rst_ni)
        counter<=3'b000;
    else begin
        unique case({reset_counter, inc_counter}) 
            2'b11, 2'b10: counter <= 3'b0;
            2'b00: counter<= counter;
            2'b01: counter <= counter +1;
        endcase
    end
end

always_ff @(posedge clk) begin: EXT_SRAM_ADDR_LOGIC
    unique case({load_ext_sram_addr_o, currently_serving_data, currently_serving_instr})
        3'b000,3'b001,3'b010,3'b011, 3'b100: ext_sram_addr_o <= ext_sram_addr_o;
        3'b110: ext_sram_addr_o <= data_addr_i + counter;
        3'b111, 3'b101: ext_sram_addr_o <= instr_addr_i + counter;
    endcase
end


always_ff @(posedge clk) begin : INSTR_RDATA_BUFFER_LOGIC
    if(load_instr_rdata_buffer)
        instr_rdata_buffer[(instr_rdata_buffer_ld_idx)*8+:8] <= ext_sram_rdata_i;
    else
        instr_rdata_buffer <= instr_rdata_buffer;
end

always_ff @(posedge clk) begin : DATA_RDATA_BUFFER_LOGIC
    if(load_data_rdata_buffer)
        data_rdata_buffer[(data_rdata_buffer_ld_idx)*8+:8] <= ext_sram_rdata_i;
    else
        data_rdata_buffer <= data_rdata_buffer;
end


//=============== END DATAPATH ELEMENTS ==============

assign currently_serving_data = data_req_i | next_state == DATA_R1 | next_state == DATA_W1;
assign currently_serving_instr = instr_req_i | next_state == INSTR_R1;

assign instr_rdata_o = instr_rdata_buffer;
assign data_rdata_o = data_rdata_buffer;
assign ext_sram_wdata_o = data_wdata_i[(ext_sram_wdata_idx)*8+:8];


always_comb begin: OUTPUT_LOGIC
    set_defaults();
    unique case(state)
        IDLE: begin
            // reset_counter = 1'b1; //reset the counter when you go to start
            if(instr_req_i | data_req_i) begin
                load_ext_sram_addr_o = 1'b1; //load address into buffer to initiate access next cycle
                inc_counter = 1'b1;
            end
        end
        INSTR_R1: begin
            instr_gnt_o = (counter == 2'b11); //set this to one to signal we are servicing the instruction request now
            ext_sram_read_o = 1'b1;
            inc_counter = (counter != 3'b100);
            instr_rdata_buffer_ld_idx = counter-1;
            load_instr_rdata_buffer = 1'b1;
            load_ext_sram_addr_o = 1'b1;
            //reset the counter before you do tht next thing
        end
        INSTR_R2: begin
            instr_rvalid_o = 1'b1;
            reset_counter = 1'b1;
        end

        DATA_R1: begin
            data_gnt_o = (counter == 2'b11); //set this to one to signal we are servicing the instruction request now
            ext_sram_read_o = 1'b1;
            inc_counter = (counter != 3'b100);
            data_rdata_buffer_ld_idx = counter-1;
            load_data_rdata_buffer = 1'b1;
            load_ext_sram_addr_o = 1'b1;
        end

        DATA_R2: begin
            data_rvalid_o = 1'b1;
            reset_counter = 1'b1; //reset the counter before you do tht next thing
        end

        DATA_W1: begin
            data_gnt_o = (counter == 2'b11); //set this to one to signal we are servicing the instruction request now
            ext_sram_write_o = data_be_i[counter-1]; //only write that byte to ext SRAM 
            inc_counter = (counter != 3'b100);
            load_ext_sram_addr_o = 1'b1;
            ext_sram_wdata_idx = counter-1;
        end

        DATA_W2: begin
            data_rvalid_o = 1'b1;
            reset_counter = 1'b1;
        end

    endcase
end

always_comb begin : NEXT_STATE_LOGIC
    unique case(state)
        IDLE: begin
            unique case({instr_req_i, data_req_i, data_we_i})
                3'b000, 3'b001: next_state = IDLE;
                3'b100, 3'b101, 3'b110, 3'b111: next_state = INSTR_R1;
                3'b010: next_state = DATA_R1;
                3'b011: next_state = DATA_W1;
            endcase
        end
        INSTR_R1: begin
            if(counter == 3'b100)
                next_state = INSTR_R2;
            else
                next_state = INSTR_R1;
        end
        INSTR_R2: begin
            next_state = IDLE;
        end
        DATA_R1: begin
            if(counter == 3'b100)
                next_state = DATA_R2;
            else
                next_state = DATA_R1;
        end
        DATA_R2: begin
            next_state = IDLE;
        end
        DATA_W1: begin
            if(counter == 3'b100)
                next_state = DATA_W2;
            else
                next_state = DATA_W1;
        end
        DATA_W2: begin
            next_state = IDLE;
        end
    endcase

end

    always_ff @(posedge clk) begin: NEXT_STATE_ASSIGNMENT
        if(!rst_ni)
            state <= IDLE;
        else
            state <= next_state;
    end

endmodule


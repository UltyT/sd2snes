`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:50:40 11/11/2017 
// Design Name: 
// Module Name:    spc7110_dcu_arbiter 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: SPC7110 DCU bus arbiter
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: This module controls the DCU's I/O ports and data ROM
//                      access. It also maintains a data buffer for DCU datarom
//                      access as well as decompressed output data.
//
//////////////////////////////////////////////////////////////////////////////////
module spc7110_dcu_arbiter(
    input CLK,
    input RESET,
    
    //PSRAM is single-ported, and not quite fast enough to service both the SFC
    //and the DCU at the same time, so SFC reads take priority. We also send our
    //own read signal, which can be used to preempt other, lower-priority access
    input sfc_rom_rd,           //SFC is reading PSRAM...
    output darb_rom_rd,         //DCU is reading PSRAM...
    
    //SFC I/O ports
    input darb_sfc_enable,      //SFC reads/writes map to DCU
    input darb_decomp_mirror,   //that stupid $50:0000-FFFF mirror
    input [3:0] sfc_dcu_port,
    input sfc_rd,
    input sfc_wr,
    input [7:0] sfc_data_in,
    output [7:0] sfc_data_out,
    
    //PSRAM control - for some reason we have a 16-bit data bus on sd2snes...
    input [15:0] psram_data,
    output [22:0] psram_addr
);

//Internal buffer RAM wiring
reg [10:0] buffer_pa_addr;
wire buffer_pa_we = 0;
wire buffer_pa_en;
wire [7:0] buffer_pa_datain = 8'h00;
wire [7:0] buffer_pa_dataout;

wire [10:0] buffer_pb_addr;
wire buffer_pb_we;
wire buffer_pb_en = 1;
wire [7:0] buffer_pb_datain;
wire [7:0] buffer_pb_dataout;

//Internal buffer RAM (8bitx2048)
//This is used to store both compressed data from ROM as well as decompressed
//data to be sent to the 65C816. It is structured as two ring buffers; the low
//buffer is filled with 1K of ROM data for the DCU to asynchronously consume,
//while the high buffer is filled with output from the DCU to be read from 4800
//TODO: is 1024 ring bytes enough?
spc7110_dcu_buffer rombuf (
    //RAM Port A: Exclusively used to service 65C816 I/O access (e.g. DMA)
    .clka(CLK),
    .wea(buffer_pa_we),
    .ena(buffer_pa_en),
    .addra(buffer_pa_addr),
    .dina(buffer_pa_datain),
    .douta(buffer_pa_dataout),
    
    //RAM Port B: DCU data prefetch (PSRAM -> Buffer -> DCU), DCU output buffer
    .clkb(CLK),
    .web(buffer_pb_we),
    .enb(buffer_pb_en),
    .addrb(buffer_pb_addr),
    .dinb(buffer_pb_datain),
    .doutb(buffer_pb_dataout)
);

//Ring Buffer Management
reg [10:0] darb_inbuf_wrloc; //Next byte to write to.
reg [10:0] darb_inbuf_rdloc; //Next byte to read from.
reg [10:0] darb_outbuf_wrloc;
reg [10:0] darb_outbuf_rdloc;

//Internal DCU wiring
reg dcu_init;
reg [1:0] dcu_init_mode;
wire dcu_datarom_wait;
wire dcu_datarom_rd;
wire [7:0] dcu_datarom_data;
wire dcu_ob_wait;
wire dcu_ob_wr;
wire [31:0] dcu_ob_data;

//Directory, pointer, and other useful registers go here.
reg [23:0] darb_directory_base;
reg [7:0] darb_directory_index;
reg [23:0] darb_index_ptr;
reg [23:0] darb_decompress_ptr;

parameter DARB_PSRAM_INACTIVE = 3'b001; //PSRAM not in use
parameter DARB_PSRAM_STORE    = 3'b010; //PSRAM data recieved, registering
parameter DARB_PSRAM_QUEUE    = 3'b100; //Writing registered data to buffer

reg [22:0] darb_psram_addr;
reg [3:0] darb_psram_ctr;
reg [2:0] darb_psram_state;
reg [15:0] darb_psram_data;

reg [2:0] darb_state;
reg [2:0] darb_state_ctr;

reg [23:0] darb_datarom_ptr;

reg [15:0] darb_length_ctr;
reg [15:0] darb_offset_ctr;

reg [31:0] darb_output_data;
reg [2:0] darb_output_ctr;

reg darb_bypass_dcu; //If enabled, DCU will remain in waitstate...

reg [7:0] darb_output;

//These are used for tracking IO access...
parameter DARB_STATE_MODEREAD = 3'b001; //read DIR into mode register
parameter DARB_STATE_ADDRREAD = 3'b010; //read DIR+1 into addr register
parameter DARB_STATE_READY    = 3'b100; //start buffering data, DCU is init'd

parameter DARB_PSRAM_TIMING = 4'd7; //70ns, plus a little more because cycles

//Ports are mapped to $4800 by address.v
parameter DARB_PORT_READ     = 4'h0;
parameter DARB_PORT_BASE0    = 4'h1;
parameter DARB_PORT_BASE1    = 4'h2;
parameter DARB_PORT_BASE2    = 4'h3;
parameter DARB_PORT_INDEX    = 4'h4;
parameter DARB_PORT_OFFSET0  = 4'h5;
parameter DARB_PORT_OFFSET1  = 4'h6;
parameter DARB_PORT_DMACHAN  = 4'h7;
parameter DARB_PORT_CRWUNK0  = 4'h8; //No documentation exists for this port.
parameter DARB_PORT_COUNTER0 = 4'h9;
parameter DARB_PORT_COUNTER1 = 4'hA;
parameter DARB_PORT_BYPASS   = 4'hB; //write 00 = bypass DCU, 02 = enable DCU
parameter DARB_PORT_STATUS   = 4'hC;

reg darb_should_begin_moderead;
reg [15:0] darb_sfc_offset_ctr;
reg [15:0] darb_sfc_length_ctr;

always @(posedge sfc_wr) begin
    if (darb_sfc_enable) begin
        case (sfc_dcu_port)
            DARB_PORT_BASE0: begin
                darb_directory_base <= (darb_directory_base & 24'hFFFF00) | (sfc_data_in);
                darb_should_begin_moderead <= 0;
            end
            DARB_PORT_BASE1: begin
                darb_directory_base <= (darb_directory_base & 24'hFF00FF) | (sfc_data_in << 8);
                darb_should_begin_moderead <= 0;
            end
            DARB_PORT_BASE2: begin
                darb_directory_base <= (darb_directory_base & 24'h00FFFF) | (sfc_data_in << 16);
                darb_should_begin_moderead <= 0;
            end
            DARB_PORT_INDEX: begin
                darb_directory_index <= sfc_data_in;
                darb_should_begin_moderead <= 0;
            end
            DARB_PORT_OFFSET0: begin
                darb_sfc_offset_ctr <= darb_sfc_offset_ctr & 16'hFF00 | sfc_data_in;
                darb_should_begin_moderead <= 0;
            end
            DARB_PORT_OFFSET1: begin
                darb_sfc_offset_ctr <= sfc_data_in << 8 | darb_sfc_offset_ctr & 16'h00FF;
                darb_should_begin_moderead <= 1;
            end
            DARB_PORT_COUNTER0: begin
                darb_sfc_length_ctr <= darb_sfc_length_ctr & 16'hFF00 | sfc_data_in;
                darb_should_begin_moderead <= 0;
            end
            DARB_PORT_COUNTER1: begin
                darb_sfc_length_ctr <= sfc_data_in << 8 | darb_sfc_length_ctr & 16'h00FF;
                darb_should_begin_moderead <= 0;
            end
            DARB_PORT_BYPASS: begin
                darb_bypass_dcu <= ~((sfc_data_in & 8'h02) >> 1);
                darb_should_begin_moderead <= 0;
            end
        endcase
    end else if (RESET) begin
        darb_directory_base <= 0;
        darb_sfc_offset_ctr <= 0;
        darb_sfc_length_ctr <= 0;
        darb_bypass_dcu <= 0;
    end
end

wire darb_readport_enable;

assign darb_readport_enable = darb_sfc_enable ? (sfc_dcu_port == DARB_PORT_READ)
                                : darb_decomp_mirror;
assign buffer_pa_en = darb_sfc_enable ? (sfc_dcu_port == DARB_PORT_READ)
                                : darb_decomp_mirror;

reg sfc_rd_last;

assign sfc_rd_posedge = sfc_rd & ~sfc_rd_last;
assign sfc_data_out = buffer_pa_en ? buffer_pa_dataout : darb_output;

reg darb_psram_pb_we;
reg [7:0] darb_psram_pb_datain;

always @(posedge CLK) begin
    //65C816 read port service.
    //We have to do it here or otherwise the Verilog synth complains
    if (sfc_rd_posedge & darb_sfc_enable) begin
        case (sfc_dcu_port)
            DARB_PORT_BASE0: begin
                darb_output <= darb_directory_base & 24'h0000FF;
            end
            DARB_PORT_BASE1: begin
                darb_output <= (darb_directory_base & 24'h00FF00) >> 8;
            end
            DARB_PORT_BASE2: begin
                darb_output <= (darb_directory_base & 24'hFF0000) >> 16;
            end
            DARB_PORT_INDEX: begin
                darb_output <= darb_directory_index;
            end
            DARB_PORT_OFFSET0: begin
                darb_output <= darb_offset_ctr & 16'h00FF;
            end
            DARB_PORT_OFFSET1: begin
                darb_output <= (darb_offset_ctr & 16'hFF00) >> 8;
            end
            DARB_PORT_COUNTER0: begin
                darb_output <= darb_length_ctr & 16'h00FF;
            end
            DARB_PORT_COUNTER1: begin
                darb_output <= (darb_length_ctr & 16'hFF00) >> 8;
            end
            DARB_PORT_BYPASS: begin
                darb_output <= ~darb_bypass_dcu << 1;
            end
            DARB_PORT_STATUS: begin
                //TODO: Is this sufficient flow control?
                //fullsnes seems to imply this is read before each byte, but
                //that would be incompatible with S-DMA usage... If we were
                //going to DMA from the read port into VRAM, then we'd wanna
                //hold this port in the busy state until the output buffer was
                //fuller...
                if (darb_outbuf_rdloc != darb_outbuf_wrloc) begin
                    darb_output <= 8'h80;
                end else begin
                    darb_output <= 8'h00;
                end
            end
        endcase
    end else if (darb_readport_enable) begin
        //WARNING: This port read does not attempt to address flow
        //control issues. It is assumed the 65C816 will heed STATUS and
        //not read READ until we tell it to. If it disobeys this then
        //ring status will become corrupted.
        if (darb_bypass_dcu) begin
            buffer_pa_addr <= darb_inbuf_rdloc & 11'h3FF;

            //We do not decrement inbuf_rdloc here because Verilog.
            //Look at the OTHER thing that decrements it...
            darb_length_ctr <= darb_length_ctr - 1;
        end else begin
            buffer_pa_addr <= darb_outbuf_rdloc | 11'h700;

            darb_outbuf_rdloc <= darb_outbuf_rdloc + 1;
            darb_length_ctr <= darb_length_ctr - 1;
        end
    end
    
    //Offsetting logic has to live here...
    if (darb_state == DARB_STATE_READY
            & darb_outbuf_wrloc != darb_outbuf_rdloc
            & darb_offset_ctr > 0) begin
        darb_offset_ctr <= darb_offset_ctr - 1;
        darb_outbuf_rdloc <= darb_outbuf_rdloc + 1;
    end
    
    sfc_rd_last <= sfc_rd;
    
    //This logic initializes the DCU buffers when it's time to start the DCU
    if (darb_should_begin_moderead) begin
        darb_state <= DARB_STATE_MODEREAD;
        darb_psram_ctr <= 0;
        darb_index_ptr <= darb_directory_base + darb_directory_index * 4;
        darb_offset_ctr <= darb_sfc_offset_ctr;
        darb_length_ctr <= darb_sfc_length_ctr;
    end
    
    if (darb_psram_ctr == 0) begin
        case (darb_state)
            DARB_STATE_MODEREAD: begin
                if (darb_state_ctr == 0) begin
                    darb_psram_addr <= darb_index_ptr >> 1;
                    darb_psram_ctr <= DARB_PSRAM_TIMING;

                    darb_state_ctr <= 1;
                end else if (darb_state_ctr > 0) begin
                    dcu_init <= 1;
                    dcu_init_mode <= psram_data >> (darb_index_ptr & 1'b1 ? 8 : 0);
                    
                    darb_index_ptr <= darb_index_ptr + 1;
                    
                    darb_state <= DARB_STATE_ADDRREAD;
                    darb_state_ctr <= 0;
                end
            end
            DARB_STATE_ADDRREAD: begin
                dcu_init <= 0;
                
                if (darb_state_ctr > 3) begin
                    darb_state <= DARB_STATE_READY;
                    darb_state_ctr <= 0;
                    
                    //Tell the PSRAM buffering logic it can start filling buffer
                    darb_psram_state <= DARB_PSRAM_INACTIVE;
                    darb_psram_ctr <= 0;
                end else if (darb_state_ctr > 0) begin
                    if (darb_index_ptr & 1 == 0 && darb_state_ctr != 3) begin
                        //16-bit ld
                        darb_decompress_ptr <= darb_decompress_ptr << 16 | psram_data >> 8 | psram_data & 8'hFF;
                        darb_index_ptr <= darb_index_ptr + 2;
                        darb_state_ctr <= darb_state_ctr + 2;
                        
                        darb_psram_addr <= (darb_index_ptr + 2) >> 1;
                        darb_psram_ctr <= DARB_PSRAM_TIMING;
                    end else if (darb_state_ctr != 3) begin
                        //8-bit ld, high byte
                        darb_decompress_ptr <= darb_decompress_ptr << 8 | psram_data & 8'hFF;
                        darb_index_ptr <= darb_index_ptr + 1;
                        darb_state_ctr <= darb_state_ctr + 1;
                        
                        darb_psram_addr <= (darb_index_ptr + 1) >> 1;
                        darb_psram_ctr <= DARB_PSRAM_TIMING;
                    end else if (darb_index_ptr & 1 == 0) begin
                        //8-bit ld, low byte
                        darb_decompress_ptr <= darb_decompress_ptr << 8 | psram_data >> 8;
                        darb_index_ptr <= darb_index_ptr + 1;
                        darb_state_ctr <= darb_state_ctr + 1;
                        
                        darb_psram_addr <= (darb_index_ptr + 1) >> 1;
                        darb_psram_ctr <= DARB_PSRAM_TIMING;
                    end
                end else begin
                    darb_psram_addr <= darb_index_ptr >> 1;
                    darb_psram_ctr <= DARB_PSRAM_TIMING;
                    
                    darb_state_ctr <= 1;
                end
            end
        endcase
    end
    
    //Priority 0 on Port B: Servicing PSRAM buffering.
    if (darb_state == DARB_STATE_READY && darb_psram_ctr == 0) begin
        case (darb_psram_state)
            DARB_PSRAM_INACTIVE: begin
                darb_psram_pb_we <= 0;
                
                //NOTE: Because PSRAM is 16b wide we need 2 free bytes of inbuf.
                //Otherwise we risk accidentally filling the ring buffer, which
                //will also empty it.
                if (((darb_inbuf_wrloc + 1 & 11'h3FF) != darb_inbuf_rdloc) &
                    ((darb_inbuf_wrloc + 2 & 11'h3FF) != darb_inbuf_rdloc)) begin
                    //We have PSRAM space, so let's start a PSRAM fetch.
                    darb_psram_state <= DARB_PSRAM_STORE;
                    darb_psram_ctr <= DARB_PSRAM_TIMING;
                    darb_psram_addr <= darb_decompress_ptr >> 1;
                end
            end
            DARB_PSRAM_STORE: begin
                darb_psram_pb_we <= 0;
                darb_psram_data <= psram_data;
                darb_psram_state <= DARB_PSRAM_QUEUE;
            end
            DARB_PSRAM_QUEUE: begin
                if (darb_decompress_ptr & 1) begin
                    //We only have one byte to write...
                    darb_psram_pb_we <= 1;
                    darb_psram_pb_datain <= darb_psram_data & 8'hFF;
                    
                    darb_psram_state <= DARB_PSRAM_INACTIVE;
                    darb_inbuf_wrloc <= darb_inbuf_wrloc + 1;
                    darb_decompress_ptr <= darb_decompress_ptr + 1;
                end else begin
                    //Otherwise, two bytes. Write the correct one first...
                    darb_psram_pb_we <= 1;
                    darb_psram_pb_datain <= darb_psram_data >> 8;
                    
                    darb_psram_state <= DARB_PSRAM_INACTIVE;
                    darb_inbuf_wrloc <= darb_inbuf_wrloc + 1;
                    darb_decompress_ptr <= darb_decompress_ptr + 1;
                end
            end
        endcase
    end
    
    //PSRAM requires 7 master cycles for valid data to be asserted. Furthermore,
    //the 65C816 can preempt our PSRAM accesses. So this logic will reset that
    //counter if our wait gets preempted. All you have to do is wait for the ctr
    //to reach zero again and then run your logic.
    if (!sfc_rom_rd & darb_psram_ctr > 0) begin
        darb_psram_ctr <= darb_psram_ctr - 1;
    end else if (sfc_rom_rd & darb_psram_ctr > 0) begin
        //We don't need to change psram_addr, we just need to reset our ctr and
        //let the countdown logic reassert psram_addr...
        darb_psram_ctr <= DARB_PSRAM_TIMING;
    end
end

parameter DIRECT_PROGROM_SIZE = 24'h100000;

assign psram_addr = darb_psram_addr + (DIRECT_PROGROM_SIZE >> 1);
assign darb_rom_rd = darb_psram_ctr != 0;

//Internal DCU
spc7110_dcu dcu (
    .CLK(CLK),
    .dcu_init(dcu_init),
    .dcu_init_mode(dcu_init_mode),
    .dcu_datarom_wait(dcu_datarom_wait),
    .dcu_datarom_rd(dcu_datarom_rd),
    .dcu_datarom_data(dcu_datarom_data),
    .dcu_ob_wait(dcu_ob_wait),
    .dcu_ob_wr(dcu_ob_wr),
    .dcu_ob_data(dcu_ob_data)
);

//DCU Service
assign dcu_datarom_wait = darb_inbuf_wrloc == darb_inbuf_rdloc | darb_psram_state == DARB_PSRAM_QUEUE | darb_bypass_dcu;
assign dcu_ob_wait = darb_output_ctr != 0;
assign dcu_datarom_data = buffer_pb_dataout;

reg darb_dcuin_pb_wlock;

always @(posedge CLK) begin
    //Priority 2 on Port B: Servicing DCU data reads.
    if ((darb_state == DARB_STATE_READY) 
        & dcu_datarom_rd
        & darb_inbuf_wrloc != darb_inbuf_rdloc
        & darb_psram_state != DARB_PSRAM_QUEUE
        & !darb_bypass_dcu) begin
        
        darb_dcuin_pb_wlock <= 1;
        darb_inbuf_rdloc <= darb_inbuf_rdloc + 1 & 10'h3FF;
    end else begin
        darb_dcuin_pb_wlock <= 0;
    end
    
    //SNES is reading our own input buffer...
    if (darb_bypass_dcu & darb_readport_enable) begin
        darb_inbuf_rdloc <= darb_inbuf_rdloc + 1;
    end
end

reg darb_dcuout_pb_we;
reg [7:0] darb_dcuout_pb_datain;

always @(posedge CLK) begin
    //Priority 3 on Port B: Servicing DCU output, which is 4 bytes wide.
    if (dcu_ob_wr & (darb_state == DARB_STATE_READY)) begin
        darb_output_data <= dcu_ob_data;
        darb_output_ctr <= 1;
    end else if (darb_state != DARB_STATE_READY) begin
        darb_output_ctr <= 0;
    end
    
    if (darb_state == DARB_STATE_READY & darb_output_ctr > 4) begin
        darb_dcuout_pb_we <= 0;
        darb_output_ctr <= 0;
    end else if ((darb_state == DARB_STATE_READY)
                & !dcu_datarom_rd
                & darb_output_ctr > 0
                & (darb_outbuf_wrloc + 1 & 11'h3FF) != darb_outbuf_rdloc
                & darb_psram_state != DARB_PSRAM_QUEUE) begin
        darb_dcuout_pb_datain <= darb_output_data & 8'hFF;
        darb_dcuout_pb_we <= 1;
        
        darb_outbuf_wrloc <= darb_outbuf_wrloc + 1;
        darb_output_ctr <= darb_output_ctr + 1;
        darb_output_data <= darb_output_data >> 8;
    end
end

//Block RAM Port B arbitration
assign buffer_pb_we = darb_psram_pb_we ? 1
                    : darb_dcuin_pb_wlock ? 0
                    : darb_dcuout_pb_we ? 1
                    : 0;
                    
assign buffer_pb_addr = darb_psram_pb_we ? darb_inbuf_wrloc
                        : darb_dcuin_pb_wlock ? darb_inbuf_rdloc
                        : darb_dcuout_pb_we ? darb_outbuf_wrloc | 11'h700
                        : 0;
                        
assign buffer_pb_datain = darb_psram_pb_we ? darb_psram_pb_datain
                          : darb_dcuout_pb_we ? darb_dcuout_pb_datain
                          : 0;

endmodule

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:50:40 11/11/2017 
// Design Name: 
// Module Name:    spc7110_direct 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: SPC7110 Data ROM Direct Access
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: For whatever reason, the SPC7110 has both bank-switched
//                      and MMIO based data ROM access.
//
//////////////////////////////////////////////////////////////////////////////////
module spc7110_direct(
    input CLK,
    input RESET,
    
    //Data ROM MMIO access is SNES triggered, and thus cannot be interrupted by
    //a higher-priority PSRAM access...
    output direct_rom_rd,         //Data ROM MMIO is accessing PSRAM...
	 output [23:0] direct_mapped_addr, //and here's the address we need.
    
    //SFC I/O ports
    input direct_sfc_enable,      //SFC reads/writes map to Data ROM MMIO
    input [3:0] sfc_direct_port,
    input sfc_rd,
	 input sfc_rd_end,
    input sfc_wr,
    input [7:0] sfc_data_in,
    output [7:0] sfc_data_out
);

reg direct_allow_read;
reg [6:0] direct_mode;
reg [23:0] direct_base;
reg [15:0] direct_offset;
reg [15:0] direct_step;

parameter DIRECT_READINC = 4'h0;
parameter DIRECT_BASE0   = 4'h1;
parameter DIRECT_BASE1   = 4'h2;
parameter DIRECT_BASE2   = 4'h3;
parameter DIRECT_OFFSET0 = 4'h4;
parameter DIRECT_OFFSET1 = 4'h5;
parameter DIRECT_STEP0   = 4'h6;
parameter DIRECT_STEP1   = 4'h7;
parameter DIRECT_MODE    = 4'h8;
parameter DIRECT_READSET = 4'hA;

parameter DIRECT_PROGROM_SIZE = 24'h100000;

wire direct_use_step = direct_mode[0];
wire direct_use_offset = direct_mode[1];
wire direct_use_signed_step = direct_mode[2];
wire direct_use_signed_offset = direct_mode[3];
wire direct_use_increment_base = !direct_mode[4];
wire direct_use_increment_offset = direct_mode[4];

//"Special Action" field...
wire direct_add_8b_offset = direct_mode[5] & !direct_mode[6];
wire direct_add_16b_offset = !direct_mode[5] & direct_mode[6];
wire direct_add_16b_offset_481A = direct_mode[5] & direct_mode[6];

//If sign extension is enabled, these wires will be used instead of the regs
wire [23:0] direct_signed_step = { {8{direct_step[15]}}, direct_step[15:0] };
wire [23:0] direct_signed_offset = { {8{direct_offset[15]}}, direct_offset[15:0] };

reg [7:0] direct_mmio_out;

assign sfc_data_out = direct_mmio_out;
//TODO: Account for small data ROM size with masking...
//SPC7110 is natively single-ROM, so we have to add the program ROM size...
//Sorry byuu
assign direct_mapped_addr = DIRECT_PROGROM_SIZE + direct_base +
                            ((direct_use_offset | sfc_direct_port == DIRECT_READSET)
                                 ? (direct_use_signed_offset ? direct_signed_offset
                                                             : direct_offset)
                                 : 0);
assign direct_rom_rd = direct_allow_read
                        & direct_sfc_enable
                        & (sfc_direct_port == DIRECT_READINC
                            | sfc_direct_port == DIRECT_READSET);

always @(posedge CLK) begin
    if (direct_sfc_enable & sfc_wr) begin
        case (sfc_direct_port)
            DIRECT_BASE0: begin
                direct_base <= (direct_base & 24'hFFFF00) | sfc_data_in;
            end
            DIRECT_BASE1: begin
                direct_base <= (direct_base & 24'hFF00FF) | (sfc_data_in << 8);
            end
            DIRECT_BASE2: begin
                direct_base <= (direct_base & 24'h00FFFF) | (sfc_data_in << 16);
                
                if (sfc_data_in != 0) begin
                    direct_allow_read <= 1;
                end
            end
            DIRECT_OFFSET0: begin
                direct_offset = (direct_offset & 16'hFF00) | sfc_data_in;
                
                //TODO: Isn't this supposed to add only the 8 bits?
                //As opposed to "add when you write the lower 8 bits...
                if (direct_add_8b_offset & direct_use_signed_offset) begin
                    direct_base <= direct_base + direct_signed_offset;
                end else if (direct_add_8b_offset) begin
                    direct_base <= direct_base + direct_offset;
                end
            end
            DIRECT_OFFSET1: begin
                direct_offset = (direct_offset & 16'h00FF) | (sfc_data_in << 8);
                
                if (direct_add_16b_offset & direct_use_signed_offset) begin
                    direct_base <= direct_base + direct_signed_offset;
                end else if (direct_add_16b_offset) begin
                    direct_base <= direct_base + direct_offset;
                end
            end
            DIRECT_STEP0: begin
                direct_step <= (direct_step & 16'hFF00) | sfc_data_in;
            end
            DIRECT_STEP1: begin
                direct_step <= (direct_step & 16'h00FF) | (sfc_data_in << 8);
            end
            DIRECT_MODE: begin
                direct_mode <= sfc_data_in;
            end
        endcase
    end else if (direct_sfc_enable & sfc_rd) begin
        case (sfc_direct_port)
            DIRECT_BASE0: begin
                direct_mmio_out <= direct_base & 24'h0000FF;
            end
            DIRECT_BASE1: begin
                direct_mmio_out <= (direct_base & 24'h00FF00) >> 8;
            end
            DIRECT_BASE2: begin
                direct_mmio_out <= (direct_base & 24'hFF0000) >> 16;
            end
            DIRECT_OFFSET0: begin
                direct_mmio_out <= direct_offset & 16'h00FF;
            end
            DIRECT_OFFSET1: begin
                direct_mmio_out <= (direct_offset & 16'hFF00) >> 8;
            end
            DIRECT_STEP0: begin
                direct_mmio_out <= direct_step & 16'h00FF;
            end
            DIRECT_STEP1: begin
                direct_mmio_out <= (direct_step & 16'hFF00) >> 8;
            end
            DIRECT_MODE: begin
                //TODO: fullsnes says this triggers the "special actions"
                direct_mmio_out <= direct_mode;
            end
        endcase
    end else if (direct_sfc_enable & sfc_rd_end) begin
        case (sfc_direct_port)
            DIRECT_READINC: begin
                if (direct_allow_read) begin
                    if (direct_use_increment_base & direct_use_step & direct_use_signed_step) begin
                        direct_base <= direct_base + direct_signed_step;
                    end else if (direct_use_increment_base & direct_use_step) begin
                        direct_base <= direct_base + direct_step;
                    end else if (direct_use_increment_base) begin
                        direct_base <= direct_base + 1;
                    end else if (direct_use_increment_offset & direct_use_step & direct_use_signed_step) begin
                        direct_offset = direct_offset + direct_signed_step;
                    end else if (direct_use_increment_offset & direct_use_step) begin
                        direct_offset = direct_offset + direct_step;
                    end else if (direct_use_increment_offset) begin
                        direct_offset = direct_offset + 1;
                    end
                end
            end
            DIRECT_READSET: begin
                if (direct_allow_read) begin
                    if (direct_add_16b_offset_481A & direct_use_signed_offset) begin
                        direct_base <= direct_base + direct_signed_offset;
                    end else if (direct_add_16b_offset_481A) begin
                        direct_base <= direct_base + direct_offset;
                    end
                    
                    //TODO: snes9x will add offset to itself if "Apply Step" is
                    //set to write to the offset, even though fullsnes and higan
                    //claim otherwise. Further testing is required to determine
                    //if $4818:b4 matters or not...
                end
            end
        endcase
    end
end

endmodule
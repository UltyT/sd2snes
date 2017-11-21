`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:44:53 11/20/2017 
// Design Name: 
// Module Name:    spc7110_banked 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: Along with the Direct access method, there's also
//                      remappable banks on the SPC7110 that map to $D, $E, and
//                      $Fnnnnn addresses. Because the CPU having it's own bank
//                      switching wasn't good enough.
//
//////////////////////////////////////////////////////////////////////////////////
module spc7110_banked(
    input CLK,
    input RESET,
    
    //SFC I/O ports
    input banked_sfc_enable,      //SFC reads/writes map to SPC7110 bank latches
    input [3:0] sfc_banked_port,
    input sfc_rd,
    input sfc_wr,
    input [7:0] sfc_data_in,
    output [7:0] sfc_data_out,
    
    //Bank latches
    //Unlike Direct, we don't route the ROM chip here, we instead latch the data
    //and send it to the address module for actual decoding
    output sram_enable,
    output [2:0] block_dn_select,
    output [2:0] block_en_select,
    output [2:0] block_fn_select
    
);

parameter BANKED_SRAMENABLE = 4'h0;
parameter BANKED_BLOCKDSEL  = 4'h1;
parameter BANKED_BLOCKESEL  = 4'h2;
parameter BANKED_BLOCKFSEL  = 4'h3;
parameter BANKED_SRAMMAP    = 4'h4;

reg banked_sram_enable;
reg [2:0] banked_dn_select;
reg [2:0] banked_en_select;
reg [2:0] banked_fn_select;

assign sram_enable = banked_sram_enable;
assign block_dn_select = banked_dn_select;
assign block_en_select = banked_en_select;
assign block_fn_select = banked_fn_select;

reg [7:0] banked_data_out;
assign sfc_data_out = banked_data_out;

always @(posedge CLK) begin
    if (banked_sfc_enable & sfc_rd) begin
        case (sfc_banked_port)
            BANKED_SRAMENABLE: begin
                banked_data_out <= (banked_sram_enable ? 8'h80 : 8'h00);
            end
            BANKED_BLOCKDSEL: begin
                banked_data_out <= block_dn_select;
            end
            BANKED_BLOCKESEL: begin
                banked_data_out <= block_en_select;
            end
            BANKED_BLOCKFSEL: begin
                banked_data_out <= block_fn_select;
            end
        endcase
    end
    
    if (banked_sfc_enable & sfc_wr) begin
        case (sfc_banked_port)
            BANKED_SRAMENABLE: begin
                banked_sram_enable <= sfc_data_in[7];
            end
            BANKED_BLOCKDSEL: begin
                banked_dn_select <= sfc_data_in;
            end
            BANKED_BLOCKESEL: begin
                banked_en_select <= sfc_data_in;
            end
            BANKED_BLOCKFSEL: begin
                banked_fn_select <= sfc_data_in;
            end
        endcase
    end
end

endmodule

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    23:27:51 11/19/2017 
// Design Name: 
// Module Name:    spc7110_alu 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: SPC7110 has a mul/div ALU... which this modern FPGA is
//                      waaaaaaay faster at than real hardware. 
//
//////////////////////////////////////////////////////////////////////////////////
module spc7110_alu(
    input CLK,
    input RESET,
    
    //SFC I/O ports
    input alu_sfc_enable,      //SFC reads/writes map to SPC7110 ALU
    input [3:0] sfc_alu_port,
    input sfc_rd,
    input sfc_wr,
    input [7:0] sfc_data_in,
    output [7:0] sfc_data_out
);

reg [7:0] alu_data_out;
assign sfc_data_out = alu_data_out;

parameter ALU_PORT_ARGUMENTA0 = 4'h0; //lower 16 bits serve as multiplicand...
parameter ALU_PORT_ARGUMENTA1 = 4'h1;
parameter ALU_PORT_ARGUMENTA2 = 4'h2; //full 32 bits only for dividend.
parameter ALU_PORT_ARGUMENTA3 = 4'h3;
parameter ALU_PORT_MULTIPLY0  = 4'h4;
parameter ALU_PORT_MULTIPLY1  = 4'h5; //Writing this port triggers multiply
parameter ALU_PORT_DIVISOR0   = 4'h6;
parameter ALU_PORT_DIVISOR1   = 4'h7; //Writing this port triggers divide
parameter ALU_PORT_RESULT0    = 4'h8;
parameter ALU_PORT_RESULT1    = 4'h9;
parameter ALU_PORT_RESULT2    = 4'hA;
parameter ALU_PORT_RESULT3    = 4'hB;
parameter ALU_PORT_RESULTMOD0 = 4'hC; //Remainder bits for divider.
parameter ALU_PORT_RESULTMOD1 = 4'hD;
parameter ALU_PORT_RESET      = 4'hE; //Bit 0 sets unsigned/signed mode...
parameter ALU_PORT_STATUS     = 4'hF;

//TODO: Actually test the latency of the Spartan3 multiply units.
//      The datasheet only mentioned Spartan3A...
//TODO: Add "accurate" latencies based on actual SPC7110 hardware latency.
//NOTE: The +1 is to account for our weird latching logic (see below)
parameter ALU_UMUL_LATENCY    = 4 + 1;
parameter ALU_SMUL_LATENCY    = 4 + 1;
parameter ALU_UDIV_LATENCY    = 35 + 1;
parameter ALU_SDIV_LATENCY    = 37 + 1;

reg alu_signed_maths;       //Are we doing a signed operation?
reg alu_lastop_division;    //Was the last operation a divide?
reg [7:0] alu_result_ctr;   //How many cycles to wait until result is valid.

//These are the registers the 65C816 writes into.
//They don't hit the hardware ALU blocks until port 5/7 are written...
reg [31:0] alu_next_arga;
reg [15:0] alu_next_multiplier;
reg [15:0] alu_next_divisor;

wire [15:0] alu_next_multiplicand = alu_next_arga[15:0];

//MULTIPLICATION HARDWARE
reg [15:0] alu_multiplicand;
reg [15:0] alu_multiplier;

wire [31:0] alu_smul_product;

spc7110_alu_smul alu_smul(
    .clk(CLK),
    .a(alu_multiplicand),
    .b(alu_multiplier),
    .p(alu_smul_product)
);

wire [31:0] alu_umul_product;

spc7110_alu_umul alu_umul(
    .clk(CLK),
    .a(alu_multiplicand),
    .b(alu_multiplier),
    .p(alu_umul_product)
);

wire [31:0] alu_product = alu_signed_maths ? alu_smul_product : alu_umul_product;

//DIVISION HARDWARE
reg [31:0] alu_dividend;
reg [15:0] alu_divisor;

wire [31:0] alu_sdiv_quotient;
wire [15:0] alu_sdiv_modulo;
wire alu_sdiv_rfd;

spc7110_alu_sdiv alu_sdiv(
    .clk(CLK),
    .rfd(alu_sdiv_rfd),
    .dividend(alu_dividend),
    .divisor(alu_divisor),
    .quotient(alu_sdiv_quotient),
    .fractional(alu_sdiv_modulo)
);

wire [31:0] alu_udiv_quotient;
wire [15:0] alu_udiv_modulo;
wire alu_udiv_rfd;

spc7110_alu_udiv alu_udiv(
    .clk(CLK),
    .rfd(alu_udiv_rfd),
    .dividend(alu_dividend),
    .divisor(alu_divisor),
    .quotient(alu_udiv_quotient),
    .fractional(alu_udiv_modulo)
);

//This output is NOT STABLE until you have counted LATENCY clocks AFTER the RFD.
reg alu_rfd_seen;
wire alu_rfd = alu_signed_maths ? alu_sdiv_rfd : alu_udiv_rfd;
wire [31:0] alu_quotient = alu_signed_maths ? alu_sdiv_quotient : alu_udiv_quotient;
wire [15:0] alu_modulo = alu_signed_maths ? alu_sdiv_modulo : alu_udiv_modulo;

//Multiply and divide share an output port.
//TODO: Are products sign extended? When are they sign extended?
wire [31:0] alu_result = alu_lastop_division ? alu_quotient : alu_product;
wire [7:0] alu_status = alu_result_ctr > 1 ? 8'h80 : 8'h00;

//These are latched once the current latency timer counts down.
reg [31:0] alu_last_result;
reg [15:0] alu_last_modulo;

always @(posedge CLK) begin
    if (alu_sfc_enable & sfc_wr) begin
        case (sfc_alu_port)
            ALU_PORT_ARGUMENTA0: begin
                alu_next_arga <= (alu_next_arga & 32'hFFFFFF00) | (sfc_data_in << 0);
            end
            ALU_PORT_ARGUMENTA1: begin
                alu_next_arga <= (alu_next_arga & 32'hFFFF00FF) | (sfc_data_in << 8);
            end
            ALU_PORT_ARGUMENTA2: begin // Attacker Unit, Number 2.
                alu_next_arga <= (alu_next_arga & 32'hFF00FFFF) | (sfc_data_in << 16);
            end
            ALU_PORT_ARGUMENTA3: begin
                alu_next_arga <= (alu_next_arga & 32'h00FFFFFF) | (sfc_data_in << 24);
            end
            ALU_PORT_MULTIPLY0: begin
                alu_next_multiplier <= (alu_next_multiplier & 16'hFF00) | (sfc_data_in << 0);
            end
            ALU_PORT_MULTIPLY1: begin
                alu_next_multiplier <= (alu_next_multiplier & 16'h00FF) | (sfc_data_in << 8);
                
                //Writes to MULTIPLY1 trigger a multiply.
                alu_multiplicand <= alu_next_multiplicand;
                alu_multiplier <= (alu_next_multiplier & 16'h00FF) | (sfc_data_in << 8);
                alu_result_ctr <= alu_signed_maths ? ALU_SMUL_LATENCY : ALU_UMUL_LATENCY;
                alu_lastop_division <= 0;
            end
            ALU_PORT_DIVISOR0: begin
                alu_next_divisor <= (alu_next_divisor & 16'hFF00) | (sfc_data_in << 0);
            end
            ALU_PORT_DIVISOR1: begin
                alu_next_divisor <= (alu_next_divisor & 16'h00FF) | (sfc_data_in << 8);
                
                //Writes to DIVISOR1 trigger a divide.
                alu_dividend <= alu_next_arga;
                alu_divisor <= (alu_next_divisor & 16'h00FF) | (sfc_data_in << 8);
                alu_result_ctr <= alu_signed_maths ? ALU_SDIV_LATENCY : ALU_UDIV_LATENCY;
                alu_lastop_division <= 1;
                alu_rfd_seen <= 0;
            end
            ALU_PORT_RESET: begin
                alu_next_arga <= 0;
                alu_next_multiplier <= 0;
                alu_next_divisor <= 0;
                
                alu_last_result <= 0;
                alu_last_modulo <= 0;
                
                //TODO: Should we enable SCLR on our hardware alu blocks, and
                //      tie it to the RESET port?
                
                alu_signed_maths <= sfc_data_in[0];
            end
        endcase
    end
    
    if (alu_sfc_enable & sfc_rd) begin
        case (sfc_alu_port)
            ALU_PORT_ARGUMENTA0: begin
                alu_data_out <= (alu_next_arga & 32'h000000FF) >> 0;
            end
            ALU_PORT_ARGUMENTA1: begin
                alu_data_out <= (alu_next_arga & 32'h0000FF00) >> 8;
            end
            ALU_PORT_ARGUMENTA2: begin
                alu_data_out <= (alu_next_arga & 32'h00FF0000) >> 16;
            end
            ALU_PORT_ARGUMENTA3: begin
                alu_data_out <= (alu_next_arga & 32'hFF000000) >> 24;
            end
            ALU_PORT_MULTIPLY0: begin
                alu_data_out <= (alu_next_multiplier & 16'h00FF) >> 0;
            end
            ALU_PORT_MULTIPLY1: begin
                alu_data_out <= (alu_next_multiplier & 16'hFF00) >> 8;
            end
            ALU_PORT_DIVISOR0: begin
                alu_data_out <= (alu_next_divisor & 16'h00FF) >> 0;
            end
            ALU_PORT_DIVISOR1: begin
                alu_data_out <= (alu_next_divisor & 16'hFF00) >> 8;
            end
            ALU_PORT_RESULT0: begin
                alu_data_out <= (alu_last_result & 32'h000000FF) >> 0;
            end
            ALU_PORT_RESULT1: begin
                alu_data_out <= (alu_last_result & 32'h0000FF00) >> 8;
            end
            ALU_PORT_RESULT2: begin
                alu_data_out <= (alu_last_result & 32'h00FF0000) >> 16;
            end
            ALU_PORT_RESULT3: begin
                alu_data_out <= (alu_last_result & 32'hFF000000) >> 24;
            end
            ALU_PORT_RESULTMOD0: begin
                alu_data_out <= (alu_last_modulo & 16'h00FF) >> 0;
            end
            ALU_PORT_RESULTMOD1: begin
                alu_data_out <= (alu_last_modulo & 16'hFF00) >> 8;
            end
            ALU_PORT_STATUS: begin
                alu_data_out <= alu_status;
            end
        endcase
    end
    
    if (alu_result_ctr > 0) begin
        //Last cycle, so latch our results. Note that latencies have to be +1
        //cause our result ctr check isn't edge triggered.
        if (alu_result_ctr == 1) begin
            alu_last_result <= alu_result;
            alu_last_modulo <= alu_lastop_division ? alu_modulo : 0;
        end
        
        //We have to wait for one RFD pulse before counting down our latency.
        if (alu_rfd_seen | alu_rfd) begin
            alu_result_ctr <= alu_result_ctr - 1;
            alu_rfd_seen <= 1;
        end
    end
    
    if (RESET) begin
        alu_signed_maths <= 0;
        alu_lastop_division <= 0;
        alu_result_ctr <= 0;
        alu_next_arga <= 0;
        alu_next_multiplier <= 0;
        alu_next_divisor <= 0;
        
        alu_multiplicand <= 0;
        alu_multiplier <= 0;
        alu_dividend <= 0;
        alu_divisor <= 0;
        
        alu_rfd_seen <= 0;
        
        alu_last_result <= 0;
        alu_last_modulo <= 0;
    end
end

endmodule

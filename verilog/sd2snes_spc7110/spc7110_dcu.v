`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    23:06:28 11/07/2017 
// Design Name: 
// Module Name:    spc7110_dcu 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: SPC7110 decompression unit
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: None of this is based on real-hardware testing, just
//                      publicly available documentation and emulator source.
//                      Accuracy not guaranteed.
//
//                      The SPC7110 implements a simplified arithmetic coder
//                      wired specifically for SNES DMA and reading from a
//                      second data ROM. Descriptions of the algorithm can be
//                      found in fullsnes, snes9x source, higan source, etc...
//
//////////////////////////////////////////////////////////////////////////////////
module spc7110_dcu(
    input CLK,
    
    input dcu_init,
    input [1:0] dcu_init_mode,
    
    //DCU <-> Data ROM
    //The DCU will fire rd whenever it needs a new input byte. It expects a new
    //compressed byte on the next CLK, unless wait is asserted. When data is
    //ready, then release wait with data asserted. It will be consumed on the
    //next CLK.
    input dcu_datarom_wait,
    output dcu_datarom_rd,
    input [7:0] dcu_datarom_data,
    
    //DCU <-> Output Buffer
    //The DCU will continuously output decoded pixels as long as it has a valid
    //graphical configuration. Another module must buffer this output and
    //provide it to the SNES side one byte at a time. If the output buffer is
    //full then you may halt the output buffer using the wait signal. After the
    //buffer is ready again you may release wait after you have committed data
    //to the buffer.
    input dcu_ob_wait,
    output dcu_ob_wr,
    output [31:0] dcu_ob_data
);

reg [9:0] dcu_state;
reg [6:0] dcu_state_ctr; //for counting reads

parameter DCU_STATE_INACTIVE    = 10'b0000000000; //DCU not configured
parameter DCU_STATE_DCINIT      = 10'b0000000100; //reset internal DCU state
parameter DCU_STATE_PREREAD     = 10'b0000001000; //initial fill of input pipeline
parameter DCU_STATE_PIXEL       = 10'b0000010000; //generate diffcode
parameter DCU_STATE_SHUFFLEMAP  = 10'b0000100000; //shuffle the colormaps around
parameter DCU_STATE_PLANE       = 10'b0001000000; //select context & exec model
parameter DCU_STATE_RENORMALIZE = 10'b0010000000; //renormalize range bits, loop
parameter DCU_STATE_DATAREAD    = 10'b0100000000; //restore bits lost during above
parameter DCU_STATE_OUTBUFFER   = 10'b1000000000; //output decompressed to buffer

reg [1:0] dcu_decompress_mode;

parameter DCU_DECOMPMODE_1BPP = 2'b00;
parameter DCU_DECOMPMODE_2BPP = 2'b01;
parameter DCU_DECOMPMODE_4BPP = 2'b10;

//When the SNES side has specified a new load address, wipe the DCU state
always @(posedge dcu_init) begin
    dcu_state <= DCU_STATE_DCINIT;
    dcu_state_ctr <= 0;
    dcu_decompress_mode <= dcu_init_mode;
end

//Input/Output states
always @(posedge CLK) begin
    case (dcu_state)
        DCU_STATE_DATAREAD: begin
            if (!dcu_datarom_wait && dcu_state_ctr > 0) begin
                dcu_input = dcu_input << 8 | dcu_datarom_data;
                dcu_datarom_rd <= 0;
                
                dcu_state <= DCU_STATE_RENORMALIZE;
                dcu_state_ctr <= 0;
            end
            
            if (dcu_state_ctr == 0) begin
                dcu_datarom_rd <= 1;
                
                dcu_state_ctr = 1;
            end
        end
        DCU_STATE_OUTBUFFER: begin
            if (dcu_state_ctr > 0) begin
                dcu_ob_wr <= 0;
            end
            
            if (!dcu_ob_wait && dcu_state_ctr > 0) begin
                dcu_state <= DCU_STATE_PIXEL;
                dcu_state_ctr <= 0;
            end
            
            if (!dcu_ob_wait && dcu_state_ctr == 0) begin
                dcu_ob_wr <= 1;
                dcu_ob_data <= dcu_output;
                
                dcu_state_ctr <= 1;
            end
        end
    endcase
end

//Arithmetic coding constants
reg [7:0] dcu_table_prob[52:0];
reg [7:0] dcu_table_lps[52:0]; //TODO: Do these require all 8 bits?
reg [7:0] dcu_table_mps[52:0];
reg dcu_table_invert[52:0];

reg [15:0] dcu_length;

//Now the contexts that drive those arithmetic coders.
//Higan says not all 74 contexts exist but I'm not 100% sure how to figure.
reg [7:0] dcu_ctx_pred[74:0];
reg dcu_ctx_swap[74:0];

reg [7:0] dcu_selected_ctx;
reg [7:0] dcu_selected_model;
reg [8:0] dcu_range;
reg [2:0] dcu_diffcode;
reg [3:0] dcu_pa;
reg [3:0] dcu_pb;
reg [3:0] dcu_pc;
reg [8:0] dcu_lps_offset;
reg [1:0] dcu_plane;
reg [7:0] dcu_index;

reg [2:0] dcu_pixel;
reg [7:0] dcu_history;
reg dcu_symbol_ismps;

reg [7:0] dcu_symbol;
reg [63:0] dcu_pixels;
reg [63:0] dcu_colormap;
reg [63:0] dcu_map;
reg [63:0] dcu_morton;

//DCU Input Registers
reg [15:0] dcu_input;
reg [4:0] dcu_input_bits;

//DCU Output Registers
reg [31:0] dcu_output;

//Decompression logic
always @(posedge CLK) begin
    case (dcu_state)
        DCU_STATE_DCINIT: begin
            //TODO: Does this run fast enough for one CLK? Probably not
            for (dcu_state_ctr = 0; dcu_state_ctr <= 75; dcu_state_ctr = dcu_state_ctr + 1) begin
                dcu_ctx_pred[dcu_state_ctr] = 8'h00;
                dcu_ctx_swap[dcu_state_ctr] = 1'b0;
            end
            
            //TODO: Support offsetting...
            
            dcu_range <= 9'h100;
            dcu_symbol <= 0;
            dcu_pixels <= 0;
            dcu_colormap <= 64'hFEDCBA9876543210;
            
            //Move to filling dcu_input/bits...
            dcu_state <= DCU_STATE_PREREAD;
            dcu_state_ctr <= 0;
        end
        DCU_STATE_PREREAD: begin
            if (!dcu_datarom_wait && dcu_state_ctr > 0) begin
                dcu_input <= dcu_input << 8 | dcu_datarom_data;
                dcu_state_ctr <= dcu_state_ctr + 1;
            end
            
            if (!dcu_datarom_wait && dcu_state_ctr > 1) begin
                dcu_state <= DCU_STATE_PIXEL;
                dcu_state_ctr <= 0;

                dcu_input_bits <= 8;

                dcu_datarom_rd <= 0;
            end else begin
                dcu_datarom_rd <= 1;

                dcu_state_ctr <= dcu_state_ctr + 1;
            end
        end
        DCU_STATE_PIXEL: begin
            dcu_map <= dcu_colormap;
            
            case (dcu_decompress_mode)
                DCU_DECOMPMODE_1BPP: begin
                    dcu_diffcode <= 0;
                    
                    dcu_state <= DCU_STATE_PLANE;
                    dcu_state_ctx <= 0;
                end
                DCU_DECOMPMODE_2BPP: begin
                    dcu_pa <= dcu_pixels >> 2 & 2'b11;
                    dcu_pb <= dcu_pixels >> 14 & 2'b11;
                    dcu_pc <= dcu_pixels >> 16 & 2'b11;
                end
                DCU_DECOMPMODE_4BPP: begin
                    dcu_pa <= dcu_pixels >> 0 & 4'b1111;
                    dcu_pb <= dcu_pixels >> 28 & 4'b1111;
                    dcu_pc <= dcu_pixels >> 32 & 4'b1111;
                end
            endcase
            
            if (dcu_decompress_mode != DCU_DECOMPMODE_1BPP) begin
                if (dcu_pa == dcu_pb == dcu_pc)
                    dcu_diffcode <= 0;
                else if (dcu_pb == dcu_pc)
                    dcu_diffcode <= 1;
                else if (dcu_pa == dcu_pc)
                    dcu_diffcode <= 2;
                else if (dcu_pa == dcu_pb)
                    dcu_diffcode <= 3;
                else
                    dcu_diffcode <= 4;

                dcu_state <= DCU_STATE_SHUFFLEMAP;
                dcu_state_ctx <= 0;
            end
        end
        DCU_STATE_SHUFFLEMAP: begin
            //There are four move-to-front ops in 2BPP mode: one for the
            //colormap, and three for map; all taking in pa, pc, pb, and pa in
            //that order. Each move-to-front searches a 64-bit register for a
            //4-bit value and, if found, moves it to the front of the list.
            //I'm sure there's a better way to handle this in Verilog...
            if (dcu_decompress_mode == DCU_DECOMPMODE_1BPP) begin
                //Technically an illegal state. Go onto the next one anyway
                dcu_state <= DCU_STATE_PLANE;
                dcu_state_ctx <= 0;
            end else if (dcu_state_ctr < 16) begin
                if (((dcu_colormap >> dcu_state_ctr & 4'b1111) & 4'b1111) != dcu_pa)
                    dcu_state_ctr <= dcu_state_ctr + 1;
                else begin
                    dcu_state_ctr = 16;
                    dcu_colormap <= dcu_colormap & (64'hFFFFFFFFFFFFFFF0 << ((dcu_state_ctr & 4'b1111) << 2))
                                  | (dcu_colormap << 4) & ~(64'hFFFFFFFFFFFFFFF0 << ((dcu_state_ctr & 4'b1111) << 2))
                                  | dcu_pa;
                end
            end else if (dcu_state_ctr < 32) begin
                if (((dcu_map >> dcu_state_ctr & 4'b1111) & 4'b1111) != dcu_pc)
                    dcu_state_ctr <= dcu_state_ctr + 1;
                else begin
                    dcu_state_ctr = 16;
                    dcu_map <= dcu_map & (64'hFFFFFFFFFFFFFFF0 << ((dcu_state_ctr & 4'b1111) << 2))
                            | (dcu_map << 4) & ~(64'hFFFFFFFFFFFFFFF0 << ((dcu_state_ctr & 4'b1111) << 2))
                            | dcu_pc;
                end
            end else if (dcu_state_ctr < 48) begin
                if (((dcu_map >> dcu_state_ctr & 4'b1111) & 4'b1111) != dcu_pb)
                    dcu_state_ctr <= dcu_state_ctr + 1;
                else begin
                    dcu_state_ctr = 16;
                    dcu_map <= dcu_map & (64'hFFFFFFFFFFFFFFF0 << ((dcu_state_ctr & 4'b1111) << 2))
                            | (dcu_map << 4) & ~(64'hFFFFFFFFFFFFFFF0 << ((dcu_state_ctr & 4'b1111) << 2))
                            | dcu_pb;
                end
            end else if (dcu_state_ctr < 64) begin
                if (((dcu_map >> dcu_state_ctr & 4'b1111) & 4'b1111) != dcu_pa)
                    dcu_state_ctr <= dcu_state_ctr + 1;
                else begin
                    dcu_state_ctr = 16;
                    dcu_map <= dcu_map & (64'hFFFFFFFFFFFFFFF0 << ((dcu_state_ctr & 4'b1111) << 2))
                            | (dcu_map << 4) & ~(64'hFFFFFFFFFFFFFFF0 << ((dcu_state_ctr & 4'b1111) << 2))
                            | dcu_pa;
                end
            end else begin
                dcu_state <= DCU_STATE_PLANE;
                dcu_state_ctx <= 0;
            end
        end
        DCU_STATE_PLANE: begin
            //Here we prep for decompressing one of our four bitplanes.
            //For whatever reason they get compressed in separate contexts...
            case (dcu_decompress_mode)
                DCU_DECOMPMODE_1BPP: begin
                    dcu_history <= (1 << dcu_pixel - 1) & dcu_symbol;
                    if (dcu_plane >= 2 & history <= 1)
                        dcu_selected_ctx <= (dcu_diffcode) * 15 + (1 << dcu_plane) + (history - 1);
                    else
                        dcu_selected_ctx <= (dcu_pixel >> 2) * 15 + (1 << dcu_pixel) + (history - 1);
                end
                DCU_DECOMPMODE_2BPP: begin
                    dcu_history <= (1 << dcu_plane - 1) & dcu_symbol;
                    dcu_selected_ctx <= (dcu_diffcode) * 15 + (1 << dcu_plane) + (history - 1);
                end
                DCU_DECOMPMODE_4BPP: begin
                    dcu_history <= (1 << dcu_plane - 1) & dcu_symbol;
                    if (dcu_plane >= 2 & history <= 1)
                        dcu_selected_ctx <= (dcu_diffcode) * 15 + (1 << dcu_plane) + (history - 1);
                    else
                        dcu_selected_ctx <= (1 << dcu_plane) + (history - 1);
                end
            endcase
            
            dcu_selected_model <= dcu_ctx_pred[dcu_selected_ctx];
            dcu_lps_offset <= dcu_range - dcu_table_prob[dcu_selected_model];
            dcu_symbol_ismps <= (dcu_input >= (lps_offset << 8));
            dcu_symbol <= dcu_symbol << 1 | (dcu_symbol_ismps ^ dcu_ctx_swap[dcu_selected_ctx]);
            
            if (dcu_symbol_ismps) //More-probable symbol
                dcu_range <= dcu_lps_offset;
            else begin //Less-probable symbol
                dcu_range <= dcu_range - dcu_lps_offset;
                dcu_input <= dcu_input - (dcu_lps_offset << 8);
            end
            
            dcu_state <= DCU_STATE_RENORMALIZE;
        end
        DCU_STATE_RENORMALIZE: begin
            //Renormalize input for the next symbol. This is the actual decoding
            //for the arithmetic coding algorithm. Everything else is context or
            //map shuffling to get the most efficiency for bitmap graphics data.
            if (dcu_range <= 8'h7F) begin
                if (dcu_symbol_ismps)
                    dcu_ctx_pred[dcu_selected_ctx] <= dcu_table_mps[dcu_selected_model];
                else
                    dcu_ctx_pred[dcu_selected_ctx] <= dcu_table_lps[dcu_selected_model];
                
                dcu_range <= dcu_range << 1;
                dcu_input <= dcu_input << 1;
                dcu_input_bits <= dcu_input_bits - 1;
                
                if (dcu_input_bits == 0)
                    dcu_state <= DCU_STATE_DATAREAD;
            end else begin
                if (dcu_symbol_ismps & dcu_table_prob[dcu_selected_model] > 8'h55)
                    dcu_ctx_swap[dcu_selected_ctx] = dcu_ctx_swap[dcu_selected_ctx] ^ 1;
                
                //End of PLANE loop
                if (dcu_plane < (1 << dcu_decompress_mode)) begin
                    dcu_state <= DCU_STATE_PLANE;
                    dcu_plane = dcu_plane + 1;
                end else begin
                    dcu_plane = 0;
                    
                    dcu_index <= dcu_symbol & (1 << (1 << dcu_decompress_mode)) - 1;
                    
                    if (dcu_decompress_mode == DCU_DECOMPMODE_1BPP) begin
                        dcu_index <= dcu_index ^ (dcu_pixels >> 15) & 1;
                    end
                    
                    dcu_pixels <= dcu_pixels << 1 | (dcu_map >> 4 * dcu_index & 4'b1111);
                    
                    //End of PIXEL loop
                    if (dcu_pixel < 8) begin
                        dcu_state <= DCU_STATE_PIXEL;
                        dcu_pixel <= dcu_pixel + 1;
                    end else begin
                        case (dcu_decompress_mode)
                            DCU_DECOMPMODE_1BPP: begin
                                dcu_output <= dcu_pixels;
                            end
                            DCU_DECOMPMODE_2BPP: begin
                                //2BPP is singly deinterleaved
                                dcu_morton <= dcu_pixels & 16'hFFFF;
                                dcu_morton <= 64'h5555555555555555 & (dcu_morton << 16 | dcu_morton >> 1);
                                dcu_morton <= 64'h3333333333333333 & (dcu_morton | dcu_morton >> 1);
                                dcu_morton <= 64'h0f0f0f0f0f0f0f0f & (dcu_morton | dcu_morton >> 2);
                                dcu_morton <= 64'h00ff00ff00ff00ff & (dcu_morton | dcu_morton >> 4);
                                dcu_morton <= 64'h0000ffff0000ffff & (dcu_morton | dcu_morton >> 8);
                                dcu_output <= dcu_morton | dcu_morton >> 16;
                            end
                            DCU_DECOMPMODE_4BPP: begin
                                //4BPP is doubly interleaved
                                dcu_morton <= dcu_pixels & 32'hFFFFFFFF;
                                dcu_morton <= 64'h5555555555555555 & (dcu_morton << 32 | dcu_morton >> 1);
                                dcu_morton <= 64'h3333333333333333 & (dcu_morton | dcu_morton >> 1);
                                dcu_morton <= 64'h0f0f0f0f0f0f0f0f & (dcu_morton | dcu_morton >> 2);
                                dcu_morton <= 64'h00ff00ff00ff00ff & (dcu_morton | dcu_morton >> 4);
                                dcu_morton <= 64'h0000ffff0000ffff & (dcu_morton | dcu_morton >> 8);
                                dcu_morton <= dcu_morton | dcu_morton >> 16;
                                dcu_morton <= dcu_morton & 32'hFFFFFFFF;
                                dcu_morton <= 64'h5555555555555555 & (dcu_morton << 32 | dcu_morton >> 1);
                                dcu_morton <= 64'h3333333333333333 & (dcu_morton | dcu_morton >> 1);
                                dcu_morton <= 64'h0f0f0f0f0f0f0f0f & (dcu_morton | dcu_morton >> 2);
                                dcu_morton <= 64'h00ff00ff00ff00ff & (dcu_morton | dcu_morton >> 4);
                                dcu_morton <= 64'h0000ffff0000ffff & (dcu_morton | dcu_morton >> 8);
                                dcu_output <= dcu_morton | dcu_morton >> 16;
                            end
                        endcase
                        
                        //Reset for next looparound
                        dcu_pixel <= 0;
                        dcu_plane <= 0;
                        
                        dcu_state <= DCU_STATE_OUTBUFFER;
                    end
                end
            end
        end
    endcase
end

//Boring stuff: here's all the arithmetic coding constant data...
initial begin
    dcu_table_prob[00] = 8'h5A; dcu_table_lps[00] = 8'd01; dcu_table_mps[00] = 8'd01; dcu_table_invert[00] = 1'b1;
    dcu_table_prob[01] = 8'h25; dcu_table_lps[01] = 8'd06; dcu_table_mps[01] = 8'd02; dcu_table_invert[01] = 1'b0;
    dcu_table_prob[02] = 8'h11; dcu_table_lps[02] = 8'd08; dcu_table_mps[02] = 8'd03; dcu_table_invert[02] = 1'b0;
    dcu_table_prob[03] = 8'h08; dcu_table_lps[03] = 8'd10; dcu_table_mps[03] = 8'd04; dcu_table_invert[03] = 1'b0;
    dcu_table_prob[04] = 8'h03; dcu_table_lps[04] = 8'd12; dcu_table_mps[04] = 8'd05; dcu_table_invert[04] = 1'b0;
    dcu_table_prob[05] = 8'h01; dcu_table_lps[05] = 8'd15; dcu_table_mps[05] = 8'd05; dcu_table_invert[05] = 1'b0;
    
    dcu_table_prob[06] = 8'h5a; dcu_table_lps[06] = 8'd07; dcu_table_mps[06] = 8'd07; dcu_table_invert[06] = 1'b1;
    dcu_table_prob[07] = 8'h3f; dcu_table_lps[07] = 8'd19; dcu_table_mps[07] = 8'd08; dcu_table_invert[07] = 1'b0;
    dcu_table_prob[08] = 8'h2c; dcu_table_lps[08] = 8'd21; dcu_table_mps[08] = 8'd09; dcu_table_invert[08] = 1'b0;
    dcu_table_prob[09] = 8'h20; dcu_table_lps[09] = 8'd22; dcu_table_mps[09] = 8'd10; dcu_table_invert[09] = 1'b0;
    dcu_table_prob[10] = 8'h17; dcu_table_lps[10] = 8'd23; dcu_table_mps[10] = 8'd11; dcu_table_invert[10] = 1'b0;
    dcu_table_prob[11] = 8'h11; dcu_table_lps[11] = 8'd25; dcu_table_mps[11] = 8'd12; dcu_table_invert[11] = 1'b0;
    dcu_table_prob[12] = 8'h0c; dcu_table_lps[12] = 8'd26; dcu_table_mps[12] = 8'd13; dcu_table_invert[12] = 1'b0;
    dcu_table_prob[13] = 8'h09; dcu_table_lps[13] = 8'd28; dcu_table_mps[13] = 8'd14; dcu_table_invert[13] = 1'b0;
    dcu_table_prob[14] = 8'h07; dcu_table_lps[14] = 8'd29; dcu_table_mps[14] = 8'd15; dcu_table_invert[14] = 1'b0;
    dcu_table_prob[15] = 8'h05; dcu_table_lps[15] = 8'd31; dcu_table_mps[15] = 8'd16; dcu_table_invert[15] = 1'b0;
    dcu_table_prob[16] = 8'h04; dcu_table_lps[16] = 8'd32; dcu_table_mps[16] = 8'd17; dcu_table_invert[16] = 1'b0;
    dcu_table_prob[17] = 8'h03; dcu_table_lps[17] = 8'd34; dcu_table_mps[17] = 8'd18; dcu_table_invert[17] = 1'b0;
    dcu_table_prob[18] = 8'h02; dcu_table_lps[18] = 8'd35; dcu_table_mps[18] = 8'd05; dcu_table_invert[18] = 1'b0;
    
    dcu_table_prob[19] = 8'h5a; dcu_table_lps[19] = 8'd20; dcu_table_mps[19] = 8'd20; dcu_table_invert[19] = 1'b1;
    dcu_table_prob[20] = 8'h48; dcu_table_lps[20] = 8'd39; dcu_table_mps[20] = 8'd21; dcu_table_invert[20] = 1'b0;
    dcu_table_prob[21] = 8'h3a; dcu_table_lps[21] = 8'd40; dcu_table_mps[21] = 8'd22; dcu_table_invert[21] = 1'b0;
    dcu_table_prob[22] = 8'h2e; dcu_table_lps[22] = 8'd42; dcu_table_mps[22] = 8'd23; dcu_table_invert[22] = 1'b0;
    dcu_table_prob[23] = 8'h26; dcu_table_lps[23] = 8'd44; dcu_table_mps[23] = 8'd24; dcu_table_invert[23] = 1'b0;
    dcu_table_prob[24] = 8'h1f; dcu_table_lps[24] = 8'd45; dcu_table_mps[24] = 8'd25; dcu_table_invert[24] = 1'b0;
    dcu_table_prob[25] = 8'h19; dcu_table_lps[25] = 8'd46; dcu_table_mps[25] = 8'd26; dcu_table_invert[25] = 1'b0;
    dcu_table_prob[26] = 8'h15; dcu_table_lps[26] = 8'd25; dcu_table_mps[26] = 8'd27; dcu_table_invert[26] = 1'b0;
    dcu_table_prob[27] = 8'h11; dcu_table_lps[27] = 8'd26; dcu_table_mps[27] = 8'd28; dcu_table_invert[27] = 1'b0;
    dcu_table_prob[28] = 8'h0e; dcu_table_lps[28] = 8'd26; dcu_table_mps[28] = 8'd29; dcu_table_invert[28] = 1'b0;
    dcu_table_prob[29] = 8'h0b; dcu_table_lps[29] = 8'd27; dcu_table_mps[29] = 8'd30; dcu_table_invert[29] = 1'b0;
    dcu_table_prob[30] = 8'h09; dcu_table_lps[30] = 8'd28; dcu_table_mps[30] = 8'd31; dcu_table_invert[30] = 1'b0;
    dcu_table_prob[31] = 8'h08; dcu_table_lps[31] = 8'd29; dcu_table_mps[31] = 8'd32; dcu_table_invert[31] = 1'b0;
    dcu_table_prob[32] = 8'h07; dcu_table_lps[32] = 8'd30; dcu_table_mps[32] = 8'd33; dcu_table_invert[32] = 1'b0;
    dcu_table_prob[33] = 8'h05; dcu_table_lps[33] = 8'd31; dcu_table_mps[33] = 8'd34; dcu_table_invert[33] = 1'b0;
    dcu_table_prob[34] = 8'h04; dcu_table_lps[34] = 8'd33; dcu_table_mps[34] = 8'd35; dcu_table_invert[34] = 1'b0;
    dcu_table_prob[35] = 8'h04; dcu_table_lps[35] = 8'd33; dcu_table_mps[35] = 8'd36; dcu_table_invert[35] = 1'b0;
    dcu_table_prob[36] = 8'h03; dcu_table_lps[36] = 8'd34; dcu_table_mps[36] = 8'd37; dcu_table_invert[36] = 1'b0;
    dcu_table_prob[37] = 8'h02; dcu_table_lps[37] = 8'd35; dcu_table_mps[37] = 8'd38; dcu_table_invert[37] = 1'b0;
    dcu_table_prob[38] = 8'h02; dcu_table_lps[38] = 8'd36; dcu_table_mps[38] = 8'd05; dcu_table_invert[38] = 1'b0;
    
    dcu_table_prob[39] = 8'h58; dcu_table_lps[39] = 8'd39; dcu_table_mps[39] = 8'd40; dcu_table_invert[39] = 1'b1;
    dcu_table_prob[40] = 8'h4d; dcu_table_lps[40] = 8'd47; dcu_table_mps[40] = 8'd41; dcu_table_invert[40] = 1'b0;
    dcu_table_prob[41] = 8'h43; dcu_table_lps[41] = 8'd48; dcu_table_mps[41] = 8'd42; dcu_table_invert[41] = 1'b0;
    dcu_table_prob[42] = 8'h3b; dcu_table_lps[42] = 8'd49; dcu_table_mps[42] = 8'd43; dcu_table_invert[42] = 1'b0;
    dcu_table_prob[43] = 8'h34; dcu_table_lps[43] = 8'd50; dcu_table_mps[43] = 8'd44; dcu_table_invert[43] = 1'b0;
    dcu_table_prob[44] = 8'h2e; dcu_table_lps[44] = 8'd51; dcu_table_mps[44] = 8'd45; dcu_table_invert[44] = 1'b0;
    dcu_table_prob[45] = 8'h29; dcu_table_lps[45] = 8'd44; dcu_table_mps[45] = 8'd46; dcu_table_invert[45] = 1'b0;
    dcu_table_prob[46] = 8'h25; dcu_table_lps[46] = 8'd45; dcu_table_mps[46] = 8'd24; dcu_table_invert[46] = 1'b0;
    
    dcu_table_prob[47] = 8'h56; dcu_table_lps[47] = 8'd47; dcu_table_mps[47] = 8'd48; dcu_table_invert[47] = 1'b1;
    dcu_table_prob[48] = 8'h4f; dcu_table_lps[48] = 8'd47; dcu_table_mps[48] = 8'd49; dcu_table_invert[48] = 1'b0;
    dcu_table_prob[49] = 8'h47; dcu_table_lps[49] = 8'd48; dcu_table_mps[49] = 8'd50; dcu_table_invert[49] = 1'b0;
    dcu_table_prob[50] = 8'h41; dcu_table_lps[50] = 8'd49; dcu_table_mps[50] = 8'd51; dcu_table_invert[50] = 1'b0;
    dcu_table_prob[51] = 8'h3c; dcu_table_lps[51] = 8'd50; dcu_table_mps[51] = 8'd52; dcu_table_invert[51] = 1'b0;
    dcu_table_prob[52] = 8'h37; dcu_table_lps[52] = 8'd51; dcu_table_mps[52] = 8'd43; dcu_table_invert[52] = 1'b0;
end

endmodule

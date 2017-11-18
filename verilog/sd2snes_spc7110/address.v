`timescale 1 ns / 1 ns
//////////////////////////////////////////////////////////////////////////////////
// Company: Rehkopf
// Engineer: Rehkopf
//
// Create Date:    01:13:46 05/09/2009
// Design Name:
// Module Name:    address
// Project Name:
// Target Devices:
// Tool versions:
// Description: Address logic w/ SaveRAM masking
//
// Dependencies:
//
// Revision:
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////
module address(
  input CLK,
  input [7:0] featurebits,  // peripheral enable/disable
  input [2:0] MAPPER,       // MCU detected mapper
  input [23:0] SNES_ADDR,   // requested address from SNES
  input [7:0] SNES_PA,      // peripheral address from SNES
  input SNES_ROMSEL,        // ROMSEL from SNES
  output [23:0] ROM_ADDR,   // Address to request from SRAM0
  output ROM_HIT,           // enable SRAM0
  output IS_SAVERAM,        // address/CS mapped as SRAM?
  output IS_ROM,            // address mapped as ROM?
  output IS_WRITABLE,       // address somehow mapped as writable area?
  input [23:0] SAVERAM_MASK,
  input [23:0] ROM_MASK,
  output msu_enable,
  output srtc_enable,
  output r213f_enable,
  output snescmd_enable,
  output nmicmd_enable,
  output return_vector_enable,
  output branch1_enable,
  output branch2_enable,
  output spc7110_dcu_enable,
);

parameter [2:0]
  FEAT_SPC7110 = 0,
  FEAT_ST0010 = 1,
  FEAT_SRTC = 2,
  FEAT_MSU1 = 3,
  FEAT_213F = 4
;

wire [23:0] SRAM_SNES_ADDR;

/* currently supported mappers:
   Index     Mapper
      000      HiROM+SPC7110
      nnn      Officially, undefined behavior.
               Unofficially, all the other mapper modes I should have wiped.
               I left everything not BSX shaped alone, so you could pair a LoROM
               or ExHiROM game with SPC7110 if you were really nuts.
               
               Also I need to stop typing out "SCP7110" when I write these
               comments...
*/

/* SPC7110: Coproc I/O @ $00-FF:$4800-4842
            SRAM       @ $30-3F:$6000-7fff (Also mirrored to $B0-$BF)
            DCU Mirror @    $50:$0000-FFFF (One @*%*ing register wtf?!)
            PROM       @ $C0-CF:$0000-FFFF (HiROM organization)
            DROM       @ $D0-DF:$0000-FFFF (Bank-switchable)
            DROM       @ $E0-EF:$0000-FFFF (Bank-switchable)
            DROM       @ $F0-FF:$0000-FFFF (Bank-switchable) */

assign IS_ROM = ((!SNES_ADDR[22] & SNES_ADDR[15])
                 |(SNES_ADDR[22]));

assign IS_SAVERAM = SAVERAM_MASK[0]
                    &(((MAPPER == 3'b000
                        || MAPPER == 3'b010
                        || MAPPER == 3'b110)
                      ? (!SNES_ADDR[22]
                         & SNES_ADDR[21]
                         & &SNES_ADDR[14:13]
                         & !SNES_ADDR[15]
                        )
/*  LoROM:   SRAM @ Bank 0x70-0x7d, 0xf0-0xff
 *  Offset 0000-7fff for ROM >= 32 MBit, otherwise 0000-ffff */
                      :(MAPPER == 3'b001)
                      ? (&SNES_ADDR[22:20]
                         & (~SNES_ROMSEL)
                         & (~SNES_ADDR[15] | ~ROM_MASK[21])
                        )
/*  Menu mapper: 8Mbit "SRAM" @ Bank 0xf0-0xff (entire banks!) */
                      :(MAPPER == 3'b111)
                      ? (&SNES_ADDR[23:20])
                      : 1'b0));

wire [2:0] SNES_PSRAM_BANK = bsx_regs[2] ? SNES_ADDR[21:19] : SNES_ADDR[22:20];

assign IS_WRITABLE = IS_SAVERAM;

/* BSX regs:
 Index  Function
    1   0=map flash to ROM area; 1=map PRAM to ROM area
    2   1=HiROM; 0=LoROM
    3   1=Mirror PRAM @60-6f:0000-ffff
    5   1=DO NOT mirror PRAM @40-4f:0000-ffff
    6   1=DO NOT mirror PRAM @50-5f:0000-ffff
    7   1=map BSX cartridge ROM @00-1f:8000-ffff
    8   1=map BSX cartridge ROM @80-9f:8000-ffff
*/

assign SRAM_SNES_ADDR = ((MAPPER == 3'b000)
                          ?(IS_SAVERAM
                            ? 24'hE00000 + ({SNES_ADDR[20:16], SNES_ADDR[12:0]}
                                            & SAVERAM_MASK)
                            : ({1'b0, SNES_ADDR[22:0]} & ROM_MASK))

                          :(MAPPER == 3'b001)
                          ?(IS_SAVERAM
                            ? 24'hE00000 + ({SNES_ADDR[20:16], SNES_ADDR[14:0]}
                                            & SAVERAM_MASK)
                            : ({2'b00, SNES_ADDR[22:16], SNES_ADDR[14:0]}
                               & ROM_MASK))

                          :(MAPPER == 3'b010)
                          ?(IS_SAVERAM
                            ? 24'hE00000 + ({SNES_ADDR[20:16], SNES_ADDR[12:0]}
                                            & SAVERAM_MASK)
                            : ({1'b0, !SNES_ADDR[23], SNES_ADDR[21:0]}
                               & ROM_MASK))
                           :(MAPPER == 3'b110)
                           ?(IS_SAVERAM
                             ? 24'hE00000 + ((SNES_ADDR[14:0] - 15'h6000)
                                             & SAVERAM_MASK)
                             :(SNES_ADDR[15]
                               ?({1'b0, SNES_ADDR[23:16], SNES_ADDR[14:0]})
                               :({2'b10,
                                  SNES_ADDR[23],
                                  SNES_ADDR[21:16],
                                  SNES_ADDR[14:0]}
                                )
                              )
                            )
                           :(MAPPER == 3'b111)
                           ?(IS_SAVERAM
                             ? SNES_ADDR
                             : (({1'b0, SNES_ADDR[22:0]} & ROM_MASK)
                                + 24'hC00000)
                            )
                           : 24'b0);

assign ROM_ADDR = SRAM_SNES_ADDR;

assign ROM_HIT = IS_ROM | IS_WRITABLE;

assign msu_enable = featurebits[FEAT_MSU1] & (!SNES_ADDR[22] && ((SNES_ADDR[15:0] & 16'hfff8) == 16'h2000));
assign srtc_enable = featurebits[FEAT_SRTC] & (!SNES_ADDR[22] && ((SNES_ADDR[15:0] & 16'hfffe) == 16'h2800));

assign r213f_enable = featurebits[FEAT_213F] & (SNES_PA == 8'h3f);

assign snescmd_enable = ({SNES_ADDR[22], SNES_ADDR[15:9]} == 8'b0_0010101);
assign nmicmd_enable = (SNES_ADDR == 24'h002BF2);
assign return_vector_enable = (SNES_ADDR == 24'h002A5A);
assign branch1_enable = (SNES_ADDR == 24'h002A13);
assign branch2_enable = (SNES_ADDR == 24'h002A4D);
endmodule

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 		P.Harvey-Smith
// 
// Create Date:    22:23:49 07/05/2009 
// Design Name: 	 Acorn Atom combined RAM and Rombox
// Module Name:    RamRom 
// Project Name: 
// Target Devices: XC9536, XC9572
// Tool versions: 
// Description: 	
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module RamRom(
    input 	[15:0] Addr,	// 6502 Address bus
    input	PHI2,			// PHI2 clock signal
	input	SpeedSW,		// Processoe speed switch
	input	DskROMSW,		// Onboard disk rom enable ($E000-$EFFF).
	 
    input 	RW,				// Read/Write from CPU
    inout 	[3:0] Data,		// 6502 Data bus (only part for RA latch).

    output 	[16:12] RA,		// Rom Address lines for banked RomBox
    output 	NRDS,			// Read data strobe
    output 	NWDS,			// Write data strobe
    output 	NRAMCS,			// Ram chip select
    output 	NROMCS,			// Rom chip select
	output  NBuffCtl		// Output buffer control
	 );
		
	// Latch for selected banked rom.
	reg [15:12] RomLatch;
	
	// Latch for jumper overide bits
	reg [3:0] 	SwitchLatch;

	// Generate the various enables by combining the position of the select jumper
	// with the apropreate bit in the SwitchLatch.
	// This way we can temporeraly overide thhe settings of the jumpers without
	// having to resort to opening the machine up.
	// Writing a bit in the SwitchLatch to 1 INVERTS the setting of the apropreate
	// jumper, writing it to 0 leaves it unchanged.
	// 2012-08-11, Changed logic so that if an external Disk ROM is enabled then the
	// Hole at $0A00 is also enabled as this is where the FDC does it's I/O.
	assign ExtRAMEN	= SwitchLatch[0];	// No longer an input for this
	assign DskRAMEN	= SwitchLatch[1] ^ ~DskROMSW;	// No longer an input for this
	assign DskROMEN	= SwitchLatch[2] ^ ~DskROMSW;

    // BeebMode enables an alternative ROM memory map where pages 7,A,C,D,E,F come from ROM pages 9,A,C,D,E,F
	assign BeebMode	= SwitchLatch[3];

	// Generate Intel type RD and WR strobes for ROM and RAM 
	assign RDS	= (PHI2 & RW);
	assign WDS	= (PHI2 & ~RW);
	
	assign NRDS	= ~RDS;
	assign NWDS	= ~WDS;

	// Tristate databus output
	//assign Data[3:0]	= 4'bz;
	
	// Chip selects common to both ROM and RAM
	assign 	ExtRomRamCS		= ((Addr>=16'hA000) && (Addr<=16'hAFFF));
	assign	ExtIsRAM		= (ExtRomRamCS && (RomLatch==4'h0));
	
	// Generate RAM chip selects, some are dependent on external lines
	assign 	LowRAMCS	= (Addr<16'h0A00);
	assign	DskRAMCS	= (DskRAMEN && ((Addr>=16'h0A00) && (Addr<=16'h0AFF)));
	assign	MidRAMCS	= ((Addr>=16'h0B00) && (Addr<=16'h6FFF));
	assign	TopRAMCS	= ((ExtRAMEN == BeebMode) && (Addr>=16'h7000) && (Addr<=16'h7FFF));
	assign 	ExtRAMCS	= (ExtRAMEN && ~BeebMode && ExtIsRAM);
	
	// Final combined RAM CS
	assign	RAMCS		= LowRAMCS || DskRAMCS || MidRAMCS || TopRAMCS || ExtRAMCS;
	assign	NRAMCS		= ~RAMCS;

	// Rombox chip select, used to select which of the banked roms to access
	// between $A000-$AFFF
	assign RomBoxCSR	= ((Addr==16'hBFFF) & RDS);
	assign RomBoxCSW	= ((Addr==16'hBFFF) & WDS);
	
	// Update ROM select latch when Rombox CS active for write
	always @(negedge RomBoxCSW)		
	begin
		RomLatch	<= Data[3:0];
	end

	// Switch Latch chip select
	assign SwitchLatchCSR	= ((Addr==16'hBFFE) & RDS);
	assign SwitchLatchCSW	= ((Addr==16'hBFFE) & WDS);
	assign JumperCSR		= ((Addr==16'hBFFD) & RDS);
	
	// Update SwitchLatch when SwitchLatch CS active for write
	always @(negedge SwitchLatchCSW)		
	begin
		SwitchLatch	<= Data[3:0];
	end
	 
	// Temp buffer for reading latches
	wire [3:0] DataOut;
	wire [3:0] JumperOrLatch;
	
	// If we are reading the rom select latch at $BFFF, just output it.
	// If we are reading the control register at $BFFE then 
	// if SwitchLatch[3]	= 0 then output the contents of the SwitchLatch
	// if SwitchLatch[3]	= 1 then output SwitchLatch[3] and the contents of the jumpers
	// In this way code running on the Atom can read both the jumpers and the latch.
	assign LatchRead		= (RomBoxCSR || SwitchLatchCSR || JumperCSR);
	assign JumperOrLatch	= JumperCSR ? { SpeedSW, ~DskROMSW, 1'b0, 1'b0 } : SwitchLatch[3:0];
	assign DataOut[3:0]		= RomBoxCSR ? RomLatch[15:12] : JumperOrLatch;
	assign Data[3:0]		= LatchRead ? DataOut[3:0] : 4'bz;
	
	// Rom Chip selects
    assign  BeebRomCS   = BeebMode && (
        ((Addr>=16'h7000) && (Addr<=16'h7FFF) && ~ExtRAMEN) ||
        ((Addr>=16'hA000) && (Addr<=16'hAFFF)) || 
        ((Addr>=16'hC000) && (Addr<=16'hFFFF)));
	assign	ExtRomCS	= ExtRAMEN ? (ExtRomRamCS && (RomLatch>4'h0)) : ExtRomRamCS;
	assign 	BasRomCS	= ((Addr>=16'hC000) && (Addr<=16'hCFFF));
	assign 	FPRomCS		= ((Addr>=16'hD000) && (Addr<=16'hDFFF));
	assign 	DskRomCS	= (DskROMEN && (Addr>=16'hE000) && (Addr<=16'hEFFF));
	assign 	MOSRomCS	= ((Addr>=16'hF000) && (Addr<=16'hFFFF));
	assign 	SysRomCS	= (BasRomCS || FPRomCS || DskRomCS || MOSRomCS);


    // Drivers for the upper address lines, which are dependent on which chip is being
	// accessed ROM or RAM and the settings of the various enable signals.
	wire [16:12] RARAM;
	wire [16:12] RAROM;	
	
	assign	RARAM[16:12]	= (Addr < 16'h8000) ? {2'b00,Addr[14:12]} : { 5'b00111 };

    // Phill's Original Code
	// assign	RAROM[16:12]	= (Addr < 16'hC000) ? {1'b0,RomLatch[15:12]} : {2'b10,~DskROMEN,Addr[13:12]};	

    assign	RAROM[16:12]	= BeebMode
     ? ((Addr >= 16'h7000 & Addr <=16'h7fff) ? 5'b01001 : {1'b0,Addr[15:12]})
     : ((Addr < 16'hC000) ? {1'b0,RomLatch[15:12]} : {2'b10,~DskROMEN,Addr[13:12]});

	assign	RA[16:12]		= RAMCS ? RARAM[16:12] : RAROM[16:12];
	
	// Rombox chip select, slectedbetwen $A000 and $AFFF
	// System rom chip select in pages $C000, $D000, $F000
	assign 	ROMCS		= (ExtRomCS || SysRomCS || BeebRomCS);
	assign	NROMCS		= ~ROMCS;
	
	// IC2 / IC3 / IC4 enable control.                                                                                     
	// Enable the buffers when an apropreate address is accessed and 
	// any of the following is true :
	// Onboard $0a00-$0aff ram is disabled
	// Onboard $e000-$efff dos rom is disabled
	// I/O in the region $BC00-$BFF0 is accessed.
	// To enable this IC5 must be removed and the NBuffCtl output linked to pin 8 
	// of it's socket.
	assign 	DskRAMBuffEn	= (~DskRAMEN && ((Addr>=16'h0A00) && (Addr<=16'h0AFF)));
	assign 	DskROMBuffEn	= (~DskROMEN && (Addr>=16'hE000) && (Addr<=16'hEFFF));
	assign 	IOBuffEn		= ((Addr>=16'hBC00) && (Addr<=16'hBFF0));
	
	assign 	BuffCtl			= (DskRAMBuffEn | DskROMBuffEn | IOBuffEn);
	assign 	NBuffCtl		= ~BuffCtl;
endmodule

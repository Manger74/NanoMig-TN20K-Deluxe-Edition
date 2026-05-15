//--------------------------------------------------------------------------//
//--------------------------------------------------------------------------//
//                                                                          //
// Copyright (c) 2009-2011 Tobias Gubener                                   //
// Copyright (c) 2017-2019 Alexey Melnikov                                  //
// Subdesign fAMpIGA by TobiFlex                                            //
//                                                                          //
// This is the cpu wrapper to generate 68K Bus signals                      //
// and configure Zorro cards                                                //
//                                                                          //
// This source file is free software: you can redistribute it and/or modify //
// it under the terms of the GNU General Public License as published        //
// by the Free Software Foundation, either version 3 of the License, or     //
// (at your option) any later version.                                      //
//                                                                          //
// This source file is distributed in the hope that it will be useful,      //
// but WITHOUT ANY WARRANTY; without even the implied warranty of           //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            //
// GNU General Public License for more details.                             //
//                                                                          //
// You should have received a copy of the GNU General Public License        //
// along with this program.  If not, see <http://www.gnu.org/licenses/>.   //
//                                                                          //
//--------------------------------------------------------------------------//
//--------------------------------------------------------------------------//
//
// NanoMig cleanup (TG68K-only, 68020-fixed, no Z3, no Toccata):
//   - FX68K removed (saves ~2000-3000 LUTs)
//   - TG68K parameters hardcoded for 68020 (saves ~200-400 LUTs)
//   - Zorro III logic removed: cfg_z3 = 0 (saves ~80-150 LUTs)
//   - Toccata autoconfig removed (saves ~40-80 LUTs)
//

`define ENABLE_TG68K
// `define ENABLE_FX68K   -- removed, FX68K not used on NanoMig

`define TG68K_A24      // limit address space of TG68K to 24 bit

// CPU_SWITCHABLE is not defined since only one CPU is enabled.
// TG68K is used for all configurations.

module cpu_wrapper
(
	input             reset,
	output reg        reset_out,

	input             clk,
	input             ph1,
	input             ph2,

	input       [1:0] cpucfg,
	input       [2:0] fastramcfg,
	input       [2:0] cachecfg,
	input             bootrom,

	output reg [23:1] chip_addr,
	input      [15:0] chip_dout,
	output reg [15:0] chip_din,
	output reg        chip_as,
	output reg        chip_uds,
	output reg        chip_lds,
	output reg        chip_rw,
	input             chip_dtack,
	input       [2:0] chip_ipl,

	input      [15:0] fastchip_dout,
	output reg        fastchip_sel,
	output            fastchip_lds,
	output            fastchip_uds,
	output            fastchip_rnw,
	output reg        fastchip_lw,
	input             fastchip_selack,
	input             fastchip_ready,

	output            ramsel,
	output     [28:1] ramaddr,
	output     [15:0] ramdin,
	input      [15:0] ramdout,
	input             ramready,
	output            ramlds,
	output            ramuds,
	output            ramshared,

	// toccata_ena / toccata_base removed: Toccata not present on NanoMig

	output reg  [1:0] cpustate,
	output reg  [3:0] cacr,
	output reg [31:0] nmi_addr
);

wire cpu_req = (cpustate != 1);

assign ramsel    = cpu_req & ~sel_nmi_vector & (sel_zram | sel_chipram | sel_kickram | sel_dd | sel_rtg);
assign ramshared = sel_dd;

// NMI
always @(posedge clk) nmi_addr <= vbr + 32'h7c;

// Zorro III removed: no Z3 RAM on Tang Nano 20K.
// sel_zram is now only Z2 RAM.
wire sel_z2ram = !cpu_addr[31:24] && (cpu_addr[23] ^ |cpu_addr[22:21]) && z2ram_ena; // addr[23:21] = 1..4
wire sel_zram  = sel_z2ram;

wire sel_dd      = (cpu_addr[31:16] == 16'h00DD) && (cpu_addr[15:13] == 'b010);
wire sel_rtg     = (cpu_addr[31:24] == 8'h02);

// don't sel_kickram when writing
wire sel_kickram   = !cpu_addr[31:24] && (&cpu_addr[23:19] || (cpu_addr[23:19] == 5'b11100)) && ckick && wr; // $f8xxxx, e0xxxx
wire sel_kicklower = !cpu_addr[31:24] && (cpu_addr[23:18] == 6'b111110);
wire sel_chipram   = !cpu_addr[31:21] && cchip;                                                               // $000000 - $1FFFFF

wire sel_nmi_vector = (cpu_addr[31:2] == nmi_addr[31:2]) && (cpustate == 2);

wire [15:0] ramdat;

assign ramlds = sel_rtg ? uds_in : lds_in;
assign ramuds = sel_rtg ? lds_in : uds_in;
assign ramdin = sel_rtg ? {cpu_dout[7:0],cpu_dout[15:8]} : cpu_dout;
assign ramdat = sel_rtg ? {ramdout[7:0], ramdout[15:8]}  : ramdout;

// Mapping for TangNano 20k
// Chip RAM, 00-1f => 00-1f
// Fast RAM, 20-5f => 20-5f
// Slow RAM, c0-d7 => 60-77
// Kick ROM, f8-ff => 78->7f
// ramaddr[21] = cpu_addr[21] | cpu_addr[23]
// All other bits passed through unmodified.
assign ramaddr[28:23] = 6'b0;
assign ramaddr[22:21] = {cpu_addr[22], cpu_addr[21] | cpu_addr[23]};
assign ramaddr[20:1]  = cpu_addr[20:1];

assign fastchip_lds = lds_in;
assign fastchip_uds = uds_in;
assign fastchip_rnw = wr;

// Register address decoding signals to reduce timing baggage on cpu_din
reg ramsel_r;
reg sel_autoconfig_r;

always @(posedge clk) begin
	ramsel_r         <= ramsel;
	sel_autoconfig_r <= sel_autoconfig;
end

reg  [31:0] cpu_addr;
reg  [15:0] cpu_dout;
wire [15:0] cpu_din = ramsel_r ? ramdat : fastchip_selack ? fastchip_dout : {sel_autoconfig_r ? autocfg_data : chip_data[15:12], chip_data[11:0]};
reg         wr;
reg         uds_in;
reg         lds_in;
reg  [15:0] chip_data;
reg  [31:0] vbr;

// TG68K is the only CPU; no runtime mux needed.
always @* begin
		cpu_dout     = cpu_dout_p;
		cpustate     = cpustate_p;
		cacr         = cacr_p;
		vbr          = vbr_p;
		wr           = wr_p;
		uds_in       = uds_p;
		lds_in       = lds_p;
		reset_out    = reset_out_p;
		chip_as      = c_as;
		chip_rw      = c_rw;
		chip_uds     = c_uds;
		chip_lds     = c_lds;
		chip_addr    = cpu_addr_p[23:1];
		chip_din     = cpu_dout_p;
		chip_data    = chipdout_i;
		// TG68K_A24: address space limited to 24 bit, fastchip unused
		cpu_addr     = { 8'h00, cpu_addr_p[23:0] };
		fastchip_sel = 0;
		fastchip_lw  = 0;
end

wire [15:0] cpu_dout_p;
wire [31:0] cpu_addr_p;
wire  [1:0] cpustate_p;
wire  [3:0] cacr_p;
wire [31:0] vbr_p;
wire        wr_p;
wire        uds_p;
wire        lds_p;
wire        reset_out_p;
wire        longword;
reg  [2:0]  cpu_ipl;
reg         chipready;

// TG68K instantiation.
// All parameters hardcoded for 68020-only operation.
// Value meanings: 0=off/16bit/user, 1=on/32bit/privileged, 2=switchable(removed)
TG68KdotC_Kernel
`ifndef VERILATOR
#(
	.sr_read(1),        // always privileged SR read (68020 mode)
	.vbr_stackframe(1), // always VBR + extended stackframe
	.extaddr_mode(1),   // always 32-bit extended addressing
	.mul_mode(1),       // always 32-bit MULS.L / MULU.L
	.div_mode(1),       // always 32-bit DIVS.L / DIVU.L
	.bitfield(1)        // always BFINS/BFEXTS/BFFFO etc.
)
`endif
cpu_inst_p
(
  .clk(clk),
  .nreset(reset),
  .clkena_in(~cpu_req | chipready | ramready | fastchip_ready),
  .data_in(cpu_din),
  .ipl(cpu_ipl),
  .ipl_autovector(1),
  .regin_out(),
  .addr_out(cpu_addr_p),
  .data_write(cpu_dout_p),
  .nwr(wr_p),
  .nuds(uds_p),
  .nlds(lds_p),
  .nresetout(reset_out_p),
  .longword(longword),

  .cpu(cpucfg),
  .busstate(cpustate_p),   // 0: fetch code, 1: no memaccess, 2: read data, 3: write data
  .cacr_out(cacr_p),
  .vbr_out(vbr_p)
);

wire cchip = turbochip_d & (!cpustate | dcache_d);
wire ckick = turbokick_d & (!cpustate | dcache_d);

reg turbochip_d;
reg turbokick_d;
reg dcache_d;
always @(posedge clk) begin
	if (~reset | ~reset_out) begin
		turbochip_d <= 0;
		turbokick_d <= 0;
		dcache_d    <= 0;
	end
	else if (~cpu_req) begin   // No mem access, safe to switch chipram access mode
		turbochip_d <= cachecfg[0];
		turbokick_d <= cachecfg[1];
		dcache_d    <= cachecfg[2];
	end
end

// cfg_z3 = 0: Zorro III not available on Tang Nano 20K.
// All Z3 autoconfig logic and z3ram_base/ena registers removed.
wire cfg_z3 = 1'b0;

// Autoconfig data ROM.
// Toccata removed (not present on NanoMig).
// Z3 branch removed (cfg_z3 = 0).
// Only Zorro II RAM autoconfig remains.
reg [3:0] autocfg_data;
always @(*) begin
	autocfg_data = 4'b1111;

	if (autocfg_card) begin
		// Zorro II RAM (up to 8 MB at $200000)
		case (chip_addr[6:1])
			6'b000000: autocfg_data = 4'b1110; // Zorro-II card, add mem, no ROM
			6'b000001:
				case (fastramcfg)
					       1: autocfg_data = 4'b0110; // 2 MB
					       2: autocfg_data = 4'b0111; // 4 MB
					default: autocfg_data = 4'b0000; // 8 MB
				endcase
			6'b001000: autocfg_data = 4'b1110; // Manufacturer ID: 0x139c
			6'b001001: autocfg_data = 4'b1100;
			6'b001010: autocfg_data = 4'b0110;
			6'b001011: autocfg_data = 4'b0011;
			6'b010011: autocfg_data = 4'b1110; // serial=1
			  default: ;
		endcase
	end
end

wire sel_autoconfig = fastramcfg && chip_addr[23:16] == 8'b11101000 && autocfg_card; // $E80000-$E8FFFF

// autocfg_card is now 1-bit: only one autoconfig slot (Z2 RAM).
// Toccata and Z3 slots removed.
reg       autocfg_card;
reg       z2ram_ena;
always @(posedge clk) begin
	reg old_uds;
	old_uds <= chip_uds;

	if (~reset | ~reset_out) begin
		autocfg_card <= 1; // autoconfig on
		z2ram_ena    <= 0;
	end
	else if (sel_autoconfig && ~chip_rw && ~chip_uds && old_uds) begin
		// Register 0x48: configure Z2 RAM base address
		if (chip_addr[6:1] == 6'b100100) begin
			z2ram_ena    <= 1;
			autocfg_card <= 0;
		end
	end
end

reg       chipreq;
always @(posedge clk) begin
	chipreq <= cpu_req & ~ramsel & ~fastchip_selack;
	cpu_ipl <= ipl_i;
end

reg ph1n, ph2n;
always @(posedge clk) begin
	ph1n <= ph1;
	ph2n <= ph2;
end

reg [15:0] chipdout_i;
reg  [2:0] ipl_i;
reg        c_as, c_rw, c_uds, c_lds;
always @(negedge clk, negedge reset) begin
	reg [1:0] stage;
	reg waitm;
	reg ready;

	if (~reset) begin
		stage <= 0;
		c_as  <= 1;
		c_rw  <= 1;
		c_uds <= 1;
		c_lds <= 1;
		ready <= 0;
	end
	else begin
		if (ph2n) begin
			waitm <= chip_dtack;
			if (~stage[0]) ipl_i <= chip_ipl;
		end

		chipready <= 0;
		if (ph1n) begin
			chipready <= ready;
			ready <= 0;
			case (stage)
				0: if (chipreq) begin
						c_as  <= 0;
						c_rw  <= wr;
						c_uds <= uds_in;
						c_lds <= lds_in;
						stage <= 1;
					end
				1: stage <= 2;
				2: begin
						chipdout_i <= chip_dout;
						if (~waitm) begin
							c_as  <= 1;
							c_rw  <= 1;
							c_uds <= 1;
							c_lds <= 1;
							ready <= 1;
							stage <= 3;
						end
					end
				3: stage <= 0;
			endcase
		end
	end
end

endmodule

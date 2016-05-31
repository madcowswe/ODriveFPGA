// QEI.v

// This file was auto-generated as a prototype implementation of a module
// created in component editor.  It ties off all outputs to ground and
// ignores all inputs.  It needs to be edited to make it do something
// useful.
// 
// This file will not be automatically regenerated.  You should check it in
// to your version control system if you want to keep it.

`timescale 1 ps / 1 ps
module QEI #(
		parameter AUTO_CLOCK_SINK_CLOCK_RATE = "-1"
	) (
		input  wire        clk,          //   clock_sink.clk
		input  wire        reset_n,      //   reset_sink.reset_n
		input  wire        MMS_read,     //             .read
		output reg [31:0] MMS_readdata, //             .readdata
		input  wire        MMS_write,
		input  wire [31:0] MMS_writedata,
		input  wire [1:0]  MMS_addr,
		input  wire [0:2]  encABZ         //    EncoderIn.encab
	);

	reg [31:0] count;

	reg clearOnZ;
	reg revDir;

	reg quadA;
	reg quadB;
	reg encZ;
	reg encAreg;
	reg encBreg;
	reg encZreg;
	always @(posedge clk) encAreg <= encABZ[0];
	always @(posedge clk) encBreg <= encABZ[1];
	always @(posedge clk) encZreg <= encABZ[2];
	always @(posedge clk) quadA <= encAreg;
	always @(posedge clk) quadB <= encBreg;
	always @(posedge clk) encZ <= encZreg;

	reg quadA_delayed, quadB_delayed;
	always @(posedge clk) quadA_delayed <= quadA;
	always @(posedge clk) quadB_delayed <= quadB;

	wire count_enable = quadA ^ quadA_delayed ^ quadB ^ quadB_delayed;
	wire count_direction = quadA ^ quadB_delayed;

	always @(posedge clk or negedge reset_n) begin : proc_count
		if(~reset_n) begin
			count <= 32'h00000000;
			clearOnZ <= 1'b0;
			revDir <= 1'b0;
		end else begin
			if(count_enable) begin
				if(count_direction ^ revDir)
					count<=count+1;
				else
					count<=count-1;
			end
			if(clearOnZ & encZ) begin
				count <= 32'h00000000;
				clearOnZ <= 1'b0;
			end
			if(MMS_write) begin
				case (MMS_addr)
					2'h0:
						count <= MMS_writedata;
					2'h1:
						clearOnZ <= MMS_writedata[0];
					2'h2:
						revDir <= MMS_writedata[0];
					default : /* default */;
				endcase
			end
		end
	end

	always @(*) begin : proc_MMSread
		MMS_readdata <= 32'h00000000;
		case (MMS_addr)
			2'h0:
				MMS_readdata <= count;
			2'h1:
				MMS_readdata[0] <= clearOnZ;
			2'h2:
				MMS_readdata[0] <= revDir;
			default : /* default */;
		endcase
	end

endmodule

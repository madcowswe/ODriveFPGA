

`timescale 1 ps / 1 ps
module Servo  (
		input  wire        clk,           //   clock_sink.clk
		input  wire        reset_n,       //   reset_sink.reset_n
		input  wire [3:0]  MMS_addr,      // avalon_slave.address
		input  wire        MMS_write,     //             .write
		input  wire [31:0] MMS_writedata, //             .writedata
		output reg  [0:2]  Udrive,      //     M0PWMout.udrive
		output reg  [0:2]  Ldrive,      //             .ldrive
		output reg         irqout       //      Trigirq.irq
	);


	reg [15:0] compvalhightoset[2:0];
	reg [15:0] compvallowtoset[2:0];
	reg [15:0] maxctrvaltoset;
	reg updateon0;
	reg updateonmax;
	reg trigon0;
	reg trigonmax;
	reg update;
	reg updateAck;

	integer i;

	reg CPUDone;
	reg En;

	always @(posedge clk or negedge reset_n) begin : proc_MMSinterface
		if(~reset_n) begin
			for (i = 0; i < 3; i = i + 1) begin
				compvalhightoset[i] <= 16'hFFFF;
				compvallowtoset[i] <= 16'h0000;
			end
			maxctrvaltoset <= 16'hFFFF;
			updateon0 <= 1'b0;
			updateonmax <= 1'b0;
			trigon0 <= 1'b0;
			trigonmax <= 1'b0;
			update <= 1'b0;
			En <= 1'b0;
			CPUDone <= 1'b0;
		end else begin
			if(updateAck) begin
				update <= 1'b0;
				CPUDone <= 1'b1;
			end else begin
				if(irqout) begin
					if(!CPUDone) begin
						En <= 1'b0;
					end
					CPUDone <= 1'b0;
				end
			end

			if(MMS_write) begin
				case (MMS_addr)
					4'h0:
						compvallowtoset[0] <= MMS_writedata[15:0];
					4'h1:
						compvalhightoset[0] <= MMS_writedata[15:0];
					4'h2:
						compvallowtoset[1] <= MMS_writedata[15:0];
					4'h3:
						compvalhightoset[1] <= MMS_writedata[15:0];
					4'h4:
						compvallowtoset[2] <= MMS_writedata[15:0];
					4'h5:
						compvalhightoset[2] <= MMS_writedata[15:0];

					4'h8:
						maxctrvaltoset <= MMS_writedata[15:0];
					4'h9:
						En <= MMS_writedata[0];
					4'hA:
						updateon0 <= MMS_writedata[0];
					4'hB:
						updateonmax <= MMS_writedata[0];
					4'hC:
						trigon0 <= MMS_writedata[0];
					4'hD:
						trigonmax <= MMS_writedata[0];

					4'hF:
						update <= MMS_writedata[0];
					default : /* default */;
				endcase
			end
		end
	end

	reg [15:0] compvalhigh[2:0];
	reg [15:0] compvallow[2:0];

	reg [15:0] ctr;
	reg [15:0] maxctrval;
	reg countup;

	always @(posedge clk or negedge reset_n) begin : proc_modulator
		if(~reset_n) begin
			ctr <= 16'h0000;
			countup <= 1'b1;
			for (i = 0; i < 3; i = i + 1) begin
				compvalhigh[i] <= 16'hFFFF;
				compvallow[i] <= 16'h0000;
			end

			maxctrval <= 16'hFFFF;
			updateAck <= 1'b0;

			irqout <= 1'b0;
		end else begin

			irqout <= 1'b0;

			if(!update) begin
				updateAck <= 1'b0;
			end

			if(countup) begin
				if(ctr == maxctrval) begin
					countup <= 0;
					if(update & !updateAck & updateonmax) begin
						updateAck <= 1'b1;
						for (i = 0; i < 3; i = i + 1) begin
							compvalhigh[i] <= compvalhightoset[i];
							compvallow[i] <= compvallowtoset[i];
						end
					end

					if(trigonmax) begin
						irqout <= 1'b1;
					end
				end else begin
					ctr <= ctr + 1;
				end
			end else begin
				if(ctr == 16'h0000) begin
					countup <= 1;
					if(update & !updateAck & updateon0) begin
						updateAck <= 1'b1;
						for (i = 0; i < 3; i = i + 1) begin
							compvalhigh[i] <= compvalhightoset[i];
							compvallow[i] <= compvallowtoset[i];
						end
					end

					if(trigon0) begin
						irqout <= 1'b1;
					end
				end else begin
					ctr <= ctr - 1;
				end
			end

			if(maxctrvaltoset != maxctrval) begin
				maxctrval <= maxctrvaltoset;
				ctr <= 16'h0000;
				countup <= 1'b1;
			end

		end
	end

	always @(posedge clk or negedge reset_n) begin : proc_compare
		if(~reset_n) begin
			Udrive[0] <= 1'b0;
			Ldrive[0] <= 1'b0;
			Udrive[1] <= 1'b0;
			Ldrive[1] <= 1'b0;
			Udrive[2] <= 1'b0;
			Ldrive[2] <= 1'b0;
		end else begin
			Udrive[0] <= (ctr > compvalhigh[0]) & En;
			Ldrive[0] <= (ctr < compvallow[0] ) & En;
			Udrive[1] <= (ctr > compvalhigh[1]) & En;
			Ldrive[1] <= (ctr < compvallow[1] ) & En;
			Udrive[2] <= (ctr > compvalhigh[2]) & En;
			Ldrive[2] <= (ctr < compvallow[2] ) & En;
		end
	end

endmodule
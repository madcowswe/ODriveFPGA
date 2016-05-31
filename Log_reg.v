// Log_reg.v

// This file was auto-generated as a prototype implementation of a module
// created in component editor.  It ties off all outputs to ground and
// ignores all inputs.  It needs to be edited to make it do something
// useful.
//
// This file will not be automatically regenerated.  You should check it in
// to your version control system if you want to keep it.

`timescale 1 ps / 1 ps
module Log_reg (
		input  wire        clk,           //   clock_sink.clk
		input  wire        reset_n,       //   reset_sink.reset_n
		input  wire        MMS_write,     // avalon_slave.write
		input  wire [9:0]  MMS_address,   //             .address
		input  wire [31:0] MMS_writedata  //             .writedata
	);

(* noprune *) reg  [31:0] log [0:3];

always @(posedge clk or negedge reset_n) begin : proc_logcapture
	if(~reset_n) begin
		//Who cares what the inital values are!?
	end else if(MMS_write) begin
		log[MMS_address] <= MMS_writedata;
	end
end

endmodule

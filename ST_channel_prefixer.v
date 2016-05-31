// ST_channel_prefixer.v

`timescale 1 ps / 1 ps
module ST_channel_prefixer (
		input  wire        clk,                 //              clock_sink.clk
		input  wire        reset_n,             //              reset_sink.reset_n
		output wire        prefix_ready,        //                  prefix.ready
		input  wire        prefix_valid,        //                        .valid
		input  wire [1:0]  prefix_data,         //                        .data
		output wire        STin_ready,          //   avalon_streaming_sink.ready
		input  wire        STin_valid,          //                        .valid
		input  wire [11:0] STin_data,           //                        .data
		input  wire [4:0]  STin_channel,        //                        .channel
		input  wire        STin_startofpacket,  //                        .startofpacket
		input  wire        STin_endofpacket,    //                        .endofpacket
		input  wire        STout_ready,         // avalon_streaming_source.ready
		output wire        STout_valid,         //                        .valid
		output wire [11:0] STout_data,          //                        .data
		output wire [6:0]  STout_channel,       //                        .channel
		output wire        STout_startofpacket, //                        .startofpacket
		output wire        STout_endofpacket    //                        .endofpacket
	);

	assign prefix_ready = STout_ready & STin_valid;
	assign STin_ready = STout_ready & prefix_valid;
	assign STout_valid = STin_valid & prefix_valid;

	assign STout_data = STin_data;
	assign STout_channel = {prefix_data, STin_channel};

	assign STout_startofpacket = STin_startofpacket;
	assign STout_endofpacket = STin_endofpacket;

endmodule

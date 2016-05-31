// ST_ch_data_demux_and_swap.v

`timescale 1 ps / 1 ps
module ST_ch_data_demux_and_swap (
		input  wire       clk,                //     clock_sink.clk
		input  wire       reset_n,            //     reset_sink.reset_n
		input  wire       STin_valid,         //          input.valid
		output wire       STin_ready,         //               .ready
		input  wire       STin_startofpacket, //               .startofpacket
		input  wire       STin_endofpacket,   //               .endofpacket
		input  wire [4:0] STin_data,          //               .data
		input  wire [1:0] STin_channel,       //               .channel
		output wire       dico_valid,         // data_in_ch_out.valid
		input  wire       dico_ready,         //               .ready
		output wire       dico_startofpacket, //               .startofpacket
		output wire       dico_endofpacket,   //               .endofpacket
		output wire [4:0] dico_channel,       //               .channel
		input  wire       cido_ready,         // ch_in_data_out.ready
		output wire [1:0] cido_data,          //               .data
		output wire       cido_valid          //               .valid
	);

	//forward swapped ch and data
	assign dico_channel = STin_data;
	assign cido_data = STin_channel;

	//Forward packet signals
	assign dico_startofpacket = STin_startofpacket;
	assign dico_endofpacket = STin_endofpacket;

	//AND ready signals
	assign STin_ready = dico_ready & cido_ready;

	//cross qualify valid signals
	assign dico_valid = STin_valid & cido_ready;
	assign cido_valid = STin_valid & dico_ready;

endmodule

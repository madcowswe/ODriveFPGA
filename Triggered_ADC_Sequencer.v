// Triggered_ADC_Sequencer.v

`timescale 1 ps / 1 ps
module Triggered_ADC_Sequencer 
	(
		input  wire        clk,                 //       clock_sink.clk
		input  wire        reset_n,             //       reset_sink.reset_n
		input  wire        chout_ready,         //   cmd_ch_as_data.ready
		output wire        chout_valid,         //                 .valid
		output wire [4:0]  chout_data,          //                 .data
		output wire        chout_startofpacket, //                 .startofpacket
		output wire        chout_endofpacket,   //                 .endofpacket
		output reg         irq_out,             // interrupt_sender.irq
		input  wire        MMS_read,            //     avalon_slave.read
		input  wire        MMS_write,           //                 .write
		input  wire [4:0]  MMS_address,         //                 .address
		output reg [31:0] MMS_readdata,        //                 .readdata
		input  wire [31:0] MMS_writedata,       //                 .writedata
		input  wire        resp_valid,          //     ADC_response.valid
		input  wire [11:0] resp_data,           //                 .data
		input  wire [4:0]  resp_channel,        //                 .channel
		input  wire        resp_startofpacket,  //                 .startofpacket
		input  wire        resp_endofpacket,    //                 .endofpacket
		input  wire        trig_in              //             Trig.irq
	);

	//regmap:
	//0-15 ctrl
	//16-23 map
	//24-31 store

	localparam REG_EN = 5'h0;
	localparam REG_IRQFLAG = 5'h1;
	localparam REG_MAXSEQ = 5'h2;

	reg [11:0] samp_store [0:7];

	reg En;
	reg [2:0] max_seq;
	reg [4:0] ch_map [0:7];
	always @(posedge clk or negedge reset_n) begin : proc_MMSwrite
		if(~reset_n) begin
			//ch_map is x
			max_seq <= 3'd0;
			irq_out <= 1'b0;
			En <= 1'b0;
		end else begin
			if(MMS_write) begin
				if(MMS_address < 5'h10) begin
					case (MMS_address)
						REG_EN:
							En <= MMS_writedata[0];
						REG_IRQFLAG:
							irq_out <= MMS_writedata[0];
						REG_MAXSEQ:
							max_seq <= MMS_writedata[2:0];
					endcase
				end else if(MMS_address >= 5'h10 && MMS_address < 5'h18) begin
					ch_map[MMS_address & 5'h0F] <= MMS_writedata[4:0];
				end
			end
			
			if(resp_endofpacket & resp_valid) begin
				irq_out <= 1'b1;
			end
		end
	end

	always @(*) begin : proc_MMSread
		MMS_readdata <= 32'b00000000000000000000000000000000;
		if(MMS_address < 5'h10) begin
			case (MMS_address)
				REG_EN:
					MMS_readdata[0]  <= En;
				REG_IRQFLAG:
					MMS_readdata[0]  <= irq_out;
				REG_MAXSEQ:
					MMS_readdata[2:0]  <= max_seq;
			endcase
		end else if(MMS_address >= 5'h10 && MMS_address < 5'h18) begin
			MMS_readdata[4:0] <= ch_map[MMS_address & 5'h0F];
		end else if(MMS_address >= 5'h18) begin
			MMS_readdata[11:0] <= samp_store[MMS_address & 5'h0F];
		end
	end

	reg [2:0] sequence_ctr;
	reg seq_running;
	always @(posedge clk or negedge reset_n) begin : proc_sequence
		if(~reset_n) begin
			sequence_ctr <= 3'b000;
			seq_running <= 1'b0;
		end else begin
			if(~En | chout_endofpacket & chout_valid & chout_ready) begin
				seq_running <= 1'b0;
			end
			if(trig_in & En) begin
				seq_running <= 1'b1;
			end

			if(chout_valid & chout_ready) begin
				sequence_ctr <= sequence_ctr + 3'd1;
				if(sequence_ctr == max_seq) begin
					sequence_ctr <= 3'd0;
				end
			end

			if(~En) begin
				sequence_ctr <= 3'b000;
			end
		end
	end

	assign chout_data = ch_map[sequence_ctr];
	assign chout_valid = seq_running;
	assign chout_startofpacket = sequence_ctr == 3'b000;
	assign chout_endofpacket = sequence_ctr == max_seq;

	reg [2:0] resp_ctr;
	always @(posedge clk or negedge reset_n) begin : proc_resp
		if(~reset_n) begin
			//samp_store is x
			resp_ctr <= 3'b000;
		end else begin
			if(resp_startofpacket) begin
				samp_store[3'd0] <= resp_data;
				resp_ctr <= 3'd1;
			end else if(resp_valid) begin
				samp_store[resp_ctr] <= resp_data;
				resp_ctr <= resp_ctr + 3'd1;
			end
		end
	end

endmodule

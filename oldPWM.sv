module PWM (
	input wire clk, // Clock
	input wire rst_n, // Asynchronous reset active low
	output wire outA [0:2],
	output wire outB [0:2]
	);

	localparam lim_rstval = 3; //32'hffffffff;
	localparam mat_rstval = 2; //32'hffffffff;
	localparam dt_rstval = 10'h3ff;

	reg [31:0]	tc;
	reg [31:0]	mat [0:2];
	reg [31:0]	mat_in [0:2];
	reg [31:0]	lim;
	reg [31:0]	lim_in;
	//reg	[9:0]	dt [0:2];
	reg [9:0]	dt_in [0:2];
	reg 		counting_up;
	reg 		preout [0:2];
	reg			dten;

	always @(posedge clk or negedge rst_n) begin : proc_cpuinterface
		if(~rst_n) begin
			dten <= 0;
			lim_in <= lim_rstval;
			dt_in[i] <= dt_rstval;

			for (int i = 0; i < 3;++i) begin
				mat_in[i] <= mat_rstval;
			end
		end else begin
			/* code */
		end
	end

	always @(posedge clk or negedge rst_n) begin : proc_pwm
		if(~rst_n) begin
			tc <= 0;
			counting_up <= 1;

			lim <= lim_rstval;
			for (int i = 0; i < 3;++i) begin
				mat[i] <= 2; //32'hffffffff;
			end
		end else begin

			//Shadow reg load
			if(tc == 32'h00000000) begin
				lim <= lim_in;
				for (int i = 0; i < 3; ++i) begin
					mat[i] <= mat_in;
				end
			end

			//count direction setting
			if(tc == 32'h00000000) begin
				counting_up = 1;
			end
			if(tc == lim) begin
				counting_up = 0;
			end

			//counting
			if(counting_up) begin
				tc++;
			end else begin
				tc--;
			end
		end
	end

	always @(*) begin : proc_preout
		for (int i = 0; i < 3; ++i) begin
			if(tc >= mat[i])
				preout[i] <= 1;
			else
				preout[i] <= 0;
		end
	end

	generate
		genvar i;
		for (int i = 0; i < 3; ++i) begin
			deadtime deadtime_inst (
				.clk(clk),
				.rst_n(rst_n)
				.dt_in(dt_in[i]),
				.dtoutA(dtoutA[i]),
				.dtoutB(dtoutB[i])
			);
		end
	endgenerate

	always @(*) begin : proc_dten
		for (int i = 0; i < 3; ++i) begin
			if(dten) begin
				outA[i] <= dtoutA[i];
				outB[i] <= dtoutB[i];
			end else begin
				outA[i] <= preout[i];
				outB[i] <= ~preout[i];
			end
		end

	end

endmodule

module deadtime (
	input wire		 clk, // Clock
	input wire		 rst_n, // Asynchronous reset active low
	input wire [9:0] dt_in,
	output reg 		 dtoutA,
	output reg		 dtoutB
	);

	reg 		last_target;
	reg [9:0]	dtctr;

	always @(posedge clk or negedge rst_n) begin : proc_deadtime
		if(~rst_n) begin
			dtctr = 0;
			dtoutA <= 0;
			dtoutB <= 0;
			last_target <= 0;
		end else begin

			//reset or decrement counter
			if(preout != last_target) begin
				dtctr = dt_in;
				last_target <= preout;
			end else begin
				if(dtctr != 10'h0) begin
					dtctr--;
				end
			end

		end
	end

	always @(*) begin : proc_dtout
		dtoutA <= 0;
		dtoutB <= 0;
		if(dtctr == 10'h0) begin
			if(last_target)
				dtoutA <= 1;
			else
				dtoutB <= 1;
		end
	end

endmodule
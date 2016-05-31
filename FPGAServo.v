module FPGAServo(
		input wire CLK_50,
		input wire ENC_A1,
		input wire ENC_A2,
		input wire ENC_A3,
		input wire ENC_B1,
		input wire ENC_B2,
		input wire ENC_B3,
		input wire ENC_Z1,
		input wire ENC_Z2,
		input wire ENC_Z3,
		inout wire [30:5] GPIO,
		//output wire LEDB1,
		//output wire LEDB2,
		//output wire LEDB3,
		//output wire LEDG1,
		//output wire LEDG2,
		//output wire LEDG3,
		//output wire LEDR1,
		//output wire LEDR2,
		//output wire LEDR3,
		output wire PWM_AH1,
		output wire PWM_AL1,
		output wire PWM_BH1,
		output wire PWM_BL1,
		output wire PWM_CH1,
		output wire PWM_CL1,
		output wire PWM_AH2,
		output wire PWM_AL2,
		output wire PWM_BH2,
		output wire PWM_BL2,
		output wire PWM_CH2,
		output wire PWM_CL2,
		output wire PWM_AH3,
		output wire PWM_AL3,
		output wire PWM_BH3,
		output wire PWM_BL3,
		output wire PWM_CH3,
		output wire PWM_CL3,
		input wire RX,
		inout wire [36:31] SGPIO,
		output wire STATUSLED_B,
		output wire STATUSLED_R,
		output wire TX,
		output wire CTS_n,
		input wire RTS_n
	);

	//force -freeze sim:/qsystem_tb/qsystem_inst/altpll_0/areset St0 0
	//force -freeze sim:/qsystem_tb/qsystem_inst/altpll_0/read St0 0
	//force -freeze sim:/qsystem_tb/qsystem_inst/altpll_0/write St0 0

	assign STATUSLED_R = RX;
	assign STATUSLED_B = TX;
	//assign TX = RX;
	//assign CTS_n = 0;
	
	wire [2:0] bollocks;
	wire [1:0] bollocks2;

	qsystem u0 (
		.clk_clk                          (CLK_50),                          //                        clk.clk
		.reset_reset_n                    (1),                    //                      reset.reset_n
		.PWM_0_PWMout_ldrive              ({PWM_AL1, PWM_BL1, PWM_CL1}),              //               PWM_0_PWMout.ldrive
		.PWM_0_PWMout_udrive              ({PWM_AH1, PWM_BH1, PWM_CH1}),              //                           .udrive
		.pwm_1_pwmout_ldrive              ({PWM_AL2, PWM_BL2, PWM_CL2}),              //               pwm_1_pwmout.ldrive
		.pwm_1_pwmout_udrive              ({PWM_AH2, PWM_BH2, PWM_CH2}),              //                           .udrive
		.pwm_2_pwmout_ldrive              ({PWM_AL3, PWM_BL3, PWM_CL3}),              //               pwm_2_pwmout.ldrive
		.pwm_2_pwmout_udrive              ({PWM_AH3, PWM_BH3, PWM_CH3}),              //                           .udrive
		.QEI_0_EncoderIn_encabz           ({ENC_A1, ENC_B1, ENC_Z1}),           //            QEI_0_EncoderIn.encabz
		.qei_1_encoderin_encabz           ({ENC_A2, ENC_B2, ENC_Z2}),           //            qei_1_encoderin.encabz
		.uart_0_external_connection_rxd   (RX),   // uart_0_external_connection.rxd
		.uart_0_external_connection_txd   (TX),   //                           .txd
		.uart_0_external_connection_cts_n (RTS_n), //                           .cts_n
		.uart_0_external_connection_rts_n (CTS_n), //                           .rts_n
		.magnet_pwm_pwmout_ldrive         ({GPIO[20], bollocks2}),         //          magnet_pwm_pwmout.ldrive
      .magnet_pwm_pwmout_udrive         (bollocks)          //                           .udrive
	);


//	reg [32:0] ctr = 0;
//	reg blinkystate = 0;
//	always @(posedge CLK_50) begin : proc_blinky
//		ctr <= ctr + 1;
//		if(ctr == 25000000) begin
//			blinkystate <= ~blinkystate;
//			ctr <= 0;
//		end
//	end
//
//	assign LEDB1 = blinkystate;

endmodule

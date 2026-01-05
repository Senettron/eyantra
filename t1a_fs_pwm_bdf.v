// Copyright (C) 2020  Intel Corporation. All rights reserved.
// Your use of Intel Corporation's design tools, logic functions 
// and other software and tools, and any partner logic 
// functions, and any output files from any of the foregoing 
// (including device programming or simulation files), and any 
// associated documentation or information are expressly subject 
// to the terms and conditions of the Intel Program License 
// Subscription Agreement, the Intel Quartus Prime License Agreement,
// the Intel FPGA IP License Agreement, or other applicable license
// agreement, including, without limitation, that your use is for
// the sole purpose of programming logic devices manufactured by
// Intel and sold by Intel or its authorized distributors.  Please
// refer to the applicable agreement for further details, at
// https://fpgasoftware.intel.com/eula.

// PROGRAM		"Quartus Prime"
// VERSION		"Version 20.1.1 Build 720 11/11/2020 SJ Lite Edition"
// CREATED		"Fri Oct 10 18:15:04 2025"

module t1a_fs_pwm_bdf(
	clk_50M,
	duty_cycle,
	motor_enable,
	motor_ena,
	motor_enb,
	motor_in1,
	motor_in2,
	motor_in3,
	motor_in4
);


input wire	clk_50M;
input wire	[3:0] duty_cycle;
input wire	motor_enable;
output wire	motor_ena;
output wire	motor_enb;
output wire	motor_in1;
output wire	motor_in2;
output wire	motor_in3;
output wire	motor_in4;

wire	SYNTHESIZED_WIRE_0;
wire	pwm_sig;
wire	[3:0] motor_dir_internal;

assign	motor_ena = pwm_sig;
assign	motor_enb = pwm_sig;
assign  motor_in1 = motor_dir_internal[3];
assign  motor_in2 = motor_dir_internal[2];
assign  motor_in3 = motor_dir_internal[1];
assign  motor_in4 = motor_dir_internal[0];




pwm_generator	b2v_inst(
	.clk_3125KHz(SYNTHESIZED_WIRE_0),
	.duty_cycle(duty_cycle),
	.pwm_signal(pwm_sig));


frequency_scaling	b2v_inst1(
	.clk_50M(clk_50M),
	.clk_3125KHz(SYNTHESIZED_WIRE_0));


// Motor direction controller
motor_controller	motor_ctrl(
	.clk_50M(clk_50M),
	.enable(motor_enable),
	.motor_dir(motor_dir_internal));


endmodule

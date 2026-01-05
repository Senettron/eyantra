
module pwm_generator(
    input clk_3125KHz,
    input [3:0] duty_cycle,
    output reg clk_195KHz, pwm_signal
);

initial begin
    clk_195KHz = 0; pwm_signal = 1;
end
//////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE //////////////////

reg [2:0] counter = 0;
reg [3:0] pwm_counter = 0;

always @ (posedge clk_3125KHz) begin
    if (!counter) clk_195KHz <= ~clk_195KHz;
    counter <= counter + 1'b1; 
	 
	 pwm_counter <= pwm_counter + 1'b1;
	 if (pwm_counter < duty_cycle) begin
		pwm_signal <= 1'b1;
	 end
	 else begin
		pwm_signal <= 1'b0;
	 end
end

//////////////////DO NOT MAKE ANY CHANGES BELOW THIS LINE //////////////////

endmodule

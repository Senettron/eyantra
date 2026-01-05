// Motor Direction Controller
// Sequences: Forward -> Left -> Right -> Reverse (3 sec each)
module motor_controller(
    input clk_50M,
    input enable,
    output reg [3:0] motor_dir  // [IN1, IN2, IN3, IN4]
);

// Counter for 3 seconds @ 50MHz = 150,000,000 cycles
reg [27:0] counter;
reg [1:0] state;

// States: 0=Forward, 1=Left, 2=Right, 3=Reverse
always @(posedge clk_50M) begin
    if (enable) begin
        counter <= 28'd0;
        state <= 2'd0;
        motor_dir <= 4'b0000;  // Stop
    end else begin
        if (counter >= 28'd149_999_999) begin
            counter <= 28'd0;
            state <= state + 1;
        end else begin
            counter <= counter + 1;
        end
        
        case (state)
            2'd0: motor_dir <= 4'b1010;  // Forward:  IN1=1, IN2=0, IN3=1, IN4=0
            2'd1: motor_dir <= 4'b0110;  // Left:     IN1=0, IN2=1, IN3=1, IN4=0
            2'd2: motor_dir <= 4'b1001;  // Right:    IN1=1, IN2=0, IN3=0, IN4=1
            2'd3: motor_dir <= 4'b0101;  // Reverse:  IN1=0, IN2=1, IN3=0, IN4=1
        endcase
    end
end

endmodule

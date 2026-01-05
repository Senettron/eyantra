// Integrated Maze Solver - uses external ultrasonic module
module maze_solver_robot(
    input clk_50M,
    input rst_n,
    input echo_front,
    input echo_left,
    input echo_right,
    output trig_front,
    output trig_left,
    output trig_right,
    output motor_ena,
    output motor_enb,
    output motor_in1,
    output motor_in2,
    output motor_in3,
    output motor_in4,
    // Debug LEDs
    output [7:0] LED              // LED[2:0]=walls, LED[5:3]=move, LED[7:6]=status
);

// Internal distance signals from ultrasonic module (raw from pins)
wire [15:0] distance_front_pin, distance_left_pin, distance_right_pin;

// Debug signals for ultrasonic module
(* keep *) wire debug_echo_front = echo_front;
(* keep *) wire debug_echo_left = echo_left;
(* keep *) wire debug_echo_right = echo_right;
(* keep *) wire debug_trig_front = trig_front;
(* keep *) wire debug_rst_n = rst_n;

// Instantiate ultrasonic sensor module
t1b_ultrasonic_top ultrasonic_module(
    .clk_50M(clk_50M),
    .reset(rst_n),
    .echo_front(echo_front),
    .echo_left(echo_left),
    .echo_right(echo_right),
    .trig_front(trig_front),
    .trig_left(trig_left),
    .trig_right(trig_right),
    .distance_front(distance_front_pin),
    .distance_left(distance_left_pin),
    .distance_right(distance_right_pin),
    .led_front(),  // Not used
    .led_left(),
    .led_right()
);

// Remap distances due to physical sensor orientation change
// Physical: left_pin→middle, mid_pin→right, right_pin→left
wire [15:0] distance_left = distance_right_pin;   // Right pin now has left sensor
wire [15:0] distance_front = distance_left_pin;   // Left pin now has middle sensor
wire [15:0] distance_right = distance_front_pin;  // Front pin now has right sensor

// Distance threshold for wall detection (80mm = 8cm)
wire wall_left_raw = (distance_left > 0) && (distance_left < 80);
wire wall_mid_raw = (distance_front > 0) && (distance_front < 80);
wire wall_right_raw = (distance_right > 0) && (distance_right < 80);

// Debounce ONLY mid (front) sensor - require 1 second continuous detection
localparam DEBOUNCE_CYCLES = 26'd20_000_000;  //  second at 50MHz

reg [25:0] wall_mid_counter = 26'd0;
reg wall_mid = 1'b0;

// Left and right use raw detection (no debounce)
wire wall_left = wall_left_raw;
wire wall_right = wall_right_raw;

// Debounce logic - only for middle sensor
always @(posedge clk_50M or negedge rst_n) begin
    if (!rst_n) begin
        wall_mid_counter <= 26'd0;
        wall_mid <= 1'b0;
    end else begin
        // Middle wall debounce
        if (wall_mid_raw) begin
            if (wall_mid_counter < DEBOUNCE_CYCLES)
                wall_mid_counter <= wall_mid_counter + 1;
            else
                wall_mid <= 1'b1;
        end else begin
            wall_mid_counter <= 26'd0;
            wall_mid <= 1'b0;
        end
    end
end

// Preserve signals for SignalTap debugging
(* keep *) wire debug_wall_left = wall_left;
(* keep *) wire debug_wall_mid = wall_mid;
(* keep *) wire debug_wall_right = wall_right;
(* keep *) wire [2:0] debug_maze_move = maze_move;
(* keep *) wire [15:0] debug_dist_left = distance_left;
(* keep *) wire [15:0] debug_dist_mid = distance_front;
(* keep *) wire [15:0] debug_dist_right = distance_right;

// Startup delay counter (wait for sensors to stabilize)
reg [23:0] startup_counter = 24'd0;
wire sensors_ready = (startup_counter == 24'd10_000_000); // 200ms at 50MHz

always @(posedge clk_50M or negedge rst_n) begin
    if (!rst_n)
        startup_counter <= 24'd0;
    else if (!sensors_ready)
        startup_counter <= startup_counter + 1;
end

// Maze explorer outputs
wire [2:0] maze_move_raw;

// Turn duration timer - different durations for different turns
localparam TURN_DURATION = 26'd25_000_000;    // 0.5 seconds for LEFT/RIGHT
localparam UTURN_DURATION = 26'd50_000_000;   // 1.0 second for U-TURN
reg [25:0] turn_timer = 26'd0;
reg [2:0] turn_command = 3'b000;
reg turning = 1'b0;
wire [25:0] current_turn_duration;

// Select duration based on turn type
assign current_turn_duration = (turn_command == U_TURN) ? UTURN_DURATION : TURN_DURATION;

// Override maze command during turn execution
wire [2:0] maze_move;

always @(posedge clk_50M or negedge rst_n) begin
    if (!rst_n) begin
        turn_timer <= 26'd0;
        turn_command <= 3'b000;
        turning <= 1'b0;
    end else begin
        // Detect new turn command (LEFT, RIGHT, or U_TURN)
        if (!turning && (maze_move_raw == LEFT || maze_move_raw == RIGHT || maze_move_raw == U_TURN)) begin
            // Start turn
            turning <= 1'b1;
            turn_command <= maze_move_raw;
            turn_timer <= 26'd0;
        end
        // Execute turn for duration
        else if (turning) begin
            if (turn_timer < current_turn_duration) begin
                turn_timer <= turn_timer + 1;
            end else begin
                // Turn complete
                turning <= 1'b0;
                turn_timer <= 26'd0;
            end
        end
    end
end

// Output actual movement command
assign maze_move = turning ? turn_command : maze_move_raw;

// Motor direction control
reg [3:0] motor_dir;
wire pwm_signal_left, pwm_signal_right;
wire clk_3125KHz;

// Base duty cycle
localparam BASE_DUTY = 4'd10;  // 67% speed
reg [3:0] duty_left, duty_right;

// Wall-following: adjust motor speeds to center robot in corridor
always @(*) begin
    reg signed [16:0] distance_diff;
    reg [3:0] speed_adjustment;
    
    // Calculate difference (left - right)
    distance_diff = distance_left - distance_right;
    
    // Base speed for both motors
    duty_left = BASE_DUTY;
    duty_right = BASE_DUTY;
    
    // Only adjust during forward movement
    if (maze_move == FORWARD) begin
        // Calculate speed adjustment (0-2 levels based on difference)
        // Moderate adaptive wall following
        if (distance_diff > 60) begin
            // Left distance larger → move left (speed up left motor)
            speed_adjustment = 4'd2;
            duty_left = (BASE_DUTY + speed_adjustment > 4'd15) ? 4'd15 : BASE_DUTY + speed_adjustment;
        end
        else if (distance_diff > 30) begin
            speed_adjustment = 4'd1;
            duty_left = (BASE_DUTY + speed_adjustment > 4'd15) ? 4'd15 : BASE_DUTY + speed_adjustment;
        end
        else if (distance_diff < -60) begin
            // Right distance larger → move right (speed up right motor)
            speed_adjustment = 4'd2;
            duty_right = (BASE_DUTY + speed_adjustment > 4'd15) ? 4'd15 : BASE_DUTY + speed_adjustment;
        end
        else if (distance_diff < -30) begin
            speed_adjustment = 4'd1;
            duty_right = (BASE_DUTY + speed_adjustment > 4'd15) ? 4'd15 : BASE_DUTY + speed_adjustment;
        end
        // else: balanced (within ±30mm), no adjustment needed
    end
end

// Movement command decode
localparam STOP    = 3'b000;
localparam FORWARD = 3'b001;
localparam LEFT    = 3'b010;
localparam RIGHT   = 3'b011;
localparam U_TURN  = 3'b100;

// Maze exploration algorithm
t2c_maze_explorer maze_brain(
    .clk(clk_50M),
    .rst_n(rst_n & sensors_ready),  // Don't start maze algorithm until sensors ready
    .left(wall_left),
    .mid(wall_mid),
    .right(wall_right),
    .move(maze_move_raw)
);

// Motor direction mapper: convert maze commands to motor IN1-IN4
always @(*) begin
    case (maze_move)
        STOP:    motor_dir = 4'b0000;  // Stop
        FORWARD: motor_dir = 4'b1010;  // Both motors forward
        LEFT:    motor_dir = 4'b1001;  // Left forward, right reverse (turn left) - SWAPPED
        RIGHT:   motor_dir = 4'b0110;  // Left reverse, right forward (turn right) - SWAPPED
        U_TURN:  motor_dir = 4'b0110;  // Same as RIGHT - turn in place for 1 second
        default: motor_dir = 4'b0000;
    endcase
end

// Assign motor outputs
assign motor_in1 = motor_dir[3];
assign motor_in2 = motor_dir[2];
assign motor_in3 = motor_dir[1];
assign motor_in4 = motor_dir[0];

// PWM generation for motor speed control (differential speeds)
frequency_scaling clk_divider(
    .clk_50M(clk_50M),
    .clk_3125KHz(clk_3125KHz)
);

pwm_generator pwm_gen_left(
    .clk_3125KHz(clk_3125KHz),
    .duty_cycle(duty_left),
    .pwm_signal(pwm_signal_left)
);

pwm_generator pwm_gen_right(
    .clk_3125KHz(clk_3125KHz),
    .duty_cycle(duty_right),
    .pwm_signal(pwm_signal_right)
);

// Connect PWM to motor enables (differential speeds for wall following)
assign motor_ena = (maze_move != STOP) ? pwm_signal_left : 1'b0;
assign motor_enb = (maze_move != STOP) ? pwm_signal_right : 1'b0;

// LED Debug Display
// LED[0] = wall_left detected
// LED[1] = wall_mid detected
// LED[2] = wall_right detected
// LED[5:3] = maze_move command (001=FWD, 010=LEFT, 011=RIGHT, 100=U_TURN)
// LED[6] = sensors_ready
// LED[7] = motor active (any movement)
(* keep *) reg [7:0] LED_reg;
assign LED[0] = wall_left;
assign LED[1] = wall_mid;
assign LED[2] = wall_right;
assign LED[5:3] = maze_move;
assign LED[6] = sensors_ready;
assign LED[7] = (maze_move != STOP);

always @(posedge clk_50M) begin
    LED_reg <= LED;  // Register for SignalTap visibility
end

endmodule

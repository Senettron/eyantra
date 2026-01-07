// Integrated Maze Solver - uses external ultrasonic module
module maze_solver_robot(
    input clk_50M,
    input rst_n,
    input echo_front,
    input echo_left,
    input echo_right,
    input ir_left,              // Left IR sensor (T10) - active low (0=obstacle detected)
    input ir_right,             // Right IR sensor (A4) - active low (0=obstacle detected)
    input ir_uturn,             // U-turn IR sensor (B4) - active low (0=trigger U-turn)
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

// Moving average filter for ultrasonic sensors (4 samples)
reg [15:0] buffer_left [0:3];
reg [15:0] buffer_front [0:3];
reg [15:0] buffer_right [0:3];
reg [17:0] sum_left, sum_front, sum_right;

integer i;
always @(posedge clk_50M or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i < 4; i = i + 1) begin
            buffer_left[i] <= 16'd0;
            buffer_front[i] <= 16'd0;
            buffer_right[i] <= 16'd0;
        end
        sum_left <= 18'd0;
        sum_front <= 18'd0;
        sum_right <= 18'd0;
    end else begin
        // Shift buffers and add new readings
        buffer_left[0] <= distance_right_pin;
        buffer_left[1] <= buffer_left[0];
        buffer_left[2] <= buffer_left[1];
        buffer_left[3] <= buffer_left[2];
        
        buffer_front[0] <= distance_left_pin;
        buffer_front[1] <= buffer_front[0];
        buffer_front[2] <= buffer_front[1];
        buffer_front[3] <= buffer_front[2];
        
        buffer_right[0] <= distance_front_pin;
        buffer_right[1] <= buffer_right[0];
        buffer_right[2] <= buffer_right[1];
        buffer_right[3] <= buffer_right[2];
        
        // Calculate sums
        sum_left <= buffer_left[0] + buffer_left[1] + buffer_left[2] + buffer_left[3];
        sum_front <= buffer_front[0] + buffer_front[1] + buffer_front[2] + buffer_front[3];
        sum_right <= buffer_right[0] + buffer_right[1] + buffer_right[2] + buffer_right[3];
    end
end

// Filtered distances (divide by 4 using right shift by 2)
wire [15:0] distance_left = sum_left[17:2];
wire [15:0] distance_front = sum_front[17:2];
wire [15:0] distance_right = sum_right[17:2];

// Distance threshold for wall detection (80mm = 10cm)
wire wall_left = (distance_left > 0) && (distance_left < 80);
wire wall_mid_raw = (distance_front > 0) && (distance_front < 100);
wire wall_right = (distance_right > 0) && (distance_right < 80);

// Debounce mid sensor for 0.1 second
localparam DEBOUNCE_CYCLES = 23'd15_000_000;  // 0.1 second at 50MHz
reg [22:0] debounce_counter = 23'd0;
reg wall_mid = 1'b0;

always @(posedge clk_50M or negedge rst_n) begin
    if (!rst_n) begin
        debounce_counter <= 23'd0;
        wall_mid <= 1'b0;
    end else begin
        if (wall_mid_raw) begin
            if (debounce_counter < DEBOUNCE_CYCLES)
                debounce_counter <= debounce_counter + 1;
            else
                wall_mid <= 1'b1;
        end else begin
            debounce_counter <= 23'd0;
            wall_mid <= 1'b0;
        end
    end
end

// Startup delay counter (wait for sensors to stabilize)
reg [23:0] startup_counter = 24'd0;
wire sensors_ready = (startup_counter == 24'd10_000_000); // 200ms at 50MHz

always @(posedge clk_50M or negedge rst_n) begin
    if (!rst_n)
        startup_counter <= 24'd0;
    else if (!sensors_ready)
        startup_counter <= startup_counter + 1;
end

// IR Collision Detection (active low - 0 means obstacle detected)
// Only active during forward movement
wire collision_left = ~ir_left;
wire collision_right = ~ir_right;

// Movement command decode
localparam STOP    = 3'b000;
localparam FORWARD = 3'b001;
localparam LEFT    = 3'b010;
localparam RIGHT   = 3'b011;
localparam U_TURN  = 3'b100;

// Maze explorer outputs
wire [2:0] maze_move_raw;

// Stop-and-wait logic for mid wall detection
localparam WAIT_DURATION = 26'd25_000_000;    // 1.0 second wait
localparam COOLDOWN_DURATION = 26'd50_000_000; // 1.0 second cooldown after turn
reg [25:0] wait_timer = 26'd0;
reg waiting = 1'b0;
reg wall_mid_confirmed = 1'b0;
reg [25:0] cooldown_timer = 26'd0;
reg in_cooldown = 1'b0;

always @(posedge clk_50M or negedge rst_n) begin
    if (!rst_n) begin
        wait_timer <= 26'd0;
        waiting <= 1'b0;
        wall_mid_confirmed <= 1'b0;
        cooldown_timer <= 26'd0;
        in_cooldown <= 1'b0;
    end else begin
        // Start cooldown when turn completes
        if (turn_completed && !in_cooldown) begin
            in_cooldown <= 1'b1;
            cooldown_timer <= 26'd0;
            wall_mid_confirmed <= 1'b0;  // Clear confirmation when cooldown starts
        end
        // Cooldown timer after turn
        else if (in_cooldown) begin
            if (cooldown_timer < COOLDOWN_DURATION) begin
                cooldown_timer <= cooldown_timer + 1;
            end else begin
                in_cooldown <= 1'b0;
                cooldown_timer <= 26'd0;
            end
        end
        // Start waiting when mid wall first detected (only if not in cooldown)
        else if (!waiting && !wall_mid_confirmed && wall_mid && !in_cooldown) begin
            waiting <= 1'b1;
            wait_timer <= 26'd0;
        end
        // Wait for 1 second
        else if (waiting) begin
            if (wait_timer < WAIT_DURATION) begin
                wait_timer <= wait_timer + 1;
            end else begin
                // Wait complete - confirm wall if still detected
                waiting <= 1'b0;
                wall_mid_confirmed <= wall_mid;  // Confirm only if still detected
                wait_timer <= 26'd0;
            end
        end
        // Clear confirmation when wall no longer detected
        else if (wall_mid_confirmed && !wall_mid) begin
            wall_mid_confirmed <= 1'b0;
        end
    end
end

// Turn duration timer - different durations for different turns
localparam TURN_DURATION = 26'd25_000_000;    // 0.5 seconds for LEFT/RIGHT
localparam UTURN_DURATION = 26'd50_000_000;   // 1.0 second for U-TURN
reg [25:0] turn_timer = 26'd0;
reg [2:0] turn_command = 3'b000;
reg turning = 1'b0;
reg turn_completed = 1'b0;
wire [25:0] current_turn_duration;

// Select duration based on turn type
assign current_turn_duration = (turn_command == U_TURN) ? UTURN_DURATION : TURN_DURATION;

// U-turn detection: ONLY ir_uturn sensor (no other conditions)
wire uturn_trigger = ~ir_uturn;

always @(posedge clk_50M or negedge rst_n) begin
    if (!rst_n) begin
        turn_timer <= 26'd0;
        turn_command <= 3'b000;
        turning <= 1'b0;
        turn_completed <= 1'b0;
    end else begin
        // Clear turn_completed flag
        if (turn_completed)
            turn_completed <= 1'b0;
            
        // Detect new turn command - U_TURN has highest priority
        if (!turning && uturn_trigger) begin
            // Start U-turn from IR sensor (highest priority)
            turning <= 1'b1;
            turn_command <= U_TURN;
            turn_timer <= 26'd0;
        end
        else if (!turning && (maze_move_raw == LEFT || maze_move_raw == RIGHT)) begin
            // Start turn from maze algorithm
            turning <= 1'b1;
            turn_command <= maze_move_raw;
            turn_timer <= 26'd0;
        end
        // Execute turn for duration
        else if (turning) begin
            if (turn_timer < current_turn_duration) begin
                turn_timer <= turn_timer + 1;
            end else begin
                // Turn complete - signal cooldown to start
                turning <= 1'b0;
                turn_timer <= 26'd0;
                turn_completed <= 1'b1;
            end
        end
    end
end

// Output actual movement command
// Priority: 1) U-turn (absolute priority), 2) Other turns, 3) Waiting (STOP), 4) IR collision, 5) Maze algorithm
wire is_uturn = (turning && turn_command == U_TURN);
wire [2:0] base_move = is_uturn ? U_TURN :
                       (waiting ? STOP : 
                       (turning ? turn_command : maze_move_raw));
// IR collision only works during forward movement (not during LEFT, RIGHT, or U_TURN)
wire [2:0] maze_move = (base_move == FORWARD && collision_left) ? LEFT :
                       (base_move == FORWARD && collision_right) ? RIGHT : base_move;

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
        // Calculate speed adjustment (0-4 levels based on difference)
        // Adaptive wall following with four levels
        if (distance_diff > 100) begin
            // Left distance larger → move left (speed up left motor)
            speed_adjustment = 4'd4;
            duty_left = (BASE_DUTY + speed_adjustment > 4'd15) ? 4'd15 : BASE_DUTY + speed_adjustment;
        end
        else if (distance_diff > 80) begin
            speed_adjustment = 4'd3;
            duty_left = (BASE_DUTY + speed_adjustment > 4'd15) ? 4'd15 : BASE_DUTY + speed_adjustment;
        end
        else if (distance_diff > 60) begin
            speed_adjustment = 4'd2;
            duty_left = (BASE_DUTY + speed_adjustment > 4'd15) ? 4'd15 : BASE_DUTY + speed_adjustment;
        end
        else if (distance_diff > 30) begin
            speed_adjustment = 4'd1;
            duty_left = (BASE_DUTY + speed_adjustment > 4'd15) ? 4'd15 : BASE_DUTY + speed_adjustment;
        end
        else if (distance_diff < -100) begin
            // Right distance larger → move right (speed up right motor)
            speed_adjustment = 4'd4;
            duty_right = (BASE_DUTY + speed_adjustment > 4'd15) ? 4'd15 : BASE_DUTY + speed_adjustment;
        end
        else if (distance_diff < -80) begin
            speed_adjustment = 4'd3;
            duty_right = (BASE_DUTY + speed_adjustment > 4'd15) ? 4'd15 : BASE_DUTY + speed_adjustment;
        end
        else if (distance_diff < -60) begin
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

// Maze exploration algorithm
t2c_maze_explorer maze_brain(
    .clk(clk_50M),
    .rst_n(rst_n & sensors_ready),  // Don't start maze algorithm until sensors ready
    .left(wall_left),
    .mid(wall_mid_confirmed),        // Use confirmed wall after wait
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
assign LED[0] = wall_left;
assign LED[1] = wall_mid;
assign LED[2] = wall_right;
assign LED[5:3] = maze_move;
assign LED[6] = sensors_ready;
assign LED[7] = (maze_move != STOP);

endmodule

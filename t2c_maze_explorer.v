// Task 2C - Adaptive Maze Explorer

module t2c_maze_explorer (
    input clk,
    input rst_n,
    input left, mid, right,
    output reg [2:0] move
);

// Movement commands
localparam STOP    = 3'b000;
localparam FORWARD = 3'b001;
localparam LEFT    = 3'b010;
localparam RIGHT   = 3'b011;
localparam U_TURN  = 3'b100;

// State registers (minimized widths)
reg        cycle;
reg [7:0]  step_count;     // Max ~130 steps
reg [2:0]  last_move;
reg [2:0]  priority_mode;  // Adaptive priority state
reg [6:0]  bot_pos;        // 0-80 (9x9 grid)
reg [1:0]  bot_facing;     // 0=N, 1=E, 2=S, 3=W
reg        deep_mode;      // Advanced exploration phase
reg        alt_path;       // Alternate path explored
reg        run_mode;       // Strategy selector

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // Initialize exploration state
        move          <= STOP;
        cycle         <= 0;
        last_move     <= STOP;
        priority_mode <= 0;
        bot_pos       <= 7'd76;   // Start at [4,8]
        bot_facing    <= 2'b00;   // Facing North
        deep_mode     <= 0;
        alt_path      <= 0;
        run_mode      <= 0;       // Always use left-wall following
        step_count    <= 0;
    end else begin
        // Phase 1: Update position and facing based on move
        if (cycle == 0) begin
            case (move)
                FORWARD: begin
                    // Move forward in current direction
                    case (bot_facing)
                        2'b00: bot_pos <= bot_pos - 7'd9;  // North
                        2'b01: bot_pos <= bot_pos + 7'd1;  // East
                        2'b10: bot_pos <= bot_pos + 7'd9;  // South
                        2'b11: bot_pos <= bot_pos - 7'd1;  // West
                    endcase
                end
                LEFT: begin
                    // Turn left and move forward
                    bot_facing <= (bot_facing + 2'b11) & 2'b11;
                    case ((bot_facing + 2'b11) & 2'b11)
                        2'b00: bot_pos <= bot_pos - 7'd9;
                        2'b01: bot_pos <= bot_pos + 7'd1;
                        2'b10: bot_pos <= bot_pos + 7'd9;
                        2'b11: bot_pos <= bot_pos - 7'd1;
                    endcase
                end
                RIGHT: begin
                    // Turn right and move forward
                    bot_facing <= (bot_facing + 2'b01) & 2'b11;
                    case ((bot_facing + 2'b01) & 2'b11)
                        2'b00: bot_pos <= bot_pos - 7'd9;
                        2'b01: bot_pos <= bot_pos + 7'd1;
                        2'b10: bot_pos <= bot_pos + 7'd9;
                        2'b11: bot_pos <= bot_pos - 7'd1;
                    endcase
                end
                U_TURN: begin
                    // Turn around and move forward
                    bot_facing <= (bot_facing + 2'b10) & 2'b11;
                    case ((bot_facing + 2'b10) & 2'b11)
                        2'b00: bot_pos <= bot_pos - 7'd9;
                        2'b01: bot_pos <= bot_pos + 7'd1;
                        2'b10: bot_pos <= bot_pos + 7'd9;
                        2'b11: bot_pos <= bot_pos - 7'd1;
                    endcase
                end
            endcase
            // Enable deep exploration mode after threshold
            if (step_count > 8'd120)
                deep_mode <= 1;
            cycle <= 1;
        end else begin
            // Phase 2: Simplified decision-making
            
            // Dead end: all three walls detected → U-turn
            if (left && mid && right) begin
                move <= U_TURN;
            end
            // Corridor: both side walls present → go straight
            else if (left && right) begin
                move <= FORWARD;
            end
            // Front + left walls → turn right
            else if (mid && left) begin
                move <= RIGHT;
            end
            // Front + right walls → turn left
            else if (mid && right) begin
                move <= LEFT;
            end
            // Only front wall → turn right
            else if (mid) begin
                move <= RIGHT;
            end
            // Only left wall → go straight
            else if (left) begin
                move <= FORWARD;
            end
            // Only right wall → go straight
            else if (right) begin
                move <= FORWARD;
            end
            // No walls → go straight
            else begin
                move <= FORWARD;
            end
            
            // Update exploration state
            last_move  <= move;
            step_count <= step_count + 1;
            cycle      <= 0;
        end
    end
end

endmodule
// Simple Quadrature Encoder Decoder
// Counts encoder pulses bidirectionally
// Use this to measure PPR (Pulses Per Revolution) by rotating motor by hand

module quad_decoder(
    input clk,           // System clock (connect to clk_50M)
    input rst,           // Reset signal (active high)
    input enc_a,         // Encoder channel A
    input enc_b,         // Encoder channel B
    output reg signed [15:0] position  // Current position count
);

// Synchronizer registers to avoid metastability
reg [1:0] a_sync, b_sync;

// Previous state for edge detection
reg [1:0] prev_state;

// Current synchronized state
wire [1:0] cur_state = {a_sync[1], b_sync[1]};

always @(posedge clk) begin
    if (rst) begin
        // Reset all registers
        a_sync <= 2'b00;
        b_sync <= 2'b00;
        prev_state <= 2'b00;
        position <= 16'd0;
    end else begin
        // Two-stage synchronizer for each encoder input
        a_sync <= {a_sync[0], enc_a};
        b_sync <= {b_sync[0], enc_b};
        
        // Detect state change and determine direction
        if (prev_state != cur_state) begin
            case ({prev_state, cur_state})
                // Forward transitions (clockwise)
                4'b0001, 4'b0111, 4'b1110, 4'b1000: 
                    position <= position + 1;
                
                // Backward transitions (counter-clockwise)
                4'b0010, 4'b0100, 4'b1101, 4'b1011: 
                    position <= position - 1;
                
                // Invalid transitions (noise or missed edges)
                default: 
                    position <= position;
            endcase
            
            prev_state <= cur_state;
        end
    end
end

endmodule

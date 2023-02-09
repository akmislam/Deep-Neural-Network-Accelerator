/*
    Akm Islam
    ESE 507
    Project 3 - Template File
*/

module memory(clk, data_in, data_out, addr, wr_en);
    parameter                   T=16, SIZE=64;
    localparam                  LOGSIZE=$clog2(SIZE);
    input [T-1:0]               data_in;
    output logic [T-1:0]        data_out;
    input [LOGSIZE-1:0]         addr;
    input                       clk, wr_en;

    logic [SIZE-1:0][T-1:0]     mem;

    always_ff @(posedge clk) begin
        data_out <= mem[addr];
        if (wr_en)
            mem[addr] <= data_in;
    end
endmodule

module f_ff(clk, reset, clear, enable_f, acc, f);
    parameter                           T = 14;                         // Parameter for number of bits for values.
    input                               clk, reset, clear, enable_f;    // Clock, reset, and enable signals.
    input signed [T-1:0]                acc;                            // Input acc (from adder).
    output logic signed [T-1:0]         f;                              // Output f.

    always_ff @ (posedge clk) begin                                     // On the positive clock edge only
        if (reset == 1 || clear == 1)                                   // When reset or clear is 1
            f <= 0;                                                     // Set f to 0.
        else if (enable_f == 1)                                         // Otherwise if enable is 1
            f <= acc;                                                   // Set f to accumulated value.
    end

endmodule

module control(clk, reset, input_valid, output_ready,
               addr_x, wr_en_x, addr_w, clear_acc, en_acc,
               input_ready, output_valid, state);

    // Parameters for Control Unit.
    parameter                   T = 14, M = 8, N = 8, P = 1;                                    // T (bits), M (rows), N (columns), P (parallel).
    localparam                  SIZE_x=$clog2(N), SIZE_w=$clog2(M*N);                           // Determine number of bits needed to represent all x addresses and w addresses.
    parameter [1:0]             STATE_INIT = 0,                                                 // State encodings.
                                STATE_STORE_X = 1, 
                                STATE_COMPUTE = 2, 
                                STATE_STALL = 3;
    
    // Internal signals for control unit.
    output logic [1:0]          state; 
    logic [1:0]                 next_state;                                                     // Holds the values for current and next states.
    logic [SIZE_x:0]            iterations_x;                                                   // Holds value of iterations of x values.
    logic [SIZE_w:0]            iterations_w;                                                   // Holds value of iterations of W values.
    logic                       en_acc_delay, clear_acc_delay;                                  // Holds delayed values for enabling and clearing accumulator.
    
    // Inputs and outputs of Control Unit.
    input                       clk, reset, input_valid, output_ready;                          // Input signals.
    output logic [SIZE_x-1:0]   addr_x;                                                         // Output port for x address. 
    output logic [SIZE_w-1:0]   addr_w;                                                         // Output port for w address
    output logic                wr_en_x, clear_acc, en_acc, input_ready, output_valid;          // Output signals

    // Register to store state.
    always_ff @ (posedge clk) begin
        if (reset)                                  // On reset,
            state <= STATE_INIT;                    // Set state to STATE_INIT.
        else begin                                  // Otherwise,
            state <= next_state;                    // State gets next state.
        end
    end

    // Register to store iteration counts.
    always_ff @ (posedge clk) begin
        if (reset) begin                            // On reset,
            iterations_x <= 0;                      // Set iterations_x to 0.
            iterations_w <= 0;                      // Set iterations_w to 0.
        end 

        else if (state == STATE_COMPUTE) begin      // If in computation state, increase each iteration.       
            if (iterations_x == N-1) begin          // If iterations_x is equal to N-1.
                iterations_x <= 0;                  // Set iterations_x back to 0.
                iterations_w <= iterations_w + 1; // Set iterations_w to next P row.
            end
            else begin                              // Otherwise,
                iterations_x <= iterations_x + 1;   // Increment iterations_x.
                iterations_w <= iterations_w + 1;   // Increment iterations_w.
            end
        end

        else if (state == STATE_STALL) begin        // If in stall state,
            iterations_x <= iterations_x;           // Hold iterations_x value.
            iterations_w <= iterations_w;           // Hold iterations_w value.
        end

        else if (state != next_state) begin         // If there is a state transition (not between Stalling and Computing),
            iterations_x <= 0;                      // Reset iterations_x.
            iterations_w <= 0;                      // Reset iterations_w.
        end

        else if (state == STATE_STORE_X && input_valid && input_ready)   // If we are currently storing x values,
            iterations_x <= iterations_x + 1;       // Increase iterations_x on each cycle where data is allowed to transmit.
    end

    // Register to delay en_acc and clear_acc signals.
    always_ff @ (posedge clk) begin
        if (reset) begin                            // On reset,
            en_acc <= 0;                            // Set en_acc to 0.
            clear_acc <= 1;                         // Set clear_acc to 0.
        end
        else begin                                  // Otherwise,
            en_acc <= en_acc_delay;                 // en_acc gets enable signal after 1 cycle.
            clear_acc <= clear_acc_delay;           // clear_acc gets signal after 1 cycle.
        end
    end

    // Output logic
    always_ff @ (posedge clk) begin
        if (reset)                                  // On reset, 
            output_valid <= 0;                      // Set output_valid to 0.
        if (state == STATE_STALL && next_state == STATE_STALL)  // If system is stalling on current and next cycle,
            output_valid <= 1;                      // Set output_valid to 1.
        else                                        // Otherwise,
            output_valid <= 0;                      // Set output_valid to 0.
    end

    // Next state logic block.
    always_comb begin
        case(state)
            STATE_INIT: begin
                // Control Signals
                addr_x = 0;                         // Set addr_x to 0.
                wr_en_x = 0;                        // Disable write to x vector memory.
                addr_w = 0;                         // Set addr_w to 0.
                clear_acc_delay = 1;                // Clear accumulator.
                en_acc_delay = 0;                   // Disable accumulation.

                //Status Signals
                input_ready = 0;                    // System is not ready to take inputs yet.

                // Next State
                if (input_valid)                    // If data is valid to receive,
                    next_state = STATE_STORE_X;     // Move to storing X values.
                else
                    next_state = STATE_INIT;        // Otherwise, remain in idle state.
            end

            STATE_STORE_X: begin                    
                // Control Signals
                addr_x = iterations_x;              // Set addr_x to current iteration of x.
                wr_en_x = 1;                        // Enable write to x vector memory.
                addr_w = 0;                         // Set addr_w to 0.
                clear_acc_delay = 0;                // Do not clear accumulator.
                en_acc_delay = 0;                   // Disable accumulation.

                //Status Signals
                input_ready = 1;                    // System is ready to take inputs.

                // Next State
                if (input_valid && iterations_x == N-1)         // If data is valid to receive, and iterations of x has exceeded expected values,
                    next_state = STATE_COMPUTE;                 // Move to computing y values.
                else                                            // Otherwise, there are still more X values to collect.
                    next_state = STATE_STORE_X;                 // Remain in storing X values state.
            end

            STATE_COMPUTE: begin
                // Control Signals
                addr_x = iterations_x;              // Set addr_x to current iteration of x.
                wr_en_x = 0;                        // Disable write to x vector memory.
                addr_w = iterations_w;              // Set addr_w to current iteration of w.
                clear_acc_delay = 0;                // Do not clear accumulator.
                en_acc_delay = 1;                   // Enable accumulation.

                //Status Signals
                input_ready = 0;                    // System is not ready to take inputs.

                // Next State
                if ((iterations_w + 1) % N == 0)    // If one row of matrix is completed,
                    next_state = STATE_STALL;       // Stall until data is transmitted.
                else                                // Otherwise, there are still more values to compute.
                    next_state = STATE_COMPUTE;     // Remain in computing Y values state.
            end

            STATE_STALL: begin
                // Control Signals
                addr_x = iterations_x;              // Set addr_x to current iteration of x.
                wr_en_x = 0;                        // Disable write to x vector memory.
                addr_w = iterations_w;              // Set addr_w to current iteration of w.
                en_acc_delay = 0;                   // Disable accumulation.

                if (output_valid && output_ready)   // If data is valid and ready to transmit,
                    clear_acc_delay = 1;            // Clear accumulator.
                else                                // Otherwise,
                    clear_acc_delay = 0;            // Do not clear accumulator.

                //Status Signals
                input_ready = 0;                    // System is not ready to take inputs.

                // Next State
                if (output_valid && output_ready) begin             // If data is valid and ready to transmit,
                    if (iterations_w == M*N/P)                        // If all values are computed,          
                        next_state = STATE_INIT;                    // Return to initial state.
                    else                                            // Otherwise, there are more values to compute
                        next_state = STATE_COMPUTE;                 // Return to computing Y values.
                end
                else                                                // Otherwise,
                    next_state = STATE_STALL;                       // Continue stalling.
            end

            default: begin
                // Control Signals
                addr_x = 0;                         // Set addr_x to 0.
                wr_en_x = 0;                        // Disable write to x vector memory.
                addr_w = 0;                         // Set addr_w to 0.
                clear_acc_delay = 1;                // Clear accumulator.
                en_acc_delay = 0;                   // Disable accumulation.

                //Status Signals
                input_ready = 0;                    // System is not ready to take inputs.

                // Next State
                next_state = STATE_INIT;            // Error must have occurred, reinitialize system.
            end

        endcase
    end

endmodule
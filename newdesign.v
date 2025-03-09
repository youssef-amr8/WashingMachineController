module top_washing_machine(
    input clk, rst, pwr_btn, start, stop, pause, door_sensor,
    input [2:0] mode,
    input water_lvl, load_detect,
    output [3:0] crnt_state,
    output door_locked,
    output [1:0] drn_count,
    output timer_done
);
    wire timer_start_pulse;
    wire timer_reset_pulse;
    wire timer_pause_pulse;
    wire timer_resume_pulse;

    // States
    parameter INIT     = 4'b0000;
    parameter IDLE     = 4'b0001;
    parameter WTR_FIL  = 4'b0010;
    parameter WASH     = 4'b0011;
    parameter DRN      = 4'b0100;
    parameter RNS      = 4'b0101;
    parameter SPN      = 4'b0110;
    parameter CMPLET   = 4'b0111;
    parameter OFF      = 4'b1000;
    parameter PAUSED   = 4'b1001;

    // Error Codes
    parameter NO_ERROR          = 3'b000;
    parameter DOOR_OPEN_ERR     = 3'b001;
    parameter WATER_LVL_ERR     = 3'b010;
    parameter LOAD_MISSING_ERR  = 3'b011;


    parameter default_counter = 5;

    // Wash times
    parameter WHITES_TIME = 20;
    parameter HEAVY_DUTY_TIME = 30;
    parameter NORMAL_TIME = 15;
    parameter ECO_TIME = 10;
    parameter QUICK_TIME = 5;
    parameter BULKY_TIME = 25;

	// Instantiation of the controller module
    washing_machine_controller controller (
        .clk(clk),
        .rst(rst),
        .pwr_btn(pwr_btn),
        .start(start),
        .stop(stop),
        .pause(pause),
        .door_sensor(door_sensor),
        .mode(mode),
        .water_lvl(water_lvl),
        .load_detect(load_detect),
        .crnt_state(crnt_state),
        .door_locked(door_locked),
        .drn_count(drn_count),
        .timer_start_pulse(timer_start_pulse),
        .timer_reset_pulse(timer_reset_pulse),
        .timer_pause_pulse(timer_pause_pulse),
        .timer_resume_pulse(timer_resume_pulse),
        .timer_done(timer_done)
    );

    // Instantiation of timer
    timer washing_machine_timer (
        .clk(clk),
        .rst(rst),
        .timer_start(timer_start_pulse),
        .timer_reset(timer_reset_pulse),
        .timer_pause(timer_pause_pulse),
        .timer_resume(timer_resume_pulse),
        .state(crnt_state),
        .mode(mode),
        .timer_finish(timer_done)
    );

endmodule
module washing_machine_controller(
    input clk, rst, pwr_btn, start, stop, pause, door_sensor,
    input [2:0] mode,
    input water_lvl, load_detect,
    output reg [3:0] crnt_state,
    output reg door_locked,
    output reg [1:0] drn_count,
    output wire timer_start_pulse,  // Pulse signals as outputs
    output wire timer_reset_pulse,
    output wire timer_pause_pulse,
    output wire timer_resume_pulse,
    input timer_done
);

    // States
    parameter INIT     = 4'b0000;
    parameter IDLE     = 4'b0001;
    parameter WTR_FIL  = 4'b0010;
    parameter WASH     = 4'b0011;
    parameter DRN      = 4'b0100;
    parameter RNS      = 4'b0101;
    parameter SPN      = 4'b0110;
    parameter CMPLET   = 4'b0111;
    parameter OFF      = 4'b1000;
    parameter PAUSED   = 4'b1001;

    // Error Codes
    parameter NO_ERROR          = 3'b000;
    parameter DOOR_OPEN_ERR     = 3'b001;
    parameter WATER_LVL_ERR     = 3'b010;
    parameter LOAD_MISSING_ERR  = 3'b011;


    parameter default_counter = 5;

    // Wash times
    parameter WHITES_TIME = 20;
    parameter HEAVY_DUTY_TIME = 30;
    parameter NORMAL_TIME = 15;
    parameter ECO_TIME = 10;
    parameter QUICK_TIME = 5;
    parameter BULKY_TIME = 25;

    // Internal one-shot pulse registers
    reg timer_start_pulse_reg;
    reg timer_reset_pulse_reg;
    reg timer_pause_pulse_reg;
    reg timer_resume_pulse_reg;

    reg [3:0] nxt_state;
	reg [3:0] temp_state;
    reg err_flag;
    reg [2:0] err_code;

    // Generate one-shot pulse signals
    assign timer_start_pulse = timer_start_pulse_reg;
    assign timer_reset_pulse = timer_reset_pulse_reg;
    assign timer_pause_pulse = timer_pause_pulse_reg;
    assign timer_resume_pulse = timer_resume_pulse_reg;


    
    timer washing_machine_timer (
    .clk(clk),
    .rst(rst),
    .timer_start(timer_start_pulse),  // Start pulse from controller
    .timer_reset(timer_reset_pulse),  // Reset pulse from controller
    .timer_pause(timer_pause_pulse),  // Pause pulse from controller
    .timer_resume(timer_resume_pulse), // Resume pulse from controller
    .state(crnt_state),                // Current state of the controller
    .mode(mode),                       // Wash mode from controller
    .timer_finish(timer_done)          // Finish signal to controller
);


    always @(posedge clk or posedge rst) begin
        if (rst) begin
            crnt_state <= INIT;
            err_flag <= 0;
            door_locked <= 0;
            drn_count <= 0;
            err_code <= NO_ERROR;

            // Clear pulse signals
            timer_start_pulse_reg <= 0;
            timer_reset_pulse_reg <= 1;  // Trigger reset on reset
            timer_pause_pulse_reg <= 0;
            timer_resume_pulse_reg <= 0;
        end else begin
            crnt_state <= nxt_state;
			timer_start_pulse_reg <= 0;
            timer_reset_pulse_reg <= 0;
        end
    end
	

    always @(*) begin
        nxt_state = crnt_state;

        // Always check for the power button (pwr_btn)
        if (pwr_btn == 0) begin
            nxt_state = OFF;  // Transition to OFF if power button is pressed
        end else begin
            case (crnt_state) //synopsys full_case parallel_case
                INIT: begin
                    timer_reset_pulse_reg = 1; 
                    err_flag     = 0;
                    err_code     = NO_ERROR;
					nxt_state = IDLE;
                end

                IDLE: begin
                     timer_start_pulse_reg = 1;
					
                    if (timer_done) begin
                        nxt_state = OFF;
                    end else if (start) begin
                        timer_reset_pulse_reg = 1; 
                        nxt_state = WTR_FIL;
                    end
                end

                WTR_FIL: begin
					timer_start_pulse_reg = 1;
					if (stop) begin
						nxt_state = DRN;  // Stop transitions to DRN
						timer_reset_pulse_reg = 1;
					end else if (!door_sensor) begin
						err_flag = 1;
						err_code = DOOR_OPEN_ERR;
						nxt_state = PAUSED;
						timer_pause_pulse_reg = 1;
					end else if (!load_detect) begin
						err_flag = 1;
						err_code = LOAD_MISSING_ERR;  // Set load missing error
						nxt_state = PAUSED;
						timer_pause_pulse_reg = 1;
					end else if (water_lvl) begin
						door_locked = 1;
						nxt_state = WASH;
						timer_reset_pulse_reg = 1; 
					end else if (!water_lvl && timer_done) begin
						err_flag = 1;
						err_code = WATER_LVL_ERR;
						nxt_state = PAUSED;
						timer_pause_pulse_reg = 1;
					end
				end


                WASH: begin
                    timer_start_pulse_reg = 1;

                    if (timer_done) begin
                        nxt_state = DRN;
						timer_reset_pulse_reg = 1; 
                    end else if (err_flag || pause) begin
                        nxt_state = PAUSED;
						temp_state = crnt_state;
						timer_pause_pulse_reg = 1;
                    end else if (stop) begin
                        nxt_state = DRN;
						timer_reset_pulse_reg = 1; 
                    end
                end

                DRN: begin
					if (!water_lvl) begin
						// Increment drn_count, ensuring it does not go beyond 2
						if (drn_count < 3) begin
							drn_count = drn_count + 1;
						end

						if (stop) begin
							nxt_state = IDLE;
							timer_reset_pulse_reg = 1; 
						end else begin
							case (drn_count) //synopsys full_case parallel_case
								2'b01: begin nxt_state = RNS; timer_reset_pulse_reg = 1; end// Transition to Rinsing
								2'b10: begin nxt_state = SPN; timer_reset_pulse_reg = 1; end // Transition to Spinning
								2'b11: begin nxt_state = CMPLET; timer_reset_pulse_reg = 1; end// Transition to Completed
							endcase
						end
					end
				end


                RNS: begin
                    timer_start_pulse_reg = 1;
					if (stop) begin
                        nxt_state = DRN;  // Stop transitions to DRN
                        timer_reset_pulse_reg = 1;
                    end 
                    else if (timer_done) begin
                        nxt_state = DRN;
						timer_reset_pulse_reg = 1; 
                    end else if (err_flag || pause) begin
                        nxt_state = PAUSED;
						temp_state = crnt_state;
						timer_pause_pulse_reg = 1;
                    end
                end

                SPN: begin
				
                    timer_start_pulse_reg = 1;
					if (stop) begin
                        nxt_state = DRN;  // Stop transitions to DRN
                        timer_reset_pulse_reg = 1;
                    end 
                    else if (timer_done) begin
                        nxt_state = DRN;
						timer_reset_pulse_reg = 1; 
                    end else if (err_flag || pause) begin
                        nxt_state = PAUSED;
						temp_state = crnt_state;
						timer_pause_pulse_reg = 1;
                    end
                end

                CMPLET: begin
                    timer_reset_pulse_reg = 1; 
                    door_locked  = 0;
                    drn_count    = 0;
                    nxt_state = IDLE;
                end

                OFF: begin
					timer_reset_pulse_reg = 1; 
					door_locked  = 0;
					drn_count    = 0;
					err_flag     = 0;
					err_code     = NO_ERROR;

					if (pwr_btn == 1) begin
						nxt_state = INIT;
					end else begin
						nxt_state = OFF;
					end
				end


                PAUSED: begin
					if (stop) begin
						nxt_state = DRN;  // Stop transitions to DRN
						timer_reset_pulse_reg = 1;
					end else if (err_flag) begin
						// Check if the specific error condition has been resolved
						case (err_code) //synopsys full_case parallel_case
							DOOR_OPEN_ERR: begin
								if (door_sensor) begin
									err_flag = 0;  // Clear error flag
									err_code = NO_ERROR;  // Reset error code
									nxt_state = temp_state;  // Resume to the saved state
									timer_resume_pulse_reg = 1;
								end
							end
							WATER_LVL_ERR: begin
								if (water_lvl) begin
									err_flag = 0;
									err_code = NO_ERROR;
									nxt_state = temp_state;
									timer_resume_pulse_reg = 1;
								end
							end
							LOAD_MISSING_ERR: begin
								if (load_detect) begin
									err_flag = 0;
									err_code = NO_ERROR;
									nxt_state = temp_state;
									timer_resume_pulse_reg = 1;
								end
							end
						endcase
					end else if (!err_flag && start) begin
						// Resume manually if the error is resolved, and user presses start
						nxt_state = temp_state;
						timer_resume_pulse_reg = 1;
						timer_pause_pulse_reg <= 0;
					end else if (!start) begin
						// Ensure no unintended transitions
						 nxt_state = PAUSED;	
						timer_resume_pulse_reg <= 0;
					end
				end
            endcase
        end
    end
	//psl default clock = rose(clk);
//psl property Reset_TO_INIT=always(rst-> eventually!crnt_state==INIT);
//psl assert Reset_TO_INIT;
//psl property Any_state_to_off=always(!pwr_btn-> eventually!(crnt_state==OFF));
//psl assert Any_state_to_off;
//psl property idle_TO_WTR_FIL=always((crnt_state==IDLE && start)-> next(crnt_state==WTR_FIL));
//psl assert idle_TO_WTR_FIL;
//psl property WTR_FIL_TO_DRN=always((crnt_state==WTR_FIL && stop)-> next(crnt_state==DRN));
//psl assert WTR_FIL_TO_DRN;
//psl property WTR_FIL_TO_WASH=always((crnt_state==WTR_FIL && water_lvl)-> eventually!(crnt_state==WASH));
//psl assert WTR_FIL_TO_WASH;
//psl property WASH_TO_DRN=always((crnt_state==WASH && timer_done)||(crnt_state==WASH && stop)-> eventually!(crnt_state==DRN));
//psl assert WASH_TO_DRN;
//psl property WASH_TO_PAUSED=always((crnt_state==WASH && err_flag)||(crnt_state==WASH && pause)-> eventually!(crnt_state==PAUSED));
//psl assert WASH_TO_PAUSED;
//psl property DRN_TO_RNS=always((crnt_state==DRN && drn_count==2'b01)-> eventually!(crnt_state==RNS));
//psl assert DRN_TO_RNS;
//psl property DRN_TO_SPN=always((crnt_state==DRN && drn_count==2'b10)-> eventually!(crnt_state==SPN));
//psl assert DRN_TO_SPN;
//psl property DRN_TO_CMPLET=always((crnt_state==DRN && drn_count==2'b11)-> next(crnt_state==CMPLET));
//psl assert DRN_TO_CMPLET;
//psl property  RNS_TO_DRN=always((crnt_state== RNS && stop)||(crnt_state== RNS && timer_done)-> next(crnt_state==DRN));
//psl assert  RNS_TO_DRN;
//psl property SPN_TO_DRN=always((crnt_state== SPN && stop)||(crnt_state== SPN && timer_done)->eventually!(crnt_state== SPN));
//psl assert SPN_TO_DRN;

//psl property  OFF_TO_INIT=always((crnt_state== OFF && pwr_btn)-> eventually!(crnt_state==INIT));
//psl assert  OFF_TO_INIT;

//psl property PAUSED_TO_temp_state=always((crnt_state== PAUSED && !err_flag)||(crnt_state== PAUSED && start)-> eventually!(crnt_state==temp_state));
//psl assert PAUSED_TO_temp_state;
endmodule
module timer(
    input clk,
    input rst,
    input timer_start,       // Pulse to start the timer
    input timer_reset,       // Pulse to reset the timer
    input timer_pause,       // Pulse to pause the timer
    input timer_resume,      // Pulse to resume the timer
    input [3:0] state,
    input [2:0] mode,
    output reg timer_finish
);
    // States
    parameter INIT     = 4'b0000;
    parameter IDLE     = 4'b0001;
    parameter WTR_FIL  = 4'b0010;
    parameter WASH     = 4'b0011;
    parameter DRN      = 4'b0100;
    parameter RNS      = 4'b0101;
    parameter SPN      = 4'b0110;
    parameter CMPLET   = 4'b0111;
    parameter OFF      = 4'b1000;
    parameter PAUSED   = 4'b1001;


    parameter default_counter = 5;

    // Wash times
    parameter WHITES_TIME = 20;
    parameter HEAVY_DUTY_TIME = 30;
    parameter NORMAL_TIME = 15;
    parameter ECO_TIME = 10;
    parameter QUICK_TIME = 5;
    parameter BULKY_TIME = 25;

    reg [31:0] timer_count;  // Timer counter
    reg timer_active;        // Tracks whether the timer is active
	
    function [31:0] get_timer_duration(input [3:0] crnt_state, input [2:0] wash_mode);
        case (crnt_state) //synopsys full_case
            IDLE, WTR_FIL, RNS, SPN: get_timer_duration = default_counter;
            WASH: begin
                case (wash_mode) //synopsys full_case parallel_case
                    3'b000: get_timer_duration = WHITES_TIME;
                    3'b001: get_timer_duration = HEAVY_DUTY_TIME;
                    3'b010: get_timer_duration = NORMAL_TIME;
                    3'b011: get_timer_duration = ECO_TIME;
                    3'b100: get_timer_duration = QUICK_TIME;
                    3'b101: get_timer_duration = BULKY_TIME;
                endcase
            end
        endcase
    endfunction

     always @(posedge clk or posedge rst) begin
        if (rst) begin
            timer_count  <= 0;
            timer_finish <= 0;
            timer_active <= 0;
        end else if (timer_reset) begin
            timer_count  <= 0;
            timer_finish <= 0;
            timer_active <= 0;
        end else if (timer_start && !timer_active) begin
            timer_count  <= get_timer_duration(state, mode);
            timer_active <= 1;
            timer_finish <= 0;
        end else if (timer_pause) begin
            timer_active <= 0;  // Pause the timer
        end else if (timer_resume) begin
            timer_active <= 1;  // Resume the timer
        end else if (timer_active && timer_count > 0) begin
            timer_count <= timer_count - 1;
            timer_finish <= 0;
        end else if (timer_count == 0) begin
            timer_finish <= 1;
            timer_active <= 0;
        end
    end
endmodule
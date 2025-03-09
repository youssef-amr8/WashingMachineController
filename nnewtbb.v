module tb_top_washing_machine;

    reg clk, rst, pwr_btn, start, stop, pause, door_sensor, water_lvl, load_detect;
    reg [2:0] mode;
    wire [3:0] crnt_state;
    wire door_locked;
    wire [1:0] drn_count;
    wire timer_done;

    top_washing_machine dut (
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
        .timer_done(timer_done)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    always @(posedge clk) begin
        if (crnt_state == 4'b0100) 
            #20 water_lvl = 0;
        else 
            #20 water_lvl = 1;
    end

    task test_mode(input [2:0] mode_to_test);
        begin
            rst = 1;
            pwr_btn = 0;
            start = 0;
            stop = 0;
            pause = 0;
            door_sensor = 1;
            water_lvl = 0;
            load_detect = 1;
            mode = 3'b000;

            #20 rst = 0;
            #10 pwr_btn = 1;
            #10 mode = mode_to_test;
            #10 start = 1;
            #10 start = 0;

            wait(crnt_state == 4'b0111);
            #50;
        end
    endtask

    task test_pause;
        begin
            rst = 1;
            pwr_btn = 0;
            start = 0;
            stop = 0;
            pause = 0;
            door_sensor = 1;
            water_lvl = 0;
            load_detect = 1;
            mode = 3'b000;

            #20 rst = 0;
            #10 pwr_btn = 1;
            #10 mode = 3'b000 + {$random} % 3'b101;
            #10 start = 1;
            #10 start = 0;

            wait(crnt_state == 4'b0010);
            pause = 1;
            #10 pause = 0;
            #10 start = 1;
            #10 start = 0;

            wait(crnt_state == 4'b0011);
            pause = 1;
            #10 pause = 0;
            #10 start = 1;
            #10 start = 0;

            wait(crnt_state == 4'b0101);
            pause = 1;
            #10 pause = 0;
            #10 start = 1;
            #10 start = 0;

            wait(crnt_state == 4'b0110);
            pause = 1;
            #10 pause = 0;
            #10 start = 1;
            #10 start = 0;

            wait(crnt_state == 4'b0111);
            #50;
        end
    endtask

    task stop_in_wtr_fil;
        begin
            rst = 1;
            pwr_btn = 0;
            start = 0;
            stop = 0;
            pause = 0;
            door_sensor = 1;
            water_lvl = 0;
            load_detect = 1;
            mode = 3'b000;

            #20 rst = 0;
            #10 pwr_btn = 1;
            #10 start = 1;
            #20 start = 0;

            wait(crnt_state == 4'b0010);
            stop = 1;
            #20 stop = 0;
            #50;
        end
    endtask

    task stop_in_wash;
        begin
            rst = 1;
            pwr_btn = 0;
            start = 0;
            stop = 0;
            pause = 0;
            door_sensor = 1;
            water_lvl = 0;
            load_detect = 1;
            mode = 3'b000;

            #20 rst = 0;
            #10 pwr_btn = 1;
            #10 start = 1;
            #20 start = 0;

            wait(crnt_state == 4'b0011);
            stop = 1;
            #20 stop = 0;
            #50;
        end
    endtask

    task stop_in_rns;
        begin
            rst = 1;
            pwr_btn = 0;
            start = 0;
            stop = 0;
            pause = 0;
            door_sensor = 1;
            water_lvl = 0;
            load_detect = 1;
            mode = 3'b000;

            #20 rst = 0;
            #10 pwr_btn = 1;
            #10 start = 1;
            #20 start = 0;

            wait(crnt_state == 4'b0101);
            stop = 1;
            #20 stop = 0;
            #50;
        end
    endtask

    task stop_in_spn;
        begin
            rst = 1;
            pwr_btn = 0;
            start = 0;
            stop = 0;
            pause = 0;
            door_sensor = 1;
            water_lvl = 0;
            load_detect = 1;
            mode = 3'b000;

            #20 rst = 0;
            #10 pwr_btn = 1;
            #10 start = 1;
            #20 start = 0;

            wait(crnt_state == 4'b0110);
            stop = 1;
            #20 stop = 0;
            #50;
        end
    endtask
	
	task test_door_open_error;
        begin
            rst = 1;
            pwr_btn = 0;
            start = 0;
            stop = 0;
            pause = 0;
            door_sensor = 0;
            water_lvl = 0;
            load_detect = 1;
            mode = 3'b000;

            #20 rst = 0;
            #10 pwr_btn = 1;
            #10 mode = 3'b000 + {$random} % 3'b101;
            #10 start = 1;
            #20 start = 0;
			wait(crnt_state == 4'b0010);
			door_sensor = 0;
			#50 door_sensor = 1;
            wait(crnt_state == 4'b0111);
            #50;
        end
    endtask

	task test_water_level_error;
		integer i; // Declare a loop counter
		begin
			rst = 1;
			pwr_btn = 0;
			start = 0;
			stop = 0;
			pause = 0;
			door_sensor = 1;
			water_lvl = 1;
			load_detect = 1;
			mode = 3'b000;

			#20 rst = 0;
			#10 pwr_btn = 1;
			#10 mode = 3'b000 + {$random} % 3'b101;
			#10 start = 1;
			#20 start = 0;

			wait(crnt_state == 4'b0010);

			// Ensure water_lvl stays 0 for #50
			for (i = 0; i < 10; i = i + 1) begin
				#1 water_lvl = 0; // Keep water_lvl at 0 for each unit of time
			end

			#50 water_lvl = 1;

			wait(crnt_state == 4'b0111);
			#50;
		end
	endtask


	task test_load_missing_error;
        begin
            rst = 1;
            pwr_btn = 0;
            start = 0;
            stop = 0;
            pause = 0;
            door_sensor = 1;
            water_lvl = 0;
            load_detect = 0;
            mode = 3'b000;

            #20 rst = 0;
            #10 pwr_btn = 1;
            #10 mode = 3'b000 + {$random} % 3'b101;
            #10 start = 1;
            #20 start = 0;
			wait(crnt_state == 4'b0010);
			load_detect = 0;
			#50 load_detect = 1;
            wait(crnt_state == 4'b0111);
            #50;
        end
    endtask
	task test_idle_auto_off;
		begin
			rst = 1;
            pwr_btn = 0;
            start = 0;
            stop = 0;
            pause = 0;
            door_sensor = 1;
            water_lvl = 0;
            load_detect = 1;
            mode = 3'b000;

            #20 rst = 0;
            #10 pwr_btn = 1;
            #10 mode = 3'b000 + {$random} % 3'b101;
			wait(crnt_state == 4'b1000)
            #10 start = 1;
            #20 start = 0;

            wait(crnt_state == 4'b0111);
            #50;
		end
	endtask
	
    initial begin
		test_idle_auto_off;
        test_mode(3'b000);
        test_mode(3'b001);
        test_mode(3'b010);
        test_mode(3'b011);
        test_mode(3'b100);
        test_mode(3'b101);
        test_pause;
        stop_in_wtr_fil;
        stop_in_wash;
        stop_in_rns;
        stop_in_spn;
		test_door_open_error;
		test_water_level_error;
		test_load_missing_error;
        $stop;
    end

endmodule
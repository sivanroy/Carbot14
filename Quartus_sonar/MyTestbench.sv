`timescale 1ns/1ns

module MyTestbench();

  logic        clk;
  logic        reset;
  logic	      trigger;
  logic			echo;
  logic [31:0]	distance;
  
	
  MySonar dut(
   	.clk(clk),
    	.reset(reset),
		.echo(echo),
    	.trigger(trigger),
		.distance(distance));

	
  // generate clock to sequence tests
  always #10 clk = ~clk;

  // initialize test
  initial
    begin
      reset = 1;
		clk = 0;
		echo = 0;
		# 10; 
		reset = 0;
		//trigger = 1; 
		# 10000;     //10µs
		//trigger = 0;
		# 200000; // the 8 pulses
		echo = 1;
		# 30000000; // echo of 30ms
		echo = 0;
		#60 
		$display("time_echo = %d", distance); //should equal 1500000
		# 8000000
		# 500000000 // wait  0.5 sec
		//trigger = 1; 
		# 10000;     //10µs
		//trigger = 0;
		# 200000; // the 8 pulses
		echo = 1;
		# 10000000; // echo of 10ms
		echo = 0;
		# 60   
		$display("time_echo = %d", distance); //should equal 500000
		# 60; 
    end

endmodule 
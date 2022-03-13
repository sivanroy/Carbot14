`timescale 1ns/1ns

module MyTestbench();

  logic        clk;
  logic        reset;
  logic	      launch;
  logic	[6:0]	Adr;
  logic [31:0]	distance;
  logic i2c_sda;
  logic i2c_scl;
  logic ready;
  logic [7:0] data_out;
  
/*module MySonarI2C(
	input logic clk,reset,launch,
	input logic [6:0] Adr,
	output logic [31:0] distance,
	inout i2c_sda,
	inout wire i2c_scl
);*/
  MySonarI2C dut(
		.ready(ready),
		.data_out(data_out),
   	.clk(clk),
    	.reset(reset),
		.launch(launch),
    	.Adr(Adr),
		.distance(distance),
		.i2c_sda(i2c_sda),
		.i2c_scl(i2c_scl));

	
//  // generate clock to sequence tests
  always #1 clk = ~clk;

  // initialize test
  initial
    begin
		//initialize
      reset = 1;
		clk = 0;
		launch = 0;
		Adr = 7'h5E;
		ready = 0;
		# 3;
		reset = 0;
		# 5;
		
		//Regselect
		launch = 1;
		# 5;
		launch = 0;
		#22;
		
		//35
		//CM
		ready = 1;
		#3;
		ready = 0;
		#22;
		//60
		
		//RegSelect
		ready = 1;
		#3;
		ready = 0;
		#22
		//85
		
		//Pause
		ready = 1;
		#3;
		ready =0;
		#47;
		//135
		
		//enter in read
		#25;
		//160
		
		//RegSelect
		data_out = 8'b01;
		ready = 1;
		#3;
		ready = 0;
		#22;
		//185
		
		//enter in read
		data_out = 8'b11;
		ready = 1;
		#3;
		ready = 0;
		#22;
		//210
		
		//S0
		ready = 1;
		#3;
		ready = 0;
		#22;
		//235
		
		//should end here		



    end

endmodule 
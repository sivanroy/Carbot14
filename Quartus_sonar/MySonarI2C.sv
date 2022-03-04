module MySonarI2C(
	input logic ready, //to testbench
	input [7:0] data_out,
	input logic clk,reset,launch,
	input logic [6:0] Adr,
	output logic [31:0] distance,
	output logic i2c_sda,
	output logic i2c_scl
	//inout i2c_sda,
	//inout wire i2c_scl
);

	localparam cm = 8'h51;
	//I2C comm initialisation
	logic rst,enable,rw;  //,ready; !!
	logic [6:0] addr;
	logic [7:0] data_in;  //,data_out; !!
	//i2c_controller sonar1(clk,rst,addr,data_in,enable,rw,data_out,ready,i2c_sda,i2c_scl);
	
	typedef enum logic [2:0] {S0,CM,RegSelect,Pause,Read} statetype;
	statetype state, nextstate;
	
	//usefull data
	logic [31:0]counter;
	logic [7:0]Reg;
	logic [2:0]done;
	logic running,i2c_end;
	logic [7:0] highByte;
	logic [7:0] lowByte;
	logic [31:0] total_distance;
	
	//counter logic
	always @(posedge clk) begin
		if (reset| (nextstate == S0 & !running) ) counter <= 0;
		if (nextstate != state) counter <= 0;
		else counter <= counter +1;
	end
	
	//nextstate  update
	always @(posedge clk) begin
		if (reset) state <= S0;
		else state <= nextstate;
	end
	
	//check for done value & running
	always @(posedge clk) begin
		if (reset) done <= 0;
		if ((state == S0) & launch) done <= 0;
		if ((state == Read) & (nextstate == S0)) done <= 0;
		if (state == CM) done <= 1;
		if ((nextstate == RegSelect) & (state == Read) &(done==1)) done <= 2;
		//done check
		running <= 1;
		if (reset) running <= 0;
		if ((state == Read) & (nextstate == S0)) running <= 0;
		if ((state == S0) & (done == 0)) running <= 0;
	end
	
	//check if comm ended
	always @(posedge clk) begin
		i2c_end <= 0;
		if (state == CM | state == Read | state == RegSelect) begin
			if (ready) i2c_end <= 1;
			end
	end
	
	// nextstate logic
	always_comb begin
		nextstate = state;
		case(state)
			S0: begin
				if (launch|running) begin
					nextstate = RegSelect;
					end
				end
			
			RegSelect: begin
				if (i2c_end & counter > 10) begin
					if (done == 0) begin
						nextstate = CM;
						end
					if (done == 1)begin
						nextstate = Pause;
						end
					if (done == 2) begin 
						nextstate = Read;
						end
					end
				end
				
			CM:begin					
				if (i2c_end & counter > 10) nextstate = S0;
				end
				
			Pause:begin
				//if(counter>= 32'd3_250_000) nextstate = Read;
				if(counter>= 32'd20) nextstate = Read;
				end
				
			Read:begin
				if (i2c_end & counter > 10) begin
					if (done == 2) nextstate = S0;
					if (done == 1) nextstate = RegSelect;
					end
				end
				
		endcase
	end
	
	
	//logic to communicate
	//addr,data_in,enable,rw,data_out,ready,i2c_sda,i2c_scl
	always_comb begin
		Reg=0; enable=0; rst=0; addr = Adr;
		rw=0 ; data_in = 8'b0;
		case(state)
			RegSelect: begin
				if (counter< 3) enable=1;
				rw=0;
				if (done == 0) begin
					data_in = 8'h00;
					end
				if (done == 1)begin
					data_in = 8'h02;
					end
				if (done == 2) begin 
					data_in = 8'h03;
					end
				end
				
			CM:begin	
				if (counter< 3) enable=1;
				rw = 0;
				data_in = 8'h51;
				end
				
			Read:begin
				if (counter< 3) enable=1;
				rw =1;
				end
				
		endcase
	end
	
	assign highByte = ((state == Read) & (done == 1))? data_out:highByte;
	assign lowByte = ((state == Read)  & (done == 2))? data_out:lowByte;
	assign total_distance = lowByte+(highByte<<8);
	assign distance = ((state == Read) & (nextstate == S0)) ?total_distance: distance;
	

endmodule

module MySonar(
	input logic clk,reset,echo,
	output logic trigger,
	output logic [31:0] distance
);
	
	typedef enum logic [1:0] {S0,Trigger,Echo,Compute} statetype;
	statetype state, nextstate;
	
	logic [31:0]counter;
	logic [31:0]count_dist;
	logic prev_echo;
	
	always @(posedge clk) begin
		if (reset) begin
			counter <= 0;
			count_dist <= 0;
		end
		else begin
			counter <= counter +1;	
			if (echo & state==Echo) count_dist <= count_dist + 1;
		end
	end
	
	
	always @(posedge clk) begin
		if (reset) state <= S0;
		else state <= nextstate;
	end	
	
	always_comb begin
		nextstate = state;
		trigger = 0; prev_echo = 0;
		case(state)
			S0: begin
				nextstate = Trigger;
				end
				
			Trigger:begin
				if (counter >= 32'd500) nextstate = Echo;
				trigger = 1;
				end
			
			Echo: begin
				if(prev_echo & !echo) nextstate = Compute;
				prev_echo = echo;
				if(count_dist == 32'd1_900_000) nextstate = Compute;
				end			
			Compute:begin
				if (counter >= 32'd3_000_000) nextstate = S0;
				end
		endcase
	end
	
	assign distance = (state == Compute)? count_dist: distance;
	

endmodule

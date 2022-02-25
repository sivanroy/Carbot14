module MySonar(
	input logic clk,set,echo,
	output logic trigger,running,
	output logic [31:0] 	distance
);
	
	typedef enum logic [1:0] {S0,Trigger,Echo,Compute} statetype;
	statetype state, nextstate;
	
	logic [31:0]counter;
	logic [31:0]count_dist;
	logic prev_echo;
	
	
	always_ff @(posedge clk) begin
		if (set) begin
			state <= S0;
			counter <= 1;
			count_dist <= 0;
			prev_echo <= 0;
		end
		if (echo) count_dist <= count_dist + 1;
		else begin
			state <= nextstate;
			counter <= counter + 1;
			prev_echo <= echo;
		end
	end
	
	//problem comes from counter +=1 !!
	always_comb begin
		case(state)
			S0:begin
				trigger = 0;
				running = 1;
				nextstate = Trigger;
				distance = 0;
			end
			
			Trigger:begin
				if(counter==32'd500) begin
					trigger = 0;
					nextstate = Echo;
				end
				else trigger = 1;
			end
			
			Echo:begin
				if(!echo & prev_echo) nextstate = Compute;
			end
				
			
			Compute:begin
					//distance = count_dist/50 /58; //50 to have us and 58 to have cm
					if (counter == 32'd3_000_000) begin
						nextstate = S0;
						running = 0;
						distance = count_dist;
					end
			end
			
			default: nextstate = S0;
		endcase
	end
	
		
	

endmodule
module encodergr2(
	 input clk,
    input reset,
    input A,
    input B,
    output reg signed [31:0] diff_count
    ); 
	 
typedef enum logic [1:0] {S0, S1, S2, S3} statetype;
statetype state, nextstate;
	 
logic [31:0] counter;		  // timer to compute the diff each 5ms
logic [31:0] odo_count;      //current value
logic [31:0] odo_count_prev; // previous value
logic count_finish;

quad_dec dq1(
    .clk(clk),
    .reset(reset),
    .A(A),
    .B(B),
    .count(odo_count));  

/*
S0 = START   	[ Starting : put counter to zero ]
S1 = WAIT	 	[ wait 5ms befor computing the diff ]
S2 = DIFF 		[ Compute the difference with th read valu and prev ]
S3 = UPDATE    [ Update the prev counter to current count] 
*/

initial 
	begin
		odo_count_prev <= 0;
	end

always @(posedge clk)
	begin
		if(reset) state <= S0;

		else state <= nextstate;
	end
		
always @(posedge clk)
	begin
		case(state)
			S0: nextstate = S1;
			S1: if (count_finish) nextstate = S2;
				 else nextstate = S1;
			S2: nextstate = S3;
			S3: nextstate = S0;
			default: nextstate = S0;
		endcase
	end
			
// increment counter :
always @(posedge clk)
	begin
		if (state == S0) begin
			counter <= 0;
		end else counter <= counter + 1;
	end
	
	
always @(posedge clk)
	begin
		if (state == S0) begin
			count_finish <= 0;
		end else if(state == S1) begin
			count_finish <= (counter >= 500000); // to have 0.001s duration
		end else if (state == S2) begin
			diff_count <= odo_count - odo_count_prev;
		end else if (state == S3) begin
			odo_count_prev <= odo_count;
		end
	end	 
endmodule

////////





module quad_dec(
    input clk,
    input reset,
    input A,
    input B,
    output reg signed [31:0] count
    );    

reg[1:0] sync, AB; // synchronization registers
typedef enum logic [1:0] {S00, S01, S10, S11} statetype;
statetype state;

always @ (posedge clk) // two-stage input synchronizer
    begin
        sync <= {A,B};
        AB <= sync;
    end

always @(posedge clk, posedge reset) // always block to compute output
    begin 
        if(reset) begin
            state <= S00;
            count <= 0;
        end else
            case(state)
                S00: if(AB == 2'b01) begin
                        count <= count-1;
                        state <= S01;
                    end else if(AB == 2'b10) begin
                        count <= count+1;
                        state <= S10;
                    end                                        
                S01: if(AB == 2'b00) begin
                        count <= count+1;
                        state <= S00;
                    end else if(AB == 2'b11) begin
                        count <= count-1;
                        state <= S11;
                    end                      
                S10: if(AB == 2'b00) begin
                        count <= count-1;
                        state <= S00;
                    end else if(AB == 2'b11) begin
                        count <= count+1;
                        state <= S11;
                    end                     
                S11: if(AB == 2'b01) begin
                        count <= count+1;
                        state <= S01;
                    end else if(AB == 2'b10) begin
                        count <= count-1;
                        state <= S10;
                    end
            endcase
    end 
endmodule
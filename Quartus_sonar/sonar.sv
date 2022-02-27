module sonar(
	input logic clk,reset,
	output logic trigger
);

	logic [31:0] counter;
	always_ff @ (posedge clk) begin
		if (reset==1'b1) counter <= 1'b0;
		else counter <= counter+1;
	end
	
endmodule



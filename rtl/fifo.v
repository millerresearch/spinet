//	Input d is sampled at posedge clk when wr is asserted
//	Output q is unbuffered

module fifo #(
	parameter
		WIDTH=16, DEPTH=8) (
	input clk,
	input rst,
	
	input [WIDTH-1:0] d,
	output [WIDTH-1:0] q,
	
	input rd,
	input wr,
	output reg full,
	output reg empty);

	localparam LOGDEPTH = $clog2(DEPTH);
	
	reg [WIDTH-1:0] mem[0:DEPTH-1];
	reg [LOGDEPTH-1:0] ri, wi;
	wire [LOGDEPTH-1:0] riplus1, wiplus1;
	assign riplus1 = ri + 1'b1;
	assign wiplus1 = wi + 1'b1;

	assign q = mem[ri];
	
	always @(posedge clk)
	if (rst) begin
		ri <= 0;
		wi <= 0;
		empty <= 1'b1;
		full <= 0;
	end else begin
		if (rd & ~empty) begin
			ri <= riplus1;
			if (~wr) begin
				if (riplus1 == wi)
					empty <= 1'b1;
				full <= 0;
			end
		end
		if (wr & ~full) begin
			mem[wi] <= d;
			wi <= wiplus1;
			if (~rd) begin
				if (wiplus1 == ri)
					full <= 1'b1;
				empty <= 0;
			end
		end
	end
		
endmodule

module spinet6 (
	input clk,
	input rst,
	output [5:0] txready,
	output [5:0] rxready,
	input [5:0] MOSI, SCK, SS,
	output [5:0] MISO
	);
	spinet #(.N(6)) SPINET (clk, rst, txready, rxready, MOSI, SCK, SS, MISO);
endmodule

module spinet #(parameter N=8, WIDTH=16, ABITS=3) (
	input clk,
	input rst,
	output [N-1:0] txready,
	output [N-1:0] rxready,
	input [N-1:0] MOSI, SCLK, SS,
	output [N-1:0] MISO
	);

	wire [WIDTH-1:0] r[N-1:0];
	genvar i;
	generate for (i = 0; i < N; i = i+1) begin
		spinode #(.WIDTH(WIDTH), .ABITS(ABITS), .ADDRESS(i)) NODE (
			.clk(clk),
			.rst(rst),
			.fromring(r[(i+N-1)%N]),
			.toring(r[i]),
			.txready(txready[i]),
			.rxready(rxready[i]),
			.MOSI(MOSI[i]),
			.SCLK(SCLK[i]),
			.SS(SS[i]),
			.MISO(MISO[i])
		);
	end endgenerate
endmodule

module spinode #(parameter WIDTH=16, ABITS=3, ADDRESS=0) (
	input clk,
	input rst,
	input [WIDTH-1:0] fromring,
	output [WIDTH-1:0] toring,
	output txready, rxready,
	input SCLK, SS, MOSI,
	output MISO
);

	wire [WIDTH-1:0] txdata, rxdata;
	wire mosivalid, mosiack, misovalid, misoack;
	wire [WIDTH-1:0] fifo_d, fifo_q;
	wire fifo_empty, fifo_full, fifo_rd, fifo_wr;
		ringnode #(.WIDTH(WIDTH), .ABITS(ABITS), .ADDRESS(ADDRESS)) NODE (
			.clk(clk),
			.rst(rst),
			.fromring(fromring),
			.toring(toring),
			.fromclient(rxdata),
			.toclient(txdata),
			.txready(txready),
			.rxready(rxready),
			.misovalid(misovalid),
			.misoack(misoack),
			.mosivalid(mosivalid),
			.mosiack(mosiack),
			.fifo_d(fifo_d),
			.fifo_q(fifo_q),
			.fifo_empty(fifo_empty),
			.fifo_full(fifo_full),
			.fifo_rd(fifo_rd),
			.fifo_wr(fifo_wr)
		);
		ringspi #(.WIDTH(WIDTH)) SPI (
			.rst(rst),
			.txdata(txdata),
			.rxdata(rxdata),
			.misovalid(misovalid),
			.misoack(misoack),
			.mosivalid(mosivalid),
			.mosiack(mosiack),
			.MOSI(MOSI),
			.SCLK(SCLK),
			.SS(SS),
			.MISO(MISO)
		);
		fifo #(.WIDTH(WIDTH), .DEPTH(16)) FIFO (
			.clk(clk),
			.rst(rst),
			.d(fifo_d),
			.q(fifo_q),
			.empty(fifo_empty),
			.full(fifo_full),
			.rd(fifo_rd),
			.wr(fifo_wr)
		);
endmodule

module ringnode #(parameter WIDTH=16, ABITS=3, ADDRESS=0) (
	input clk,
	input rst,
	input [WIDTH-1:0] fromring,
	output [WIDTH-1:0] toring,
	input [WIDTH-1:0] fromclient,
	output [WIDTH-1:0] toclient,
	output txready, rxready,
	input mosivalid, misoack,
	output reg mosiack, misovalid,
	output [WIDTH-1:0] fifo_d,
	input [WIDTH-1:0] fifo_q,
	input fifo_empty, fifo_full,
	output reg fifo_rd,
	output fifo_wr
	);

	wire [ABITS-1:0] address = ADDRESS;
	reg [1:0] mosivalid_sync, misoack_sync;
	always @(posedge clk or posedge rst)
	if (rst) begin
		mosivalid_sync <= 0;
		misoack_sync <= 0;
	end else begin
		mosivalid_sync <= {mosivalid_sync[0],mosivalid};
		misoack_sync <= {misoack_sync[0],misoack};
	end

	localparam
		FULL = WIDTH-1, ACK = WIDTH-2,
		DST = (WIDTH-2) - ABITS, SRC = (WIDTH-2) - 2*ABITS;

	reg [WIDTH-1:0] rxbuf, txbuf, ringbuf;
	reg busy;
	reg xmit, recv, seize;	// combinational signals
	assign toring = ringbuf;
	assign toclient = rxbuf;
	assign rxready = rxbuf[FULL];
	assign txready = ~txbuf[FULL];
	assign fifo_wr = recv;
	assign fifo_d = fromring;

	reg [WIDTH-1:0] outpkt;
	always @(*) begin
		outpkt = fromring;
		xmit = 0;
		recv = 0;
		seize = 0;
		case (fromring[FULL:ACK])
		2'b00:	       // free slot - transmit if ready
			if (txbuf[FULL] & ~busy) begin
				outpkt = txbuf;
				outpkt[SRC +: ABITS] = address;
				outpkt[FULL] = 1;
				seize = 1;
			end
		2'b10, 2'b11:  // payload - return ack to sender
			if (fromring[DST +: ABITS] == address && !fifo_full) begin
				outpkt[FULL:ACK] = 2'b01;
				recv = 1;
			end
		2'b01:        // ack
			if (fromring[SRC +: ABITS] == address) begin
				outpkt[ACK] = 0;
				xmit = 1;
			end
		endcase
	end

	always @(posedge clk or posedge rst)
	if (rst) begin
		ringbuf <= 0;
		rxbuf <= 0;
		txbuf <= 0;
		busy <= 0;
		mosiack <= 0;
		misovalid <= 0;
		fifo_rd <= 0;
	end else begin
		ringbuf <= outpkt;
		fifo_rd <= 0;
		if (rxbuf[FULL] == 0 && !fifo_empty) begin
			fifo_rd <= 1;
			rxbuf <= fifo_q;
			misovalid <= ~misovalid;
		end else if (misoack_sync[1] == misovalid)
			rxbuf[FULL] <= 0;
		if (mosivalid_sync[1] != mosiack) begin
			txbuf <= fromclient;
			mosiack <= ~mosiack;
		end else if (xmit)
			txbuf[FULL] <= 0;
		if (seize)
			busy <= 1;
		else if (xmit)
			busy <= 0;
	end
endmodule

module ringspi #(parameter WIDTH=16) (
	input rst,
	input SCLK, SS, MOSI,
	output MISO,
	input misovalid,
	output reg misoack,
	output reg mosivalid,
	input mosiack,
	output [WIDTH-1:0] rxdata,
	input [WIDTH-1:0] txdata
);

	localparam LOGWIDTH = $clog2(WIDTH);
	reg [WIDTH:0] shiftreg;
	assign MISO = shiftreg[WIDTH];
	reg [LOGWIDTH-1:0] bitcount;
	reg [WIDTH-1:0] inbuf;
	assign rxdata = inbuf;

	// capture incoming data on falling clock edge
	always @(negedge SCLK or posedge SS)
		if (SS) begin
			bitcount <= 0;
			shiftreg[0] <= 0;
		end else begin
			if (bitcount != WIDTH-1) begin
				bitcount <= bitcount + 1;
				shiftreg[0] <= MOSI;
			end else begin
				bitcount <= 0;
				if (mosiack == mosivalid) begin
					inbuf <= {shiftreg[WIDTH-1:1],MOSI};
				end
			end
		end

	// set up next outgoing data on rising clock edge
	always @(posedge SCLK or posedge SS)
		if (SS) begin
			shiftreg[WIDTH:1] <= 0;
		end else begin
			if (bitcount == 0) begin
				if (misovalid != misoack) begin
					shiftreg[WIDTH:1] <= txdata;
				end else
					shiftreg[WIDTH:1] <= 0;
			end else
				shiftreg[WIDTH:1] <= shiftreg[WIDTH-1:0];
		end

	// handshake with node
	always @(negedge SCLK or posedge rst)
		if (rst) begin
			mosivalid <= 0;
		end else begin
			if (bitcount == WIDTH-1 && mosiack == mosivalid)
				mosivalid <= ~mosivalid;
		end
	always @(posedge SCLK or posedge rst)
		if (rst) begin
			misoack <= 0;
		end else begin
			if (bitcount == 0 && misovalid != misoack)
				misoack <= ~misoack;
		end

endmodule

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
	wire [WIDTH-1:0] txfifo_d, txfifo_q;
	wire txfifo_empty, txfifo_full, txfifo_rd, txfifo_wr;
	wire [WIDTH-1:0] rxfifo_d, rxfifo_q;
	wire rxfifo_empty, rxfifo_full, rxfifo_rd, rxfifo_wr;
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
			.txfifo_d(txfifo_d),
			.txfifo_q(txfifo_q),
			.txfifo_empty(txfifo_empty),
			.txfifo_full(txfifo_full),
			.txfifo_rd(txfifo_rd),
			.txfifo_wr(txfifo_wr),
			.rxfifo_d(rxfifo_d),
			.rxfifo_q(rxfifo_q),
			.rxfifo_empty(rxfifo_empty),
			.rxfifo_full(rxfifo_full),
			.rxfifo_rd(rxfifo_rd),
			.rxfifo_wr(rxfifo_wr)
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
		genvar i;
		fifo #(.WIDTH(WIDTH), .DEPTH(16)) TXFIFO (
			.clk(clk),
			.rst(rst),
			.d(txfifo_d),
			.q(txfifo_q),
			.empty(txfifo_empty),
			.full(txfifo_full),
			.rd(txfifo_rd),
			.wr(txfifo_wr)
		);
		fifo #(.WIDTH(WIDTH), .DEPTH(16)) RXFIFO (
			.clk(clk),
			.rst(rst),
			.d(rxfifo_d),
			.q(rxfifo_q),
			.empty(rxfifo_empty),
			.full(rxfifo_full),
			.rd(rxfifo_rd),
			.wr(rxfifo_wr)
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
	output [WIDTH-1:0] txfifo_d, rxfifo_d,
	input [WIDTH-1:0] txfifo_q, rxfifo_q,
	input txfifo_empty, rxfifo_empty,
	input txfifo_full, rxfifo_full,
	output reg txfifo_rd, rxfifo_rd,
	output txfifo_wr, rxfifo_wr
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
	reg txwrite;
	reg xmit, recv, seize;	// combinational signals
	assign toring = ringbuf;
	assign toclient = rxbuf;
	assign rxready = rxbuf[FULL];
	assign txready = ~txfifo_full;
	assign rxfifo_wr = recv;
	assign txfifo_wr = txwrite;
	assign rxfifo_d = fromring;
	assign txfifo_d = txbuf;

	reg [WIDTH-1:0] outpkt;
	always @(*) begin
		outpkt = fromring;
		xmit = 0;
		recv = 0;
		seize = 0;
		case (fromring[FULL:ACK])
		2'b00:	       // free slot - transmit if ready
			if (~txfifo_empty & ~busy) begin
				outpkt = txfifo_q;
				outpkt[SRC +: ABITS] = address;
				outpkt[FULL] = 1;
				seize = 1;
			end
		2'b10, 2'b11:  // payload - return ack to sender
			if (fromring[DST +: ABITS] == address && !rxfifo_full) begin
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
		rxfifo_rd <= 0;
		txfifo_rd <= 0;
		txwrite <= 0;
	end else begin
		ringbuf <= outpkt;
		rxfifo_rd <= 0;	// default
		txfifo_rd <= 0; // default
		txwrite <= 0;	// default
		// packets from net to client
		if (rxbuf[FULL] == 0 && !rxfifo_empty) begin
			rxfifo_rd <= 1;
			rxbuf <= rxfifo_q;
			misovalid <= ~misovalid;
		end else if (misoack_sync[1] == misovalid)
			rxbuf[FULL] <= 0;
		// packets from client to net
		if (mosivalid_sync[1] != mosiack) begin
			txbuf <= fromclient;
			mosiack <= ~mosiack;
		end else if (txbuf[FULL] & ~txfifo_full) begin
			txwrite <= 1;
			txbuf[FULL] <= 0;
		end
		// limit one packet in transit per sender
		if (seize) begin
			txfifo_rd <= 1;
			busy <= 1;
		end else if (xmit)
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

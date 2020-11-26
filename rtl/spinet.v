module spinet6 (
	input clk,
	input rst,
	input [37:0] io_in,
	output [37:0] io_out
	);
	spinet #(.N(6)) SPINET (
		.clk(clk),
		.rst(rst),
		.MOSI   ({io_in[0],io_in[ 8],io_in[14],io_in[20],io_in[26],io_in[32]}),
		.SCLK   ({io_in[1],io_in[ 9],io_in[15],io_in[21],io_in[27],io_in[33]}),
		.SS     ({io_in[2],io_in[10],io_in[16],io_in[22],io_in[28],io_in[34]}),
		.MISO   ({io_out[3],io_out[11],io_out[17],io_out[23],io_out[29],io_out[35]}),
		.txready({io_out[4],io_out[12],io_out[18],io_out[24],io_out[30],io_out[36]}),
		.rxready({io_out[7],io_out[13],io_out[19],io_out[25],io_out[31],io_out[37]}) 
	);
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
			.mosiack(mosiack)
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
	output reg mosiack, misovalid
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
	assign toring = ringbuf;
	assign toclient = rxbuf;
	assign rxready = rxbuf[FULL];
	assign txready = ~txbuf[FULL];
	reg xmit, recv, seize;

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
			if (fromring[DST +: ABITS] == address && !rxbuf[FULL]) begin
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
	end else begin
		ringbuf <= outpkt;
		if (recv) begin
			rxbuf <= fromring;
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

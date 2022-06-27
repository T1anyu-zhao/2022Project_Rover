module sobel (
	input       		 clk, rst, in_valid,
   input [7:0]  		 red_in,green_in,blue_in,
   input [23:0]		 sobelthreshold,
   output  				 data_out
);

wire [23:0] grey;
wire [7:0] grey_in;
assign grey = (red_in*19595 + green_in*38469 + blue_in*7472) >> 16;
assign grey_in = grey[7:0];

 kernal_gen erosion(
		.clk(clk),
		.rst(rst),
		.in_valid(in_valid),
		.in_ready(), 
		.data_in(grey_in),
		.kernal_11(row_11),
		.kernal_12(row_12),
		.kernal_13(row_13),
		.kernal_21(row_21),
		.kernal_22(row_22),
		.kernal_23(row_23),
		.kernal_31(row_31),
		.kernal_32(row_32),
		.kernal_33(row_33),
		.out_valid(),
		.out_ready()
);

wire  [7:0]  row_11,row_12,row_13;
wire	[7:0]  row_21,row_22,row_23;
wire  [7:0]  row_31,row_32,row_33;

wire [9:0] Gx_neg,Gx_pos;
wire [9:0] Gx;
assign Gx_neg = row_11 + (row_21<<1) + row_31;
assign Gx_pos = row_13 + (row_23<<1) + row_33;
assign Gx = (Gx_neg >= Gx_pos) ? (Gx_neg - Gx_pos) : (Gx_pos -Gx_neg);

wire [9:0] Gy_pos,Gy_neg;
wire [9:0] Gy;

assign Gy_pos = row_11 + (row_12<<1) + row_13;
assign Gy_neg = row_31 + (row_32<<1) + row_33;
assign Gy = (Gy_neg >= Gy_pos) ? (Gy_neg -Gy_pos) : (Gy_pos - Gy_neg);



wire [23:0] G_square;
assign G_square = (Gx*Gx)+(Gy*Gy);

assign data_out = (G_square <= sobelthreshold) ? 1'b0 :1'b1;

endmodule

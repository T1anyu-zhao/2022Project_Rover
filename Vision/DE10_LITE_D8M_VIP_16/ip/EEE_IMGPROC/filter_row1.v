//use the filter to deal with multi pixels to get rid of the noisees.

module filter_gaussian (
	input       		 clk, rst, in_valid,
   input [7:0]  		 data_in,
   output[7:0]  		 data_out
);

//input pixel and generating kernal for filter;

kernal_gen gaussian(
		.clk(clk),
		.rst(rst),
		.in_valid(in_valid),
		.in_ready(), 
		.data_in(data_in),
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
wire  [15:0] tmp;
wire  [7:0]  mean;

//output the pixels;
assign tmp = (row_11)
            +(row_12<<1)
            +(row_13)
				+(row_21<<1)
				+(row_22<<2)
				+(row_23<<1)
				+(row_31)
				+(row_32<<1)
				+(row_33);
				
assign mean	= tmp>>4;
assign data_out = (~rst) ? 0 : mean;

endmodule


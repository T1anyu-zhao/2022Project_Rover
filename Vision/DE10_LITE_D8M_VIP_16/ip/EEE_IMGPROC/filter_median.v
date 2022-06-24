module filter_median (
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

//internal wiring
wire [7:0]  row_11,row_12,row_13;
wire [7:0]  row_21,row_22,row_23;
wire [7:0]  row_31,row_32,row_33;
wire [7:0]  max1,max2,max3;
wire [7:0]  mid1,mid2,mid3;
wire [7:0]  min1,min2,min3;
//find median of these 5 pixels
// find max of each row
assign max1 = (row_11>=row_12 && row_11>=row_13) ? row_11:
				  (row_12>=row_11 && row_12>=row_13) ? row_12: row_13;
assign max2 = (row_21>=row_22 && row_21>=row_23) ? row_21:
				  (row_22>=row_21 && row_22>=row_23) ? row_22: row_23;
assign max3 = (row_31>=row_32 && row_31>=row_33) ? row_31:
				  (row_32>=row_31 && row_32>=row_33) ? row_32: row_33;				  
//find min of each row
assign min1 = (row_11<=row_12 && row_11<=row_13) ? row_11:
				  (row_12<=row_11 && row_12<=row_13) ? row_12: row_13;
assign min2 = (row_21<=row_22 && row_21<=row_23) ? row_21:
				  (row_22<=row_21 && row_22<=row_23) ? row_22: row_23;
assign min3 = (row_31<=row_32 && row_31<=row_33) ? row_31:
				  (row_32<=row_31 && row_32<=row_33) ? row_32: row_33;
//find mid of each row
assign mid1 = (min1<=row_11 && row_11<=max1) ? row_11:
				  (min1<=row_12 && row_12<=max1) ? row_12: row_13;
assign mid2 = (min1<=row_21 && row_21<=max1) ? row_21:
				  (min1<=row_22 && row_22<=max1) ? row_22: row_23;
assign mid3 = (min1<=row_31 && row_31<=max1) ? row_31:
				  (min1<=row_32 && row_32<=max1) ? row_32: row_33;				  

// find max mid and min of MAX,MID,MIN
wire [7:0] max_min, mid_mid, min_max;
assign max_min = (max1<=max2 && max1<=max3) ? max1 :
					  (max2<=max1 && max2<=max3) ? max2 : max3;
assign min_max = (min1>=min2 && min1>=min3) ? min1 :
					  (min2>=min1 && min2>=min3) ? min2 : min3;
assign mid_mid = ((mid2<=mid1 && mid1<=mid3)||(mid3<=mid1 && mid1<=mid2)) ? mid1:
					  ((mid1<=mid2 && mid2<=mid3)||(mid3<=mid2 && mid2<=mid1)) ? mid2: mid3;
//then the mid of precious three;
wire [7:0] median;
assign median = ((max_min<=mid_mid&&mid_mid<=min_max)||(min_max<=mid_mid&&mid_mid<=max_min)) ? mid_mid:
					 ((mid_mid<=max_min&&max_min<=min_max)||(min_max<=max_min&&max_min<=mid_mid)) ? max_min: min_max;
					  
assign data_out = (~rst) ? 0 : median;

endmodule


				  

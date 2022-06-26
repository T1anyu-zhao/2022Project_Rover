module filter_erosion (
	input       		 clk, rst, in_valid,
   input [7:0]  		 data_in,
   output[7:0]  		 data_out
);


 kernal_gen erosion(
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

wire  [3:0]  count1,count2,count3,count4,count5,count6;
wire  [7:0]  row_11,row_12,row_13;
wire	[7:0]  row_21,row_22,row_23;
wire  [7:0]  row_31,row_32,row_33;
wire			 tmp1,tmp2,tmp3,tmp4,tmp5,tmp6;

assign count1 = row_11[5]+row_12[5]+row_13[5]+row_21[5]+row_22[5]+row_23[5]+row_31[5]+row_32[5]+row_33[5];
assign count2 = row_11[4]+row_12[4]+row_13[4]+row_21[4]+row_22[4]+row_23[4]+row_31[4]+row_32[4]+row_33[4];
assign count3 = row_11[3]+row_12[3]+row_13[3]+row_21[3]+row_22[3]+row_23[3]+row_31[3]+row_32[3]+row_33[3];
assign count4 = row_11[2]+row_12[2]+row_13[2]+row_21[2]+row_22[2]+row_23[2]+row_31[2]+row_32[2]+row_33[2];
assign count5 = row_11[1]+row_12[1]+row_13[1]+row_21[1]+row_22[1]+row_23[1]+row_31[1]+row_32[1]+row_33[1];
assign count6 = row_11[0]+row_12[0]+row_13[0]+row_21[0]+row_22[0]+row_23[0]+row_31[0]+row_32[0]+row_33[0];
assign tmp1 = (count1 >= 8'd6) ? {1'b1} :{1'b0};
assign tmp2 = (count2 >= 8'd6) ? {1'b1} :{1'b0};
assign tmp3 = (count3 >= 8'd6) ? {1'b1} :{1'b0};
assign tmp4 = (count4 >= 8'd6) ? {1'b1} :{1'b0};
assign tmp5 = (count5 >= 8'd8) ? {1'b1} :{1'b0};
assign tmp6 = (count6 >= 8'd6) ? {1'b1} :{1'b0};

assign data_out = {2'b0,tmp1,tmp2,tmp3,tmp4,tmp5,tmp6};

endmodule 
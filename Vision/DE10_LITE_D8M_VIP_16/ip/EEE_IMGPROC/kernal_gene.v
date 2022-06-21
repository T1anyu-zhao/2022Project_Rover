module kernal_gen (
input					clk,rst,in_valid,in_ready, 
input[7:0] 			data_in,
output reg[7:0] 	kernal_11,kernal_12,kernal_13,
output reg[7:0]	kernal_21,kernal_22,kernal_23,
output reg[7:0] 	kernal_31,kernal_32,kernal_33,
output 				out_valid,out_ready
);

//internal wiring
reg[10:0] counter;
wire [7:0] row1_data,row2_data,row3_data;
parameter ROW_WIDTH = 640;

assign row3_data = data_in;
assign out_ready = in_ready;
assign out_valid = out_valid;

always @(*) begin
	kernal_13 <= row1_data;
	kernal_23 <= row2_data;
	kernal_33 <= row3_data;
end

always @(posedge clk) begin 
	if (~rst | counter == ROW_WIDTH -1) begin
		counter <= 0;
		end
	else if (in_valid) begin
		counter <= counter +1;
	end
end 

always @(posedge clk) begin
	if(~rst | counter == ROW_WIDTH -1) begin
		kernal_11 <= 0;
		kernal_12 <= 0;
		kernal_21 <= 0;
		kernal_22 <= 0;
		kernal_31 <= 0;
		kernal_32 <= 0;
		end
	else if(in_valid) begin
		kernal_11 <= kernal_12;
		kernal_12 <= kernal_13;
		kernal_21 <= kernal_22;
		kernal_22 <= kernal_23;
		kernal_31 <= kernal_32;
		kernal_32 <= kernal_33;
		end
end


shiftram kernal_reg (
	.clken(in_valid),
	.clock(clk),
	.shiftin(row3_data),
	.shiftout(),
	.taps0x(row2_data),
	.taps1x(row1_data)
);

endmodule





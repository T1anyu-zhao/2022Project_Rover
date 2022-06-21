module  rgb2hsv(
	input                         clk,rst,	
	input  [7:0]                  red,green,blue,	
	output [8:0] 			      hue,sat,val//  hue 0 - 360	
);

wire [7:0]  max,min,diff;
wire [32:0] hue_f, sat_f, diff_f;
//find max and min

assign max = (red >= green)&&(red >= blue) ? red :
             (green >= red)&&(green >= blue) ? green :blue;
assign min = (red <= green)&&(red <= blue) ? red :
             (green <= red)&&(green <= blue) ? green :blue;    

//v
assign val = max;

//diff

assign diff = max - min;
assign diff_f = {diff,24'b0};

//s
assign sat_f = (max == 0) ? 0 : (diff_f/max)*8'hff; 
assign sat = sat_f[31:24];

//h
parameter degree = {8'b111100,24'b0};//degree of 60 in fixed point
assign hue_f =  (max == red)   ? (green - blue)*(degree/diff):
                (max == green) ? ((blue - red)*(degree/diff)+2*degree):
                                 ((red - green)*(degree/diff)+4*degree);

assign hue = hue_f[32:24];

endmodule
module dis_measure (
input [10:0] x_max,x_min,
output[31:0] distance
);

parameter focal = 650;
//decleration
//distance / actual width = pixel width / focal distance;
assign distance = (x_max-x_min)*40/focal;


endmodule
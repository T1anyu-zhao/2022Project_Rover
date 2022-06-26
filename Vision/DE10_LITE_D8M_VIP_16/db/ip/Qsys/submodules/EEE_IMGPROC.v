    module EEE_IMGPROC(
	// global clock & reset
	clk,
	reset_n,
	
	// mm slave
	s_chipselect,
	s_read,
	s_write,
	s_readdata,
	s_writedata,
	s_address,

	// stream sink
	sink_data,
	sink_valid,
	sink_ready,
	sink_sop,
	sink_eop,
	
	// streaming source
	source_data,
	source_valid,
	source_ready,
	source_sop,
	source_eop,
	
	// conduit
	mode
	
);


// global clock & reset
input	clk;
input	reset_n;

// mm slave
input							s_chipselect;
input							s_read;
input							s_write;
output	reg	[31:0]	s_readdata;
input	[31:0]				s_writedata;
input	[2:0]					s_address;


// streaming sink
input	[23:0]            	sink_data;
input								sink_valid;
output							sink_ready;
input								sink_sop;
input								sink_eop;

// streaming source
output	[23:0]			  	   source_data;
output								source_valid;
input									source_ready;
output								source_sop;
output								source_eop;

// conduit export
input                         mode;

////////////////////////////////////////////////////////////////////////
//
parameter IMAGE_W = 11'd640;
parameter IMAGE_H = 11'd480;
parameter MESSAGE_BUF_MAX = 256;
parameter MSG_INTERVAL = 6;
parameter BB_COL_DEFAULT = 24'h00ff00;


wire [7:0]   red, green, blue, grey;
wire [7:0]   red_out, green_out, blue_out;

wire         sop, eop, in_valid, out_ready;
////////////////////////////////////////////////////////////////////////

integer i;
//Filter light declearation
//wire[7:0]	red_filt, green_filt, blue_filt;
//Aplly 1-dimension 

//hsv decleration
wire [8:0]hue;//colour
wire [8:0]sat;//saturation
wire [8:0]val;//bright

rgb2hsv h1 (
	.clk(clk),
	.rst(reset_n),
	.red(red),
	.green(green),
	.blue(blue),
	.hue(hue),
	.sat(sat),
	.val(val)
);
// Detect red areas
// Find boundary of cursor box
// Highlight detected areas


//setting the colour;
parameter COLOUR_N = 6;
wire [23:0] colour_high,colour_high_without,block_top_high;
wire [23:0] l_green_set, d_green_set, red_set, pink_set, yellow_set, d_blue_set,edge_colour;
wire l_green_det, d_green_det, red_det, pink_det, yellow_det, d_blue_det;
wire l_green_high, d_green_high, red_high, pink_high, yellow_high, d_blue_high;

//reg fir_detect [COLOUR_N:0];
//reg sec_detect [COLOUR_N:0];

//set the desired colour here;
assign l_green_set ={8'h28,8'hFF,8'h28};//light green
assign d_green_set ={8'h00,8'h60,8'h30};//dark green
assign red_set    ={8'hff,8'h00,8'h00};//red
assign pink_set	  ={8'hfc,8'h6d,8'ha3};//pink
assign yellow_set ={8'hf9,8'hf9,8'h00};//yellow
assign d_blue_set  ={8'h00,8'h4b,8'h97};//dark blue
assign edge_colour = 24'hffffff;//white
//set the detection range same order as setting;
assign l_green_det = ( (100<=hue)&&(hue<=130) && (86<=sat) &&(sat<=200) && (158<=val) && (val<=255));
assign d_green_det = ( (100<=hue)&&(hue<=160) && (50<=sat) &&(sat<=256) && (50<=val)  && (val<=150));//val120- 100 163
assign red_det 	 = ( (11<=hue)  &&(hue<=22)  && (140<=sat) &&(sat<=240) && (100<=val) && (val<=200));//sat200+val195-
assign pink_det 	 = ( (0<=hue)&&(hue<=12) && (100<=sat)&&(sat<=184) && (200<=val) && (val<=255));
assign yellow_det  = ( (45<=hue) &&(hue<=70)  && (110<=sat)&&(sat<=255) && (150<=val) && (val<=255));//sat180+val220+
assign d_blue_det  = ( (160<=hue)&&(hue<=270) && (46<=sat) &&(sat<=256) && (46<=val)  && (val<=255));//170-236 val120_

/*
( (104<=hue)&&(hue<=168) && (88<=sat)&&(sat<=255) && (0<=val) && (val<=140));
assign colour_detect[2] = ( (0<=hue)&&(hue<=32) && (195<=sat)&&(sat<=235) && (50<=val) && (val<=255));
assign colour_detect[3] = 0//( (0<=hue)&&(hue<=32) && (77<=sat)&&(sat<=195) && (164<=val) && (val<=255));
assign colour_detect[4] = ( (40<=hue)&&(hue<83) && (140<=sat)&&(sat<=255) && (159<=val) && (val<=255));
assign colour_detect[5] = ( (168<=hue)&&(hue<=238) && (69<=sat)&&(sat<=227) && (0<=val) && (val<=130));
*/

//erosion filter

wire [7:0] six_detect,detect_filt;
assign six_detect = {2'b0,l_green_det,d_green_det,red_det,pink_det,yellow_det,d_blue_det};

filter_erosion six_colour_detection(
	.clk(clk),
	.rst(reset_n),
	.in_valid(in_valid),
	.data_in(six_detect),
	.data_out(detect_filt)
);

assign l_green_high = detect_filt[5];
assign d_green_high = detect_filt[4];
assign red_high     = detect_filt[3];
assign pink_high    = detect_filt[2];
assign yellow_high  = detect_filt[1];
assign d_blue_high  = detect_filt[0];

//building detection//
wire building_edge;
wire [23:0] threshold_setting;
assign threshold_setting = 24'h0000ff;
sobel build_detect(
	.clk(clk), 
	.rst(reset_n), 
	.in_valid(in_valid),
	.red_in(red),
	.green_in(green),
	.blue_in(blue),
	.sobelthreshold(threshold_setting),
	.data_out(building_edge)
);

assign grey = green[7:1] + red[7:2] + blue[7:2]; //Grey = green/2 + red/4 + blue/4
//assign colour_high =  (fir_detect[0] && sec_detect[0]) ? colour_set[0] :
//					  (fir_detect[1] && sec_detect[1]) ? colour_set[1] :
//					  (fir_detect[2] && sec_detect[2]) ? colour_set[2] :
//					  (fir_detect[3] && sec_detect[3]) ? colour_set[3] :
//					  (fir_detect[4] && sec_detect[4]) ? colour_set[4] :
//					  (fir_detect[5] && sec_detect[5]) ? colour_set[5] : {grey, grey, grey};
assign colour_high = l_green_high ? l_green_set:
					 d_green_high ? d_green_set:
					 red_high	  ? red_set    :
					 pink_high    ? pink_set   :
					 yellow_high  ? yellow_set :
					 d_blue_high  ? d_blue_set : 
					 building_edge ? edge_colour : {grey,grey,grey};

assign colour_high_without = l_green_det ? l_green_set:
					 d_green_det ? d_green_set:
					 red_det	  ? red_set    :
					 pink_det    ? pink_set   :
					 yellow_det  ? yellow_set :
					 d_blue_det  ? d_blue_set : {grey,grey,grey};
					 
assign block_top_high = ( y > 200 ) ? colour_high : {grey,grey,grey};


// Show bounding box
//genvar j;
wire [23:0] new_image,new_image_without,image_out;
wire bb_active_lg,bb_active_dg,bb_active_r,bb_active_p,bb_active_y,bb_active_db;

assign bb_active_lg= (x == left_lg) || (x == right_lg); // | (y == top[j]) | (y == bottom[j]);
assign bb_active_dg= (x == left_dg) || (x == right_dg);
assign bb_active_r = (x == left_r ) || (x == right_r );
assign bb_active_p = (x == left_p ) || (x == right_p );
assign bb_active_y = (x == left_y ) || (x == right_y );
assign bb_active_db= (x == left_db) || (x == right_db);

assign new_image = bb_active_lg ? l_green_set
                 : bb_active_dg ? d_green_set
                 : bb_active_r  ? red_set
                 : bb_active_p  ? pink_set
                 : bb_active_y  ? yellow_set
                 : bb_active_db ? d_blue_set : block_top_high;
/*					  
assign new_image_without = bb_active_lg ? l_green_set
                 : bb_active_dg ? d_green_set
                 : bb_active_r  ? red_set
                 : bb_active_p  ? pink_set
                 : bb_active_y  ? yellow_set
                 : bb_active_db ? d_blue_set : colour_high_without;
*/

//assign bb_active = (x == left) | (x == right);//| (y == top) | (y == bottom); 
//assign new_image = bb_active ? bb_col : colour_high;

// Switch output pixels depending on mode switch
// Don't modify the start-of-packet word - it's a packet discriptor
// Don't modify data in non-video packets
assign {red_out, green_out, blue_out} = (mode & ~sop & packet_video) ? new_image : {red,green,blue};
//assign image_out = mode ? new_image : new_image_without;
//assign {red_out, green_out, blue_out} = (~sop & packet_video) ? image_out : {red, green, blue};

//Count valid pixels to tget the image coordinates. Reset and detect packet type on Start of Packet.
reg [10:0] x, y;
reg packet_video;
always@(posedge clk) begin
	if (sop) begin
		x <= 11'h0;
		y <= 11'h0;
		packet_video <= (blue[3:0] == 3'h0);
	end
	else if (in_valid) begin
		if (x == IMAGE_W-1) begin
			x <= 11'h0;
			y <= y + 11'h1;
		end
		else begin
			x <= x + 11'h1;
		end
	end
end

//Find first and last red pixels
reg [10:0] x_max_lg,x_max_dg,x_max_r,x_max_p,x_max_y,x_max_db;
reg [10:0] y_max;
reg [10:0] x_min_lg,x_min_dg,x_min_r,x_min_p,x_min_y,x_min_db;
reg [10:0] y_min;

always@(posedge clk) begin     //Update bounds
	if (l_green_high & in_valid) begin	
			if (x < x_min_lg) x_min_lg <= x;
			if (x > x_max_lg) x_max_lg <= x;
			//if (y < y_min[i]) y_min[i] <= y;
			//y_max[i] <= y;
	end
	if (d_green_high & in_valid) begin	
			if (x < x_min_dg) x_min_dg <= x;
			if (x > x_max_dg) x_max_dg <= x;
	end
	if (red_high & in_valid) begin	
			if (x < x_min_r) x_min_r <= x;
			if (x > x_max_r) x_max_r <= x;
	end
	if (pink_high & in_valid) begin	
			if (x < x_min_p) x_min_p <= x;
			if (x > x_max_p) x_max_p <= x;
	end
	if (yellow_high & in_valid) begin	
			if (x < x_min_y) x_min_y <= x;
			if (x > x_max_y) x_max_y <= x;
	end
	if (d_blue_high & in_valid) begin	
			if (x < x_min_db) x_min_db <= x;
			if (x > x_max_db) x_max_db <= x;
	end

	if (sop & in_valid) begin	//Reset bounds on start of packet
			x_min_lg <= IMAGE_W-11'h1;
			x_min_dg <= IMAGE_W-11'h1;
			x_min_r  <= IMAGE_W-11'h1;
			x_min_p  <= IMAGE_W-11'h1;
			x_min_y  <= IMAGE_W-11'h1;
			x_min_db <= IMAGE_W-11'h1;
			x_max_lg <= 0;
			x_max_dg <= 0;
			x_max_r  <= 0;
			x_max_p  <= 0;
			x_max_y  <= 0;
			x_max_db <= 0;
			//y_min[i] <= IMAGE_H-11'h1;
			//y_max[i] <= 0;
	end
end

//Process bounding box at the end of the frame.
reg [1:0]   msg_state;
reg [10:0]  left_lg,left_dg,left_r,left_p,left_y,left_db;
reg [10:0]  right_lg,right_dg,right_r,right_p,right_y,right_db;
reg [10:0]  top 	[COLOUR_N:0];
reg [10:0]  bottom	[COLOUR_N:0];
reg [7:0]   frame_count;
always@(posedge clk) begin

	if (eop & in_valid & packet_video) begin  //Ignore non-video packets
		
		//Latch edges for display overlay on next frame
		left_lg <= x_min_lg;
		left_dg <= x_min_dg;
		left_r  <= x_min_r;
		left_p  <= x_min_p;
		left_y  <= x_min_y;
		left_db <= x_min_db;
		right_lg <= x_max_lg;
		right_dg <= x_max_dg;
		right_r  <= x_max_r;
		right_p  <= x_max_p;
		right_y  <= x_max_y;
		right_db <= x_max_db;
		//Start message writer FSM once every MSG_INTERVAL frames, if there is room in the FIFO
		frame_count <= frame_count - 1;

		if (frame_count == 0 && msg_buf_size < MESSAGE_BUF_MAX - 3) begin
			msg_state <= 2'b01;
			frame_count <= MSG_INTERVAL-1;
		end
	end

	//Cycle through message writer states once started
	if (msg_state != 2'b00) msg_state <= msg_state + 2'b01;

end
	
//Generate output messages for CPU
reg [31:0] msg_buf_in; 
wire [31:0] msg_buf_out;
reg msg_buf_wr;
wire msg_buf_rd, msg_buf_flush;
wire [7:0] msg_buf_size;
wire msg_buf_empty;

`define RED_BOX_MSG_ID "RBB"

always@(*) begin	//Write words to FIFO as state machine advances
	case(msg_state)
		2'b00: begin
			msg_buf_in = 32'b0;
			msg_buf_wr = 1'b0;
		end
		2'b01: begin
			msg_buf_in = `RED_BOX_MSG_ID;	//Message ID
			msg_buf_wr = 1'b1;
		end
		2'b10: begin
			msg_buf_in = {5'b0, x_min_lg, 5'b0, x_max_lg};	//Top left coordinate
			msg_buf_wr = 1'b1;//01a000/0000 0001 
		end
		2'b11: begin
			msg_buf_in = {5'b0, x_min_dg, 5'b0, x_max_dg}; //Bottom right coordinate
			msg_buf_wr = 1'b1;
		end
	endcase
end


//Output message FIFO
MSG_FIFO	MSG_FIFO_inst (
	.clock (clk),
	.data (msg_buf_in),
	.rdreq (msg_buf_rd),
	.sclr (~reset_n | msg_buf_flush),
	.wrreq (msg_buf_wr),
	.q (msg_buf_out),
	.usedw (msg_buf_size),
	.empty (msg_buf_empty)
	);


//Streaming registers to buffer video signal
STREAM_REG #(.DATA_WIDTH(26)) in_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(sink_ready),
	.valid_out(in_valid),
	.data_out({red,green,blue,sop,eop}),
	.ready_in(out_ready),
	.valid_in(sink_valid),
	.data_in({sink_data,sink_sop,sink_eop})
);

STREAM_REG #(.DATA_WIDTH(26)) out_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(out_ready),
	.valid_out(source_valid),
	.data_out({source_data,source_sop,source_eop}),
	.ready_in(source_ready),
	.valid_in(in_valid),
	.data_in({red_out, green_out, blue_out, sop, eop})
);


/////////////////////////////////
/// Memory-mapped port		 /////
/////////////////////////////////

// Addresses
`define REG_STATUS    			0
`define READ_MSG    				1
`define READ_ID    				2
`define REG_BBCOL					3

//Status register bits
// 31:16 - unimplemented
// 15:8 - number of words in message buffer (read only)
// 7:5 - unused
// 4 - flush message buffer (write only - read as 0)
// 3:0 - unused


// Process write

reg  [7:0]   reg_status;
reg	[23:0]	bb_col;

always @ (posedge clk)
begin
	if (~reset_n)
	begin
		reg_status <= 8'b0;
		bb_col <= BB_COL_DEFAULT;
	end
	else begin
		if(s_chipselect & s_write) begin
		   if      (s_address == `REG_STATUS)	reg_status <= s_writedata[7:0];
		   if      (s_address == `REG_BBCOL)	bb_col <= s_writedata[23:0];
		end
	end
end


//Flush the message buffer if 1 is written to status register bit 4
assign msg_buf_flush = (s_chipselect & s_write & (s_address == `REG_STATUS) & s_writedata[4]);


// Process reads
reg read_d; //Store the read signal for correct updating of the message buffer

// Copy the requested word to the output port when there is a read.
always @ (posedge clk)
begin
   if (~reset_n) begin
	   s_readdata <= {32'b0};
		read_d <= 1'b0;
	end
	
	else if (s_chipselect & s_read) begin
		if   (s_address == `REG_STATUS) s_readdata <= {16'b0,msg_buf_size,reg_status};
		if   (s_address == `READ_MSG) s_readdata <= {msg_buf_out};
		if   (s_address == `READ_ID) s_readdata <= 32'h1234EEE2;
		if   (s_address == `REG_BBCOL) s_readdata <= {8'h0, bb_col};
	end
	
	read_d <= s_read;
end

//Fetch next word from message buffer after read from READ_MSG
assign msg_buf_rd = s_chipselect & s_read & ~read_d & ~msg_buf_empty & (s_address == `READ_MSG);
						


endmodule

/*
filter_median r (
	.clk(clk),
	.rst(reset_n),
	.in_valid(in_valid),
	.data_in(red),
	.data_out(red_filt),
);

filter_median g (
	.clk(clk),
	.rst(reset_n),
	.in_valid(in_valid),
	.data_in(green),
	.data_out(green_filt),
);

filter_median b (
	.clk(clk),
	.rst(reset_n),
	.in_valid(in_valid),
	.data_in(blue),
	.data_out(blue_filt),
);
*/


/*
filter_erosion d_green(
	.clk(clk),
	.rst(reset_n),
	.in_valid(in_valid),
	.data_in(d_green_det),
	.data_out(colour_detect[1])
);
/*
filter_erosion red1(
	.clk(clk),
	.rst(reset_n),
	.in_valid(in_valid),
	.data_in(red_det),
	.data_out(colour_detect[2])
);

filter_erosion pink(
	.clk(clk),
	.rst(reset_n),
	.in_valid(in_valid),
	.data_in(pink_det),
	.data_out(colour_detect[3])
);

filter_erosion yellow(
	.clk(clk),
	.rst(reset_n),
	.in_valid(in_valid),
	.data_in(yellow_det),
	.data_out(colour_detect[4])
);

/*filter_erosion d_blue(
	.clk(clk),
	.rst(reset_n),
	.in_valid(in_valid),
	.data_in(d_blue_det),
	.data_out(colour_detect[5])
);
*/

/*
initial begin
	for(i=0; i<COLOUR_N; i=i+1)
		fir_detect[i] <= 0;
		sec_detect[i] <= 0;
end

always @(posedge clk) begin
	fir_detect [0] <= sec_detect [0];
	sec_detect [0] <= colour_detect [0];
	fir_detect [1] <= sec_detect [1];
	sec_detect [1] <= colour_detect [1];
	fir_detect [2] <= sec_detect [2];
	sec_detect [2] <= colour_detect [2];
	fir_detect [3] <= sec_detect [3];
	sec_detect [3] <= colour_detect [3];
	fir_detect [4] <= sec_detect [4];
	sec_detect [4] <= colour_detect [4];
	fir_detect [5] <= sec_detect [5];
	sec_detect [5] <= colour_detect [5];
end
*/


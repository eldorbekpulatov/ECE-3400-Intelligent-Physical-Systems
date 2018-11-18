 `define SCREEN_WIDTH 176
`define SCREEN_HEIGHT 144

///////* DON'T CHANGE THIS PART *///////
module DE0_NANO(
	CLOCK_50,
	GPIO_0_D,
	GPIO_1_D,
	LED,
	KEY
);

//=======================================================
//  PARAMETER declarations
//=======================================================
localparam RED = 8'b111_000_00;
localparam GREEN = 8'b000_111_00;
localparam BLUE = 8'b000_000_11;
localparam WHITE = 8'b111_111_11;
localparam BLACK = 8'b000_000_00;

//=======================================================
//  PORT declarations
//=======================================================

//////////// CLOCK - DON'T NEED TO CHANGE THIS //////////
input 		          		CLOCK_50;

//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
output 		    [33:0]		GPIO_0_D;
//////////// GPIO_0, GPIO_1 connect to GPIO Default //////////
input 		    [33:20]		GPIO_1_D;
input 		     [1:0]		KEY;

//////////////////LEDS ON BOARD/////////////////////
output [7:0] LED;

///// PIXEL DATA /////
wire [7:0]	pixel_data_RGB332;

//DATA FROM CAMERA
wire [7:0] camera_data;
wire pclk;
wire href;
wire vsync;

assign camera_data = GPIO_1_D[27:20];
assign pclk = GPIO_1_D[33];
assign href = GPIO_1_D[31];
assign vsync = GPIO_1_D[30];


//assign camera_data = GPIO_1_D[27:20];
//assign pclk = GPIO_1_D[28];
//assign href = GPIO_1_D[29];
//assign vsync = GPIO_1_D[30];



///// READ/WRITE ADDRESS /////
reg [14:0] X_ADDR = 15'b1000;
reg [14:0] Y_ADDR = 15'b1000;
wire [14:0] WRITE_ADDRESS;
reg [14:0] READ_ADDRESS; 

//assign WRITE_ADDRESS = X_ADDR + Y_ADDR*(`SCREEN_WIDTH);

///// VGA INPUTS/OUTPUTS /////
wire 			VGA_RESET;
wire [7:0]	VGA_COLOR_IN;
wire [9:0]	VGA_PIXEL_X;
wire [9:0]	VGA_PIXEL_Y;
wire [7:0]	MEM_OUTPUT;
wire			VGA_VSYNC_NEG;
wire			VGA_HSYNC_NEG;
reg			VGA_READ_MEM_EN;

assign GPIO_0_D[5] = VGA_VSYNC_NEG;
assign VGA_RESET = ~KEY[0];

///// I/O for Img Proc /////
wire [8:0] RESULT;

/* WRITE ENABLE */
wire W_EN;

////* LOCAL WIRES FOR PLL *////
wire clk50MHz;
wire clk25MHz;
wire clk24MHz;

assign GPIO_0_D[33] = clk24MHz; // Use something other than 33?

//assign GPIO_0_D[26] = clk24MHz; // Use something other than 33?

assign GPIO_0_D[0] = RESULT[0];
assign GPIO_0_D[1] = RESULT[1];
///////* INSTANTIATE PLL HERE *///////
PLLClks pll(
	.inclk0(CLOCK_50),
	.c0(clk24MHz),
	.c1(clk25MHz),
	.c2(clk50MHz)
);

///////* M9K Module *///////
Dual_Port_RAM_M9K mem(
	.input_data(pixel_data_RGB332),
	.w_addr(WRITE_ADDRESS),
	.r_addr(READ_ADDRESS),
	.w_en(W_EN),
	.clk_W(CLOCK_50),
	.clk_R(clk25MHz), // DO WE NEED TO READ SLOWER THAN WRITE??
	.output_data(MEM_OUTPUT)
);

///////* VGA Module *///////
VGA_DRIVER driver (
	.RESET(VGA_RESET),
	.CLOCK(clk25MHz),
	.PIXEL_COLOR_IN(VGA_READ_MEM_EN ? MEM_OUTPUT : BLACK),
	.PIXEL_X(VGA_PIXEL_X),
	.PIXEL_Y(VGA_PIXEL_Y),
	.PIXEL_COLOR_OUT({GPIO_0_D[9],GPIO_0_D[11],GPIO_0_D[13],GPIO_0_D[15],GPIO_0_D[17],GPIO_0_D[19],GPIO_0_D[21],GPIO_0_D[23]}),
   .H_SYNC_NEG(GPIO_0_D[7]),
   .V_SYNC_NEG(VGA_VSYNC_NEG)
);

///////* Image Processor *///////
IMAGE_PROCESSOR proc(
	.PIXEL_IN(MEM_OUTPUT),
	.CLK(clk25MHz),
	.VGA_PIXEL_X(VGA_PIXEL_X),
	.VGA_PIXEL_Y(VGA_PIXEL_Y),
	.VGA_VSYNC_NEG(VGA_VSYNC_NEG),
	.RESULT(RESULT)
);


///////* Downsampler *///////////
wire [7:0] ledavg;
wire [7:0] pic;
DOWNSAMPLER dsample(
	.PCLK(pclk),  //new byte from camera on each posedge
	.VSYNC(vsync), //new frame from camera on each negedge
	.HREF(href), //new row from camera on each posedge
	.CAMERA_IN(camera_data), //[7:0] from camera
	.READY(W_EN), //write to RAM on this posedge 
	.RAM_ADDR(WRITE_ADDRESS), //which RAM address to write data to [14:0]
	.DATA_2_RAM(), //what data to write to ram [7:0]
	.CV_2_RAM(),
	.CV_3_RAM(),
	.CV_4_RAM(pixel_data_RGB332),
	.LEDAVG(ledavg)
);
assign LED = ledavg;
//assign pixel_data_RGB332 = (edges === 8'b0) ? pic : edges;


///////* Update Read Address *///////
always @ (VGA_PIXEL_X, VGA_PIXEL_Y) begin
		READ_ADDRESS = (VGA_PIXEL_X + VGA_PIXEL_Y*`SCREEN_WIDTH);
		X_ADDR = VGA_PIXEL_X;
		Y_ADDR = VGA_PIXEL_Y;
		if(VGA_PIXEL_X>(`SCREEN_WIDTH-1) || VGA_PIXEL_Y>(`SCREEN_HEIGHT-1))begin
				VGA_READ_MEM_EN = 1'b0;
		end
		else begin
				VGA_READ_MEM_EN = 1'b1;
		end
end



///////* WRITE FLAG TO MEM *///////
//always @ (VGA_PIXEL_X, VGA_PIXEL_Y) begin
//		X_ADDR = VGA_PIXEL_X;
//		Y_ADDR = VGA_PIXEL_Y;
//		
//		if ((VGA_PIXEL_X >= 80 && VGA_PIXEL_X <= 96) || (VGA_PIXEL_Y >= 64 && VGA_PIXEL_Y <= 80)) begin
//			pixel_data_RGB332 = RED;
//		end else  
//			pixel_data_RGB332 = WHITE;
//		
//		if(VGA_PIXEL_X>(`SCREEN_WIDTH-1) || VGA_PIXEL_Y>(`SCREEN_HEIGHT-1))begin
//				W_EN = 1'b0;
//		end
//		else begin
//				W_EN = 1'b1;
//		end
//end

	
endmodule 
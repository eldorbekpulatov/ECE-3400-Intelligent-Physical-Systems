`define TOTAL_SCREEN_WIDTH 795
`define TOTAL_SCREEN_HEIGHT 525
`define VISIBLE_SCREEN_WIDTH 640
`define VISIBLE_SCREEN_HEIGHT 480

module DOWNSAMPLER (
	PCLK,  //new byte from camera on each posedge
	VSYNC, //new frame from camera on each negedge
	HREF, //new row from camera on each posedge
	CAMERA_IN, //[7:0] from camera
	READY, //write to RAM on this posedge 
	RAM_ADDR, //which RAM address to write data to [14:0]
	DATA_2_RAM //what data to write to ram [7:0]
);
input [7:0] CAMERA_IN;
input PCLK;
input VSYNC;
input HREF;

output [7:0] DATA_2_RAM;
output READY;
output [14:0] RAM_ADDR;

reg READY = 1'b0;
reg CYCLE = 1'b0;
reg [15:0] TEMP;
reg [7:0] DATA_2_RAM;
reg [9:0] X = 10'b0;
reg [9:0] Y = 10'b0; 

reg last_sync = 0;
reg last_href = 0;

assign RAM_ADDR = X + Y*(`SCREEN_WIDTH);

always @(posedge PCLK) begin
	if(VSYNC & ~last_sync) begin // posedge vsync
		Y = 10'b0;	
		X = 10'b0;
		CYCLE = 1'b0;
	end 
	else if (~HREF & last_href) begin // negedge href
		Y = Y + 10'b1;  
		X = 10'b0;
		CYCLE = 1'b0;
	end
	else begin
		Y = Y; 
		if (HREF) begin			
			if (CYCLE == 1'b0) begin
				DATA_2_RAM[4:2] = CAMERA_IN[7:5]; //GREEN
				DATA_2_RAM[1:0] = CAMERA_IN[2:0]; //BLUE
				CYCLE = 1'b1;
				READY = 1'b0;
				X = X;
			end 
			else begin				
				DATA_2_RAM[7:5] = CAMERA_IN[2:0]; //RED
				READY = 1'b1;
				CYCLE = 1'b0;
				X = X + 1'b1;			
			end
			
		end else begin
			X = 10'b0;
		end 
	end
	last_sync = VSYNC;
	last_href = HREF;
end

endmodule

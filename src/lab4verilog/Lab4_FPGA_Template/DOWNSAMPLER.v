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
	DATA_2_RAM, //what data to write to ram [7:0]
	CV_2_RAM, //Shape Segmentation
	CV_3_RAM, //Edge Detection
	CV_4_RAM,  // Shape + Edge
	LEDAVG
);
input [7:0] CAMERA_IN;
input PCLK;
input VSYNC;
input HREF;

output [7:0] DATA_2_RAM;
output [7:0] CV_2_RAM;
output [7:0] CV_3_RAM;
output [7:0] CV_4_RAM;
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

reg [7:0] sum;
reg [7:0] temp;
reg [7:0] redStuff;
reg [7:0] blueStuff;
reg [7:0] blueStuffForEdges;

reg [7:0] redEdgeStuff;
reg [7:0] blueEdgeStuff; 


assign RAM_ADDR = X + Y*(`SCREEN_WIDTH);
assign CV_2_RAM = redStuff | blueStuff; //Shape Detection
assign CV_3_RAM = (redOrBlueOrNaught === 2'b00) ? 8'b0 : ((redOrBlueOrNaught === 2'b01) ? redEdgeStuff : blueEdgeStuff); //Edge Detection  //redEdgeStuff | 
assign CV_4_RAM = CV_2_RAM | CV_3_RAM;

reg [2:0] redShift [3:0];
reg [2:0] blueShift [3:0];

reg [7:0] blueSum;
reg [7:0] redSum;

output [7:0] LEDAVG; //Outputs the environmental brightness
reg [19:0] brightnessSum;
reg [19:0] sum2bAvg;
wire [19:0] AVG;  //adjust thresholds according to the average brightness
assign AVG = sum2bAvg / (20'd76032);
assign LEDAVG = AVG[7:0];

reg [17:0] redAccumulator;
reg [17:0] blueAccumulator;
reg [1:0] redOrBlueOrNaught;


always @(posedge PCLK) begin
	if(VSYNC & ~last_sync) begin // posedge vsync
		Y = 10'b0;	
		X = 10'b0;
		CYCLE = 1'b0;
		sum2bAvg <= brightnessSum;
		brightnessSum <= 20'b0;
		redAccumulator <= 18'b0;
		blueAccumulator <= 18'b0;
		if((redAccumulator < 18'd10000) & (blueAccumulator < 18'd10000))begin
			redOrBlueOrNaught <= 2'b00;
		end
		else if (redAccumulator > blueAccumulator)begin
			redOrBlueOrNaught <= 2'b01;
		end
		else begin
			redOrBlueOrNaught <= 2'b10;
		end
	end 
	else if (~HREF & last_href) begin // negedge href
		Y = Y + 10'b1;  
		X = 10'b0;
		CYCLE = 1'b0;
		brightnessSum <= brightnessSum;
		blueAccumulator <= blueAccumulator;
		redAccumulator <= redAccumulator;
	end
	else begin
		Y = Y; 
		if (HREF) begin			
			if (CYCLE == 1'b0) begin
				temp = CAMERA_IN;
				DATA_2_RAM[4:2] = CAMERA_IN[7:5]; //GREEN
				DATA_2_RAM[1:0] = CAMERA_IN[1:0]; //BLUE
				CYCLE = 1'b1;
				READY = 1'b0;
				X = X;
			end 
			else begin				
				DATA_2_RAM[7:5] = CAMERA_IN[2:0]; //RED
				
				brightnessSum <= CAMERA_IN[2:0] + temp[7:5] + temp[2:0] + brightnessSum;
				
				//BLUE THRESH DETECT 
				//if(temp[2:0] > 3'b0 & DATA_2_RAM[4:2] < 3'b100 & DATA_2_RAM[7:5] < 3'b100) begin
				if(temp[2:0] > 3'b0 & DATA_2_RAM[4:2] < (AVG[2:0]+3'b1) & DATA_2_RAM[7:5] < (AVG[2:0]+3'b1)) begin	
					blueStuff[1:0] = temp[1:0];
					blueStuffForEdges[2:0] = temp[2:0];
					blueStuff[7:2] = 6'b0;
					blueStuffForEdges[7:3] = 5'b0;
					blueAccumulator <= blueAccumulator + temp[2:0];
				end
				else begin
					blueStuff[7:0] = 8'b0;
					blueStuffForEdges[7:0] = 8'b0;
				end	
				
				//RED THRESH DETECT if(DATA_2_RAM[7:5] > 3'b0 & DATA_2_RAM[4:2] < 3'b110 & temp[2:0] < 3'b101) begin
				if(DATA_2_RAM[7:5] > 3'b0 & DATA_2_RAM[4:2] < (AVG[2:0]+3'b1) & temp[2:0] < (AVG[2:0]+3'b1)) begin
					redStuff[7:5] = DATA_2_RAM[7:5];
					redStuff[4:0] = 5'b0;
					redAccumulator <= redAccumulator + DATA_2_RAM[7:5];
				end
				else begin
					redStuff[7:0] = 8'b0;
				end
				
				//Blue Edge Stuff
				blueShift[3] <= blueStuffForEdges[2:0];

				blueShift[0] <= blueShift[1];
				blueShift[1] <= blueShift[2];
				blueShift[2] <= blueShift[3];
				
				
				blueSum = blueShift[0]+blueShift[1]+blueShift[2]+blueShift[3];
				if((blueSum > 8'b100) & ~(blueShift[3] > 3'b0)) begin
					blueEdgeStuff <= 8'b11111111;
				end
				else begin
					blueEdgeStuff <= 8'b0;
				end
				
				//Red Edge Stuff
				redShift[3] <= redStuff[7:5];
				
				redShift[0] <= redShift[1];
				redShift[1] <= redShift[2];
				redShift[2] <= redShift[3];
				
				redSum = redShift[0]+redShift[1]+redShift[2]+redShift[3];
				
				if((redSum > 8'b111) & ~(redShift[3] > 3'b0)) begin
					redEdgeStuff <= 8'b00011100;
				end
				else begin
					redEdgeStuff <= 8'b0;
				end
				
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

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
	CV_2_RAM, //Shape Segmentation Output
	CV_3_RAM, //Edge Detection Output
	CV_4_RAM,  // Shape + Edge Output
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
assign CV_3_RAM = (redOrBlueOrNaught === 2'b00) ? 8'b0 : ((redOrBlueOrNaught === 2'b01) ? redEdgeStuff : blueEdgeStuff); //Edge Detection
assign CV_4_RAM = CV_2_RAM | CV_3_RAM;

reg [2:0] redShift [3:0];
reg [2:0] blueShift [4:0];

reg [7:0] blueSum;
reg [7:0] redSum;

output [7:0] LEDAVG; //Outputs the environmental brightness
reg [19:0] brightnessSum;
reg [19:0] sum2bAvg;
wire [19:0] AVG;  //adjust thresholds according to the average brightness
assign AVG = sum2bAvg / (20'd76032);


reg [17:0] redAccumulator;
reg [17:0] blueAccumulator;
reg [1:0] redOrBlueOrNaught;


/////////////////VARS FOR SHAPE DETECTION FROM EDGES////////////////////
//these change constantly
reg [7:0] edgesTopScreen;
reg [7:0] edgesMidScreen;
reg [7:0] edgesBotScreen;

//these only change on each vsync
reg [7:0] edgesTopStore;
reg [7:0] edgesMidStore;
reg [7:0] edgesBotStore;

//change constantly
reg [15:0] edgesYAccumTop;
reg [15:0] edgesYAccumMid;
reg [15:0] edgesYAccumBot;

//change only on vsync
reg [15:0] topAVG;
reg [15:0] midAVG;
reg [15:0] botAVG;

assign LEDAVG = shapeResult;//{topAVG[7:3],midAVG[7:3]};

reg [9:0] lastEdge;

reg edgeDetectBlue;
reg edgeDetectRed;
reg noShapeCondition;

reg [7:0] shapeResult;
////////////////////////////////////////////////////////////////////////
reg smolDistTopBot;
reg smolDistTopMid;
always @(*)begin
		smolDistTopBot = (topAVG - botAVG < 16'd12) | (botAVG - topAVG < 16'd12);
		smolDistTopMid = (topAVG - midAVG < 16'd12) | (midAVG - topAVG < 16'd12);
		shapeResult[5] = smolDistTopBot & smolDistTopMid; //rectangle
		shapeResult[7] = (botAVG > midAVG) & (midAVG > topAVG) & ~shapeResult[5]; //triangle
		shapeResult[6] = (midAVG > topAVG) & (midAVG > botAVG) & ~shapeResult[5]; //diamond
		shapeResult[0] = (edgesTopStore < 8'd15)|(edgesMidStore < 8'd35)|(edgesBotStore < 8'd15); //Treasure Present?
		shapeResult[3:2] = redOrBlueOrNaught; //Red or Blue?
end

always @(posedge PCLK) begin
	if(VSYNC & ~last_sync) begin // New pixel frame
		Y = 10'b0;	
		X = 10'b0;
		CYCLE = 1'b0;
		sum2bAvg <= brightnessSum; //Store the last brightness sum
		brightnessSum <= 20'b0;    //reset brightness sum to zero
		redAccumulator <= 18'b0;   //reset red color accumulator to zero
		blueAccumulator <= 18'b0;  //reset blue color accumulator to zero
		if((redAccumulator < 18'd5000) & (blueAccumulator < 18'd5000))begin
			redOrBlueOrNaught <= 2'b00; //If both red and blue are below a threshold, no color
		end
		else if ((((redAccumulator/3)*2) > blueAccumulator))begin
			redOrBlueOrNaught <= 2'b01; 
		end
		else begin
			redOrBlueOrNaught <= 2'b10;
		end
		
		//Averaging for Shape Detection Filters
		topAVG <= edgesYAccumTop / {8'b0,edgesTopScreen};
		midAVG <= edgesYAccumMid / {8'b0,edgesMidScreen};
		botAVG <= edgesYAccumBot / {8'b0,edgesBotScreen};
		
		edgesTopStore <= edgesTopScreen;
		edgesMidStore <= edgesMidScreen;
		edgesBotStore <= edgesBotScreen;
		noShapeCondition <= 1'b0;//(edgesTopScreen < 8'd30)|(edgesMidScreen < 8'd30)|(edgesBotScreen < 8'd30); 
		
		//Reset Shape Detection Filters
		edgesTopScreen <= 8'b1;
		edgesMidScreen <= 8'b1;
		edgesBotScreen <= 8'b1;
		edgesYAccumTop <= 16'b0;
		edgesYAccumMid <= 16'b0;
		edgesYAccumBot <= 16'b0;
	end 
	else if (~HREF & last_href) begin // New line of pixels
		Y = Y + 10'b1;  
		X = 10'b0;
		CYCLE = 1'b0;
		brightnessSum <= brightnessSum;  //hold state
		blueAccumulator <= blueAccumulator;
		redAccumulator <= redAccumulator;
		
		//Shape Detection Filters
		if((Y < 10'd48) & (lastEdge > 10'd20)) begin
			edgesYAccumTop <= edgesYAccumTop + {6'b0,lastEdge};
			edgesTopScreen <= edgesTopScreen + 1'b1;
		end
		else if((Y >= 10'd48) & (Y < 10'd96) & (lastEdge > 10'd20)) begin
			edgesYAccumMid <= edgesYAccumMid + {6'b0,lastEdge};
			edgesMidScreen <= edgesMidScreen + 1'b1;
		end
		else if((Y >= 10'd96) & (lastEdge > 10'd20)) begin
			edgesYAccumBot <= edgesYAccumBot + {6'b0,lastEdge};
			edgesBotScreen <= edgesBotScreen + 1'b1;
		end 
		lastEdge <= 10'b0;
	end
	else begin
		Y = Y; 
		if (HREF) begin			
			if (CYCLE == 1'b0) begin
				temp = CAMERA_IN;
				DATA_2_RAM[4:2] = CAMERA_IN[7:5]; //GREEN component
				DATA_2_RAM[1:0] = CAMERA_IN[1:0]; //BLUE component
				CYCLE = 1'b1;
				READY = 1'b0;
				X = X;
			end 
			else begin				
				DATA_2_RAM[7:5] = CAMERA_IN[2:0]; //RED component
				
				//Sum together R,G,B components into brightness sum
				brightnessSum <= CAMERA_IN[2:0] + temp[7:5] + temp[2:0] + brightnessSum;
				
				////vvvv///////Shape Segmentation///////vvvv////////
				
				//BLUE THRESHOLD DETECT
				if(temp[2:0] > 3'b0 & DATA_2_RAM[4:2] < (AVG[2:0]) & DATA_2_RAM[7:5] < (AVG[2:0])) begin	
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
				
				//RED THRESHOLD DETECT
				if(DATA_2_RAM[7:5] > 3'b0 & DATA_2_RAM[4:2] < (AVG[2:0]) & temp[2:0] < (AVG[2:0])) begin
					redStuff[7:5] = DATA_2_RAM[7:5];
					redStuff[4:0] = 5'b0;
					redAccumulator <= redAccumulator + DATA_2_RAM[7:5];
				end
				else begin
					redStuff[7:0] = 8'b0;
				end
				
				////vvvv///////Edge Detection///////vvvv////////////
				
				//Blue Edge Stuff
				
				//FIR Filter y[n] = x[n]+x[n-1]+x[n-2]+x[n-3]+x[n-4]
				blueShift[4] <= blueStuffForEdges[2:0];
				blueShift[0] <= blueShift[1];
				blueShift[1] <= blueShift[2];
				blueShift[2] <= blueShift[3];
				blueShift[3] <= blueShift[4];
				blueSum = blueShift[0]+blueShift[1]+blueShift[2]+blueShift[3]+blueShift[4];
				
				//Subtract image from filter to detect right blue edge
				if((blueSum > 8'b100) & ~(blueShift[3] > 3'b0)) begin
					blueEdgeStuff <= 8'b11111111;  //blue edges represented as white
					edgeDetectBlue = 1'b1; 
					//need a blocking assign so this can act as an intermediate wire
				end
				else begin
					blueEdgeStuff <= 8'b0;
					edgeDetectBlue = 1'b0;
				end
				
				//Red Edge Stuff
				
				//FIR Filter y[n] = x[n]+x[n-1]+x[n-2]+x[n-3]
				redShift[3] <= redStuff[7:5];
				redShift[0] <= redShift[1];
				redShift[1] <= redShift[2];
				redShift[2] <= redShift[3];
				redSum = redShift[0]+redShift[1]+redShift[2]+redShift[3];
				
				//Subtract image from filter to detect right red edge
				if((redSum > 8'b111) & ~(redShift[3] > 3'b0)) begin
					redEdgeStuff <= 8'b00011100; //red edges represented as green
					edgeDetectRed = 1'b1;
					//need a blocking assign so this can act as an intermediate wire
				end
				else begin
					redEdgeStuff <= 8'b0;
					edgeDetectRed = 1'b0;
				end
				
				//Pick out the rightmost edge to use for calculations
				if(edgeDetectRed | edgeDetectBlue) begin
					lastEdge <= X;
				end
				else begin
					lastEdge <= lastEdge;
				end
				
				
				
				//Resets and increments
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

`define SCREEN_WIDTH 176
`define SCREEN_HEIGHT 144
`define NUM_BARS 3
`define BAR_HEIGHT 48

module IMAGE_PROCESSOR (
	PIXEL_IN,
	CLK,
	VGA_PIXEL_X,
	VGA_PIXEL_Y,
	VGA_VSYNC_NEG,
	RESULT
);


//=======================================================
//  PORT declarations
//=======================================================
input	[7:0]	PIXEL_IN;
input 		CLK;

input [9:0] VGA_PIXEL_X;
input [9:0] VGA_PIXEL_Y;
input			VGA_VSYNC_NEG;

//reg prevPixY;
output [8:0] RESULT;
reg [2:0] shapeColor;

reg [9:0] edgeXAccumulator [143:0];
reg [7:0] totalTop;
reg [7:0] totalMid;
reg [7:0] totalBot;
reg [9:0] yPrev;
reg [9:0] lastEdge;

reg [16:0] topSum [0:71];
reg [16:0] midSum [36:107];
reg [16:0] botSum [72:143];

wire [16:0] topAVG;
wire [16:0] midAVG;
wire [16:0] botAVG;

reg [7:0] lastTotalTop;
reg [7:0] lastTotalMid;
reg [7:0] lastTotalBot;
//[71:0]
//[143:72]


//reg [17:0] redAccumulator; //25,344 * 7 = 177,408
//
//reg [17:0] blueAccumulator;  //25,344 * 3 = 76,032



//wire blueEdge;
//shiftReg11bit blueWindow(
// .CLK(CLK), 
// .SI({1'b0,PIXEL_IN[1:0]}),
// .SO(),
// .SUM(),
// .edgeDetected(blueEdge)
//);
//
//wire redEdge;
//shiftReg11bit redWindow(
// .CLK(CLK), 
// .SI(PIXEL_IN[7:5]),
// .SO(),
// .SUM(),
// .edgeDetected(redEdge)
//);





//
//always @(posedge CLK) begin
//	if(VGA_VSYNC_NEG) begin
////		if(((lastTotalTop+lastTotalBot) < 8'd10) | (lastTotalMid < 8'd10))begin
////		  shapeColor <= 3'b100;
////		end 
////		else if(((midAVG - topAVG) > 17'd10) & ((midAVG - botAVG) > 17'd10)) begin
////			shapeColor <= 3'b01; //diamond
////		end
////		else if(((botAVG - midAVG) > 17'd10) & ((midAVG - topAVG) > 17'd10))begin
////			shapeColor <= 3'b10; //triangle
////		end
////		else begin
////			shapeColor <= 3'b11;
////		end
////		totalTop <= 8'b0;
////		totalMid <= 8'b0;
////		totalBot <= 8'b0;
//		lastEdge <= 10'b0;
//	end
//	else if(VGA_PIXEL_X != 10'b0) begin
//		edgeXAccumulator[VGA_PIXEL_Y] <= lastEdge;
//	end
//	else begin
//		if(((PIXEL_IN === 8'b111_111_11) | (PIXEL_IN === 8'b000_111_00)))begin
//			lastEdge <= VGA_PIXEL_X;
//		end
//		else begin
//			lastEdge <= lastEdge;
//		end
//	end
//	yPrev <= VGA_PIXEL_Y;
//
//end
//
//assign topAVG = topSum [71]  / {9'b0,lastTotalTop};
//assign midAVG = midSum [107] / {9'b0,lastTotalMid};
//assign botAVG = botSum [143] / {9'b0,lastTotalBot};
//
//assign RESULT = edgeXAccumulator[31];
//
endmodule

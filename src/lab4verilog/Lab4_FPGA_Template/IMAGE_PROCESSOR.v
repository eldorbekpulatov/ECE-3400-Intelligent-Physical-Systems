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

output [8:0] RESULT;

reg [17:0] redAccumulator; //25,344 * 7 = 177,408

reg [17:0] blueAccumulator;  //25,344 * 3 = 76,032

always @(posedge CLK) begin
	if(VGA_VSYNC_NEG) begin
		redAccumulator <= 18'b0;
		blueAccumulator <= 18'b0;
	end
	else begin
		redAccumulator <= redAccumulator + (18'b0 | (PIXEL_IN & 8'b11100000));
		blueAccumulator <= blueAccumulator + (18'b0 | (PIXEL_IN & 8'b00000011));
	end
end

assign RESULT[0] = (redAccumulator > 18'd25000);
assign RESULT[1] = (blueAccumulator > 18'd1000);

endmodule

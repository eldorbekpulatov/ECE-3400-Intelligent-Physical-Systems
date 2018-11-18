module edgeComputeUnit (CLK, RESET, edgeCoordXIn, edgeCoordYIn);

input CLK;
input RESET;
input [9:0] edgeCoordXIn;
input [9:0] edgeCoordYIn;
wire [15:0] midX;
wire [15:0] midY;


integer i;

reg [15:0] incSumX [199:0];
reg [15:0] incSumY [199:0];

reg [7:0] shiftX [199:0];
reg [7:0] shiftY [199:0];

reg [9:0] count;

//////Hardware For X Component//////
always @(posedge CLK) begin
	if(RESET)begin
	for(i = 0; i < 200; i = i+1)
			shiftX[i] <= 8'b0;
			count <= 10'b0;
	end
	else begin
		shiftX[199] <= edgeCoordXIn[7:0];
		count <= count + 10'b0000000001;
		for(i = 0; i < 199; i = i+1)
			shiftX[i] <= shiftX[i+1];
	end
end

always @(*) begin
    for (i = 1; i<200; i = i + 1) begin
         if (i == 1) begin
             incSumX[i] = shiftX[i] + shiftX[i-1];
         end
         else begin
             incSumX[i] = incSumX[i-1] + shiftX[i];
         end
    end
end

//////Hardware For Y Component//////
always @(posedge CLK) begin
	if(RESET)begin
	for(i = 0; i < 200; i = i+1)
			shiftY[i] <= 8'b0;
	end
	else begin
		shiftY[199] <= edgeCoordYIn[7:0];
		for(i = 0; i < 199; i = i+1)
			shiftX[i] <= shiftX[i+1];
	end
end

always @(*) begin
    for (i = 1; i<200; i = i + 1) begin
         if (i == 1) begin
             incSumY[i] = shiftY[i] + shiftY[i-1];
         end
         else begin
             incSumY[i] = incSumY[i-1] + shiftY[i];
         end
    end
end
////////////////////////////////////////////

///////////Hardware for Multicycle Variance Computation//////////
assign midX = (incSumX[199] / ({6'b0,count}));
assign midY = (incSumY[199] / ({6'b0,count}));


endmodule


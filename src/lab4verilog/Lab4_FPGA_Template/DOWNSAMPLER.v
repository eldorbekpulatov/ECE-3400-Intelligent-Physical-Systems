module DOWNSAMPLER(
	PCLK,  //new byte from camera on each posedge
	HREF, //new row from camera on each posedge
	CAMERA_IN, //[7:0] from camera
	READY, //write to RAM on this posedge 
	DATA_2_RAM //what data to write to ram [7:0]
);

input [7:0] CAMERA_IN;
input PCLK;
input HREF;

output reg READY;
output [7:0] DATA_2_RAM;

reg byteCount = 1'b0; 
reg [7:0] rgb332 = 8'b0;

assign DATA_2_RAM = rgb332;

always@(posedge PCLK) begin
	//New byte ready to be read
	if(HREF) begin
		if(~byteCount) begin
			rgb332[7:5] = CAMERA_IN[7:5];
			rgb332[4:2] = CAMERA_IN[2:0];
			byteCount = 1'b1;
			READY = 1'b0;
		end 
		else begin
			rgb332[1:0] = CAMERA_IN[4:3];
			byteCount = 1'b0;
			READY = 1'b1;
		end
	end
	else begin
		rgb332 = 8'b0;
		byteCount = 1'b0;
		READY = 1'b0;
	end
end

endmodule 
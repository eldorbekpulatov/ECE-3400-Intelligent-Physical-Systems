#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(9, 10); // CE, CSN
long long address = 0x0000000068LL;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
   if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    String stuff = "";
    stuff += String(int(text[0]));
    stuff += ","+String(int(text[1]));
    byte hellaStuff = text[2];
  if(wallNorth(hellaStuff)){
    stuff+=",north=true";
  }else{
    stuff+=",north=false";
  }
  if(wallEast(hellaStuff)){
    stuff+=",east=true";
  }else{
    stuff+=",east=false";
  }
  if(wallWest(hellaStuff)){
    stuff+=",west=true";
  }else{
    stuff+=",west=false";
  }
  if(wallSouth(hellaStuff)){
    stuff+=",south=true";
  }else{
    stuff+=",south=false";
  }

  if(tShape(hellaStuff)!="none"){
    stuff+=",tshape=" +(tShape(hellaStuff));
    stuff+=",tcolor=" +(tColor(hellaStuff));
  Serial.println(stuff);
  }
}
}

boolean wallNorth(byte data){
  return((data >> 7) & 1 == 0b1);
}
boolean wallEast(byte data){
  return((data >> 6) & 1 == 0b1);
}
boolean wallSouth(byte data){
  return((data >> 5) & 1 == 0b1);
}
boolean wallWest(byte data){
  return((data >> 4) & 1 == 0b1);
}
boolean isRobot(byte data){
  return((data >> 3) & 1 == 0b1);
}

String tShape(byte data){
  boolean bit0 = ((data)&1);
  boolean bit1 = ((data>>1)&1);
  if(!bit1&&bit0){
    return "square";
  }
  else if(bit1&&!bit0){
    return "triangle";
  }
  else if(bit1&&bit0){
    return "circle";
  }else{
    return "none";
  }
}

String tColor(byte data){
  boolean bit0 = ((data)&1);
  boolean bit1 = ((data>>1)&1);
  boolean bit2 = ((data>>2)&1);
  if(!bit0 && !bit1 && !bit2){
    return "none";
  }
  else if(bit2){
    return "blue";
  }
  else{
    return "red";
  }
}


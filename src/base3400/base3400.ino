#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
// 
RF24 radio(9, 10); // CE, CSN

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0x0000000044LL, 0x0000000045LL };


void setup() {
  Serial.begin(9600);
  setupRadio();
}

void loop() {
    if ( radio.available() )
    {
      
//      bool done = false;
//      while (!done)
//      {
        // Fetch the payload, and see if this was the last one.
        //done = radio.read( &got_time, sizeof(unsigned long) );
        char text[32] = "";
        radio.read(&text, sizeof(text)); // done = 
       // Serial.println(text);
        String guiMsg = "";
        guiMsg += String(int(text[0]));
        guiMsg += ","+String(int(text[1]));
        byte mazeInfo = text[2];
        if(wallNorth(mazeInfo)){
          guiMsg+=",north=true";
        } else {
          guiMsg+=",north=false";
        }
        if(wallEast(mazeInfo)){
          guiMsg+=",east=true";
        } else {
          guiMsg+=",east=false";
        }
        if(wallWest(mazeInfo)){
          guiMsg+=",west=true";
        } else {
          guiMsg+=",west=false";
        }
        if(wallSouth(mazeInfo)){
          guiMsg+=",south=true";
        } else {
          guiMsg+=",south=false";
        }

    if(tShape(mazeInfo)!="none"){
      guiMsg+=",tshape=" +(tShape(mazeInfo));
      guiMsg+=",tcolor=" +(tColor(mazeInfo));
    }
    Serial.println(guiMsg);
   // delay(20);

  //  }
 }
}

/**** SET UP ****/

void setupRadio(){
  radio.begin();
  
  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);
  
  //radio.setAutoAck(true);
  // set the channel
  //radio.setChannel(0x50);
  // set the power
  // RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_MED=-6dBM, and RF24_PA_HIGH=0dBm.
  //radio.setPALevel(RF24_PA_MIN);
  //RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
  //radio.setDataRate(RF24_250KBPS);
  
  radio.setPayloadSize(3);
  //RECEIVE CODE (RECEIVE)
  
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1,pipes[0]);

  radio.startListening();
  //radio.printDetails();
}

/**** GENERAL ****/

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

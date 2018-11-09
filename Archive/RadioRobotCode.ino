#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(9, 10); // CE, CSN
long long address = 0x0000000068LL;

byte maze[9][9];
int startPosX = 0;
int startPosY = 0;

void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.stopListening();
  Serial.begin(9600);
  byte x;
  for(int i = 0; i < 9; i++){
    for(int j = 0; j < 9; j++){
      x = 0xFF;
      maze[i][j] = x;
    }
  }
}



void loop() {
    for(int i = 0; i < 9; i++){
    for(int j = 0; j < 9; j++){
      char msg[3];
      msg[0] = char(i);
      msg[1] = char(j);
      msg[2] = char(maze[i][j]);
      radio.write(msg, 3);
      Serial.println(msg);
      delay(1000);
    }
  }
    //radio.write("hello", 5);
    //delay(1000);
    //radio.write("world", 5);
    //delay(1000);
}



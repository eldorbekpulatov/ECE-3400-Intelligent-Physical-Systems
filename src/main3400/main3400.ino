/**
 * TEAM CAPTCHA FALL 2018
 * Joseph Primmer   | Vicente Caycedo
 * Eldor Bekpulatov | Francis Rayos del Sol
 */

#define LOG_OUT 1 // use the log output function
#define FFT_N 256 // set to 256 point fft

// Servo Library
#include <Servo.h>
// FFT Library 
#include <FFT.h>
// Radio Libraries
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define rightSensorPin A0
#define leftSensorPin A1
#define micInputPin A3
#define leftWall 4
#define frontWall 3
#define rightWall 2
#define lineSensorThreshold 930
#define FIRsampRate 20
#define FIRwindowSize 60
#define DetectionRate 100

Servo rightWheel;
Servo leftWheel;

RF24 radio(9, 10); // CE, CSN
// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0x0000000044LL, 0x0000000045LL };

byte maze[9][9];
//bool onStack [9][9];

/* Robot starts in top left square of map, facing south */
int posX = 0; // row
int posY = 0; // col

/* Robot's orientation
 *  0: North
 *  1: East
 *  2: South
 *  3: West
 */
int orientation = 2;

/************* CLASSES ***************/
struct Coordinate{
    Coordinate(int x, int y){this->x = x; this->y = y;}
    int x, y;
};

class Stack{
  public:
   void push(Coordinate c){
    stack_x[i] = c.x;
    stack_y[i] = c.y;
    i = i+1;
   }

   Coordinate pop(){
    if(i > 0){
      i = i - 1;
      return Coordinate(stack_x[i], stack_y[i]);
    } 
    }

   Coordinate peep(){
    if (i > 0 ){
      return Coordinate(stack_x[i-1], stack_y[i-1]);
    }}
   
   bool isEmpty(){
    return i < 1; 
   }

  private:
    byte stack_x [81];
    byte stack_y [81];
    byte i = 0; 
};

/*********** MAIN ***************/
void setup() {
  Serial.begin(115200);
  setupMotors();
  setupWallSensors();
  setupRadio();
  resetMaze();
}

void loop() {
  stopMoving();
  while(!startSignalDetected()){} // Do not move until signal detected
  dfs();
  while(1);
}

/*************** SET UP ****************/

/* Sets up left and right motors for movement*/
void setupMotors() {
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
  leftWheel.attach(5);
  rightWheel.attach(6);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
}

void setupWallSensors() {
  pinMode(leftWall, INPUT);
  pinMode(frontWall, INPUT);
  pinMode(rightWall, INPUT);
}

void setupRadio() {
  radio.begin();
  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);
  //radio.setAutoAck(true);
  // set the channel
  //radio.setChannel(0x50);
  // set the power
  // RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_MED=-6dBM, and RF24_PA_HIGH=0dBm.
  //radio.setPALevel(RF24_PA_HIGH);
  //RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
  //radio.setDataRate(RF24_250KBPS);
  radio.setPayloadSize(3);
  //SEND CODE (TRANSMIT)
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);
  //radio.printDetails();
  
}

void resetMaze() {
  /*
   * INFORMATION ENCODING FOR MAZE MAPPING
   * 1 Byte, x, is used to map information for each square
   * x[7:4] = North, East, South, West Wall detected
   * x[3]   = Explored
   * x[2:0] = Treasure encoding
   *      000 - undefined
   *      001 - red square
   *      010 - red triangle
   *      011 - red circle
   *      100 - undefined
   *      101 - blue square
   *      110 - blue triangle
   *      111 - blue circle
   */
  byte x;
  for(int i = 0; i < 9; i++){
    for(int j = 0; j < 9; j++){
      maze[i][j] = 0x00;
    }
  }
}



/**** MAZE TRAVERSAL ****/

boolean canTurnRight() {
  return !digitalRead(rightWall);
}

boolean canMoveStraight() {
  return !digitalRead(frontWall);
}

boolean canTurnLeft() {
  return !digitalRead(leftWall);
}

bool isVisited(Coordinate c){
  if ((c.y < 9 && c.y > -1) && (c.x < 9 && c.x > -1)){
    return !((maze[c.x][c.y] | 0b11110111) ^ 0b11111111);
  }else{ return true; }
}

int getDirection(Coordinate c){
  int dir;
  if (posX == c.x){
    if (posY > c.y){ dir = 3; }
    else{ dir = 1; }
  }else{
    if (posX > c.x){ dir = 0; }
    else{ dir = 2;}
  }
  return dir;
}

/* Updates robot's position in maze */
void updatePos(){
  switch (orientation) {
    case 0: posX--; break;
    case 1: posY++; break;
    case 2: posX++; break;
    case 3: posY--; break;
    default: break;
  }
}

/* Updates robot's orientation in maze 
 *  param turn: 0 if straight, 1 if turned right, 
 *              2 if turned back, 3 if turned left
 */
void updateOrientation(int turn){
  // dont update orientation if went straight
  if(turn == 1){ //Turned right
    orientation = (orientation + 1) % 4;
  } else if(turn == 2){ //Turned back
    orientation = (orientation + 2) % 4;
  }else if (turn == 3){ //Turned left
    orientation--;
    if (orientation == -1){
      orientation = 3;
    }
  } 
}

Coordinate getFrontNeighbor(){
  switch(orientation){
    case 0: return Coordinate(posX - 1, posY);
    case 1: return Coordinate(posX, posY + 1);
    case 2: return Coordinate(posX + 1, posY);
    case 3: return Coordinate(posX, posY - 1);
    default: return Coordinate(-1,-1);
  }
}

Coordinate getLeftNeighbor(){
  switch(orientation){
    case 0: return Coordinate(posX, posY - 1);
    case 1: return Coordinate(posX - 1, posY);
    case 2: return Coordinate(posX, posY + 1);
    case 3: return Coordinate(posX + 1, posY);
    default: return Coordinate(-1,-1);
  }
}

Coordinate getRightNeighbor(){
  switch(orientation){
    case 0: return Coordinate(posX, posY + 1);
    case 1: return Coordinate(posX + 1, posY);
    case 2: return Coordinate(posX, posY - 1);
    case 3: return Coordinate(posX - 1, posY);
    default: return Coordinate(-1,-1);
  }
}

/**** MOVEMEMENT ****/
/* Follows the line and returns true when intersection found and false otherwise*/
boolean followLine(){
  int rightLine = analogRead(rightSensorPin);
  int leftLine = analogRead(leftSensorPin);
  if(rightLine < lineSensorThreshold && leftLine < lineSensorThreshold){
    return true; // Intersection encountered
  } else if(sampleRobotDetect()){
    stopMoving(); //Stand still if a robot is seen
  }
  else if(leftLine < lineSensorThreshold) {
    // Left sensor white
    rightWheel.write(90); //nudge left
    leftWheel.write(130);
  } else if(rightLine < lineSensorThreshold) {
    // Right sensor white
    rightWheel.write(50); //nudge right
    leftWheel.write(90);
  } else {
    // Both sensors black
    leftWheel.write(130);
    rightWheel.write(40);
  }
  return false;
}

/* Stops servo motors */
void stopMoving(){
  rightWheel.write(90);
  leftWheel.write(90);
}

/* Turns 180 degrees */
void turnAround(){
  leftWheel.write(135);
  rightWheel.write(135);
  delay(800);
  int rightLine = analogRead(rightSensorPin);
  while(rightLine > lineSensorThreshold){
    rightLine = analogRead(rightSensorPin);
  }
}

/* Turns right */
void turnRight(){
  delay(100);
  leftWheel.write(130);
  rightWheel.write(90);
  delay(800);
  int rightLine = analogRead(rightSensorPin);
  while(rightLine > lineSensorThreshold){
    rightLine = analogRead(rightSensorPin);
  }
}

/* Turns left */
void turnLeft(){
  delay(100);
  leftWheel.write(90);
  rightWheel.write(40);
  delay(800);
  int leftLine = analogRead(leftSensorPin);
  while(leftLine > lineSensorThreshold){
    leftLine = analogRead(leftSensorPin);
  }
}

/************ STEP MOVEMENTS **************/

void goStraight(){
  leftWheel.write(130);
  rightWheel.write(40);
  delay(200);
  //updateOrientation(0); //went straight
  while(!followLine()){} 
  stopMoving();
}

void goLeft(){
  turnLeft();
  updateOrientation(3); //turned left
  while(!followLine()){}; 
  stopMoving();
}
void goRight(){
  turnRight();
  updateOrientation(1); //turned right
  while(!followLine()){}; 
  stopMoving();
}
void goBack(){
  turnAround();
  updateOrientation(2); //turned back
  while(!followLine()){}; 
  stopMoving();
}

void goToDir(int dir){
  int rem = dir - orientation;
  switch(rem){
    case -3: goRight(); break;
    case -2: goBack(); break;
    case -1: goLeft(); break;
    case 0: goStraight(); break;
    case 1: goRight(); break;
    case 2: goBack(); break;
    case 3: goLeft(); break; 
    default: break;
  }
  updatePos();
}

Coordinate getNotVisited(Coordinate src) {
  if(!isVisited(getRightNeighbor()) && canTurnRight()) { return getRightNeighbor(); }
  else if (!isVisited(getFrontNeighbor()) && canMoveStraight()) { return getFrontNeighbor(); }
  else if (!isVisited(getLeftNeighbor()) && canTurnLeft()) { return getLeftNeighbor(); }
  // Should not need to check behind 
  else {
    return Coordinate(-1,-1); // no unvisited neighbors
  }
}  

void dfs(){

  Stack s;
  s.push(Coordinate(0,0)); // TODO: check
  mapMaze();
  while(!s.isEmpty()){
    Coordinate curr = getNotVisited(s.peep());
    Serial.print("X: ");
    Serial.print(curr.x);
    Serial.print(" Y: ");
    Serial.println(curr.y);
    if ( curr.x != -1 && curr.y != -1 ) { // Curr x and y are -1 if there are no nodes to visit
        s.push(curr);
        goToDir(getDirection(curr));
        mapMaze(); 
    } else {
        s.pop();
        goToDir(getDirection(s.peep()));
    }
  }
  
}

/**************** GENERAL *********************/

/* Creates mapping of current square and sends info to base */
void mapMaze(){
  byte squareInfo;
  //Setting explored
  squareInfo |= 0b00001000; 
  //Setting walls (depends on orientation)
  switch (orientation) {
        case 0:
            if(!canTurnRight()){
              squareInfo |= 0b01000000;
            }
            if(!canMoveStraight()){
              squareInfo |= 0b10000000;  
            }
            if(!canTurnLeft()){
              squareInfo |= 0b00010000;
            }
            break;
        case 1:
            if(!canTurnRight()){
              squareInfo |= 0b00100000;
            }
            if(!canMoveStraight()){
              squareInfo |= 0b01000000;  
            }
            if(!canTurnLeft()){
              squareInfo |= 0b10000000;
            }
            break;
        case 2:
            if(!canTurnRight()){
              squareInfo |= 0b00010000;
            }
            if(!canMoveStraight()){
              squareInfo |= 0b00100000;  
            }
            if(!canTurnLeft()){
              squareInfo |= 0b01000000;
            }
            break;
        case 3:
            if(!canTurnRight()){
              squareInfo |= 0b10000000;
            }
            if(!canMoveStraight()){
              squareInfo |= 0b00010000;  
            }
            if(!canTurnLeft()){
              squareInfo |= 0b00100000;
            }
            break;
        default:
          break;
  }

  // TODO: Fix treasure
  squareInfo = squareInfo & 0b11111000; // Resets last three bits
 // squareInfo = squareInfo | 0b1; //Sets last bit to temporarily indicate red square.

  maze[posX][posY] = squareInfo;
  transmitMsg();
}

/* Returns true if start whistle detected */
int samples[FIRwindowSize];
int sIndex = 0;
int sRateCount = 0;
boolean startSignalDetected(){
  if(sRateCount >= FIRsampRate){
    samples[sIndex] = analogRead(micInputPin);
    sRateCount = 0;
    sIndex++;
  }else{
    delay(1);
    sRateCount++;
  }
  if(sIndex > FIRwindowSize){
    sIndex = 0;
  }long sum = 0;
  for(int i = 0; i < FIRwindowSize; i++){
    sum += samples[i];
  }
  int output = sum/FIRwindowSize;
  if(output > 589)return true;
  else return false;
}

/* Transmits info to base */
void transmitMsg(){
  int numTries = 0;
  radio.stopListening();
  bool ok;
      do{
        Serial.print("hey");
        Serial.println(numTries);
        char msg[3];
        msg[0] = char(posX);
        msg[1] = char(posY);
        msg[2] = char(maze[posX][posY]);
        ok = radio.write(msg, 3);
        //delay(20);
        numTries++;
      }while(!ok && numTries < 100);
      if(numTries > 50){
        delay(10);
        setupRadio();
      }
}

/*********** SENSORS **********/

/* Samples the robot detect method and stores in detectState every (detection rate) calls, returns detectionState
 * Need this workaround because otherwise constantly doing fft makes robot react too slow 
 */
int detectCount = 0;
boolean detectState = false;
boolean sampleRobotDetect(){
  if(detectCount > DetectionRate){
    if(robotDetect()){
      detectState = true;
    }else{
      detectState = false;
    }
    detectCount = 0;
  }
  detectCount++;
  return detectState;
}

/* Returns true if robot detected */
boolean robotDetect(){
   ADMUX = 0x42;
    int init_adcsra = ADCSRA;
    ADCSRA = 0xe5;
    cli();  // UDRE interrupt slows this way down on arduino1.0
    for (int i = 0 ; i < 512 ; i += 2) { // save 256 samples
      while(!(ADCSRA & 0x10)); // wait for adc to be ready
      ADCSRA = 0xf5; // restart adc
      byte m = ADCL; // fetch adc data
      byte j = ADCH;
      int k = (j << 8) | m; // form into an int
      k -= 0x0200; // form into a signed int
      k <<= 6; // form into a 16b signed int
      fft_input[i] = k; // put real data into even bins
      fft_input[i+1] = 0; // set odd bins to 0
    }
    fft_window(); // window the data for better frequency response
    fft_reorder(); // reorder the data before doing the fft
    fft_run(); // process the data in the fft
    fft_mag_log(); // take the output of the
    ADCSRA = init_adcsra;
    sei();
    if(fft_log_out[42] > 50){
        return true;
     } else {
        return false;
     }
}

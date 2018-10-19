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
#include <nRF24L01.h>
#include <RF24.h>

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
long long address = 0x0000000068LL;

byte maze[9][9];
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

void setup() {
  Serial.begin(115200);
  setupMotors();
  setupWallSensors();
  setupRadio();
  resetMaze();
}

void loop() {
  stopMoving();
  while(!startSignalDetected()){}
  while(true) { 
    rightHandFollow();
  }
}

/**** SET UP ****/

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
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.stopListening();
}

void resetMaze() {
  /*
   * INFORMATION ENCODING FOR MAZE MAPPING
   * 1 Byte, x, is used to map information for each square
   * x[7:4] = North, East, South, West Wall detected
   *  x[3]  = Explored
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

/* Circles maze by keeping right hand on wall */
void rightHandFollow(){
  while(!followLine()){} // Keep moving straight until intersection is reached
    if(canTurnRight()){
      Serial.println("turn right");
      turnRight();
      followLine();
    } else if(canMoveStraight()){
      Serial.println("straight");
    } else if(canTurnLeft()){
      Serial.println("turn left");
      turnLeft();
      followLine();
    } else {
      Serial.println("stop");
      stopMoving(); // FIXME: do 180 deg turn
    }
    //updatePos();
  //}
}

/* Updates robot's position in maze */
void updatePos(){
  switch (orientation) {
    case 0:
      posX--;
      break;
    case 1:
      posY++;
      break;
    case 2:
      posX++;
      break;
    case 3:
      posY--;
      break;
    default:
      break;
  }
}

/* Updates robot's orientation in maze 
   param turn: 0 if turned left, 1 if turned right.
*/
void updateOrientation(int turn){
  if(turn){ //Turned right
    orientation = (orientation + 1) % 4;
  } else {
    orientation--;
    if (orientation == -1){
      orientation = 3;
    }
  }
}

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
  // Update neighbor squares if possible
  // TODO: Is it necessary?
  // TODO:

  // TODO: Fix treasure
  squareInfo = squareInfo & 0b11111000; //Resets last three bits
  squareInfo = squareInfo | 0b1; //Sets last bit to temporarily indicate red square.

  maze[posX][posY] = squareInfo;
  transmitMsg();
}

/**** GENERAL ****/

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
  char msg[3];
  msg[0] = char(posX);
  msg[1] = char(posY);
  msg[2] = char(maze[posX][posY]);
  radio.write(msg, 3);
}

/**** MOVEMEMENT ****/

/* Stops servo motors */
void stopMoving(){
  rightWheel.write(90);
  leftWheel.write(90);
}

/* Turns right */
void turnRight(){
  //updateOrientation(1);
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
  //updateOrientation(0);
  delay(100);
  leftWheel.write(90);
  rightWheel.write(40);
  delay(800);
  int leftLine = analogRead(leftSensorPin);
  while(leftLine > lineSensorThreshold){
    leftLine = analogRead(leftSensorPin);
  }
}

/* Follows the line and returns true when intersection found and false otherwise*/
boolean followLine(){
  int rightLine = analogRead(rightSensorPin);
  int leftLine = analogRead(leftSensorPin);
  if(rightLine < lineSensorThreshold && leftLine < lineSensorThreshold){
    // TODO: possibly stop moving to give time to make decisions
    // mapMaze();
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
    rightWheel.write(40); //nudge right
    leftWheel.write(90);
  } else {
    // Both sensors black
    leftWheel.write(130);
    rightWheel.write(40);
  }
  return false;
}

/**** SENSORS ****/

boolean canTurnRight() {
  return !digitalRead(rightWall);
}

boolean canMoveStraight() {
  return !digitalRead(frontWall);
}

boolean canTurnLeft() {
  return !digitalRead(leftWall);
}

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



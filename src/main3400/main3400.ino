/** 
 * TEAM CAPTCHA FALL 2018
 * Joseph Primmer   | Vicente Caycedo
 * Eldor Bekpulatov | Francis Rayos del Sol
 */



#define LOG_OUT 1 // use the log output function
#define FFT_N 256 // set to 256 point fft
#include <Servo.h>
#include <FFT.h>
#define rightSensorPin A0
#define leftSensorPin A1
#define micInputPin A3
#define leftWall 4
#define frontWall 3
#define rightWall 2
#define lineSensorThreshold 900

Servo rightWheel;
Servo leftWheel;

void setup() {
  Serial.begin(115200);
  setupMotors();
  setupWallSensors();
}

void loop() {
  rightHandFollow();
  /*
  stopMoving();
  if (digitalRead(leftWall)) {
    Serial.println("Left wall detected");
  } 
  if (digitalRead(frontWall)) {
    Serial.println("Front wall detected");
  } 
  if (digitalRead(rightWall)) {
    Serial.println("Right wall detected");
  }
  Serial.println("     ");
  delay(1000);
  */
}

/**** SET UP ****/

/* Sets up left and right motors for movement*/
void setupMotors() {
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
  leftWheel.attach(10);
  rightWheel.attach(11);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
}

void setupWallSensors() {
  pinMode(leftWall, INPUT);
  pinMode(frontWall, INPUT);
  pinMode(rightWall, INPUT);
}

/**** MAZE TRAVERSAL ****/

/* Circles maze by keeping right hand on wall */
void rightHandFollow(){
  while(!followLine()){} // Keep moving straight until intersection is reached
  if(canTurnRight()){
    turnRight();
    followLine();
  } else if(canMoveStraight()){
    followLine();
  } else if(canTurnLeft()){
    turnLeft();
    followLine();
  } else {
    stopMoving(); // FIXME: do 180 deg turn
  }
}

/**** GENERAL ****/

/* Returns true if start whistle detected */
boolean startSignalDetected(){
  int micValue = analogRead(micInputPin);
  if( micValue > 580 ){
    return true;
  } else {
    return false;
  }
}

/**** MOVEMEMENT ****/

/* Stops servo motors */
void stopMoving(){
  rightWheel.write(90);
  leftWheel.write(90);
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

/* Follows the line and returns true when intersection found */
boolean followLine(){
  int rightLine = analogRead(rightSensorPin);
  int leftLine = analogRead(leftSensorPin);
  if(rightLine < lineSensorThreshold && leftLine < lineSensorThreshold){
    return true; // Intersection encountered
  } else if(leftLine < lineSensorThreshold) {
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

// TODO: canMoveRight, canMoveLeft, canMoveStraight

boolean canTurnRight() {
  return !digitalRead(rightWall);
}

boolean canMoveStraight() {
  return !digitalRead(frontWall);
}

boolean canTurnLeft() {
  return !digitalRead(leftWall);
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
      }else{
        return false;
      }
}



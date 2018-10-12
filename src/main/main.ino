/** 
 * TEAM CAPTCHA FALL 2018
 * Joseph Primmer   | Vicente Caycedo
 * Eldor Bekpulatov | Francis Rayos del Sol
 */

#include <Servo.h>
#include <FFT.h>

#define LOG_OUT 1 // use the log output function
#define FFT_N 256 // set to 256 point fft
#define rightSensorPin A0
#define leftSensorPin A1
#define micInputPin A3
#define lineSensorThreshold 850

Servo rightWheel;
Servo leftWheel;

int state = 0; 

void setup() {
  Serial.begin(115200);
  setupMotors();
}

void loop() {

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

/**** GENERAL ****/

/* Stops servo motors */
void stopMoving(){
  rightWheel.write(90);
  leftWheel.write(90);
}

/* Returns true if start whistle detected */
boolean startSignalDetected(){
  int micValue = analogRead(micInputPin);
  if( micValue > 580 ){
    return true;
  } else {
    return false;
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

/* Follows the line */
void followLine(){
  int rightLine = analogRead(rightSensorPin);
  int leftLine = analogRead(leftSensorPin);
  if(rightLine < lineSensorThreshold && leftLine < lineSensorThreshold){
    // Intersection encountered
    // TODO: encountered intersection
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
}

/* Returns true if robot detected */
boolean robotDetected(){
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


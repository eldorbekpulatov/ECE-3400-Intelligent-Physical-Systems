
/*
fft_adc_serial.pde
guest openmusiclabs.com 7.7.14
example sketch for testing the fft library.
it takes in data on ADC0 (Analog0) and processes them
with the fft. the data is sent out over the serial
port at 115.2kb.
*/

#define LOG_OUT 1 // use the log output function
#define FFT_N 256 // set to 256 point fft
#include <FFT.h> // include the library
#include <Servo.h>
#define rightSense A0
#define leftSense A1
#define sensorThreshold 850
Servo rightWheel;
Servo leftWheel;
const int micInputPin = A3; 

int topState = 0;
int figure8State = 0;

int micValue = 0; 
void setup() {
  Serial.begin(115200); // use the serial port
  pinMode(rightSense, INPUT);
  pinMode(leftSense, INPUT);
  rightWheel.attach(11);
  leftWheel.attach(10);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
}
void loop() {

}

boolean startSignal(){
  micValue = analogRead(micInputPin);
  if(micValue > 580){
    return true;
  }else{
    return false;
  }
}

void doNothing(){
  rightWheel.write(90);
  leftWheel.write(90);
}

void turnRight(){
  delay(100);
  leftWheel.write(130);
  rightWheel.write(90);
  delay(800);
  int rightLine = analogRead(rightSense);
  while(rightLine > sensorThreshold){
    rightLine = analogRead(rightSense);
    leftWheel.write(130);
    rightWheel.write(90);
  }
}

void turnLeft(){
  delay(100);
  leftWheel.write(90);
  rightWheel.write(40);
  delay(800);
  int leftLine = analogRead(leftSense);
  while(leftLine > sensorThreshold){
    leftLine = analogRead(leftSense);
    leftWheel.write(90);
    rightWheel.write(40);
  }
}

void figureEight(){
  if(figure8State < 4){
    turnLeft();
    figure8State++;
  }else{
    turnRight();
    figure8State++;
  }
  if(figure8State > 7){
    figure8State = 0;
  }
  topState = 0;
  
}

void followLine(){
  int rightLine = analogRead(rightSense);
  int leftLine = analogRead(leftSense);
  if(rightLine < sensorThreshold && leftLine < sensorThreshold){
    Serial.println("both sensors white");
    //This if statement contains actions the robot should take if there is an intersection encountered
    topState = 1; //go to the next state in the top state machine
  }
  else if(leftLine < sensorThreshold){
    Serial.println("left sensor white");
    rightWheel.write(90); //nudge left
    leftWheel.write(130);
  }else if(rightLine < sensorThreshold){
    Serial.println("right sensor white");
    rightWheel.write(40); //nudge right
    leftWheel.write(90);
  }else{
    Serial.println("both sensor black");
    leftWheel.write(130);
    rightWheel.write(40);
  }
}

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


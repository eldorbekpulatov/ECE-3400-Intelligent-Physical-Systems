// the setup function runs once when you press reset or power the board
Servo rightWheel;
Servo leftWheel;
int input = A0;

void setup() {
  Serial.begin(9600);
  rightWheel.attach(11);
  leftWheel.attach(10);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  dance();
}

void dance(){
  rightWheel.write(0);
  leftWheel.write(0);
  delay(1000);
  leftWheel.write(180);
  delay(1000);
  leftWheel.write(0);
  rightWheel.write(180);
  delay(1000);
  leftWheel.write(180);
  rightWheel.write(180);
  delay(1000);
}

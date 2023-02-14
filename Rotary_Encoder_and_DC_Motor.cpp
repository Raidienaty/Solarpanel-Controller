/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-actuator
 */

// constants won't change
/*#define CLK 4
#define DT 3
#define SW 2
int correct_position = 240;
int counter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir = "";
unsigned long lastButtonPress = 0;*/
int correct_position = 240;
const int ENA_PIN = 12; // the Arduino pin connected to the EN1 pin L298N
const int IN1_PIN = 10; // the Arduino pin connected to the IN1 pin L298N
const int IN2_PIN = 11;

int temp, counter = 0;// the Arduino pin connected to the IN2 pin L298N
//const int ENB_PIN = 10; // the Arduino pin connected to the EN2 pin L298N
//const int IN3_PIN = 9; // the Arduino pin connected to the IN1 pin L298N
//const int IN4_PIN = 8; // the Arduino pin connected to the IN2 pin L298N
// the setup function runs once when you press reset or power the board
 void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
  counter++;
  }else{
  counter--;
  }
  }
   
  void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
  counter--;
  }else{
  counter++;
  }
  }

void setup() {
  // initialize digital pins as outputs.
 Serial.begin (9600);

  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 
  
  pinMode(3, INPUT_PULLUP); // internalเป็น pullup input pin 3
//Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);
   
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);// Read the initial state of CLK
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  //pinMode(ENB_PIN, OUTPUT);
  //pinMode(IN3_PIN, OUTPUT);
  //pinMode(IN4_PIN, OUTPUT);

  digitalWrite(ENA_PIN, HIGH);
  //digitalWrite(ENB_PIN, HIGH);
}

// the loop function runs over and over again forever
void loop() {
 if( counter != temp ){
  Serial.println (counter);
  temp = counter;
  }
  
   if(counter < (correct_position - 10)){
      // extend the actuator
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    }
    else if(counter > (correct_position + 10)){
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    }
    
    else{
      // retracts the actuator
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
    }

/*
    //while(counter!=correct_position){
      
        currentStateCLK = digitalRead(CLK);  // Read the current state of CLK
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1) {
    if (digitalRead(DT) != currentStateCLK) {
      counter --;
      currentDir = "CCW";
    } 
    else {
      counter ++;
      currentDir = "CW";
    }
    Serial.print("Direction: ");
    Serial.print(currentDir);
    Serial.print(" | Counter: ");
    Serial.println(counter);
  }
  lastStateCLK = currentStateCLK;
      
    if(correct_position > counter){
      // extend the actuator
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    }
    else if(correct_position == counter){
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    }
    
    else{
      // retracts the actuator
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
    }*/
//}
 /* // extend the actuator
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);

  delay(20000); // actuator will stop extending automatically when reaching the limit

  // retracts the actuator
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);

  delay(20000); // actuator will stop retracting automatically when reaching the limit*/

}

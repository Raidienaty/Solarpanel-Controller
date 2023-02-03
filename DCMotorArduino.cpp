int ENB_PIN = 4;
int IN3_PIN = 3;
int IN4_PIN = 2;
int DCA = 11;
int DCB = 10;
int targetValueDCMotorEncoder = 100;
int counter = 0; 
int aState;
int aLastState;  

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(DCA, INPUT);
  pinMode(DCB, INPUT);
  aLastState = digitalRead(DCA);
  digitalWrite(ENB_PIN, HIGH);

}

void readEncoder(){
  aState = digitalRead(DCA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (aState != aLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(DCB) != aState) { 
       counter ++;
     } else {
       counter --;
     }
     Serial.print("Position: ");
     Serial.println(counter);
   }
   else
   {
    //  Serial.print("Failed to change state!\n");
   }
   aLastState = aState; // Updates the previous state of the outputA with the current state
 }
void loop() {
  // put your main code here, to run repeatedly:
readEncoder();
  if (counter > targetValueDCMotorEncoder-4 && counter < targetValueDCMotorEncoder+4)
  {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);  
  }
  else if (counter > 300)
  {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);  
  }
  else if (counter < -300)
  {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);  
  }
  else if (counter < targetValueDCMotorEncoder)
  {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW); 
  }
  else if (counter > targetValueDCMotorEncoder)
  {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);  
  }

}

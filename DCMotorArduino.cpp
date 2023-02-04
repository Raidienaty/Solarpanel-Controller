struct DCMotor
{
    // Control Pin Variables
    const int ENB_PIN = 4;
    const int IN3_PIN = 3;
    const int IN4_PIN = 2;

    // Are we going CW or CCW?
    bool direction = 1;

    // Encoder Pin Variables
    const int ENCODER_A_PIN = 11;
    const int ENCODER_B_PIN = 10;

    // Target Encoder Count
    const int TARGET_VALUE = 100;

    // Encoder Count Variables
    int encoderCounts = 0;
    int previousState = 0;
    int currentState = 0;
} dcMotor;

// Make DC Motor go clockwise
void clockwiseMotion()
{
    digitalWrite(dcMotor.IN3_PIN, HIGH);
    digitalWrite(dcMotor.IN4_PIN, LOW);

    dcMotor.direction = 1;
}

// Make DC Motor go counterclockwise
void counterclockwiseMotion()
{
    digitalWrite(dcMotor.IN3_PIN, LOW);
    digitalWrite(dcMotor.IN4_PIN, HIGH);

    dcMotor.direction = 0;
}

// Stop all motion on motor
void stopMotion()
{
    digitalWrite(dcMotor.IN3_PIN, LOW);
    digitalWrite(dcMotor.IN4_PIN, LOW);
}

// Switch directions of travel with DC Motor
void switchMotorDirection()
{
    if (dcMotor.direction)
    {
        counterclockwiseMotion();
    }
    else
    {
        clockwiseMotion();
    }

    dcMotor.direction = !dcMotor.direction;
}

// Read the encoders and attempt to increment value
void readEncoder()
{
    // Reads the "current" state of encoder A
    dcMotor.currentState = digitalRead(dcMotor.ENCODER_A_PIN);

    // If the previous and the current state of encoder A are different, that means a Pulse has occured
    if (dcMotor.currentState != dcMotor.previousState)
    {
        // If the outputB state is different to encoder A state, that means the encoder is rotating clockwise
        if (digitalRead(dcMotor.ENCODER_B_PIN) != dcMotor.currentState)
        {
            dcMotor.encoderCounts++;
        }
        else
        {
            dcMotor.encoderCounts--;
        }
        Serial.print("Position: ");
        Serial.println(dcMotor.encoderCounts);
    }
    else
    {
        //  Serial.print("Failed to change state!\n");
    }

    // Updates the previous state of the outputA with the current state
    dcMotor.previousState = dcMotor.currentState;
}

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);

    // Set the dc motor control pins to be output mode
    pinMode(dcMotor.ENB_PIN, OUTPUT);
    pinMode(dcMotor.IN3_PIN, OUTPUT);
    pinMode(dcMotor.IN4_PIN, OUTPUT);

    // Set the encoder read pins to be input
    pinMode(dcMotor.ENCODER_A_PIN, INPUT);
    pinMode(dcMotor.ENCODER_B_PIN, INPUT);

    dcMotor.previousState = digitalRead(dcMotor.ENCODER_A_PIN);
    digitalWrite(dcMotor.ENB_PIN, HIGH);
}

void loop()
{
    // put your main code here, to run repeatedly:
    readEncoder();

    // Did we hit our target?
    if (dcMotor.encoderCounts > dcMotor.TARGET_VALUE-4 && dcMotor.encoderCounts < dcMotor.TARGET_VALUE+4)
    {
        stopMotion();
    }
    // Are we above our range?
    else if (dcMotor.encoderCounts > 300 || dcMotor.encoderCounts < -300)
    {
        switchMotorDirection();  
    }
    else if (dcMotor.encoderCounts < dcMotor.TARGET_VALUE)
    {
        clockwiseMotion();
    }
    else if (dcMotor.encoderCounts > dcMotor.TARGET_VALUE)
    {
        counterclockwiseMotion();
    }
}
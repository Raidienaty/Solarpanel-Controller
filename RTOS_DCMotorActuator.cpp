#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

const int ENA_PIN = 7;
const int IN1_PIN = 6;
const int IN2_PIN = 5;

int targetValuePitch = 30;

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

TaskHandle_t Task1Handle;
TaskHandle_t Task2Handle;

// ********************************************************************************
//                               Actuator Functions
// ********************************************************************************

void retractActuator() 
{
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
}

void stopActuator()
{
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
}

void extendActuator()
{
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
}

// ********************************************************************************
//                               Actuator Functions
// ********************************************************************************

// ********************************************************************************
//                               DCMotor Functions
// ********************************************************************************

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

// ********************************************************************************
//                               DCMotor Functions
// ********************************************************************************

void Actuator(void *pvParameters)
{
    while (true)
    {
        // Read normalized values 
        Vector normAccel = mpu.readNormalizeAccel();

        // Calculate Pitch & Roll
        int pitchDegrees = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
        int rollDegrees = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

        // Output
        Serial.print(" Pitch = ");
        Serial.print(pitch);
        Serial.print(" Roll = ");
        Serial.print(roll);

        Serial.println();

        delay(10);

        if (pitch < targetValuePitch - 3)
        {
            extendActuator();
        }
        else if (pitch > targetValuePitch + 3)
        {
            retractActuator();
        }
        else
        {
            stopActuator();
        }

        delay(10);
    }
}

void DCMotor(void *pvParameters)
{
    while (true)
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
}

void setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    // ********************************************************************************
    //                                 Actuator Setup
    // ********************************************************************************

    // Setup pin outputs for actuator
    pinMode(ENA_PIN, OUTPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);

    digitalWrite(ENA_PIN, HIGH);

    Serial.println("Initialize MPU6050");

    // Confirm we have connection to the board
    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
    {
        Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        delay(500);
    }

    // ********************************************************************************
    //                                 Actuator Setup
    // ********************************************************************************

    // ********************************************************************************
    //                                 DCMotor Setup
    // ********************************************************************************

    // Set the dc motor control pins to be output mode
    pinMode(dcMotor.ENB_PIN, OUTPUT);
    pinMode(dcMotor.IN3_PIN, OUTPUT);
    pinMode(dcMotor.IN4_PIN, OUTPUT);

    // Set the encoder read pins to be input
    pinMode(dcMotor.ENCODER_A_PIN, INPUT);
    pinMode(dcMotor.ENCODER_B_PIN, INPUT);

    dcMotor.previousState = digitalRead(dcMotor.ENCODER_A_PIN);
    digitalWrite(dcMotor.ENB_PIN, HIGH);

    // ********************************************************************************
    //                                 DCMotor Setup
    // ********************************************************************************

    xTaskCreate(Actuator, "Actuator", 128, NULL, 1, &Actuator);
    xTaskCreate(DCMotor, "DCMotor", 128, NULL, 1, &DCMotor);
}

void loop()
{
    vTaskDelay(portMAX_DELAY);
}

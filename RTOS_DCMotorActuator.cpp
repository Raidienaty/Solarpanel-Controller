#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <MPU6050.h>
#include <string>

MPU6050 mpu;

// ********************************************************************************
//                               Rotary Encoder Functions
// ********************************************************************************

#define CLK 12
#define DT 13

int counter = 0;
int currentStateCLK;
int lastStateCLK;
std::string currentDir ="";

// ********************************************************************************
//                               Rotary Encoder Functions
// ********************************************************************************


// ********************************************************************************
//                               Actuator Pinout
// ********************************************************************************

const int ENA_PIN = 7;
const int IN1_PIN = 6;
const int IN2_PIN = 5;

// ********************************************************************************
//                               Actuator Pinout
// ********************************************************************************

int targetValuePitch = Elevation;

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
    int TARGET_VALUE = Azimuth;

    // Encoder Count Variables
    int encoderCounts = 0;
    int previousState = 0;
    int currentState = 0;
} dcMotor;

TaskHandle_t Actuator;
TaskHandle_t DCMotor;
TaskHandle_t ReceiveGoals;

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

int calculatePitchDegrees(Vector normAccel)
{
    return -1 * (
        atan2(
            normAccel.XAxis, 
            sqrt(
                normAccel.YAxis*normAccel.YAxis 
                + normAccel.ZAxis*normAccel.ZAxis
            )
        )
        * 180.0
    ) / M_PI;
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
   // Read the current state of CLK
	currentStateCLK = digitalRead(CLK);

	// If last and current state of CLK are different, then pulse occurred
	// React to only 1 state change to avoid double count
	if (currentStateCLK != lastStateCLK  && currentStateCLK == 1)
    {
		// If the DT state is different than the CLK state then
		// the encoder is rotating CCW so decrement
		if (digitalRead(DT) != currentStateCLK)
        {
			counter--;
			currentDir = "CCW";
		}
        else
        {
			// Encoder is rotating CW so increment
			counter++;
			currentDir = "CW";
		}

		Serial.print("Direction: ");
		Serial.print(currentDir);
		Serial.print(" | Counter: ");
		Serial.println(counter);
	}

	// Remember last CLK state
	lastStateCLK = currentStateCLK;
}

// ********************************************************************************
//                               Actuator Functions
// ********************************************************************************

void Actuator(void *pvParameters)
{
    while (true)
    {
        // Read normalized values 
        Vector normAccel = mpu.readNormalizeAccel();

        // Calculate Pitch & Roll
        int pitchDegrees = calculatePitchDegrees(normAccel);

        // Output
        Serial.print(" Pitch = ");
        Serial.print(pitchDegrees);

        Serial.println();

        delay(10);

        if (pitchDegrees < targetValuePitch - 3)
        {
            extendActuator();
        }
        else if (pitchDegrees > targetValuePitch + 3)
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

// ********************************************************************************
//                               Actuator Functions
// ********************************************************************************

// ********************************************************************************
//                               Receive Data Functions
// ********************************************************************************

void ReceiveGoals()
{
    if (Serial1.available()) 
    {
        // Allocate the JSON document
        // This one must be bigger than the sender's because it must store the strings
        StaticJsonDocument<300> doc;

        // Read the JSON document from the "link" serial port
        DeserializationError err = deserializeJson(doc, Serial1);

        if (err == DeserializationError::Ok) 
        {
            // Print the values
            // (we must use as<T>() to resolve the ambiguity)
            Serial.print("Elevation = ");
            Serial.println(doc["Elevation"].as<int>());
            Serial.print("Azimuth = ");
            Serial.println(doc["Azimuth"].as<int>());
            int Elevation = (doc["Elevation"].as<int>());
            int Azimuth = (doc["Azimuth"].as<int>());
        } 
        else 
        {
            // Print error to the "debug" serial port
            Serial.print("deserializeJson() returned ");
            Serial.println(err.c_str());
        
            // Flush all bytes in the "link" serial port buffer
            while (Serial1.available() > 0)
            {
                Serial1.read();
            }
        }
    }
}

// ********************************************************************************
//                               Receive Data Functions
// ********************************************************************************

// ********************************************************************************
//                               DC Motor Functions
// ********************************************************************************

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

// ********************************************************************************
//                               DC Motor Functions
// ********************************************************************************

void setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    while (!Serial) continue;

    // Initialize the "link" serial port
    // Use a low data rate to reduce the error ratio
    Serial1.begin(9600);

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

    // ********************************************************************************
    //                                Rotary Encoder Setup
    // ********************************************************************************

    pinMode(CLK, INPUT);
    pinMode(DT, INPUT);

    // ********************************************************************************
    //                                Rotary Encoder Setup
    // ********************************************************************************

    xTaskCreate(Actuator, "Actuator", 128, NULL, 1, &Actuator);
    xTaskCreate(DCMotor, "DCMotor", 128, NULL, 1, &DCMotor);
    xTaskCreate(ReceiveGoals, "ReceiveGoals", 128, NULL, 1, &ReceiveGoals)
}

void loop()
{
    vTaskDelay(portMAX_DELAY);
}
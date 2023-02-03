#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
/* Firgelli Automations
 * Limited or no support: we do not have the resources for Arduino code support
 * 
 * Program demos how a motor driver controls direction & speed of a linear actuator 
 */
 
const int ENA_PIN = 7;
const int IN1_PIN = 6;
const int IN2_PIN = 5;

int targetValuePitch = 30;

void setup() 
{
    Serial.begin(115200);

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
}
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


void loop()
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
// Wind Rate Variables
const int MAX_WIND_RATE_KNOTS = 35;
const int MIN_WIND_RETURN_RATE_KNOTS = 30;
int windRateCounter = 0;
bool flatModeTriggered = 0;

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float getWindSpeed()
{
    float sensorValue = analogRead(A0);
    // Serial.print("Analog Value =");
    // Serial.println(sensorValue);

    float voltage = (sensorValue / 1024) * 5; //Arduino ADC resolution 0-1023
    // Serial.print("Voltage =");
    // Serial.print(voltage);
    // Serial.println(" V");

    return mapfloat(voltage, 0.4, 2, 0, 32.4);
}

void sendFlatMode()
{
    // send signal to actuator to go flat
}

void sendNormalMode()
{
    // send signal to go back to normal operations
}

void loop()
{
    float windSpeed = 0.0f;
    float speedKnots = 0.0f;

    windSpeed = getWindSpeed();

    speedKnots = windSpeed * 1.943844;

    // if we're currently in recovery mode or not
    if (flatModeTriggered)
    {
        // if we are at our 10m passed at wind rate, return to normal
        if (windRateCounter >= 10)
        {
            windRateCounter = 0;
            flatModeTriggered = false;
            sendNormalMode();
        }
        // if we're currently above the wind rate, log it and delay for 1s
        else if (speedKnots < MIN_WIND_RETURN_RATE_KNOTS)
        {
            windRateCounter++;
            delay(60000);
        }
    }
    else
    {
        // if we are at our 20s passed at max wind rate, trigger flatmode
        if (windRateCounter >= 20)
        {
            windRateCounter = 0;
            flatModeTriggered = true;
            sendFlatMode();
        }
        // if we're currently above the wind rate, log it and delay for 1s
        else if (speedKnots > MAX_WIND_RATE_KNOTS)
        {
            windRateCounter++;
            delay(1000);
        }
    }
}
// <30 10m 
// >35 20s
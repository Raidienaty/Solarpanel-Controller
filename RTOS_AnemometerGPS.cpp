#include <Arduino_FreeRTOS.h>
#include <semphr.h>

// Title: RTOS_AnemometerGPS.cpp
// Purpose:
//  Measures current wind speed. If wind speed is above a maximum rate, sends signal
//  to rotate panel. Once in recovery mode, will only rotate based on sub-rate wind
//  speeds held for 10 minutes.

#define ANEMOMETER_PIN A0

// Wind Rate Variables
const int MAX_WIND_RATE_KNOTS = 35;
const int MIN_WIND_RETURN_RATE_KNOTS = 30;
int windRateCounter = 0;
bool flatModeTriggered = 0;

TaskHandle_t GPSHandle;
TaskHandle_t AnemometerHandle;

void GPS(void *pvParameters)
{
    while(true)
    {
        // GPS
    }
}

void Anemometer(void *pvParameters)
{
    while(true)
    {
        // Anemometer
    }
}

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);

    Serial.println("Anemometer is starting up...");
    Serial.println("GPS is starting up...");

    // ********************************************************************************
    //                                 GPS Setup Code
    // ********************************************************************************

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // the parser doesn't care about other sentences at this time
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz

    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);

    delay(1000);

    // Ask for firmware version
    GPSSerial.println(PMTK_Q_RELEASE);

    // ********************************************************************************
    //                                 GPS Setup Code
    // ********************************************************************************

    pinMode(LED_BUILTIN, OUTPUT);

    xTaskCreate(GPS, "GPSTask", 128, NULL, 1, &GPSHandle);
    xTaskCreate(Anemometer, "AnemometerTask", 128, NULL, 1, &AnemometerHandle);
}

void loop()
{
    vTaskDelay(portMAX_DELAY);
}

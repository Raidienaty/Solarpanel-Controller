#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Adafruit_GPS.h>
#include <ArduinoJson.h>
// Title: RTOS_AnemometerGPS.cpp
// Purpose:
//  Measures current wind speed. If wind speed is above a maximum rate, sends signal
//  to rotate panel. Once in recovery mode, will only rotate based on sub-rate wind
//  speeds held for 10 minutes.
//  GPS sends longitude and latitude to the Arduino MEGA which then calculates the
//  Suns elevation and azimuth which are then sent to the other Arduino MEGA 
#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false
#define ANEMOMETER_PIN A0
uint32_t timer = millis();

double rad_to_deg = 180/PI;
double fract_year_rad = 0;
double fract_year_deg = 0;
double day = 0;
double hour = 0;
double minute = 0;
double sec = 0;
double long_;
//double long_rad = long_ / rad_to_deg;
double lat_deg;
double lat_rad;
double eqtime = 0;
double decl_rad = 0;
double decl_deg = 0;
double off_set = 0;
double true_solar_time = 0;
double time_zone = -5;
double Solar_Hour_Angle_deg = 0;
double Solar_Hour_Angle_rad = 0;
double Zenith_rad = 0;
double Zenith_deg = 0;
double Elevation_deg = 0;
double Elevation_rad = 0;
double Azimuth_rad = 0;
double Azimuth_deg = 0;
// Wind Rate Variables
const int MAX_WIND_RATE_KNOTS = 35;
const int MIN_WIND_RETURN_RATE_KNOTS = 30;
int windRateCounter = 0;
bool flatModeTriggered = 0;

TaskHandle_t GLOBALHandle;
TaskHandle_t AnemometerHandle;
TaskHandle_t SendGoalsHandle;

// ********************************************************************************
//                               Anemometer Functions
// ********************************************************************************

// Sends signal to actuator arduino to go into emergency mode
void sendFlatMode()
{
    // send signal to actuator to go flat
}

// Sends signal to actuator arduino to return to normal operation
void sendNormalMode()
{
    // send signal to go back to normal operations
}

// Math stuff I don't understand
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Reads anemometer and converts to float value speed
float getWindSpeed()
{
    float sensorValue = analogRead(ANEMOMETER_PIN);
    // Serial.print("Analog Value =");
    // Serial.println(sensorValue);

    float voltage = (sensorValue / 1024) * 5; //Arduino ADC resolution 0-1023
    // Serial.print("Voltage =");
    // Serial.print(voltage);
    // Serial.println(" V");

    return mapfloat(voltage, 0.4, 2, 0, 32.4);
}

// ********************************************************************************
//                               Anemometer Functions
// ********************************************************************************

// ********************************************************************************
//                                  GPS Functions
// ********************************************************************************

int calculateDayOfYear(int day, int month, int year)
{
    // Given a day, month, and year (4 digit), returns 
    // the day of year. Errors return 999.

    int daysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};

    // Verify we got a 4-digit year
    if (year < 1000)
    {
        return 999;
    }

    // Check if it is a leap year, this is confusing business
    // See: https://support.microsoft.com/en-us/kb/214019
    if (year % 4  == 0)
    {
        if (year % 100 != 0)
        {
            daysInMonth[1] = 29;
        }
        else
        {
            if (year % 400 == 0)
            {
                daysInMonth[1] = 29;
            }
        }
    }

    // Make sure we are on a valid day of the month
    if (day < 1) 
    {
        return 999;
    } 
    else if (day > daysInMonth[month-1])
    {
        return 999;
    }

    int doy = 0;
    for (int i = 0; i < month - 1; i++)
    {
        doy += daysInMonth[i];
    }

    doy += day;
    return doy;
}

// ********************************************************************************
//                                  GPS Functions
// ********************************************************************************

void GLOBAL(void *pvParameters)
{
    while (true)
    {
        // read data from the GPS in the 'main loop'
        char gpsInformation = GPS.read();
        // if you want to debug, this is a good time to do it!
        if (GPSECHO)
        {
            if (gpsInformation)
            {
                Serial.print(gpsInformation);
            }
        }

        // if a sentence is received, we can check the checksum, parse it...
        if (GPS.newNMEAreceived()) 
        {
            // a tricky thing here is if we print the NMEA sentence, or data
            // we end up not listening and catching other sentences!
            // so be very wary if using OUTPUT_ALLDATA and trying to print out data
            if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
            {
                return; // we can fail to parse a sentence in which case we should just wait for another
            }
        }

        // approximately every 2 seconds or so, print out the current stats
        if (millis() - timer > 2000)
        {
            timer = millis(); // reset the timer

            Serial.print("\nTime: ");

            if (GPS.hour < 10)
            {
                Serial.print('0');
            }
            Serial.print(GPS.hour, DEC); Serial.print(':');
            
            if (GPS.minute < 10)
            {
                Serial.print('0');
            }
            
            Serial.print(GPS.minute, DEC); Serial.print(':');
            
            if (GPS.seconds < 10)
            {
                Serial.print('0');
            }
            
            Serial.print(GPS.seconds, DEC); Serial.print('.');
            
            if (GPS.milliseconds < 10) {
                Serial.print("00");
            }
            else if (GPS.milliseconds > 9 && GPS.milliseconds < 100)
            {
                Serial.print("0");
            }
            
            Serial.println(GPS.milliseconds);
            Serial.print("Date: ");
            Serial.print(GPS.day, DEC); Serial.print('/');
            Serial.print(GPS.month, DEC); Serial.print("/20");
            Serial.println(GPS.year, DEC);
            Serial.print("Fix: "); Serial.print((int)GPS.fix);
            Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
            
            if (GPS.fix)
            {
                Serial.print("Location: ");
                Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
                Serial.print(", ");
                Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
                Serial.println("Location in Degrees");
                Serial.print(GPS.latitudeDegrees, 8);
                Serial.print(", ");
                Serial.println(GPS.longitudeDegrees, 8);
                Serial.print("Speed (knots): "); Serial.println(GPS.speed);
                Serial.print("Angle: "); Serial.println(GPS.angle);
                Serial.print("Altitude: "); Serial.println(GPS.altitude);
                Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
                Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
            }

            day = calculateDayOfYear(GPS.day, GPS.month, GPS.year+2000);
            hour = GPS.hour + time_zone;
                
            fract_year_rad = ((2*PI)/365)*(day - 1 + ((hour-12)/24));

            fract_year_deg = fract_year_rad * rad_to_deg;

            eqtime = 229.18*(.000075 + .001868*cos(fract_year_rad) - .032077*sin(fract_year_rad)
            - .014615*cos(2*fract_year_rad) - .040849*sin(2*fract_year_rad));

            printf("eqtime: %f \n", eqtime);

            decl_rad = 0.006918 - 0.399912*cos(fract_year_rad) + 0.070257*sin(fract_year_rad) 
            - 0.006758*cos(2*fract_year_rad) + 0.000907*sin(2*fract_year_rad)
            - 0.002697*cos(3*fract_year_rad) + 0.00148*sin(3*fract_year_rad);

            printf("decl_rad: %f \n", decl_rad);

            decl_deg = decl_rad * rad_to_deg;

            printf("decl_deg: %f \n", decl_deg);

            long_ = GPS.longitudeDegrees;
            lat_deg = GPS.latitudeDegrees;
            lat_rad = lat_deg / rad_to_deg;

            off_set = eqtime + (4*long_) - (60*time_zone);

            minute = GPS.minute;

            sec = GPS.seconds;

            true_solar_time = GPS.hour*60 + minute + (sec/60) + off_set;

            Solar_Hour_Angle_deg = (true_solar_time/4)-180;

            Solar_Hour_Angle_rad = (Solar_Hour_Angle_deg)/rad_to_deg;

            Zenith_rad = acos(sin(lat_rad)*sin(decl_rad) + cos(lat_rad)*cos(decl_rad)*cos(Solar_Hour_Angle_rad));

            Zenith_deg = Zenith_rad*rad_to_deg;

            Elevation_deg = 90 - Zenith_deg;

            Serial.print("Elevation_deg: ");
            Serial.println(Elevation_deg);

            Elevation_rad = Elevation_deg / rad_to_deg;

            Azimuth_rad = -(acos(-((sin(lat_rad)*cos(Zenith_rad)-sin(decl_rad))/(cos(lat_rad)*sin(Zenith_rad))))) + (2*PI);

            Azimuth_deg = (Azimuth_rad * rad_to_deg);

            Serial.print("Azimuth_deg: ");
            Serial.println(Azimuth_deg);
        }
    }
}

void Anemometer(void *pvParameters)
{
    while (true)
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
}

void SendGoals()
{
      // Values we want to transmit
  int Elevation = Elevation_deg();
  int Azimuth = Azimuth_deg();

  // Print the values on the "debug" serial port
  Serial.print("Elevation = ");
  Serial.println(Elevation);
  Serial.print("Azimuth = ");
  Serial.println(Azimuth);
  Serial.println("---");

  // Create the JSON document
  StaticJsonDocument<200> doc;
  doc["Elevation"] = Elevation;
  doc["Azimuth"] = Azimuth;

  // Send the JSON document over the "link" serial port
  serializeJson(doc, Serial1);

  // Wait
  delay(5000);
}
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    while (!Serial) continue;

     // Initialize the "link" serial port
     // Use a low data rate to reduce the error ratio
    Serial1.begin(9600);
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

    xTaskCreate(GLOBAL, "GLOBALTask", 128, NULL, 1, &GLOBALHandle);
    xTaskCreate(Anemometer, "AnemometerTask", 128, NULL, 1, &AnemometerHandle);
    xTaskCreate(SendGoals, "SendGoalsTask", 128, NULL, 1, &SendGoalsHandle);
}

void loop()
{
    vTaskDelay(portMAX_DELAY);
}

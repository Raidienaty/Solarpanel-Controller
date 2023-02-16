
#include <Adafruit_GPS.h>

// what's the name of the hardware serial port?
#define GPSSerial Serial2
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

const float pi = 3.14159265358979323846;
const float rad = pi/180.0;
const float deg = 180.0/pi;
uint32_t timer = millis();
    double day = 0;
    double hour = 0;
    double minute = 0;
    double sec = 0;
// Set your location here, using longitude and latitude
float longitude;
float latitude;

// Calculate the Julian day number
unsigned int calculateJulianDay(int year, int month, int day)
{
  if (month <= 2)
  {
    year -= 1;
    month += 12;
  }
  int A = year / 100;
  int B = 2 - A + A / 4;
  unsigned int jd = floor(365.25 * (year + 4716)) + floor(30.6001 * (month + 1)) + day + B - 1524.5;
  return jd;
}

// Calculate the Julian century
float calculateJulianCentury(unsigned int jd)
{
  return (jd - 2451545.0) / 36525.0;
}

// Calculate the mean solar noon
float calculateMeanSolarNoon(float julianCentury)
{
  return 2451545.0009 + julianCentury * 36525.0;
}

// Calculate the mean longitude of the sun
float calculateMeanLongitude(float julianCentury)
{
  float L0 = 280.46645 + julianCentury * (36000.76983 + julianCentury * 0.0003032);
  while (L0 > 360.0)
  {
    L0 -= 360.0;
  }
  while (L0 < 0.0)
  {
    L0 += 360.0;
  }
  return L0;
}

// Calculate the mean anomaly of the sun
float calculateMeanAnomaly(float julianCentury)
{
  float M = 357.52910 + julianCentury * (35999.05030 - 0.0001559 * julianCentury);
  return M;
}

// Calculate the equation of center
float calculateEquationOfCenter(float julianCentury, float meanAnomaly)
{
  float C = (1.914600 - julianCentury * (0.004817 + 0.000014 * julianCentury)) * sin(meanAnomaly * rad);
  C += (0.019993 - 0.000101 * julianCentury) * sin(2 * meanAnomaly * rad);
  C += 0.000290 * sin(3 * meanAnomaly * rad);
  return C;
}

// Calculate the ecliptic longitude
float calculateEclipticLongitude(float meanLongitude, float equationOfCenter)
{
  float L = meanLongitude + equationOfCenter;
  return L;
}

// Calculate the obliquity of the ecliptic
float calculateObliquityOfEcliptic(float julianCentury)
{
  float epsilon0 = 23.439291111;
  float omega = 125.04 - 1934.136 * julianCentury;
  float epsilon = epsilon0 + 0.00256 * cos(omega * rad);
  return epsilon;
}

// Convert the ecliptic coordinates to equatorial coordinates
void convertEclipticToEquatorial(float eclipticLongitude, float obliquityOfEcliptic, float &rightAscension, float &declination)
{
  float Y = sin(eclipticLongitude * rad) * cos(obliquityOfEcliptic * rad);
  float X = cos(eclipticLongitude * rad);
  float alpha = atan2(Y, X) * deg;
  while (alpha < 0.0)
  {
    alpha += 360.0;
  }
  while (alpha > 360.0)
  {
    alpha -= 360.0;
  }
  rightAscension = alpha;

  float delta = asin(sin(eclipticLongitude * rad) * sin(obliquityOfEcliptic * rad)) * deg;
  return delta;
}

// Convert the equatorial coordinates to horizontal coordinates
void convertEquatorialToHorizontal(float rightAscension, float declination, float latitude, float &localHourAngle, float &elevation, float &azimuth)
{
  float hourAngle = (GPS.hour + GPS.minute / 60.0 + GPS.seconds / 3600.0) * 15.0 - rightAscension;
  while (hourAngle < -180.0)
  {
    hourAngle += 360.0;
  }
  while (hourAngle > 180.0)
  {
    hourAngle -= 360.0;
  }
  localHourAngle = hourAngle;

  float H = localHourAngle * rad;
  float lat = latitude * rad;
  float delta = declination * rad;
  float sinElevation = sin(lat) * sin(delta) + cos(lat) * cos(delta) * cos(H);
  elevation = asin(sinElevation) * deg;

  float cosAzimuth = (sin(delta) - sin(lat) * sinElevation) / (cos(lat) * cos(elevation * rad));
  azimuth = acos(cosAzimuth) * deg;
  if (sin(H) > 0.0)
  {
    azimuth = 360.0 - azimuth;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic parsing test!");
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
  GPSSerial.println(PMTK_Q_RELEASE);;
}

int calculateDayOfYear(int day, int month, int year){
    int daysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};

    // Verify we got a 4-digit year
    if (year < 1000 || year > 9999) // Check if the year is not 4-digit.
    {
        return 999;
    }

    // Check if it is a leap year
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
    if (day < 1 || day > daysInMonth[month-1]) // Check if the day is not valid.
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

void loop() {
  int year = GPS.year+2000; // Get the current year
  int month = GPS.month; // Get the current month
  int day = calculateDayOfYear(GPS.day, month, year);

  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
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
    if (GPS.fix) {
      latitude = GPS.latitudeDegrees;
      longitude = GPS.longitudeDegrees;
    }
  }

 unsigned int julianDay = calculateJulianDay(year, month, day); // Calculate the Julian day number
float julianCentury = calculateJulianCentury(julianDay); // Calculate the Julian century
float meanSolarNoon = calculateMeanSolarNoon(julianCentury); // Calculate the mean solar noon
float meanLongitude = calculateMeanLongitude(julianCentury); // Calculate the mean longitude of the sun
float meanAnomaly = calculateMeanAnomaly(julianCentury); // Calculate the mean anomaly of the sun
float equationOfCenter = calculateEquationOfCenter(julianCentury, meanAnomaly); // Calculate the equation of center
float eclipticLongitude = calculateEclipticLongitude(meanLongitude, equationOfCenter); // Calculate the ecliptic longitude
float obliquityOfEcliptic = calculateObliquityOfEcliptic(julianCentury); // Calculate the obliquity of the ecliptic
float rightAscension, declination;
convertEclipticToEquatorial(eclipticLongitude, obliquityOfEcliptic, rightAscension, declination); // Convert the ecliptic coordinates to equatorial coordinates
float localHourAngle, elevation, azimuth;
convertEquatorialToHorizontal(rightAscension, declination, latitude, localHourAngle, elevation, azimuth); // Convert the equatorial coordinates to horizontal coordinates
Serial.print("The sun's elevation is ");
Serial.print(elevation);
Serial.print(" degrees and its azimuth is ");
Serial.print(azimuth);
Serial.println(" degrees.");
delay(5000);
}

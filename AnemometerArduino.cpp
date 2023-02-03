void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}
void Anemometer()
{float sensorValue = analogRead(A0);
  Serial.print("Analog Value =");
  Serial.println(sensorValue);
 
  float voltage = (sensorValue / 1024) * 5; //Arduino ADC resolution 0-1023
  Serial.print("Voltage =");
  Serial.print(voltage);
  Serial.println(" V");
 
  float wind_speed = mapfloat(voltage, 0.4, 2, 0, 32.4);
  float speed_mph = ((wind_speed *3600)/1609.344);
  float speed_knot = ((wind_speed * 1.943844));
  Serial.print("Wind Speed =");
  Serial.print(wind_speed);
  Serial.println("m/s");
  Serial.print(speed_mph);
  Serial.println("mph");
  Serial.print(speed_knot);
  Serial.println("knots");

  Serial.println(" ");
  delay(1000);
}
void loop() {
  // put your main code here, to run repeatedly:
Anemometer();
}
float mapfloat([float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
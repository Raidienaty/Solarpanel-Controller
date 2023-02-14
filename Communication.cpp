const int TX_PIN = 0;
const int RX_PIN = 0;

void setup()
{
    Serial.begin(115200);
    pinMode(TX_PIN, OUTPUT);
    pinMode(RX_PIN, INPUT);
}

void loop()
{
    int readValue = digitalRead(TX_PIN);  

    if (readValue == LOW)
    {
        Serial.print("Read low value!");
        

        Serial.print("Writing high value!");
        digitalWrite(TX_PIN, HIGH);
    } 
    else
    {
        Serial.print("Reading high value!");

        Serial.print("Writing low value!");
        digitalWrite(RX_PIN, LOW);
    }
    delay(10);       
}
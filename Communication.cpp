// rx end for receiving transmission and echoing result to tx
// With serial input either from USB channel or BT channel
//    Receive with start and end markers combined with parsing x,y

#include <SoftwareSerial.h>

SoftwareSerial BTSerial(15, 14); // RX, TX

byte id = 'r';    // initialise to tx or rx as desired (one for each device)

void setup()
{
    Serial.begin(115200);
    BTSerial.begin(115200);
}

void loop()
{
    while(BTSerial.available())
    {
        if (id == 'r')
        {
            int readValue = (int)BTSerial.read();

            if (readValue == -1)
            {
                Serial.println("Failed to read value!");
            }

            Serial.print("Read: ");
            Serial.println(readValue);
        }
        else if (id == 't')
        {
            int writeValue = 10;

            int valueWritten = BTSerial.write(writeValue);

            if (valueWritten != writeValue)
            {
                Serial.print("Possibly failed to write value! Written value: ");
                Serial.println(valueWritten);
            }
        }
    }
}
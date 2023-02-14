#include <Arduino_FreeRTOS.h>
#include <semphr.h>

TaskHandle_t Task1Handle;
TaskHandle_t Task2Handle;

void Task1(void *pvParameters)
{
  for (;;)
  {
    // Task 1 code here
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }
}

void Task2(void *pvParameters)
{
  for (;;)
  {
    // Task 2 code here
    Serial.println("Task 2 Running");
    delay(2000);
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  xTaskCreate(Task1, "Task1", 128, NULL, 1, &Task1Handle);
  xTaskCreate(Task2, "Task2", 128, NULL, 1, &Task2Handle);
}

void loop()
{
  vTaskDelay(portMAX_DELAY);
}

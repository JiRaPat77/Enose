#include <Adafruit_AHTX0.h>
#include <ArduinoJson.h>  // เพิ่มไลบรารี ArduinoJson

Adafruit_AHTX0 aht;
sensors_event_t humidity, temp;

unsigned long previousMillis = 0;
const long interval = 60000*10; 


void setup() {
  Serial.begin(115200);
  if (!aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }
  Serial.println();
  Serial.println("Starting sensor readings...");
}

void loop() {
  unsigned long currentMillis = millis(); 

  
  if (currentMillis - previousMillis >= interval) {
  
    previousMillis = currentMillis; 

    aht.getEvent(&humidity, &temp);

    
    StaticJsonDocument<256> doc;

   
    doc["temperature"] = temp.temperature;
    doc["humidity"] = humidity.relative_humidity;

  
    serializeJson(doc, Serial);
    Serial.println(); 
  }
  

}

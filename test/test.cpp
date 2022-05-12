#include <Arduino.h>
#include <unity.h>
#include <ESP8266HTTPClient.h>
void testCall()
{
    HTTPClient http;
    WiFiClient client;
    String url = "http://203.150.243.87/";
  
    http.begin(client, url);
    int httpCode = http.GET();
    if (httpCode == 200)
    {
        String content = http.getString();
        Serial.println(content);            
        
    }

    http.end();
}
void setup()
{

    Serial.begin(115200);
    pinMode(2, OUTPUT);
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via  Serial.DTR/RTS
    delay(2000);

    UNITY_BEGIN();
    // RUN_TEST(printdata);
    // RUN_TEST(createFile);
    // RUN_TEST(countfile);
    // RUN_TEST(testList);
    RUN_TEST(testCall);
    UNITY_END();
}

void loop()
{
    digitalWrite(2, HIGH);
    delay(200);
    digitalWrite(2, LOW);
    delay(200);
}

#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino.h>

const char* ssid = "YIWEI_Laptop";
const char* password = "1234567890";

//Your Domain name with URL path or IP address with path
//For server name , could we change it into another node?
//
String serverName = "http://146.169.169.105:8000/datastream";

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
//Below is the timer
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;  
// Set timer to 5 seconds (5000)
//Below is the timer
//Below is the timer 
unsigned long timerDelay = 5000;

void setup() {
  Serial.begin(115200); 

  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
 
  //Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");
}

void loop() {
  //Send an HTTP POST request every 10 minutes
  //if ((millis() - lastTime) > timerDelay) {
    //Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED){
      HTTPClient http;
      
      //String serverPath = serverName + "?Mode=W";
      //How can I get the data in the way like https://randomnerdtutorials.com/esp32-http-get-post-arduino/
      String serverPath = serverName;
      // Your Domain name with URL path or IP address with path
      http.begin(serverPath.c_str());
      Serial.println(serverPath.c_str());//Test what is input
      // Send HTTP GET request
      int httpResponseCode = http.GET();
      
      if (httpResponseCode>0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();
    }
    else {
      Serial.println("WiFi Disconnected");
    }
    delay(600000);//Adding delay to show the content whether is HTML
    //lastTime = millis();
  //}
}
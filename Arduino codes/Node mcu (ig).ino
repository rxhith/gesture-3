

#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
//#include <FirebaseESP8266.h> // Adjust the library name for ESP32 if necessary
#include <Firebase_ESP_Client.h>

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "Wifi_top_floor"
#define WIFI_PASSWORD "87654321"
// Insert Firebase project API Key
#define API_KEY "AIzaSyBH73I4JZngWn34khOlLCtASl1xBQ50if8"

// Insert RTDB URL
#define DATABASE_URL "https://fireeeeee-2fd33-default-rtdb.firebaseio.com/"

// Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;

bool signupOK = false;

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  
  /* Assign the api key (required) */
  config.api_key = API_KEY;
  
  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;
  
  /* Sign up */


#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
//#include <FirebaseESP8266.h> // Adjust the library name for ESP32 if necessary
#include <Firebase_ESP_Client.h>

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "Wifi_top_floor"
#define WIFI_PASSWORD "87654321"
// Insert Firebase project API Key
#define API_KEY "AIzaSyBH73I4JZngWn34khOlLCtASl1xBQ50if8"

// Insert RTDB URL
#define DATABASE_URL "https://fireeeeee-2fd33-default-rtdb.firebaseio.com/"

// Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;

bool signupOK = false;

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  
  /* Assign the api key (required) */
  config.api_key = API_KEY;
  
  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;
  
  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("Sign up successful");
    signupOK = true;
  }
  else{
    Serial.printf("Sign up failed. Reason: %s\n", config.signer.signupError.message.c_str());
  }
  
  /* Assign the callback function for the long-running token generation task */
  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  
    String receivedData = Serial.readStringUntil('\n');
//    String receivedData = Serial.read();
    
    // Process the received data
     Serial.println("Received data: " + String(receivedData));
    
    // Split the received data by commas
    int commaIndex1 = receivedData.indexOf(',');
    int commaIndex2 = receivedData.indexOf(',', commaIndex1 + 1);
    
    // Extract the latitude, longitude, and card value from the received data
    int flexValue1 = receivedData.substring(0, commaIndex1).toInt();
    int flexValue2 = receivedData.substring(commaIndex1 + 1, commaIndex2).toInt();
    int flexValue3 = receivedData.substring(commaIndex2 + 1).toInt();

    
    // Print the extracted values
//    Serial.print(flexValue1);
//    Serial.print(",");
//    Serial.print(flexValue2);
//    Serial.print(",");
//    Serial.println(flexValue3);
    
    if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0)){
      sendDataPrevMillis = millis();
      
      // Write an Int number to the database path "test/flexValue1"
      if (Firebase.RTDB.setInt(&fbdo, "test/string", flexValue1)){
        Serial.println("Latitude write successful");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      }
      else {
        Serial.println("Latitude write failed");
        Serial.println("REASON: " + fbdo.errorReason());
      }
      
      // Write an Int number to the database path "test/flexValue2"
      if (Firebase.RTDB.setInt(&fbdo, "test/string2", flexValue2)){
        Serial.println("Longitude write successful");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      }
      else {
        Serial.println("Longitude write failed");
        Serial.println("REASON: " + fbdo.errorReason());
      }
      
      // Write an Int number to the database path "test/flexValue3"
      if (Firebase.RTDB.setInt(&fbdo, "test/string3", flexValue3)){
        Serial.println("Card value write successful");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      }
      else {
        Serial.println("Card value write failed");
        Serial.println("REASON: " + fbdo.errorReason());
      }
    }
  
  delay(100);
}￼Enter  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("Sign up successful");
    signupOK = true;
  }
  else{
    Serial.printf("Sign up failed. Reason: %s\n", config.signer.signupError.message.c_str());
  }
  
  /* Assign the callback function for the long-running token generation task */
  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  
    String receivedData = Serial.readStringUntil('\n');
//    String receivedData = Serial.read();
    
    // Process the received data
     Serial.println("Received data: " + String(receivedData));
    
    // Split the received data by commas
    int commaIndex1 = receivedData.indexOf(',');
    int commaIndex2 = receivedData.indexOf(',', commaIndex1 + 1);
   // Extract the latitude, longitude, and card value from the received data
    int flexValue1 = receivedData.substring(0, commaIndex1).toInt();
    int flexValue2 = receivedData.substring(commaIndex1 + 1, commaIndex2).toInt();
    int flexValue3 = receivedData.substring(commaIndex2 + 1).toInt();

    
    // Print the extracted values
//    Serial.print(flexValue1);
//    Serial.print(",");
//    Serial.print(flexValue2);
//    Serial.print(",");
//    Serial.println(flexValue3);
    
    if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0)){
      sendDataPrevMillis = millis();
      
      // Write an Int number to the database path "test/flexValue1"
      if (Firebase.RTDB.setInt(&fbdo, "test/string", flexValue1)){
        Serial.println("Latitude write successful");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      }
      else {
        Serial.println("Latitude write failed");
        Serial.println("REASON: " + fbdo.errorReason());

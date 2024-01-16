#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
//#include <WiFiClient.h>
//#include <WiFiManager.h>
 
#define HTTPS_PORT 443
 
const String ssid = "OPPO F7";
const String password = "12345678";

const String baseurl = "https://chopper-iot-backend.vercel.app";
 
const String uid = "j5u7MVpaZFQrKMaB5zAUhqFHP8x2"; // uid per user | beda user beda uid
 
uint64_t timeNow = 0;
 
uint64_t timePrev_upload = 0;
// uint64_t interval_upload = 1 * 60 * 1000; // 1 minute(s)
uint64_t interval_upload = 15 * 1000; // 1 minute(s)
 
uint64_t timePrev_get = 0;
uint64_t interval_get = 500; // 5 second(s)

String isActive;
String chopperLevel;
String values, sensor_data;
float volumemasuk;
int kondisi_relay;
int statusspeed;
int kondisi_relay_kirim;
int statusspeed_kirim;
int kondisimotorsekarang;
int kondisimotorsementara = 0;
int statusspeedsekarang;
int statusspeedsementara = 1;
int is_active;
int chopper_level;

unsigned long interval = 100; // the time we need to wait
unsigned long previousMillis = 0; // millis() returns an unsigned long.

unsigned long interval2 = 1000; // the time we need to wait
unsigned long previousMillis2 = 0; // millis() returns an unsigned long.
 
void setup()
{
    Serial.begin(115200);
    WiFi.begin(ssid, password);
 
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting...");
    }

//  WiFiManager wifiManager;
//  wifiManager.resetSettings();
//  wifiManager.autoConnect("Smart Trash Crusher");
//
// if (!wifiManager.autoConnect("Smart Trash Crusher"))
//  {
//    Serial.println("Gagal Terhubung Ke Internet");
//  }
//  else
//  {
//    Serial.println("Terhubung Ke Internet");
//  }
    
}


 
String getIsActive();
String getChopperLevel();
void uploadChopperData();
void setIsAvtive(int is_active);
void setChopperLevel(int chopper_level);
 
void loop()
{
    kirimarduino();
    kirimfirebase();
    timeNow = millis();
 
    if (timeNow - timePrev_get >= interval_get)
    {
        timePrev_get = timeNow;
        if (WiFi.status() == WL_CONNECTED)
        {
              isActive = getIsActive();
//            Serial.print("is active     : ");
//            Serial.println(isActive);
 
              chopperLevel = getChopperLevel();
//            Serial.print("chopper level : " + chopperLevel);
//            Serial.println(chopperLevel);
        }
          
          if (isActive == "1" && chopperLevel == "1") {
            kondisi_relay_kirim = 1;
          }
          else if (isActive == "1" && chopperLevel == "2"){
            kondisi_relay_kirim = 2;
          }
          else if (isActive == "0" && (chopperLevel == "1" || chopperLevel == "2")) {
            kondisi_relay_kirim = 0;
          }
          
          String datakirim = String(kondisi_relay_kirim);
          Serial.println(datakirim);
    }
 
    if (timeNow - timePrev_upload >= interval_upload)
    {
        timePrev_upload = timeNow;
 
        if (WiFi.status() == WL_CONNECTED)
        {
            uploadChopperData();
        }
    }
}
 
/*
Requests:
1. is_active     | web -> nodemcu | response: 0 1 -> 0 off, 1 on
2. chopper_level | web -> nodemcu | response: 1 2 -> speed level 1 and 2
3. chopper_data  | nodemcu -> web | send: data={uid},{volume}
 
ada lagi?
*/
 
struct httpResponse
{
    String payload;
    String error;
};
 
struct httpResponse getHttpResponse(HTTPClient *http, int *httpResponseCode)
{
    static httpResponse response{
        payload : "",
        error : ""
    };
 
//    Serial.println();
 
    if (httpResponseCode > 0)
    {
//        Serial.print("HTTP Response code: ");
//        Serial.println(*httpResponseCode);
        String payload = http->getString();
//        Serial.println(payload);
        response.payload = payload;
    }
    else
    {
//        Serial.print("Error code: ");
//        Serial.println(*httpResponseCode);
        response.error = "error";
    }
 
    return response;
}
 
String getIsActive()
{
    // setup https connection
    String url = baseurl + "/events/" + uid + "/is_active";
 
    WiFiClientSecure client;
    client.setInsecure();
    client.connect(url, HTTPS_PORT);
 
    HTTPClient http;
    http.begin(client, url);
 
    // get data
    int httpResponseCode = http.GET();
 
    httpResponse response = getHttpResponse(&http, &httpResponseCode);
 
    // Free resources
    http.end();
 
    return response.payload;
}
 
String getChopperLevel()
{
    // setup https connection
    String url = baseurl + "/events/" + uid + "/chopper_level";
 
    WiFiClientSecure client;
    client.setInsecure();
    client.connect(url, HTTPS_PORT);
 
    HTTPClient http;
    http.begin(client, url);
 
    // get data
    int httpResponseCode = http.GET();
 
    httpResponse response = getHttpResponse(&http, &httpResponseCode);
 
    // Free resources
    http.end();
 
    return response.payload;
}
 
void uploadChopperData()
{
    // random data (nanti bagian ini aja yang nilainya diganti sama nilai asli sensor)
    float volume = volumemasuk; // random 0 - 99
 
    // set string urutan data
    String data = "data=";
    data += uid;
    data += "," + String(volume);
//    Serial.println ("volume :" + data);
 
    // setup https connection
    String url = baseurl + "/chopper_data";
 
    WiFiClientSecure client;
    client.setInsecure();
    client.connect(url, HTTPS_PORT);
 
    HTTPClient http;
    http.begin(client, url);
 
    // send data
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    int httpResponseCode = http.POST(data);
 
    httpResponse response = getHttpResponse(&http, &httpResponseCode);
 
    // Free resources
    http.end();
}

void kirimfirebase() {
  //Baca data dari arduino
  unsigned long currentMillis = millis(); // grab current time
  if ((unsigned long)(currentMillis - previousMillis) >= interval) {
    bool Sr = false;

    while (Serial.available() > 0) {

      //get sensor data from serial put in sensor_data
      sensor_data = Serial.readString();
      Sr = true;

    }

    if (Sr == true && sensor_data.length() > 2) {

      values = sensor_data;

      //get comma indexes from values variable
      int fristCommaIndex = values.indexOf('#');
      int secondCommaIndex = values.indexOf('#', fristCommaIndex + 1);
//      int thirdCommaIndex = values.indexOf('#', secondCommaIndex + 1);

      //get sensors data from values variable by  spliting by commas and put in to variables
      String stringvolume = values.substring(0, fristCommaIndex);
      String stringkondisi_relay = values.substring(fristCommaIndex + 1, secondCommaIndex);
//      String stringstatusspeed = values.substring(secondCommaIndex + 1);

      volumemasuk = stringvolume.toFloat();
      kondisi_relay = stringkondisi_relay.toInt();
      if (kondisi_relay == 1) {
        is_active = 1 ;     // 0 atau 1
        chopper_level = 1; // 1 atau 2
//        Serial.println("Motor 1");
      } else if (kondisi_relay == 2) {
        is_active = 1 ;     // 0 atau 1
        chopper_level = 2; // 1 atau 2
//        Serial.println("Motor 2");
      } else if (kondisi_relay == 0) {
        is_active = 0 ;     // 0 atau 1
//        Serial.println("Motor 0");
      }
//      statusspeed = stringstatusspeed.toInt();

//        Serial.println(volumemasuk);
//        Serial.println(kondisi_relay);
      //  Serial.println(statusspeed);
    }

    //Kirim data ke firebase
//    Firebase.setInt(firebaseData, "/FirebaseIOT/volume", volume);
//    Firebase.setInt(firebaseData, "/FirebaseIOT/kondisimotor", kondisi_relay);
//    Firebase.setInt(firebaseData, "/FirebaseIOT/statusspeed", statusspeed);

    previousMillis = currentMillis;
  }
}

void kirimarduino() {
  //Kirim data ke arduino
  unsigned long currentMillis2 = millis(); // grab current time
  if ((unsigned long)(currentMillis2 - previousMillis2) >= interval2) {
    //Kirim data ke arduino
  if (isActive == "1" && chopperLevel == "1") {
    kondisi_relay_kirim = 1;
  }
  else if (isActive == "1" && chopperLevel == "2"){
    kondisi_relay_kirim = 2;
  }
  else if (isActive == "0" && (chopperLevel == "1" || chopperLevel == "2")) {
    kondisi_relay_kirim = 0;
  }
  
  String datakirim = String(kondisi_relay_kirim);
  Serial.println(datakirim);
  previousMillis2 = currentMillis2;
}
}

void setIsAvtive(int is_active)
{
  // set string urutan data
  String data = "data=";
  data += uid;
  data += ",is_active";
  data += "," + String(is_active);

  // setup https connection
  String url = baseurl + "/events";

  WiFiClientSecure client;
  client.setInsecure();
  client.connect(url, HTTPS_PORT);

  HTTPClient http;
  http.begin(client, url);

  // send data
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int httpResponseCode = http.PUT(data);

  httpResponse response = getHttpResponse(&http, &httpResponseCode);

  // Free resources
  http.end();
}

void setChopperLevel(int chopper_level)
{
  // random data (nanti bagian ini aja yang nilainya diganti sama nilai asli sensor)

  // set string urutan data
  String data = "data=";
  data += uid;
  data += ",chopper_level";
  data += "," + String(chopper_level);

  // setup https connection
  String url = baseurl + "/events";

  WiFiClientSecure client;
  client.setInsecure();
  client.connect(url, HTTPS_PORT);

  HTTPClient http;
  http.begin(client, url);

  // send data
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int httpResponseCode = http.PUT(data);

  httpResponse response = getHttpResponse(&http, &httpResponseCode);

  // Free resources
  http.end();
}

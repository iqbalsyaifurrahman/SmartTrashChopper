//Inisiasi OLED & lidar
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>
Adafruit_SH1106 lcd(-1);
VL53L0X sensor;

////Inisiasi softwareserial
//#include <SoftwareSerial.h>
//SoftwareSerial NodeMCU1(3, 2);


//smoothing bacaan lidar
const int numReadings = 20;
int readings0 [numReadings];      // the readings from the analog input
int readings1 [numReadings];
int readIndex0;
int readIndex1; // the index of the current reading
int total0;                  // the running total
int average0;                    // the average
int total1;                  // the running total
int average1;
int i = -1;
boolean statusvolume = false;

//Untuk volume
int l = 3600;
float t = 500;
float temptinggi0;
int volume0;
float temptinggi1;
int volume1;
int tinggi [1];
unsigned long interval = 60000; // the time we need to wait
unsigned long previousMillis = 0; // millis() returns an unsigned long.
unsigned long interval3 = 1000; // the time we need to wait
unsigned long previousMillis3 = 0; // millis() returns an unsigned long.


//Untuk data firebase
unsigned long interval2 = 2000; // the time we need to wait
unsigned long previousMillis2 = 0; // millis() returns an unsigned long.
int kondisirelay_masuk;
int statusspeed_masuk;
String values, sensor_data;
int kondisirelay_sementara;
int statusspeed_sementara;


//Inisiasi pushbutton
#define btnUp   12 // Dalam counter UP dan Start
#define btnOk   9 // Dalam counter Set
#define btnDown 10  // Dalam counter Down, Pause ( tekan sekali), Reset (Tekan lama dan angkat)
#define btnBack 11
#define speed1  8
#define speed2  7


boolean statusBtnUp   = false;
boolean statusBtnOk   = false;
boolean statusBtnDown = false;
boolean statusBtnBack = false;

boolean statusAkhirBtnUp   = false;
boolean statusAkhirBtnOk   = false;
boolean statusAkhirBtnDown = false;
boolean statusAkhirBtnBack = false;

boolean UP   = false;
boolean OK   = false;
boolean DOWN = false;
boolean BACK = false;

int halaman   = 0;
int menuItem  = 1;
int menutimer = 1;
int menuutama = 1;
bool ganti;

//Inisiasi status motor
boolean statusmotor = false;
boolean statustempmotor = false;

//Control speed (Buat control speed tinggal masukin variabel ini)
int statusspeed = 1;

//Inisiasi timer
#include "Countimer.h"
Countimer tdown;
int set_detik = 0;
int set_menit = 0;
int set_jam = 0;
int last_set_detik = 0;
int last_set_menit = 0;
int last_set_jam = 0;
int set = 0;
bool kondisi_set = 0;
int kondisi_relay = 0;  //Kondisi controller motor status on/off baik manual atau counter
bool kondisi_reset = 0;
int kondisi_timer = 0;
unsigned long lastmillis;

//logobitmap
const unsigned char logo [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x03, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x07, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x3f, 0xff, 0xf0, 0x00, 0x00, 0x00,
  0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x0f, 0x0f, 0xff, 0xf8, 0x00, 0x00, 0x00,
  0x00, 0x07, 0x80, 0xfe, 0xf0, 0x7f, 0x8f, 0xc7, 0xc0, 0x1f, 0xc7, 0xff, 0xfc, 0x00, 0x00, 0x00,
  0x00, 0x03, 0xe0, 0xff, 0xf8, 0xff, 0x8f, 0x87, 0xc0, 0x1f, 0xe3, 0xff, 0xfe, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xf1, 0xc6, 0x38, 0xc3, 0x8e, 0x03, 0x00, 0x3f, 0xf1, 0xff, 0xfe, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x71, 0xc6, 0x39, 0xc3, 0x8c, 0x03, 0x00, 0x7f, 0xf9, 0xff, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x0c, 0xf1, 0xce, 0x31, 0xe7, 0x0c, 0x07, 0x00, 0xff, 0xfc, 0xff, 0xff, 0xb8, 0x00, 0x00,
  0x00, 0x1f, 0xe1, 0xce, 0x30, 0xff, 0x1c, 0x07, 0x00, 0xff, 0xfc, 0xff, 0xff, 0xf0, 0x00, 0x00,
  0x00, 0x07, 0xc1, 0x8e, 0x30, 0x77, 0x1c, 0x07, 0x01, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xfc, 0x3f, 0xff, 0xe0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xfc, 0x3f, 0xff, 0xc0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xf8, 0x1f, 0xff, 0x80, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xf0, 0x1f, 0xff, 0x80, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xf0, 0x1f, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xe0, 0x7f, 0xfe, 0x0c, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x1f, 0xc0, 0x07, 0xfe, 0x3e, 0x00, 0x00,
  0x00, 0x07, 0xf8, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0xfe, 0x00, 0x00,
  0x00, 0x0f, 0xf8, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x1f, 0x83, 0x80, 0x00, 0x03, 0xff, 0x00, 0x00,
  0x00, 0x0f, 0xf0, 0x00, 0x00, 0x01, 0xc0, 0x0f, 0xff, 0xc0, 0x00, 0x00, 0x0f, 0xff, 0x80, 0x00,
  0x00, 0x01, 0xc2, 0x63, 0x98, 0xf1, 0xdc, 0x07, 0xff, 0xe0, 0x00, 0x00, 0x3f, 0xff, 0x80, 0x00,
  0x00, 0x03, 0xc7, 0xcf, 0xf9, 0xf9, 0xfe, 0x01, 0xff, 0xe0, 0x00, 0x00, 0x3f, 0xff, 0xc0, 0x00,
  0x00, 0x03, 0x87, 0x0e, 0x79, 0xc1, 0xce, 0x03, 0xff, 0xf0, 0x00, 0x00, 0x1f, 0xff, 0xe0, 0x00,
  0x00, 0x03, 0x86, 0x1c, 0x39, 0xe3, 0x8e, 0x03, 0xff, 0xf8, 0x00, 0x00, 0x1f, 0xff, 0xe0, 0x00,
  0x00, 0x03, 0x8e, 0x1c, 0x30, 0x7b, 0x8e, 0x07, 0xff, 0xf8, 0x00, 0x00, 0x0f, 0xff, 0xf0, 0x00,
  0x00, 0x03, 0x8e, 0x1e, 0x72, 0x3b, 0x8e, 0x07, 0xff, 0xfc, 0x00, 0x00, 0x07, 0xff, 0xf8, 0x00,
  0x00, 0x07, 0x8e, 0x0f, 0xf3, 0xf3, 0x8c, 0x0f, 0xff, 0xfe, 0x00, 0x00, 0x07, 0xff, 0xf0, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x00, 0xc0, 0x00, 0x0f, 0xff, 0xfe, 0x00, 0x00, 0x03, 0xff, 0xf0, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xf2, 0x00, 0x00, 0x01, 0xff, 0xe0, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xe0, 0x00, 0x00, 0x01, 0xff, 0x80, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xc0, 0x06, 0x00, 0x00, 0x00, 0x10, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xc0, 0x06, 0x00, 0x00, 0x00, 0x60, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x80, 0x0f, 0xff, 0xff, 0xff, 0xe0, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xc0, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xc0, 0x00,
  0x00, 0x01, 0xfc, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0x80, 0x00,
  0x00, 0x07, 0xfc, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0x80, 0x00,
  0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0x00, 0x00,
  0x00, 0x0f, 0x00, 0xdb, 0x8e, 0x7c, 0xff, 0x0f, 0x87, 0xc0, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
  0x00, 0x0e, 0x01, 0xfb, 0x8c, 0xec, 0xff, 0x1d, 0xc7, 0xc0, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00,
  0x00, 0x0e, 0x01, 0xc3, 0x8c, 0xe0, 0xe3, 0x38, 0xe7, 0x00, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00,
  0x00, 0x0f, 0x01, 0xc3, 0x9c, 0x78, 0xe3, 0x3f, 0xee, 0x00, 0x7f, 0xff, 0xff, 0xfc, 0x00, 0x00,
  0x00, 0x07, 0x89, 0xc3, 0x9c, 0x1c, 0xc7, 0x38, 0x0e, 0x00, 0x3f, 0xff, 0xff, 0xf8, 0x00, 0x00,
  0x00, 0x07, 0xf9, 0x83, 0xf8, 0xfc, 0xc7, 0x3f, 0x8e, 0x00, 0x1f, 0xff, 0xff, 0xf8, 0x00, 0x00,
  0x00, 0x01, 0xfb, 0x81, 0xf0, 0xf9, 0xc7, 0x1f, 0x8e, 0x00, 0x0f, 0xff, 0xff, 0xe0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

//void TCA9548A(uint8_t bus) {
//  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
//  Wire.write(1 << bus);          // send byte to select bus
//  Wire.endTransmission();
//}


void setup() {
  //Untuk OLED
  Wire.begin();
  Serial.begin(115200); 
  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();
//  delay(1000); 
  //Untuk lidar

  
  //  TCA9548A(0);
  //  TCA9548A(1);
  //  TCA9548A(2);
  //  NodeMCU1.begin(115200);
  
  lcd.begin(SH1106_SWITCHCAPVCC, 0x3C);
  lcd.clearDisplay();
  lcd.drawBitmap(0, 0, logo, 128, 64, WHITE);
  lcd.display();
  delay(2000);
  lcd.clearDisplay();

  //inisiasi value firebase
  int kondisirelay_sementara = kondisi_relay;
//  int statusspeed_sementara = statusspeed;

  // pin push button
  pinMode(btnUp,   INPUT);
  pinMode(btnOk,   INPUT);
  pinMode(btnDown, INPUT);
  pinMode(btnBack, INPUT);
  pinMode(speed1,   OUTPUT);
  pinMode(speed2,   OUTPUT);


  for (int thisReading0 = 0; thisReading0 < numReadings; thisReading0++) {
    readings0[thisReading0] = 0;
  }
  for (int thisReading1 = 0; thisReading1 < numReadings; thisReading1++) {
    readings1[thisReading1] = 0;
  }

  //untuk rtc
//  rtc.begin();
  //    rtc.setDate(9, 8, 2021);   //mensetting tanggal 07 april 2018
  //    rtc.setTime(16, 15, 00);     //menset jam 22:00:00
  //    rtc.setDOW(1);     //menset hari "Sabtu"

  //countdown
  tdown.setInterval(print_time, 1000);

//  serialvolume2 ();
}

void print_time() {
  set_detik = set_detik - 1;
  if (set_detik < 0) {
    set_detik = 59;
    set_menit = set_menit - 1;
  }
  if (set_menit < 0) {
    set_menit = 59;
    set_jam = set_jam - 1;
  }
}

void loop() {

//  Serial.println(kondisi_relay);
  tampil();
  tdown.run();
  datafirebase();
//  serialvolume();
  serialvolume2 ();


  //Untuk pushbutton
  statusBtnUp   = digitalRead(btnUp);
  statusBtnOk   = digitalRead(btnOk);
  statusBtnDown = digitalRead(btnDown);
  statusBtnBack = digitalRead(btnBack);
  //
  saatUpDitekan();
  saatOkDitekan();
  saatDownDitekan();
  saatBackDitekan();


  //  untuk button up
  if (UP && halaman == 1) {
    UP = false;
    menuItem --;
    if (menuItem < 1)menuItem = 3;
  }
  if (UP && halaman == 0) {
    UP = false;
    menuutama --;
    if (menuutama < 1)menuutama = 2;
  }
  if (digitalRead (btnUp) == HIGH && halaman == 3) {
//    UP = false;
    if (set == 0 && (set_detik != 0 || set_menit != 0 || set_jam != 0)) {
      tdown.start();
      statusmotor = 1;
      kondisi_timer = 1;
      controlmotorhidup();
      String datakirim2 = String(volume0) + "#" + String(kondisi_relay);
      Serial.println(datakirim2);
//      serialvolume2();
    }
    if (set == 1) {
      set_detik++;
      last_set_detik = set_detik;
    }
    if (set == 2) {
      set_menit++;
      last_set_menit = set_menit;
    }
    if (set == 3) {
      set_jam++;
      last_set_jam = set_jam;
    }
    if (set_detik > 59) {
      set_detik = 0;
    }
    if (set_menit > 59) {
      set_menit = 0;
    }
    if (set_jam > 99) {
      set_jam = 0;
    }
    delay(200);
  }



  //untuk button down
  if (DOWN && halaman == 1) {
        DOWN = false;
    menuItem ++;
    if (menuItem > 3)menuItem = 1;
  }
  if (DOWN && halaman == 0) {
    DOWN = false;
    menuutama ++;
    if (menuutama > 2)menuutama = 1;
  }

  if (digitalRead (btnDown) == HIGH && halaman == 3) {
    lastmillis = millis();
    kondisi_reset = 0;
    while (digitalRead (btnDown) == HIGH && set == 0) {
      if (millis() - lastmillis > 500) {
        tdown.stop();
        controlmotormati();
        kondisi_timer = 0;
        set_detik = 0;
        set_menit = 0;
        set_jam = 0;
      }
    }
    if (kondisi_reset == 0) {
      if (set == 0 && statusmotor == 1) {
        tdown.stop();
        controlmotormati();
        kondisi_timer = 0;
        statusmotor = 0;
        String datakirim2 = String(volume0) + "#" + String(kondisi_relay);
        Serial.println(datakirim2);
//        serialvolume2();
      }
      if (set == 1) {
        set_detik--;
        last_set_detik = set_detik;
      }
      if (set == 2) {
        set_menit--;
        last_set_menit = set_menit;
      }
      if (set == 3) {
        set_jam--;
        last_set_jam = set_jam;
      }
      if (set_detik < 0) {
        set_detik = 59;
      }
      if (set_menit < 0) {
        set_menit = 59;
      }
      if (set_jam < 0) {
        set_jam = 99;
      }
    }
    delay(200);
  }
  //

  //  untuk button ok
  if (OK) {
        OK = false;
    if (halaman == 0 && menuutama == 1) {
      halaman = 1;
    } else if (halaman == 0 && menuutama == 2) {
      halaman = 2;
//      serialvolume2 ();
      statusvolume = true;
      String datakirim2 = String(volume0) + "#" + String(kondisi_relay);
      Serial.println(datakirim2);
    } else if (halaman == 1 && menuItem == 1 && statusmotor == false) {
      statusmotor = true;
      controlmotorhidup();
      if ( set_detik != 0 || set_menit != 0 || set_jam != 0) {
        tdown.start();
      }
      String datakirim2 = String(volume0) + "#" + String(kondisi_relay);
      Serial.println(datakirim2);
//      serialvolume2 ();
    } else if (halaman == 1 && menuItem == 1 && statusmotor == true) {
      statusmotor = false;
      controlmotormati();
      tdown.stop();
      String datakirim2 = String(volume0) + "#" + String(kondisi_relay);
      Serial.println(datakirim2);
//      serialvolume2 ();
    } else if (halaman == 1 && menuItem == 2) {
      halaman = 3;
    } else if (halaman == 1 && menuItem == 3) {
      if (statusmotor == true) {
        statusspeed++;
        if (statusspeed > 2)statusspeed = 1;
        controlmotorhidup();
      } else {
        statusspeed++;
        if (statusspeed > 2)statusspeed = 1;
      }
      String datakirim2 = String(volume0) + "#" + String(kondisi_relay);
      Serial.println(datakirim2);
//      serialvolume2 ();
    } 
    else if (halaman == 3) {
      if (kondisi_set == 0 && kondisi_timer == 0) {
        //        kondisi_set = 1;
        set++;
        if (set > 3)set = 0;
      } else {
        kondisi_set = 0;
      }
    }
  }



  //  untuk button back
  if (BACK) {
        BACK = false;
    if (halaman == 1 ) {
      halaman = 0;
    }
    else if (halaman == 2) {
      halaman = 0;
      statusvolume = false;
    }
    else if (halaman == 3) {
      halaman = 1;
    }
  }

  // Kondisi motor saat countdown berlangsung
  if (set_detik == 0 && set_menit == 0 && set_jam == 0 && kondisi_timer == 1 ) {
    kondisi_timer = 0;
    statusmotor = 0;
    controlmotormati();
    tdown.stop();
    String datakirim2 = String(volume0) + "#" + String(kondisi_relay);
    Serial.println(datakirim2);
//    serialvolume2();
  }

  //   Masukan pin motor
  //  if (kondisi_relay == 1) {
  //  digitalWrite(relay, HIGH);
  //  } else {
  //  digitalWrite(relay, LOW);
  //  }

  delay(100);

}

void controlmotorhidup() {
  if (statusspeed == 1) {
    digitalWrite(speed2, LOW);
    delay(100);
    digitalWrite(speed1, HIGH);
    kondisi_relay = 1;
  } else if (statusspeed == 2) {
    digitalWrite(speed1, LOW);
    delay(100);
    digitalWrite(speed2, HIGH);
    kondisi_relay = 2;
  }
}

void controlmotormati() {
  digitalWrite(speed1, LOW);
  digitalWrite(speed2, LOW);
  kondisi_relay = 0;
}


//--------------------------------------------------------------------------------
void saatUpDitekan() {
  if (statusBtnUp != statusAkhirBtnUp) {
    if (statusBtnUp == 0) {
      UP = true;
    }
    delay(50);
  }
  statusAkhirBtnUp = statusBtnUp;
}

void saatOkDitekan() {
  if (statusBtnOk != statusAkhirBtnOk) {
    if (statusBtnOk == 0) {
      OK = true;
    }
    delay(50);
  }
  statusAkhirBtnOk = statusBtnOk;
}

void saatDownDitekan() {
  if (statusBtnDown != statusAkhirBtnDown) {
    if (statusBtnDown == 0) {
      DOWN = true;
    }
    delay(50);
  }
  statusAkhirBtnDown = statusBtnDown;
}

void saatBackDitekan() {
  if (statusBtnBack != statusAkhirBtnBack) {
    if (statusBtnBack == 0) {
      BACK = true;
    }
    delay(50);
  }
  statusAkhirBtnBack = statusBtnBack;
}
//----------------------------------------------------------------------------

//semua yang tampil di lcd ada di fungsi ini
void tampil() {

  if (halaman == 0) {
    lcd.clearDisplay();
    lcd.setTextSize(1);
    lcd.setTextColor(WHITE);
    lcd.setCursor(7, 5);
    lcd.print(F("SMART TRASH CRUSHER"));
    lcd.setTextSize(1);
    lcd.setTextColor(WHITE);
    lcd.setCursor(0, 15);
    //    lcd.print(kondisirelay_masuk);
    //    lcd.setCursor(35, 25);
    //    lcd.print(statusspeed_masuk);
    //
    //        lcd.print(rtc.getDOWStr());
    //        lcd.print(F(", "));
    //        lcd.print(rtc.getDateStr());
    //        lcd.setCursor(35, 25);
    //        lcd.print(rtc.getTimeStr());
    if (menuutama == 1) {
      lcd.setCursor(5, 30);
      lcd.setTextColor(WHITE);
      lcd.print(F("> CONTROL MOTOR "));
    } else if (menuutama != 1) {
      lcd.setCursor(5, 30);
      lcd.setTextColor(WHITE);
      lcd.print(F("  CONTROL MOTOR"));
    }
    if (menuutama == 2) {
      lcd.setCursor(5, 45);
      lcd.setTextColor(WHITE);
      lcd.print(F("> STATISTIK"));
    } else {
      lcd.setCursor(5, 45);
      lcd.setTextColor(WHITE);
      lcd.print(F("  STATISTIK"));
    }

  }
  if (halaman == 1) {
    lcd.clearDisplay();
    lcd.setTextSize(1);
    lcd.setTextColor(WHITE);
    lcd.setCursor(25, 5);
    lcd.print(F("CONTROL MOTOR"));
    if (menuItem == 1 && statusmotor == false ) {
      lcd.setCursor(5, 25);
      lcd.setTextColor(WHITE);
      lcd.print(F("> STATUS MOTOR: OFF "));
    } else if (menuItem != 1 && statusmotor == false ) {
      lcd.setCursor(5, 25);
      lcd.setTextColor(WHITE);
      lcd.print(F("  STATUS MOTOR: OFF"));
    }
    if (menuItem == 1 && statusmotor == true ) {
      lcd.setCursor(5, 25);
      lcd.setTextColor(WHITE);
      lcd.print(F("> STATUS MOTOR: ON"));
    } else if (menuItem != 1 && statusmotor == true ) {
      lcd.setCursor(5, 25);
      lcd.setTextColor(WHITE);
      lcd.print(F("  STATUS MOTOR: ON"));
    }
    if (menuItem == 2) {
      lcd.setCursor(5, 40);
      lcd.setTextColor(WHITE);
      lcd.print(F("> TIMER"));
    } else if (menuItem != 2) {
      lcd.setCursor(5, 40);
      lcd.setTextColor(WHITE);
      lcd.print(F("  TIMER"));
    }
    if (menuItem == 3) {
      lcd.setCursor(5, 55);
      lcd.setTextColor(WHITE);
      lcd.print(F("> SPEED MOTOR : "));
      lcd.print(statusspeed);
    } else if (menuItem != 3) {
      lcd.setCursor(5, 55);
      lcd.setTextColor(WHITE);
      lcd.print(F("  SPEED MOTOR : "));
      lcd.print(statusspeed);
    }
  } else if (halaman == 2) {
    lcd.clearDisplay();
    lcd.setTextSize(2);
    lcd.setTextColor(WHITE);
    lcd.setCursor(25, 0);
    lcd.print(F("Volume"));
    lcd.setTextSize(2);
    lcd.setCursor(5, 30);
    lcd.print(volume0);
    lcd.setTextSize(2);
    lcd.setCursor(50, 30);
    lcd.print(F(" Liter"));

  }
  else if (halaman == 3) {
    lcd.clearDisplay();
    lcd.setTextSize(2);
    if (set == 0) {
      lcd.setTextColor(WHITE);
      lcd.setCursor(35, 0);
      lcd.print(F("Timer"));
    } else if (set == 1) {
      lcd.setTextColor(WHITE);
      lcd.setCursor(10, 0);
      lcd.print(F("Set Detik"));
    } else if (set == 2) {
      lcd.setTextColor(WHITE);
      lcd.setCursor(10, 0);
      lcd.print(F("Set Menit"));
    } else if (set == 3) {
      lcd.setTextColor(WHITE);
      lcd.setCursor(25, 0);
      lcd.print(F("Set Jam"));
    }

    lcd.setTextSize(2);
    lcd.setCursor(15, 25);
    if (set_jam <= 9) {
      lcd.print(F("0"));
    }
    lcd.print(set_jam);
    lcd.print(F(":"));
    if (set_menit <= 9) {
      lcd.print(F("0"));
    }
    lcd.print(set_menit);
    lcd.print(F(":"));
    if (set_detik <= 9) {
      lcd.print(F("0"));
    }
    lcd.print(set_detik);
    lcd.print(F("   "));

  }

  lcd.display();

}

void serialvolume () {
  //Untuk volume
  unsigned long currentMillis = millis(); // grab current time
  if ((unsigned long)(currentMillis - previousMillis) >= interval) {
//  readIndex1 = 0;
//  total1 = 0;
//  average1 = 0;
//  temptinggi1 = 0;
//  volume1 = 0;
//    while (readIndex1  <= 19) {
//      //    Serial.println(total);
//      readings1[readIndex1] = sensor.readRangeContinuousMillimeters();
//      total1 = total1 + readings1[readIndex1];
//      readIndex1 = readIndex1 + 1;
//    }
//    if (readIndex1 = 20) {
//      average1 = total1 / 20;
//    }
//    average1 = average1 - 50;
//    if ( average1 < 0 ) {
//      average1 = 0;
//    } else if ( average1 > t) {
//      average1 = t;
//    }
//    temptinggi1 = t - average1;
//    volume1 = (l * temptinggi1) / 100;
//
//    //Kirim data ke NodeMCU
    String datakirim = String(volume0) + "#" + String(kondisi_relay);
    Serial.println(datakirim);
    previousMillis = currentMillis;
  }
}

void datafirebase() {
  unsigned long currentMillis2 = millis(); // grab current time
  if ((unsigned long)(currentMillis2 - previousMillis2) >= interval2) {
    bool Sr = false;

    while (Serial.available() > 0) {

      //get sensor data from serial put in sensor_data
      sensor_data = Serial.readString();
      Sr = true;

    }

    if (Sr == true) {

      values = sensor_data;


//      //get comma indexes from values variable
//      int fristCommaIndex = values.indexOf('#');
//      int secondCommaIndex = values.indexOf('#', fristCommaIndex + 1);
//
//      //get sensors data from values variable by  spliting by commas and put in to variables
//      String stringkondisi_relay = values.substring(0, fristCommaIndex);
//      String stringstatusspeed = values.substring(fristCommaIndex + 1, secondCommaIndex);
//
      kondisirelay_masuk = values.toInt();
//      Serial.println(kondisirelay_masuk);
//      statusspeed_masuk = stringstatusspeed.toInt();

      if (kondisirelay_sementara != kondisirelay_masuk) {
//        Serial.println(kondisirelay_masuk);
        if (kondisirelay_masuk == 1) {
            statusmotor = true;
            statusspeed = 1;
            controlmotorhidup();
        }
        else if (kondisirelay_masuk == 2) {
            statusmotor = true;
            statusspeed = 2;
            controlmotorhidup();
        }
        else if (kondisirelay_masuk == 0){
           statusmotor = false;
           controlmotormati();
        }
        kondisirelay_sementara = kondisirelay_masuk;
      }
//      if (statusspeed_sementara != statusspeed_masuk) {
//        if (statusspeed_masuk == 1) {
//          statusspeed = 1;
////          serialvolume2 ();
//        }
//        if (statusspeed_masuk == 2) {
//          statusspeed = 2;
////          serialvolume2 ();
//        }
//        statusspeed_sementara = statusspeed_masuk;
//      }
    }
            
    //        Serial.println("Status Speed : " + statusspeed_masuk);
    previousMillis2 = currentMillis2;
  }
}

void serialvolume2 () {
  unsigned long currentMillis3 = millis();
  if ((unsigned long)(currentMillis3 - previousMillis3) >= interval3) {
    readIndex0 = 0;
    total0 = 0;
    average0 = 0;
    temptinggi0 = 0;
    volume0 = 0;
    while (readIndex0 <= 3) {
      readings0[readIndex0] = sensor.readRangeContinuousMillimeters();
      total0 = total0 + readings0[readIndex0];
      readIndex0 = readIndex0 + 1;
    }
    if (readIndex0 = 4) {
      average0 = total0 /4;
    }
    average0 = average0 - 50;
    if ( average0 < 0 ) {
      average0 = 0;
    } else if ( average0 > t) {
      average0 = t;
    }
    temptinggi0 = t - average0;
    volume0 = (l * temptinggi0) / 1000;
    previousMillis3 = currentMillis3;
  }
}

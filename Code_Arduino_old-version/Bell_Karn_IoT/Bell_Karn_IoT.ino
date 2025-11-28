#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiMulti.h>
#include "sps30.h"
#include <Wire.h>

#include <Adafruit_ADS1015.h>
#include "SHT31.h"

#define WIFI_STA_NAME "tongngii"//"HUAWEI-74D7"
#define WIFI_STA_PASS "0881927559"//"92981219"

Adafruit_ADS1115 ads(0x48);
Adafruit_ADS1115 ads2(0x49);

#define MQTT_SERVER "thingsboard.weaverbase.com"
#define MQTT_PORT 1883
#define MQTT_USERNAME "3TrtbxMFVtcWEqf85Xv0"
#define MQTT_PASSWORD "123456"
#define MQTT_NAME "PICO_W"
#define MQTT_TOPIC "v1/devices/me/telemetry"

WiFiClient client;
PubSubClient mqtt(client);



char json_0[300];
StaticJsonDocument<200> jsonDocument_0;
char jsonString_0[200];

char json_1[300];
StaticJsonDocument<200> jsonDocument_1;
char jsonString_1[200];

char json_2[300];
StaticJsonDocument<200> jsonDocument_2;
char jsonString_2[200];

char json_3[300];
StaticJsonDocument<200> jsonDocument_3;
char jsonString_3[200];

char json_4[300];
StaticJsonDocument<200> jsonDocument_4;
char jsonString_4[200];

char json_5[300];
StaticJsonDocument<200> jsonDocument_5;
char jsonString_5[200];

char json_6[300];
StaticJsonDocument<200> jsonDocument_6;
char jsonString_6[200];

uint32_t start;
uint32_t stop;
SHT31 sht;

String gps_data;
float PASS_LAT = 0 , PASS_LON = 0 ;

  float aqi_us = 0;
  int pm_min = 0;
  int aqi_max = 0;
  int pmdiff = 0;
  int aqi_min = 0;
  int aqi_level = 0;
  
// function prototypes (sometimes the pre-processor does not create prototypes themself on ESPxx)
void serialTrigger(char * mess);
void ErrtoMess(char *mess, uint8_t r);
void Errorloop(char *mess, uint8_t r);
void GetDeviceInfo();
bool read_all();
long lastMsg = 0;

WiFiMulti multi;

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 1000;    // the debounce time; increase if the output flickers
 
int pinInterrupt = 16;
 
int Count = 0;
 
void onChange()
{
  if ( digitalRead(pinInterrupt) == LOW )
    Count++;
}

void setup() {
  Wire.begin();
  pinMode(27, OUTPUT); // LED
  pinMode(17, OUTPUT); // relay

  pinMode(14, OUTPUT); // spk

  pinMode( pinInterrupt, INPUT_PULLUP);// set the interrupt pin
  attachInterrupt( digitalPinToInterrupt(pinInterrupt), onChange, FALLING);
  
  digitalWrite(27, HIGH); 
  digitalWrite(14, LOW);
  digitalWrite(17, LOW);
  
  int16_t ret;
  uint8_t auto_clean_days = 4;
  uint32_t auto_clean;
  
  Serial.begin(115200);
 
  sensirion_i2c_init();
  
  while (sps30_probe() != 0) {
    Serial.print("SPS sensor probing failed\n");
    delay(500);
  }
  Serial.print("SPS sensor probing successful\n");
  
  ret = sps30_set_fan_auto_cleaning_interval_days(auto_clean_days);
  if (ret) {
    Serial.print("error setting the auto-clean interval: ");
    Serial.println(ret);
  }

  ret = sps30_start_measurement();
  if (ret < 0) {
    Serial.print("error starting measurement\n");
  }

#ifndef PLOTTER_FORMAT
  Serial.print("measurements started\n");
#endif /* PLOTTER_FORMAT */

#ifdef SPS30_LIMITED_I2C_BUFFER_SIZE
  Serial.print("Your Arduino hardware has a limitation that only\n");
  Serial.print("  allows reading the mass concentrations. For more\n");
  Serial.print("  information, please check\n");
  Serial.print("  https://github.com/Sensirion/arduino-sps#esp8266-partial-legacy-support\n");
  Serial.print("\n");
  delay(2000);
#endif

  delay(1000);
  
  multi.addAP(WIFI_STA_NAME, WIFI_STA_PASS);

  if (multi.run() != WL_CONNECTED) {
    Serial.println("Unable to connect to network, rebooting in 10 seconds...");
    delay(10000);
    
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  //serialTrigger((char *) "SPS30-Example1: Basic reading. press <enter> to start");

  Serial.println(F("Trying to connect"));

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
 

  sht.begin(0x44);    //Sensor I2C Address

  Wire.setClock(100000);
  uint16_t stat = sht.readStatus();
  Serial.print(stat, HEX);
  Serial.println();

  digitalWrite(27, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(14, HIGH);
  digitalWrite(17, HIGH);
  delay(1000);                       // wait for a second
  digitalWrite(27, LOW);  
  digitalWrite(14, LOW);  // turn the LED off by making the voltage LOW
  delay(1000);

  ads.begin();
  ads2.begin();
  //am2315.begin();
}


void loop() {

if ((millis() - lastDebounceTime) > debounceDelay)
  {
    lastDebounceTime = millis();
    
    Serial.print((Count * 8.75)/100);
    
    Count = 0;
    
    Serial.println("m/s");
    
  }
  
  long now = millis();

  //Check time for pub mqtt  
  if (now - lastMsg > 300000) {
  lastMsg = now;

  digitalWrite(17, LOW);
  delay(20000);
  
  struct sps30_measurement m;
  char serial[SPS30_MAX_SERIAL_LEN];
  uint16_t data_ready;
  int16_t ret;
  
    do {
    ret = sps30_read_data_ready(&data_ready);
    if (ret < 0) {
      Serial.print("error reading data-ready flag: ");
      Serial.println(ret);
    } else if (!data_ready)
      Serial.print("data not ready, no new measurement available\n");
    else
      break;
    delay(100); /* retry in 100ms */
  } while (1);

  ret = sps30_read_measurement(&m);
  if (ret < 0) {
    Serial.print("error reading measurement\n");
  } else {

#ifndef PLOTTER_FORMAT
    Serial.print("PM  1.0: ");
    Serial.println(m.mc_1p0);
    Serial.print("PM  2.5: ");
    Serial.println(m.mc_2p5);
    Serial.print("PM  4.0: ");
    Serial.println(m.mc_4p0);
    Serial.print("PM 10.0: ");
    Serial.println(m.mc_10p0);

#ifndef SPS30_LIMITED_I2C_BUFFER_SIZE
    Serial.print("NC  0.5: ");
    Serial.println(m.nc_0p5);
    Serial.print("NC  1.0: ");
    Serial.println(m.nc_1p0);
    Serial.print("NC  2.5: ");
    Serial.println(m.nc_2p5);
    Serial.print("NC  4.0: ");
    Serial.println(m.nc_4p0);
    Serial.print("NC 10.0: ");
    Serial.println(m.nc_10p0);

    Serial.print("Typical particle size: ");
    Serial.println(m.typical_particle_size);
#endif

    Serial.println();

#else
    // since all values include particles smaller than X, if we want to create buckets we 
    // need to subtract the smaller particle count. 
    // This will create buckets (all values in micro meters):
    // - particles        <= 0,5
    // - particles > 0.5, <= 1
    // - particles > 1,   <= 2.5
    // - particles > 2.5, <= 4
    // - particles > 4,   <= 10

    Serial.print(m.nc_0p5);
    Serial.print(" ");
    Serial.print(m.nc_1p0  - m.nc_0p5);
    Serial.print(" ");
    Serial.print(m.nc_2p5  - m.nc_1p0);
    Serial.print(" ");
    Serial.print(m.nc_4p0  - m.nc_2p5);
    Serial.print(" ");
    Serial.print(m.nc_10p0 - m.nc_4p0);
    Serial.println();


#endif /* PLOTTER_FORMAT */

  }

  PASS_LAT = 14.059264450400837;
  PASS_LON = 99.42554581259911;
  
 
  
  int16_t batt, wind_dir_raw, co, so2; // max_adc = 26000, low_adc = 0
  
  batt = ads.readADC_SingleEnded(0);
  wind_dir_raw = ads.readADC_SingleEnded(1);
  co  = ads2.readADC_SingleEnded(0);
  so2 = ads2.readADC_SingleEnded(1);
  if(batt < 0)batt = 0;
  if(batt > 26000)batt = 26000;
  
  if(wind_dir_raw < 0)wind_dir_raw = 0;
  if(wind_dir_raw > 26000)wind_dir_raw = 26000;
  
  if(co < 0)co = 0;
  if(co > 26000)co = 26000;

  if(so2 < 0)so2 = 0;
  if(so2 > 26000)so2 = 26000;
  
  batt = map(batt, 0, 26000, 0, 100);
  batt = map(batt, 80, 100, 0, 100);
  wind_dir_raw = map(wind_dir_raw, 0, 26000, 1, 360);

  co = map(co, 0, 26000, 0, 5000);
  co = map(co, 600, 3000, 0, 1000);
  
  so2 = map(so2, 0, 26000, 0, 5000);
  so2 = map(so2, 600, 3000, 0, 20);

  if(co < 0)co = 0;
  if(co > 1000)co = 1000;

  if(so2 < 0)so2 = 0;
  if(so2 > 20)so2 = 20;

  digitalWrite(17, HIGH);
  
  sht.read();
  
  delay(1000);
  Serial.print("Temperature:");
  Serial.print(sht.getTemperature(), 1);
  Serial.print("\t");
  Serial.print("Humidity:");
  Serial.println(sht.getHumidity(), 1);

  for(int i=0;i<3;i++)
{
    if (mqtt.connected() == false) {
    Serial.print("MQTT connection... ");
    if (mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("connected");
      //mqtt.subscribe("/KRU/data");
    } else {
      Serial.println("failed");
      delay(5000);
    }
  }
  
  char Temp_tx[5];
  String temp_str;
  temp_str=String(sht.getTemperature(), 1);
  Serial.println(temp_str);
  temp_str.toCharArray(Temp_tx,5);
  mqtt.publish("KRU/temp",Temp_tx);
  
  char hum_tx[5];
  String hum_str;
  hum_str=String(sht.getHumidity(), 1);
  Serial.println(hum_str);
  hum_str.toCharArray(hum_tx,5);
  mqtt.publish("KRU/humid",hum_tx);
  
  char co_tx[6];
  String co_str;
  co_str=String(co);
  co_str.toCharArray(co_tx,5);
  mqtt.publish("KRU/co",co_tx);

  char so_tx[6];
  String so_str;
  so_str=String(so2);
  so_str.toCharArray(so_tx,5);
  mqtt.publish("KRU/so2",so_tx);

  char wsp_tx[6];
  String wsp_str;
  wsp_str=String((Count * 8.75)/100);
  wsp_str.toCharArray(wsp_tx,5);
  mqtt.publish("KRU/windspd",wsp_tx);

  char wdr_tx[6];
  String wdr_str;
  wdr_str=String(wind_dir_raw);
  wdr_str.toCharArray(wdr_tx,5);
  mqtt.publish("KRU/winddir",wdr_tx);

  char pm_25[6];
  String pm25_str;
  pm25_str=String(m.mc_2p5);
  pm25_str.toCharArray(pm_25,5);
  mqtt.publish("KRU/pm25",pm_25);

  char pm_10[6];
  String pm10_str;
  pm10_str=String(m.nc_10p0);
  pm10_str.toCharArray(pm_10,5);
  mqtt.publish("KRU/pm10",pm_10);

  char batt_tx[6];
  String batt_str;
  batt_str=String(batt);
  batt_str.toCharArray(batt_tx,5);
  mqtt.publish("KRU/batt",batt_tx);  
  
  //14.078199, 100.604132
  char lat_tx[19];
  String lat_str;
  lat_str=PASS_LAT;
  lat_str.toCharArray(lat_tx,18);
  mqtt.publish("KRU/lat",lat_tx);
  //mqtt.publish("/doophoon_12/0");

  char lon_tx[19];
  String lon_str;
  lon_str=PASS_LON;
  lon_str.toCharArray(lon_tx,18);
  mqtt.publish("KRU/lon",lon_tx);
  //mqtt.publish("/doophoon_12/0");
  
  mqtt.loop();
}
  }
}

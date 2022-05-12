#include <Arduino.h>
#include <ModbusMaster.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Setup_var.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <DNSServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <AsyncElegantOTA.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

AsyncWebServer server_ota(80);
ESP8266WiFiMulti wifiMulti;
const uint32_t connectTimeoutMs = 5000;
//const char* mqtt_server = "test.mosquitto.org";
const char* mqtt_server = "192.168.110.126";
WiFiClient espClient;
PubSubClient client(espClient);
LiquidCrystal_I2C lcd(0x27,20,4);
DHT dht(DHTPIN, DHT11);
SoftwareSerial DPMSerial;
ModbusMaster node;
/*fungsi digunakan untuk Digital Interrupt Push buttons
*/
void IRAM_ATTR interrupthandler(){
  lcd.clear();
  move_display += 1;
  if (move_display >= 4){
    move_display = 0;
  }
  time_get_pm2120 = millis();
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  DPMSerial.begin(9600, SWSERIAL_8E2, max485_rx, max485_tx);
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, Degree);
  pinMode(INT_EXT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_EXT), interrupthandler, CHANGE);
  inisialisasi_t();
  start_millis();
  lcd.clear();
  snprintf(topiq1, 32, "kwh/%s/log", id_dev);
  snprintf(topiq2, 32, "kwh/%s/reset", id_dev);
}
void loop() {
  // put your main code here, to run repeatedly:
  if (reset_pm2120 == true){
    lcd.setCursor(0,0); lcd.printf("PM2120 ID:"); lcd.printf(":%s", id_dev);
    lcd.setCursor(0,1); lcd.printf("Reset Energy.....");
    if (millis() - time_reset_pm2120 >= 2000){
      time_reset_pm2120 = millis();
      rst_energy = get_int16_PM2120(ID_meter1, reset_energy1);
      Serial.println(rst_energy);
      lcd.setCursor(0,2); lcd.printf("data: %d", rst_energy);
      reset_pm2120 = false;
      lcd.setCursor(0,3); lcd.printf(">SUKSES RESET ENERGY");
      lastMsg = 0;
      send_mqtt_json(1);
      lcd.clear();
    }
  }else if(reset_pm2120 == false){
    taks_handler();
  }
}
void taks_handler(){
  AsyncElegantOTA.loop();
  display_lcd(move_display);
  get_temperature();
  get_PM2120();
  scan_wifi();
  if (energy_pm2120 >= 0.01){
    send_mqtt_json(0);
  }
}
void inisialisasi_t(){
  //inisialisasi_wifimulti_dan_OTA_UPDATE();
  inisialisasi_wifi_dan_OTA_UPDATE();
  dht.begin();
  Serial.println("Inisialisasi DHT Berhasil");
  lcd.setCursor(0, 3); lcd.printf("->init DHT OK");
  randomSeed(micros());
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}
void inisialisasi_wifimulti_dan_OTA_UPDATE(){
  Serial.println((String) "Connected to WiFi Local IP:" + local_IP);
  lcd.setCursor(0, 0); lcd.printf("->Connected WiFI IP:");
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP("AVIAN SSO", "avian2021");
  wifiMulti.addAP("AIC-Lab", "LabAIC2020");
  wifiMulti.addAP("Fahmi", "tanyadulu");
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(1000);
    Serial.print('.');
  }
  local_IP = WiFi.localIP().toString();
  soft_APIP = WiFi.softAPIP().toString();
  scan_ssid = WiFi.SSID();
  Serial.println((String) "Connected to WiFi Local IP:" + local_IP);
  if (WiFi.status() == WL_CONNECTED)
  {

  } else{
    Serial.println("Can't connect! Enter WiFi config mode...");
  }
  lcd.setCursor(2, 1); lcd.print(local_IP);
  lcd.setCursor(0, 2); lcd.printf("->SSID:"); lcd.print(scan_ssid);
  server_ota.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
  request->send(200, "text/plain", "Hi! I am ESP8266. KWH Meter ready to get Frimware updatae");
  });
  AsyncElegantOTA.begin(&server_ota);
  server_ota.begin();
  Serial.println("server started you can open with ip/update");
}
void inisialisasi_wifi_dan_OTA_UPDATE(){
    lcd.setCursor(0, 0); lcd.printf("->Connected WiFI IP:");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) 
    {
     delay(500);
     Serial.print("*");
    }
    local_IP = WiFi.localIP().toString();
    soft_APIP = WiFi.softAPIP().toString();
    scan_ssid = WiFi.SSID();
    Serial.println((String) "Connected to WiFi Local IP:" + local_IP);
    if (WiFi.status() == WL_CONNECTED)
    {

    } else{
      Serial.println("Can't connect! Enter WiFi config mode...");
    }
    lcd.setCursor(2, 1); lcd.print(local_IP);
    lcd.setCursor(0, 2); lcd.printf("->SSID:"); lcd.print(scan_ssid);
    server_ota.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP8266. KWH Meter ready to get Frimware updatae");
    });
    AsyncElegantOTA.begin(&server_ota);
    server_ota.begin();
    Serial.println("server started you can open with ip/update");
}
void reconnectwifi_multi(){
  if (wifiMulti.run(connectTimeoutMs) == WL_CONNECTED) {
    
  }else {
    Serial.println("WiFi not connected!");
  }
}
void reconnectwifi(){
  if(WiFi.status() != WL_CONNECTED)
  {
    lcd.clear();
    display_lcd(102);
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    while(WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin(ssid, password);
      Serial.print(".");
      lcd.setCursor(0, 1); lcd.print(local_IP);
      lcd.setCursor(0, 2); lcd.printf("->SSID:"); lcd.print(scan_ssid);
      lcd.setCursor(0, 3); lcd.printf("error koneksi wifi");
      delay(5000);     
    }
    Serial.println("x");
    Serial.println("\nConnected.");
    lcd.clear();
  }
}
void start_millis(){
  time_serial = millis();
  lcd_time = millis();
  scan_wifi_time = millis();
}
String getValue(String data, char separator, int index){
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
      if (data.charAt(i) == separator || i == maxIndex) {
          found++;
          strIndex[0] = strIndex[1] + 1;
          strIndex[1] = (i == maxIndex) ? i+1 : i;
      }
    }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
void callback(char* topic, byte* payload, unsigned int length) {
  //Serial.println((String)"Message arrived [" + topic + "]");
  String topic_set = (String)topic;
  String topic_kind = getValue(topic_set, '/', 0);
  String topic_id = getValue(topic_set, '/', 1);
  String topic_task = getValue(topic_set, '/', 2);
  char message[length + 1];
  strncpy (message, (char*)payload, length);
  message[length] = '\0';
  Serial.println(message);
  if(topic_kind == "kwh"){
    if(topic_id == id_dev){
      if(topic_task == "reset"){
        lcd.clear();
        time_reset_pm2120 = millis();
        reset_pm2120 = true;
      }
    }
  }
}
void reconnect(){
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.publish(topiq2, "Connect kwh to web");
      client.subscribe(topiq2);
      client.publish(topiq1, "Connect to web dev");
      client.subscribe(topiq1);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      lcd.clear();
      lcd.setCursor(0,0); lcd.printf("Attempting MQTT");
      lcd.setCursor(0,1); lcd.printf("connection failed,-");
      lcd.setCursor(0,2); lcd.printf("rc:"); lcd.print(client.state());
      lcd.setCursor(0,3); lcd.printf("coba 5 detik lagi");
      delay(1000);
    }
  }
  lcd.setCursor(0,1); lcd.printf("connection sukes,-");
  lcd.clear();
}
void send_mqtt_json(uint8_t index_json){
  if (!client.connected()) {
    reconnectwifi();
    //reconnectwifi_multi();
    reconnect();
  }
  client.loop();
  StaticJsonDocument<256> doc;
  switch (index_json)
  {
  case 0:
    doc["device_id"] = id_dev;
    doc["signal"] = scan_rssi;
    doc["kwh"] = energy_pm2120;
    doc["daya"] = power_pm2120;
    doc["volt"] = voltage_pm2120;
    doc["arus"] = current_pm2120;
    doc["time"] = gabung_time;
    doc["ip"] = local_IP;
    doc["note"] = " ";
    doc["suhu"] = temperature;
    break;
  case 1:
    doc["device_id"] = id_dev;
    doc["reset"] = "berhasil";
    interval_msg = 0;
    break;
  }
  if (millis() - lastMsg > interval_msg){
    lastMsg = millis();
    char msg[256];
    int b =serializeJson(doc, msg);
    Serial.print("publishing bytes = "); Serial.println(b,DEC);
    client.publish(topiq1, msg);
    interval_msg = 120000;
  }
}
float get_float_PM2120(uint8_t _addr, uint16_t _REG){
  node.begin(_addr,DPMSerial);
  uint16_t hasil = node.readHoldingRegisters(_REG, 2);
  if (hasil == node.ku8MBSuccess){
    union { uint32_t _x; float _f;} _u;
    float _z = _u._f;
    _u._x = ((unsigned long)node.getResponseBuffer(0) << 16) | node.getResponseBuffer(1);
    _z = _u._f;
    return _z;
  }else {
    Serial.println((String) "koneksi modbus poll gagal dengan alamat : " + (String)_addr + " dengan register :" + _REG);
    set_var_pm = false;
    return 0;
  }
}
uint16 get_int16_PM2120(uint8_t addr, uint16_t REG){
  node.begin(addr, DPMSerial);
  uint16_t result = node.readHoldingRegisters(REG, 2);
  uint16_t time_Result[2];
  uint8_t j;
  if (result == node.ku8MBSuccess){
    for (j = 0; j < 2; j++)
    {
        time_Result[j] = node.getResponseBuffer(j);
    }
    return time_Result[0];
  } else {
    Serial.println((String) "koneksi modbus poll gagal dengan alamat : " + (String)addr + " dengan register :" + REG);
    set_var_time = false;
    return 0;   
  }
}
void get_PM2120(){
  if (millis() - time_get_pm2120 >= 1020){
    time_get_pm2120 = millis();
    get_pm2120_move += 1;
    if (get_pm2120_move >= 2){
      get_pm2120_move = 0;
    }
  }
  switch (get_pm2120_move)
  {
  case 0:
      set_var_pm = true;
      for (uint8_t i = 0; i < total_reg1; i++){
        DATA_METER1 [i] = get_float_PM2120(ID_meter1, Reg_addr_pm2120[i]);
      }
      if (set_var_pm == true){
        voltage_pm2120 = DATA_METER1[0];
        current_pm2120 = DATA_METER1[1];
        power_pm2120 = DATA_METER1[2];
        energy_pm2120 = DATA_METER1[3];
        sa_pm2120 = DATA_METER1[4];  
        qa_pm2120 = DATA_METER1[5];
        pf_pm2120 = DATA_METER1[6];
        frequency_pm2120 = DATA_METER1[7];
        Serial.printf("V: %fVAC | I: %fA | P: %fkW | E: %fkWh | SA: %fKVA | Q: %fKVAR | Pf: %f | F: %f\r\n", 
        voltage_pm2120, current_pm2120, power_pm2120, energy_pm2120, sa_pm2120, qa_pm2120, pf_pm2120, frequency_pm2120);
      }
    break;
  case 1:
    set_var_time = true;
    for (uint8_t i = 0; i < total_reg_time; i++){
      TIME_METER1 [i] = get_int16_PM2120(ID_meter1, reg_addr_time[i]);
    }
    if (set_var_time == true){
      tahun = TIME_METER1[0];
      bulan = TIME_METER1[1];
      hari = TIME_METER1[2];
      jam = TIME_METER1[3];
      menit = TIME_METER1[4];
      detik = TIME_METER1[5];
      Serial.printf("TIME:%d:%d:%d %d/%d/%d\r\n",detik, menit, jam, hari, bulan, tahun);
      snprintf(gabung_time, 50, "%2d-%02d-%02d %02d:%02d:%02d", tahun, bulan, hari, jam, menit, detik);
    }
  }
}
void get_temperature(){
  if (millis() - dht_time >= 2000){
    dht_time = millis();
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
    fahrenheit = dht.readTemperature(true);
    hic = dht.computeHeatIndex(temperature, humidity, false);
    hif = dht.computeHeatIndex(fahrenheit, humidity);
    if (isnan(humidity) || isnan(temperature) || isnan(fahrenheit)) {
      lcd.clear();
      lcd.setCursor(1,0);
      Serial.println(F("Failed to read from DHT sensor!"));
      lcd.printf("Error read DHT sensor!");
    }
  }
}
void scan_wifi(){
  if(millis() - scan_wifi_time >= 500){
    scan_wifi_time = millis();
    int n = WiFi.scanNetworks();
    if(n == 0){
      Serial.println("tidak ada koneksi internet");
      display_lcd(102);
    } else { 
      scan_rssi = WiFi.RSSI();
      rssi_percen = dBmtoPercentage(scan_rssi);
      //Serial.println((String) scan_ssid + " RSSI :" + scan_rssi + "dBm (" + rssi_percen + "%) " + local_IP);
      if (rssi_percen < 5){
        bar_rssi = 0;
      } else if(rssi_percen >= 5 && rssi_percen <= 25){
        bar_rssi = 1;
      } else if(rssi_percen >= 25 && rssi_percen <= 50){
        bar_rssi = 2;
      } else if(rssi_percen >= 50 && rssi_percen <= 75){
        bar_rssi = 3;
      } else if(rssi_percen >= 75 && rssi_percen <= 100){
        bar_rssi = 4;
      }
    }
  }
} 
int dBmtoPercentage(int dBm){
  int quality;
    if(dBm <= RSSI_MIN){
        quality = 0;
    }
    else if(dBm >= RSSI_MAX){  
        quality = 100;
    }else{
        quality = 2 * (dBm + 100);
    }
     return quality;
}
void display_lcd(uint8_t index_display){
  if (millis() - lcd_time >= 500){
    lcd_time = millis();
    switch (index_display)
    {
    case 0:
      lcd.setCursor(0,0); lcd.printf("E:"); lcd.print(energy_pm2120,3); lcd.printf("kWh");
      lcd.setCursor(0,1); lcd.printf("V:"); lcd.print(voltage_pm2120,1); lcd.printf("V"); 
      lcd.setCursor(10,1); lcd.printf("I:"); lcd.print(current_pm2120); lcd.printf("A");
      lcd.setCursor(0,2); lcd.printf("P:"); lcd.print(power_pm2120,1); lcd.printf("W");
      lcd.setCursor(10,2); lcd.printf("S:"); lcd.print(sa_pm2120,1); lcd.printf("KVA"); 
      lcd.setCursor(0,3); lcd.printf("pf:"); lcd.print(pf_pm2120);
      lcd.setCursor(10,3); lcd.printf("T:"); lcd.print(temperature, 1); lcd.write(0); lcd.printf("C");
      break;
    case 1:
      lcd.setCursor(0,0); lcd.printf("SSID:"); lcd.print(scan_ssid);
      lcd.setCursor(0,1); lcd.printf("RSSI:"); lcd.print(scan_rssi);
      lcd.setCursor(10,1); lcd.print(rssi_percen); lcd.print(char('%'));
      lcd.setCursor(15,1); lcd.printf("Bar:"); lcd.print(bar_rssi);
      lcd.setCursor(0,2); lcd.printf("IP:"); lcd.print(local_IP);
      lcd.setCursor(0,3); lcd.printf("APIP:"); lcd.print(soft_APIP);
      break;
    case 2:
      lcd.setCursor(0,0); lcd.printf("All Temperature:");
      lcd.setCursor(0,1); lcd.printf("T :"); lcd.print(temperature, 1); lcd.write(0); lcd.printf("C"); 
      lcd.setCursor(11,1); lcd.printf("F :"); lcd.print(fahrenheit, 1); lcd.write(0); lcd.printf("F"); 
      lcd.setCursor(0,2); lcd.printf("Hu:"); lcd.print(humidity, 1); lcd.write(0);
      lcd.setCursor(11,2); lcd.printf("Ic:"); lcd.print(hic, 1); lcd.write(0);lcd.printf("C");
      lcd.setCursor(0,3); lcd.printf("If:"); lcd.print(hif, 1); lcd.write(0); lcd.printf("F"); 
      lcd.setCursor(12,3); lcd.printf("Press->");
      break;
    case 3:
      lcd.setCursor(0,0); lcd.printf("PM2120 ID"); lcd.printf(":%s", id_dev);
      lcd.setCursor(0,1); lcd.printf("TIME: %d:%d:%d", jam, menit, detik);
      lcd.setCursor(0,2); lcd.printf("DATE: %d/%d/%d",hari, bulan, tahun);
      lcd.setCursor(0,3); lcd.printf("send MQTT: sukses");
      break;
    case 102: 
      lcd.setCursor(0,0); lcd.printf("Failed Connect WiFi");
      lcd.setCursor(0,1); lcd.printf("ssid:"); lcd.print(scan_ssid);
      lcd.setCursor(0,2); lcd.printf("Finding the Nearest");
      lcd.setCursor(0,3); lcd.printf("WiFi to Connected");
      break;
    }
  }
}











/*
C:\Program Files\Mosquitto>mosquitto_sub -h test.mosquitto.org -t "kwh/1/log" -v
mosquitto_sub -h test.mosquitto.org -t "general/currenttime" -v
mosquitto_sub -h test.mosquitto.org -t "kwh/1/reset" -v
Mosquitto>mosquitto_pub -h test.mosquitto.org -t "kwh/1/reset" -m 0
data selesai
subs
C:\Program Files\Mosquitto>mosquitto_sub -h 192.168.110.126 -t "kwh/1/log" -v
mosquitto_sub -h 192.168.110.126 -t "general/currenttime" -v
mosquitto_sub -h 192.168.110.126 -t "kwh/1/reset" -v
publish
mosquitto_sub -h 192.168.110.126 -t "kwh/1/log" -m data
mosquitto_pub -h 192.168.110.126 -t "kwh/1/reset" -m 0
mosquitto_sub -h 192.168.110.126 -t "general/currenttime" -m data
*/
/*
Deklarasi variable pin
pin i2c terbalik GPIO5->SDA, GPIO4->SCL, DHT11->GPIO13, 

*/
#define max485_rx 14
#define max485_tx 12
#define DHTPIN 13
#define INT_EXT 2
#define RSSI_MAX -50
#define RSSI_MIN -70

//definisi register pm2120 from https://www.aggsoft.com/serial-data-logger/tutorials/modbus-data-logging/schneider-electric-em6400ng-pm2100-pm2200.htm
#define reset_energy1        13624 //reset energy 3199, 3200 atau 3251 13624

#define REG_tahun            1836 //tahun
#define REG_bulan            1837 //bulan
#define REG_hari             1838 //hari
#define REG_jam              1839 //jam
#define REG_menit            1840 //menit
#define REG_detik            1841 //detik

#define REG_ULL_avg         3025 //Voltage L L Average (VAC)
#define REG_It              3009 //Current avg (A)
#define REG_Pt              3059 //Active power Total (KW)
#define REG_energy          2701 //Active Positif energy delivered (into load) 	(kWh)2699
#define REG_St              3075 //Apparent power Total (KVA)
#define REG_Qt              3067 //Reactive power Total (kVAR)
#define REG_Pft             3083 //Power factor total
#define REG_Hz              3109 //Frequency

#define total_reg1 8
#define total_reg_time 6
#define ID_meter1 1 //Addres pm2120
#define MSG_BUFFER_SIZE	(50)
const char* id_dev = "2";
const char* ssid = "AVIAN SSO";
const char* password = "avian2021";
///pass aic lab: LabAIC2020
// const char* ssid = "AIC-Lab";
// const char* password = "LabAIC2020";

uint16_t Reg_addr_pm2120[8] = {
 REG_ULL_avg,
 REG_It,
 REG_Pt,
 REG_energy,
 REG_St,
 REG_Qt,
 REG_Pft,
 REG_Hz,
};
uint16_t reg_addr_time[6] = {
  REG_tahun,
  REG_bulan,
  REG_hari,
  REG_jam,
  REG_menit,
  REG_detik,
};
float DATA_METER1 [total_reg1];
uint16_t TIME_METER1 [total_reg_time];

unsigned long time_serial = 0;
unsigned long dht_time = 0;
unsigned long lcd_time = 0;
unsigned long scan_wifi_time = 0;
unsigned long time_get_pm2120 = 0;
unsigned long time_reset_pm2120 = 0;
unsigned long lastMsg = 0;
unsigned long interval_msg = 50000;

float humidity = 0.0, temperature = 0.0, fahrenheit = 0.0;
float hic = 0.0, hif = 0.0;
float voltage_pm2120 = 0.0, current_pm2120 = 0.0, power_pm2120 = 0.0;
float energy_pm2120 = 0.0, frequency_pm2120 = 0.0, pf_pm2120 = 0.0;
float sa_pm2120 = 0.0, qa_pm2120 = 0.0;
uint16_t rst_energy = 0;
uint8_t move_display = 0, move_display_prev, get_pm2120_move = 0;
String scan_ssid, local_IP, soft_APIP;
int scan_rssi, rssi_percen;
uint8_t bar_rssi;
uint16_t tahun = 0, jam = 0, detik = 0, menit = 0, hari = 0, bulan = 0;
bool set_var_pm = true, set_var_time = true, reset_pm2120 = false;
char gabung_time[50];
char msg[MSG_BUFFER_SIZE];
char topiq1[32];
char topiq2[32];
byte Degree[] = {
  B00111,
  B00101,
  B00111,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};

void taks_handler();
void inisialisasi_t();
void inisialisasi_wifimulti_dan_OTA_UPDATE();
void inisialisasi_wifi_dan_OTA_UPDATE();
void reconnectwifi_multi();
void reconnectwifi();
void start_millis();
String getValue(String data, char separator, int index);
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void send_mqtt_json(uint8_t index_json);
float get_float_PM2120(uint8_t _addr, uint16_t _REG);
uint16 get_int16_PM2120(uint8_t addr, uint16_t REG);
void get_PM2120();
void get_temperature();
void scan_wifi();
int dBmtoPercentage(int dBm);
void display_lcd(uint8_t index_display);

















; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

[env:esp07]
platform = espressif8266
board = esp07
framework = arduino
monitor_speed = 115200
lib_deps = 
	marcoschwartz/LiquidCrystal_I2C@^1.1.4
	4-20ma/ModbusMaster@^2.0.1
	adafruit/DHT sensor library@^1.4.3
	adafruit/Adafruit Unified Sensor@^1.1.4
	knolleary/PubSubClient@^2.8
	bblanchon/ArduinoJson@^6.18.5
	me-no-dev/ESPAsyncTCP@^1.2.2
	ayushsharma82/AsyncElegantOTA@^2.2.6
	ottowinter/ESPAsyncTCP-esphome@^1.2.3
	esphome/ESPAsyncWebServer-esphome@^2.1.0
























  backup program 2


  #include <Arduino.h>
#include <ModbusMaster.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Setup_var.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <DNSServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* mqtt_server = "192.168.110.126";
WiFiClient espClient;
PubSubClient client(espClient);
LiquidCrystal_I2C lcd(0x27,20,4);
DHT dht(DHTPIN, DHT11);
SoftwareSerial DPMSerial;
ModbusMaster node;
/*fungsi digunakan untuk Digital Interrupt Push buttons
*/
void IRAM_ATTR interrupthandler(){
  lcd.clear();
  move_display += 1;
  if (move_display >= 4){
    move_display = 0;
  }
  time_get_pm2120 = millis();
}
int dBmtoPercentage(int dBm){
  int quality;
    if(dBm <= RSSI_MIN){
        quality = 0;
    }
    else if(dBm >= RSSI_MAX){  
        quality = 100;
    }else{
        quality = 2 * (dBm + 100);
    }
     return quality;
}
void send_data(){
  if (energy_pm2120 >= 0.01){
    send_mqtt_json(1);
    // send_mqtt_json(0);
  }
  if (reset_pm == true || rst_count >= 200){
    send_mqtt_json(2);
    delay(100);
    ESP.restart();
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  DPMSerial.begin(9600, SWSERIAL_8E2, max485_rx, max485_tx);
  Wire.setClock(10000);
  lcd.init();
  lcd.backlight();
  pinMode(INT_EXT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_EXT), interrupthandler, CHANGE);
  inisialisasi_t();
  lcd.clear();
  snprintf(topiq1, 32, "kwh/%s/log", id_dev);
  snprintf(topiq2, 32, "kwh/%s/reset", id_dev);
  snprintf(topiq3, 32, "kwh/%s/status", id_dev);
}
void loop() {
  // put your main code here, to run repeatedly:
  display_lcd(move_display);
  //get_temperature();
  // get_PM2120();
  reconnectwifi();
  energy_pm2120 = 100;
  send_data();
}
void inisialisasi_t(){
  inisialisasi_wifi();
  dht.begin();
  Serial.println("Inisialisasi DHT Berhasil");
  lcd.setCursor(0, 3); lcd.printf("->init DHT OK");
  randomSeed(micros());
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}
void inisialisasi_wifi(){
    lcd.setCursor(0, 0); lcd.printf("->Connected WiFI IP:");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) 
    {
     delay(500);
     Serial.print("*");
    }
    local_IP = WiFi.localIP().toString();
    scan_ssid = WiFi.SSID();
    mac_address = WiFi.macAddress();
    Serial.println((String) "Connected to WiFi Local IP:" + local_IP);

    lcd.setCursor(2, 1); lcd.print(local_IP);
    lcd.setCursor(0, 2); lcd.printf("->SSID:"); lcd.print(scan_ssid);
}
void reconnectwifi(){
  if(WiFi.status() != WL_CONNECTED)
  {
    lcd.clear();
    display_lcd(102);
    while(WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin(ssid, password);
      delay(5000);
      rst_count += 1;
      if(rst_count >= 20){
        ESP.restart();
      }
    }
    lcd.clear();
  } else {
    scan_rssi = WiFi.RSSI();
    rssi_percen = dBmtoPercentage(scan_rssi);
  }
}
String getValue(String data, char separator, int index){
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
      if (data.charAt(i) == separator || i == maxIndex) {
          found++;
          strIndex[0] = strIndex[1] + 1;
          strIndex[1] = (i == maxIndex) ? i+1 : i;
      }
    }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
void callback(char* topic, byte* payload, unsigned int length) {
  String topic_set = (String)topic;
  String topic_kind = getValue(topic_set, '/', 0);
  String topic_id = getValue(topic_set, '/', 1);
  String topic_task = getValue(topic_set, '/', 2);
  char message[length + 1];
  strncpy (message, (char*)payload, length);
  message[length] = '\0';
  Serial.println(message);
  if(topic_kind == "kwh"){
    if(topic_id == id_dev){
      if(topic_task == "reset"){
        reset_pm = true;
      }
    }
  }
}
void reconnect(){
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      client.subscribe(topiq3);
      client.subscribe(topiq2);
      client.subscribe(topiq1);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      lcd.clear();
      lcd.setCursor(0,0); lcd.printf("Attempting MQTT");
      lcd.setCursor(0,1); lcd.printf("connection failed,-");
      lcd.setCursor(0,2); lcd.printf("rc:"); lcd.print(client.state());
      lcd.setCursor(0,3); lcd.printf("coba 5 detik lagi");
      delay(1000);
    }
  }
  lcd.setCursor(0,1); lcd.printf("connection sukes,-");
  lcd.clear();
}
void send_mqtt_json(uint8_t index_json){
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  StaticJsonDocument<256> doc;
  if (index_json == 0){
    doc["device_id"] = id_dev;
    doc["kwh"] = energy_pm2120;
    char msg1[256];
    int a = serializeJson(doc, msg1);
    if (jam == 9 && flag_jam_9 == false){
      interval_time_log = 1024;
      rst_count += 1;
    }else if(jam != 9){
      interval_time_log = 90000;
      flag_jam_9 = false;
    }
    if (millis() - lastMsg > interval_time_log){
      lastMsg = millis();
      Serial.print("send log:"); Serial.println(a,DEC);
      client.publish(topiq1, msg1);
      if(jam == 9){
        flag_jam_9 = true;
      }
    }
  }else if(index_json == 1){
    doc["device_id"] = id_dev;
    doc["signal"] = scan_rssi;
    doc["kwh"] = energy_pm2120;
    doc["kvar"] = qa_pm2120;
    doc["volt"] = voltage_pm2120;
    doc["arusA"] = current_a;
    doc["arusB"] = current_b;
    doc["arusC"] = current_c;
    doc["arusT"] = current_pm2120;
    doc["pf"] = pf_pm2120;
    doc["suhu"] = temperature;
    doc["time"] = gabung_time;
    doc["ip"] = local_IP;
    doc["mac"] = mac_address;
    doc["note"] = "";
    char msg2[256];
    int b = serializeJson(doc, msg2);
    if (millis() - msg_status_time >= 1020){
      msg_status_time = millis();
      Serial.print("send status:"); Serial.println(b,DEC);
      client.publish(topiq3, msg2);
    }
  }else if(index_json == 2){
    doc["device_id"] = id_dev;
    doc["reset"] = 1;
    char msg3[256];
    int c = serializeJson(doc, msg3);
    Serial.print("send status:"); Serial.println(c,DEC);
    client.publish(topiq2, msg3);
  }
}
float get_float_PM2120(uint8_t _addr, uint16_t _REG){
  node.begin(_addr,DPMSerial);
  uint16_t hasil = node.readHoldingRegisters(_REG, 2);
  if (hasil == node.ku8MBSuccess){
    union { uint32_t _x; float _f;} _u;
    float _z = _u._f;
    _u._x = ((unsigned long)node.getResponseBuffer(0) << 16) | node.getResponseBuffer(1);
    _z = _u._f;
    return _z;
  }else {
    Serial.println((String) "koneksi modbus poll gagal dengan alamat : " + (String)_addr + " dengan register :" + _REG);
    set_var_pm = false;
    return 0;
  }
}
uint16 get_int16_PM2120(uint8_t addr, uint16_t REG){
  node.begin(addr, DPMSerial);
  uint16_t result = node.readHoldingRegisters(REG, 2);
  uint16_t time_Result[2];
  uint8_t j;
  if (result == node.ku8MBSuccess){
    for (j = 0; j < 2; j++)
    {
        time_Result[j] = node.getResponseBuffer(j);
    }
    return time_Result[0];
  } else {
    Serial.println((String) "koneksi modbus poll gagal dengan alamat : " + (String)addr + " dengan register :" + REG);
    set_var_time = false;
    return 0;   
  }
}
void get_PM2120(){
  if (millis() - time_get_pm2120 >= 1020){
    time_get_pm2120 = millis();
    get_pm2120_move += 1;
    if (get_pm2120_move >= 2){
      get_pm2120_move = 0;
    }
  }
  switch (get_pm2120_move)
  {
  case 0:
      set_var_pm = true;
      for (uint8_t i = 0; i < total_reg1; i++){
        DATA_METER1 [i] = get_float_PM2120(ID_meter1, Reg_addr_pm2120[i]);
      }
      if (set_var_pm == true){
        voltage_pm2120 = DATA_METER1[0];
        current_pm2120 = DATA_METER1[1];
        current_a = DATA_METER1[2];
        current_b = DATA_METER1[3];
        current_c = DATA_METER1[4];
        power_pm2120 = DATA_METER1[5];
        power_pm2120 *= -1;
        energy_pm2120 = DATA_METER1[6];
        sa_pm2120 = DATA_METER1[7];  
        qa_pm2120 = DATA_METER1[8];
        pf_pm2120 = DATA_METER1[9];
        frequency_pm2120 = DATA_METER1[10];
        Serial.printf("V: %fVAC | It: %fA | Ia: %fA | Ib: %fA | Ic: %fA | P: %fkW | E: %fkWh | SA: %fKVA | Q: %fKVAR | Pf: %f | F: %f\r\n", 
        voltage_pm2120, current_pm2120, current_a, current_b, current_c, power_pm2120, energy_pm2120, sa_pm2120, qa_pm2120, pf_pm2120, frequency_pm2120);
      }
    break;
  case 1:
    set_var_time = true;
    for (uint8_t i = 0; i < total_reg_time; i++){
      TIME_METER1 [i] = get_int16_PM2120(ID_meter1, reg_addr_time[i]);
    }
    if (set_var_time == true){
      tahun = TIME_METER1[0];
      bulan = TIME_METER1[1];
      hari = TIME_METER1[2];
      jam = TIME_METER1[3];
      menit = TIME_METER1[4];
      detik = TIME_METER1[5];
      Serial.printf("TIME:%d:%d:%d %d/%d/%d\r\n",detik, menit, jam, hari, bulan, tahun);
      snprintf(gabung_time, 50, "%2d-%02d-%02d %02d:%02d:%02d", tahun, bulan, hari, jam, menit, detik);
    }
  }
}
void get_temperature(){
  if (millis() - dht_time >= 2000){
    dht_time = millis();
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
    fahrenheit = dht.readTemperature(true);
    hic = dht.computeHeatIndex(temperature, humidity, false);
    hif = dht.computeHeatIndex(fahrenheit, humidity);
    if (isnan(humidity) || isnan(temperature) || isnan(fahrenheit)) {
      lcd.clear();
      lcd.setCursor(1,0);
      Serial.println(F("Failed to read from DHT sensor!"));
      lcd.printf("Error read DHT sensor!");
    }
  }
}
void display_lcd(uint8_t index_display){
  if (millis() - lcd_time >= 500){
    lcd_time = millis();
    switch (index_display)
    {
    case 0:
      lcd.setCursor(0,0); lcd.printf("E:"); lcd.print(energy_pm2120,3); lcd.printf("kWh");
      lcd.setCursor(0,1); lcd.printf("V:"); lcd.print(voltage_pm2120,1); lcd.printf("V"); 
      lcd.setCursor(10,1); lcd.printf("I:"); lcd.print(current_pm2120); lcd.printf("A");
      lcd.setCursor(0,2); lcd.printf("P:"); lcd.print(power_pm2120,1); lcd.printf("W");
      lcd.setCursor(10,2); lcd.printf("S:"); lcd.print(sa_pm2120,1); lcd.printf("KVA"); 
      lcd.setCursor(0,3); lcd.printf("pf:"); lcd.print(pf_pm2120);
      lcd.setCursor(10,3); lcd.printf("T:"); lcd.print(temperature, 1); lcd.write(0); lcd.printf("C");
      break;
    case 1:
      lcd.setCursor(0,0); lcd.printf("SSID:"); lcd.print(scan_ssid);
      lcd.setCursor(0,1); lcd.printf("RSSI:"); lcd.print(scan_rssi);
      lcd.setCursor(10,1); lcd.print(rssi_percen); lcd.print(char('%'));
      lcd.setCursor(15,1); lcd.printf("Bar:"); lcd.print(bar_rssi);
      lcd.setCursor(0,2); lcd.printf("IP:"); lcd.print(local_IP);
      lcd.setCursor(0,3); lcd.print(mac_address);
      break;
    case 2:
      lcd.setCursor(0,0); lcd.printf("All Temperature:");
      lcd.setCursor(0,1); lcd.printf("T :"); lcd.print(temperature, 1); lcd.write(0); lcd.printf("C"); 
      lcd.setCursor(11,1); lcd.printf("F :"); lcd.print(fahrenheit, 1); lcd.write(0); lcd.printf("F"); 
      lcd.setCursor(0,2); lcd.printf("Hu:"); lcd.print(humidity, 1); lcd.write(0);
      lcd.setCursor(11,2); lcd.printf("Ic:"); lcd.print(hic, 1); lcd.write(0);lcd.printf("C");
      lcd.setCursor(0,3); lcd.printf("If:"); lcd.print(hif, 1); lcd.write(0); lcd.printf("F"); 
      lcd.setCursor(12,3); lcd.printf("Press->");
      break;
    case 3:
      lcd.setCursor(0,0); lcd.printf("PM2120 ID"); lcd.printf(":%s", id_dev);
      lcd.setCursor(0,1); lcd.printf("TIME: %d:%d:%d", jam, menit, detik);
      lcd.setCursor(0,2); lcd.printf("DATE: %d/%d/%d",hari, bulan, tahun);
      lcd.setCursor(0,3); lcd.printf("send MQTT: sukses");
      break;
    case 102: 
      lcd.setCursor(0,0); lcd.printf("Failed Connect WiFi");
      lcd.setCursor(0,1); lcd.printf("ssid:"); lcd.print(scan_ssid);
      lcd.setCursor(0,2); lcd.printf("Finding the Nearest");
      lcd.setCursor(0,3); lcd.printf("WiFi to Connected");
      break;
    }
  }
}





sett updatae


/*
C:\Program Files\Mosquitto>mosquitto_sub -h test.mosquitto.org -t "kwh/1/log" -v
mosquitto_sub -h test.mosquitto.org -t "general/currenttime" -v
mosquitto_sub -h test.mosquitto.org -t "kwh/1/reset" -v
Mosquitto>mosquitto_pub -h test.mosquitto.org -t "kwh/1/reset" -m 0
data selesai
subs
C:\Program Files\Mosquitto>mosquitto_sub -h 192.168.110.126 -t "kwh/1/log" -v
mosquitto_sub -h 192.168.110.126 -t "general/currenttime" -v
mosquitto_sub -h 192.168.110.126 -t "kwh/1/reset" -v
publish
mosquitto_sub -h 192.168.110.126 -t "kwh/1/log" -m data
mosquitto_pub -h 192.168.110.126 -t "kwh/1/reset" -m 0
mosquitto_sub -h 192.168.110.126 -t "general/currenttime" -m data
*/
/*
Deklarasi variable pin
pin i2c terbalik GPIO5->SDA, GPIO4->SCL, DHT11->GPIO13, 

*/
#define max485_rx 14
#define max485_tx 12
#define DHTPIN 13
#define INT_EXT 2
#define RSSI_MAX -50
#define RSSI_MIN -70

//definisi register pm2120 from https://www.aggsoft.com/serial-data-logger/tutorials/modbus-data-logging/schneider-electric-em6400ng-pm2100-pm2200.htm
#define reset_energy1        3200 //reset energy 6209 3199, 3200 atau 3251 13624

#define REG_tahun            1836 //tahun
#define REG_bulan            1837 //bulan
#define REG_hari             1838 //hari
#define REG_jam              1839 //jam
#define REG_menit            1840 //menit
#define REG_detik            1841 //detik

#define REG_ULL_avg         3025 //Voltage L L Average (VAC)
#define REG_Ia              2999 //Current avg (A)
#define REG_Ib              3001 //Current avg (A)
#define REG_Ic              3003 //Current avg (A)
#define REG_It              3009 //Current avg (A)
#define REG_Pt              3059 //Active power Total (KW)
#define REG_energy          2701 //Active Positif energy delivered (into load) 	(kWh)2699
#define REG_St              3075 //Apparent power Total (KVA)
#define REG_Qt              3067 //Reactive power Total (kVAR)
#define REG_Pft             3083 //Power factor total
#define REG_Hz              3109 //Frequency

#define total_reg1 11
#define total_reg_time 6
#define ID_meter1 1 //Addres pm2120
#define MSG_BUFFER_SIZE	(50)
const char* id_dev = "3";
// const char* ssid = "AVIAN SSO";
// const char* password = "avian2021";
///pass aic lab: LabAIC2020
const char* ssid = "AIC-Lab";
const char* password = "LabAIC2020";

uint16_t Reg_addr_pm2120[11] = {
 REG_ULL_avg,
 REG_It,
 REG_Ia,
 REG_Ib,
 REG_Ic,
 REG_Pt,
 REG_energy,
 REG_St,
 REG_Qt,
 REG_Pft,
 REG_Hz,
};
uint16_t reg_addr_time[6] = {
  REG_tahun,
  REG_bulan,
  REG_hari,
  REG_jam,
  REG_menit,
  REG_detik,
};
float DATA_METER1 [total_reg1];
uint16_t TIME_METER1 [total_reg_time];

unsigned long dht_time = 0;
unsigned long lcd_time = 0;
unsigned long time_get_pm2120 = 0;
unsigned long lastMsg = 0;
unsigned long msg_status_time = 0;
unsigned long interval_time_log = 60000;

long address_eeprom = 0;
long address_eeprom_bulan = 6;

float humidity = 0.0, temperature = 0.0, fahrenheit = 0.0;
float hic = 0.0, hif = 0.0;
float voltage_pm2120 = 0.0, current_pm2120 = 0.0, power_pm2120 = 0.0;
float current_a = 0.0, current_b = 0.0, current_c = 0.0;
float energy_pm2120 = 0.0, frequency_pm2120 = 0.0, pf_pm2120 = 0.0;
float sa_pm2120 = 0.0, qa_pm2120 = 0.0;
uint16_t rst_energy = 0;
uint8_t move_display = 0, get_pm2120_move = 0;
String scan_ssid, local_IP, mac_address;
int scan_rssi, rssi_percen;
uint8_t bar_rssi;
uint8_t rst_count = 0;
uint16_t tahun = 0, jam = 0, detik = 0, menit = 0, hari = 0, bulan = 0;
bool set_var_pm = true, set_var_time = true;
bool reset_pm = false;
bool flag_jam_9 = false;
char gabung_time[50];
char msg[MSG_BUFFER_SIZE];
char topiq1[32];
char topiq2[32];
char topiq3[32];

void inisialisasi_t();
void inisialisasi_wifi();
void reconnectwifi();
String getValue(String data, char separator, int index);
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void send_mqtt_json(uint8_t index_json);
float get_float_PM2120(uint8_t _addr, uint16_t _REG);
uint16 get_int16_PM2120(uint8_t addr, uint16_t REG);
void get_PM2120();
void get_temperature();
void display_lcd(uint8_t index_display);
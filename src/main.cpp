#include <Arduino.h>
#include <ArduinoOTA.h>
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
  interval_lcd = 1000;
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
    if (jam != jam_prev){
      send_mqtt_json(3);
      jam_prev = jam;
    }
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
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, Degree);
  pinMode(INT_EXT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_EXT), interrupthandler, CHANGE);
  inisialisasi_t();
  lcd.clear();
  snprintf(topiq2, 32, "kwh/%s/reset", id_dev);
  snprintf(topiq3, 32, "kwh/%s/status", id_dev);
  snprintf(topiq1, 32, "kwh/%s/log", id_dev);
  ArduinoOTA.begin();
}
void loop() {
  // put your main code here, to run repeatedly:
  ArduinoOTA.handle();
  display_lcd(move_display);
  get_temperature();
  get_PM2120();
  // energy_pm2120 = 1;
  reconnectwifi();
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
    uint32_t notConnectedCounter = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
     delay(100);
     Serial.print("*");
     notConnectedCounter++;
      if(notConnectedCounter > 150) { // Reset board if not connected after 5s
          Serial.println("Resetting due to Wifi not connecting...");
          ESP.restart();
      }
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
      client.subscribe(topiq2);
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
  if(index_json == 1){
    doc["device_id"] = id_dev; 
    doc["signal"] = scan_rssi;
    doc["kwh"] = energy_pm2120; //energy
    doc["kvar"] = qa_pm2120;    //daya apparent
    doc["kva"] = sa_pm2120;
    doc["volt"] = voltage_pm2120; // tegangan
    doc["arusA"] = current_a;   //arus line a
    doc["arusB"] = current_b;   //arus line b
    doc["arusC"] = current_c;   //arus line c
    // doc["arusT"] = current_pm2120;// arus rata rata
    doc["pf"] = pf_pm2120;      //power factor
    doc["suhu"] = temperature;  //temperature C
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
  }else if(index_json == 3){
    doc["device_id"] = id_dev;
    doc["kwh"] = energy_pm2120; //energy
    doc["time"] = gabung_time;
    char msg1[256];
    int c = serializeJson(doc, msg1);
    Serial.print("send status:"); Serial.println(c,DEC);
    client.publish(topiq1, msg1);
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
        qa_pm2120 *= -1;
        pf_pm2120 = DATA_METER1[9];
        pf_pm2120 *= -1;
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
      lcd.printf("Error DHT sensor!");
    }
  }
}
void display_lcd(uint8_t index_display){
  if (millis() - lcd_time >= interval_lcd){
    if(interval_lcd == 1000){
      interval_lcd = 500;
    }
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
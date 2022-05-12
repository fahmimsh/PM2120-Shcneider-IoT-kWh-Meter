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

//REGISTER PM212O
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

const char* id_dev = "2"; // LV T638 RINJANI
const char* ssid = "AVIAN SSO";
const char* password = "avian2021";
///pass aic lab: LabAIC2020
// const char* ssid = "AIC-Lab";
// const char* password = "LabAIC2020";

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
unsigned long msg_status_time = 0;

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
uint16_t jam_prev = 0;
bool set_var_pm = true, set_var_time = true;
bool reset_pm = false;
bool flag_jam_9 = false;
char topiq2[32];
char topiq3[32];
char topiq1[32];
char gabung_time[50];
uint16_t interval_lcd = 500;
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
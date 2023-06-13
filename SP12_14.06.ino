/*<TEAM_ID>,<MISSION_TIME>,<PACKET_COUNT>,<PACKET_TYPE>,<SP_ALTITUDE>,<SP_TEMP>,<SP_ROTATION_RATE>,<SP_VOLTAGE>
  led D5, BUZZER D2, RESET BUTONU D6
  reset buton D6
  voltage divider a0*/
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>
#include <SPI.h>
#include <SD.h>
#include <MPU9255.h>// include MPU9255 library
File myFile;
MPU9255 mpu;
SoftwareSerial NANO(7, 8);
#define pin_buzzer 3//3//2
#define pin_led 5//5
//#define pin_reset_button 6//6
const int chipSelect = 10;
Adafruit_BMP280 bmp;
int cmd_calibration = 0;
int software_state;
int lowbyte, highbyte;
int packet_count = 1;
unsigned long time_telemetry = 0;
double sp1_voltage;
float rpm;
int i , sayac;
float b, toplam, ortalama;
float gForceX, gForceY, gForceZ;
float totaldeg;
long accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;
unsigned long time1 = 0;
double ref_altitude;
unsigned long timer_mission;
int reset_ = 0;
String cmd_received[6];
double sp_bmp_altitude;
double sp_bmp_temperature;
int deger = 0;
float vout;
float vin;
int R1 = 1000;
int R2 = 10000;
int cmd_on = 0;
long timer_command = 0;
int team_ID = 3210;
int a;
float rot;
String giden ;
int rst = 0;
#define SP1 0
#define SP2 1
//*******************************
unsigned char packet[20];
#pragma pack(push,1)
typedef struct
{
  uint8_t startcondition ;
  uint16_t TeamID;
  uint8_t saat;
  uint8_t dk;
  uint8_t sn;
  uint16_t PacketCount;
  bool PacketType;
  int16_t SPAltitude;
  int16_t SPTemp;
  int16_t RotationRate;
  uint32_t Voltage;
  uint8_t stopcondition ;
} GrizuPayloadTypedef;
#pragma pack(pop)
//***************************
GrizuPayloadTypedef MyPayload;
void setup() {
  Serial.begin(19200);
  NANO.begin(19200);
  Wire.begin();
  bmp.begin();
  SD.begin(chipSelect);
  mpu.init();// Initialize MPU9255 chip
  mpu.set_gyro_scale(3);//value/16.4 = degree per second
  //pinMode(pin_reset_button, INPUT);
  pinMode(pin_led, OUTPUT);
  pinMode(pin_buzzer, OUTPUT);
  digitalWrite(pin_buzzer , HIGH);
  delay(1000);
  digitalWrite(pin_buzzer , LOW);
  myFile = SD.open("Flight_3205_SP2.txt", FILE_WRITE);
  myFile.println("TEAM_ID,MISSION_TIME,PACKET_COUNT,PACKET_TYPE,MODE,SP1_RELEASED,SP2_RELEASED,ALTITUDE,TEMP,VOLTAGE,GPS_TIME,GPS_LATITUDE,GPS_LONGITUDE,GPS_ALTITUDE,GPS_SATS,SOFTWARE_STATE,SP1_PACKET_COUNT,SP2_PACKET_COUNT,CMD_ECHO ");

}

void loop() {

  digitalWrite(pin_led, HIGH);
  millis();
  mpu.read_gyro();
  voltage();
  RPM();
   Serial.println( mpu.gz);
  Serial.println(rpm);
  delay(80);
  if (sayac == 1)
  {
    if (millis() - timer_command > 1000)
    {
      digitalWrite(pin_buzzer, LOW);
      sayac = 0;
    }
  }
  if (rst == 0 )
  {
    _reset();
    rst++;
  }
  received_serial();
  sensor_bmp();
  if (cmd_on == 1)
  {
    if (millis() - time_telemetry >= 990)
    { time_telemetry = millis();
      giden = team_ID;
      giden += ",";
      if (hour() < 10)
        giden += "0";
      giden += hour();
      giden += ":";
      if (minute() < 10)
        giden += "0";
      giden += minute();
      giden += ":";
      if (second() < 10)
        giden += "0";
      giden += second();
      giden += ",";
      giden += packet_count;
      giden += ",";
      giden += "SP2";
      giden += ",";
      giden += sp_bmp_altitude;
      giden += ",";
      giden += sp_bmp_temperature;
      giden += ",";
      giden += abs(rpm);
      giden += ",";
      giden +=  vin;
      giden += ",";
      NANO.println(giden);
      Serial.println(giden);
      packet_count++;
      giden = "";
      eeprom_write(100, 101, packet_count);
      myFile = SD.open("Flight_3210_SP2.csv", FILE_WRITE);
      myFile.println(giden);
      myFile.close();
    }
  }
}

void received_serial()
{
  if (NANO.available() > 0 )
  {
    NANO.setTimeout(10);
    for ( i = 0; i < 6; i++) {
      cmd_received [i] = NANO.readStringUntil (',');
    } Serial.print(cmd_received[0]); Serial.print(cmd_received[1]); Serial.print(cmd_received[2]); Serial.print(cmd_received[3]);
    if (cmd_received[0] == "CMD" && cmd_received[1] == "3210" && cmd_received[2] == "SP2" && cmd_received[3] == "ON")
    {
      cmd_on = 1;
      eeprom_write(10, 11, cmd_on);
      digitalWrite(pin_buzzer, HIGH);
      timer_command = millis();
      sayac = 1;
    }
    if (cmd_received[0] == "CMD" && cmd_received[1] == "3210" && cmd_received[2] == "SP2" && cmd_received[3] == "OFF")
    {
      cmd_on = 0;
      eeprom_write(10, 11, cmd_on);
      digitalWrite(pin_buzzer, HIGH);
      timer_command = millis();
      sayac = 1;
    }
    if (cmd_received[0] == "CMD" && cmd_received[1] == "3210" && cmd_received[2] == "CBN")
    {
      cmd_calibration = 1;
      sifirla();
      digitalWrite(pin_buzzer, HIGH);
      timer_command = millis();
      sayac = 1;
    }
    if (cmd_received[0] == "CMD" && cmd_received[1] == "3210" && cmd_received[2] == "ST")
    {
      long int  hh = cmd_received[3].toInt();
      long  int  mm = cmd_received[4].toInt();
      long int  ss = cmd_received[5].toInt();
      sensor_rtc(hh, mm, ss);
      digitalWrite(pin_buzzer, HIGH);
      timer_command = millis();
      sayac = 1;
    }
    NANO.flush();
  }
}
int sensor_rtc(int hh, int mm, int ss) {
  setTime(hh, mm, ss, 10, 6, 2021);
}
void sensor_bmp()
{
  sp_bmp_altitude = bmp.readAltitude() - ref_altitude;
  sp_bmp_temperature = bmp.readTemperature();
}

int eeprom_write(int low, int high, int values)
{
  byte lowbyte = lowByte(values);
  byte highbyte = highByte(values);
  EEPROM.update(low, lowbyte);
  EEPROM.update(high, highbyte);
}
int eeprom_read(int read_low, int read_high)
{
  byte low = EEPROM.read(read_low);
  byte high = EEPROM.read(read_high);
  int recovery = low + (high << 8);
  return recovery;
}
float voltage()
{
  deger = analogRead(A0);
  vin = (deger * 4.17) / 1023.0;//SP2 referans 4.17 SP1 referansı:
  //vin = (vout / R2) * (R1 + R2);
  return vin;
}
void EEPROM_clear() {
  for (int i = 0 ; i < 120 ; i++) {
    EEPROM.update(i, 0);
    delay(10);
  }
}
float RPM() {
  float DPS = mpu.gz / 32.8;//degree per second
  rpm = DPS / 6;
  return rpm;
}
void sifirla()
{
  digitalWrite(pin_buzzer, HIGH);
  digitalWrite(pin_led, HIGH);
  timer_command = millis();
  sayac = 1;
  EEPROM_clear();
  packet_count = 1;
  eeprom_write(100, 101, packet_count);
  if (cmd_calibration == 1)
  {
    ref_altitude = bmp.readAltitude();
    eeprom_write(50, 51, ref_altitude);
  }
}
void _reset()
{
  ref_altitude = eeprom_read(50, 51);
  cmd_on = eeprom_read(10, 11);
  packet_count = eeprom_read(100, 101);
}

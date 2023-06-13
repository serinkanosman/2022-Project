//Conainer Pin Out; Buzzer--D20 LED--D17 Reset Buton--D16 Melt of Fishline 1--D29 Melt of Fishline 2--D37 GY91--SDA2,SCL2 Gerilim Bölücü--A16 Kamera--D14 Servo Motor--D38
#define TIME_HEADER  "T"
#include <TimeLib.h>
#include <SD.h>
#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <medianFilter.h>
////////////////bonus///////////////////////
#include <Adafruit_BNO055.h>
#include <PID_v1.h>
#define encodPinA1      14                       // Quadrature encoder A pin
#define encodPinB1      15                       // Quadrature encoder B pin
#define M1              17                       // PWM outputs to L298N H-Bridge motor driver module
#define M2              16
//*******************************************
unsigned long timer_hsr = 0;
unsigned long prev_timer_hsr = 0;

//***************************************
double kp = 8 , ki = 0.5 , kd = 0.01;            // modify for optimal performance  kp = 8 , ki = 0.5 , kd = 0.01
double input = 0, outputt = 0, setpoint = 0;
volatile long encoderPos = 0;
int gyrooperational ;
int prevgyro = 0;
int counter = 0 ;
int careful;//a
PID myPID(&input, &outputt, &setpoint, kp, ki, kd, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'
//******************************************
double xPos = 0, yPos = 0, headingVel = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample
//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
/////////////////////////////////////////////
int cam_pin = 38;
/////////////////////////////////////////////
int i, j, rl;
int f = 0;
int altitude_change = 0;
/////////////FILTER/////////////////////////
medianFilter Filter; //Median filter is used for the SIMP deviations
///////////Voltage divide//////////////////////////
float vout;
float vin;
///////////HYPSOMETRIC FORMULA & SIMP///////////////////////////
float temp = 288.15;
float sbt = 0.0065;
float Po = 101325;
float sbt_pow = 0.1902;
double TEMP = 15;
long int SIMP  ;
////////////SD CARD//////////////////////////
File myFile;
const int chipSelect = BUILTIN_SDCARD;
////////////COMMANDS Received//////////////////////////
int cmd_controll_mode = 0;
int cmd_controll_telemetry = 0;
int cmd_controll_calibration = 0;
/////////Telemetry////////////////////////////
unsigned long sp1_relayed = 0; // 5 min controll
unsigned long sp2_relayed = 0;
///////////////////////////////////////////////
int packet_controll;
int team_ID = 3205;
int packet_count = 1;
String Telemetry;
int software_state = 0;
char F, S, N, R;
char SP1_Released = 'N'; // N not released , R released
char SP2_Released = 'N' ;
String cmd_echo = "Grizu263-CanSat21"; //last received command
char mode = 'F'; // F flight mode, S simulation mode
int packet_type = 0;
////////////ALTITUDE (BMP280 & SIM)//////////////////////////
double bmp_altitude = 0;
double bmp_altitude_calibrate = 0;
double bmp_temperature = 0;
double altitude_simp = 0;
double altitude_prev = 0;
double altitude_simp_calibrate = 0;
double altitude_operational = 0;
double altitude_raw = 0;
Adafruit_BMP280 bmp;
///////////EEPROM///////////////////////////
int lowbyte, highbyte;
///////////Relayed SP packet counts///////////////////////////
int SP1_packet_count = 0 ;
int SP2_packet_count = 0 ;
int sp1_stop = 0;
int sp2_stop = 0;
///////////TIMERS///////////////////////////
unsigned long timer_telemetry = 0;
unsigned long trig = 0;
///////////SERIAL RECIVED///////////////////////////
String cmd_received[6];
///////////////////////////PINOUT config/////////////////////////////
//const int pin_reset        = 16 ;
const int pin_buzzer       = 24 ;//23 20 -24*
const int pin_led          = 25 ;//22 17 -25*
const int pin_fishline_sp1 = 29 ;
const int pin_fishline_sp2 = 37 ;
///////////////////////////XBEEs RX & TX config/////////////////////////////
const int XBEE_GCS_Rx = 7;
const int XBEE_GCS_Tx = 8;
////////////////////////////////////////////////////////
SoftwareSerial XBEE_GCS(XBEE_GCS_Rx, XBEE_GCS_Tx);
///////////////////////////*********************/////////////////////////////
#define cmd_controll_telemetry_high 31
#define cmd_controll_telemetry_low 30
#define cmd_controll_mode_high 41
#define cmd_controll_mode_low 40
#define software_state_high 21
#define software_state_low 20
#define bmp_calibrate_high 61
#define bmp_calibrate_low 60
#define SP1_packet_count_low 100
#define SP1_packet_count_high 101
#define SP2_packet_count_low 102
#define SP2_packet_count_high 103
#define altitude_simp_calibrate_high 111
#define altitude_simp_calibrate_low 110
#define _packet_high 151
#define _packet_low 150
#define simp_altitude_low 152
#define simp_altitude_high 153
/////////////////////////////////
int Kt = 0;
int a = 0 ;
int b = 0;
int rst = 0;
void setup() {
  Serial.begin(9600);
  Serial5.begin(19200);//sp1
  Serial4.begin(19200);//sp2
  Serial3.begin(19200);//c
  bmp.begin();
  SD.begin(chipSelect);
  //SD.remove("Flight_3205_C.csv");
  pinMode(pin_led , OUTPUT);
  pinMode(pin_buzzer , OUTPUT);
  pinMode(cam_pin, OUTPUT);
  pinMode(pin_fishline_sp1 , OUTPUT);
  pinMode(pin_fishline_sp2 , OUTPUT);
  digitalWrite(pin_buzzer , HIGH );
  delay (1992);
  digitalWrite(pin_buzzer , LOW);
  myFile = SD.open("Flight_3205_C.csv", FILE_WRITE);
  myFile.println("TEAM_ID,MISSION_TIME,PACKET_COUNT,PACKET_TYPE,MODE,SP1_RELEASED,SP2_RELEASED,ALTITUDE,TEMP,VOLTAGE,GPS_TIME,GPS_LATITUDE,GPS_LONGITUDE,GPS_ALTITUDE,GPS_SATS,SOFTWARE_STATE,SP1_PACKET_COUNT,SP2_PACKET_COUNT,CMD_ECHO ");
  myFile.close();
  myFile = SD.open("Flight_3205_SP1.csv", FILE_WRITE);
  myFile.println("TEAM_ID,MISSION_TIME,PACKET_COUNT,PACKET_TYPE,ALTITUDE,TEMP,VOLTAGE,ROTATION_RATE");
  myFile.close();
  myFile = SD.open("Flight_3205_SP2.csv", FILE_WRITE);
  myFile.println("TEAM_ID,MISSION_TIME,PACKET_COUNT,PACKET_TYPE,ALTITUDE,TEMP,VOLTAGE,ROTATION_RATE");
  myFile.close();
  //////////////////////////////////bno+bonus///////////////////////////
  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  attachInterrupt(14, encoder, FALLING);               // update encoder position
  //analogWriteFrequency(A10, 480000);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    // while (1);
  }
  delay(1000);
  ///////////////////////////////////*** kamera tetik ***///////////////////////////////////////
}
void loop() {
  digitalWrite(pin_led, HIGH);
  millis();
  if (rst == 0) {
    reset_();
    rst++;
  }
  camera_stabilization();
  cmd_();
  sensor_bmp();
  sensor_voltage();
  sent();
  relay_payloads();
  software_states_();
  release_trigger_shutdown();
}
void cmd_()
{
  if (XBEE_GCS.available() > 0 )
  {
    XBEE_GCS.setTimeout(10);
    for ( i = 0; i < 6; i++) {
      cmd_received [i] = XBEE_GCS.readStringUntil (',');
    }
    Serial.print(cmd_received[0]); Serial.print(cmd_received[1]); Serial.print(cmd_received[2]); Serial.println(cmd_received[3]);

    if (cmd_received[0] == "CMD" && cmd_received[1] == "3205" && cmd_received[2] == "SIMP" )
    {
      cmd_echo = "";
      SIMP = cmd_received[3].toInt();
      cmd_echo = "SP";
      cmd_echo += cmd_received[3];
      altitude_simp = ((((pow(Po / SIMP, sbt_pow) - 1) * (temp)) / sbt) );//hypsometr
      //altitude_raw = ((((pow(Po / SIMP, sbt_pow) - 1) * (temp)) / sbt) * 100);
      //Serial.println(altitude_raw);
      //altitude_raw = Filter.run(altitude_raw) ;
      //altitude_simp = altitude_raw / 100;
      //Serial.println(altitude_simp);
      eeprom_write(simp_altitude_low, simp_altitude_high, altitude_simp);
    }
    if (cmd_received[0] == "2")
    {
      cmd_echo = "MANUEL_SEPERATE1";
      analogWrite(pin_fishline_sp1, 140);
      digitalWrite(pin_buzzer, HIGH);
      delay(1000);
      analogWrite(pin_fishline_sp1, 0);
      digitalWrite(pin_buzzer, LOW);
      cmd_received[0] = "";
    }
    if (cmd_received[0] == "3")
    {
      cmd_echo = "MANUEL_SEPERATE2";
      analogWrite(pin_fishline_sp2, 140);
      digitalWrite(pin_buzzer, HIGH);
      delay(1000);
      analogWrite(pin_fishline_sp2, 0);
      digitalWrite(pin_buzzer, LOW);
      cmd_received[0] = "";
    }
    if (cmd_received[0] == "CMD" && cmd_received[1] == "3205" && cmd_received[2] == "CX" && cmd_received[3] == "ON")
    {
      cmd_controll_telemetry = 1;
      cmd_echo = "CXON";
      eeprom_write(cmd_controll_telemetry_low, cmd_controll_telemetry_high, cmd_controll_telemetry);
    }
    if (cmd_received[0] == "CMD" && cmd_received[1] == "3205" && cmd_received[2] == "CX" && cmd_received[3] == "OFF") {
      cmd_controll_telemetry = 0;
      cmd_echo = "CXOFF";
      telemetry_();
      eeprom_write(cmd_controll_telemetry_low, cmd_controll_telemetry_high, cmd_controll_telemetry);
    }
    if (cmd_received[0] == "CMD" && cmd_received[1] == "3205" && cmd_received[2] == "SIM" && cmd_received[3] == "ENABLE")
    {
      cmd_echo = "ENABLE";
    }
    if (cmd_received[0] == "CMD" && cmd_received[1] == "3205" && cmd_received[2] == "SIM" && cmd_received[3] == "DISABLE" )
    {
      cmd_echo = "DISABLE";
      cmd_controll_mode = 0;
      mode = 'F';
      eeprom_write(cmd_controll_mode_low, cmd_controll_mode_high, cmd_controll_mode);
    }
    if (cmd_received[0] == "CMD" && cmd_received[1] == "3205" && cmd_received[2] == "SIM" && cmd_received[3] == "ACTIVATE" )
    {
      cmd_controll_mode = 1;
      cmd_echo = "ACTIVATE";
      mode = 'S';
      eeprom_write(cmd_controll_mode_low, cmd_controll_mode_high, cmd_controll_mode);
    }
    if (cmd_received[0] == "CMD" && cmd_received[1] == "3210" && cmd_received[2] == "SP1" && cmd_received[3] == "ON")
    {
      cmd_echo = "SP1ON";
      Serial5.println("CMD,3210,SP1,ON,");
    }
    if (cmd_received[0] == "CMD" && cmd_received[1] == "3210" && cmd_received[2] == "SP1" && cmd_received[3] == "OFF")
    {
      cmd_echo = "SP1OFF";
      Serial5.println("CMD,3210,SP1,OFF,");
    }
    if (cmd_received[0] == "CMD" && cmd_received[1] == "3210" && cmd_received[2] == "SP2" && cmd_received[3] == "ON")
    {
      Serial4.println("CMD,3210,SP2,ON,");
      cmd_echo = "SP2ON";
    }
    if (cmd_received[0] == "CMD" && cmd_received[1] == "3210" && cmd_received[2] == "SP2" && cmd_received[3] == "OFF")
    {
      cmd_echo = "SP2OFF";
      Serial4.println("CMD,3210,SP2,OFF,");
    }
    if (cmd_received[0] == "CMD" && cmd_received[1] == "3205" && cmd_received[2] == "CBN")
    {
      Serial4.println("CMD,3210,CBN,");
      Serial5.println("CMD,3210,CBN,");
      cmd_controll_calibration = 1;
      cmd_controll_cbn();
      cmd_echo = "CBN";
    }
    if (cmd_received[0] == "CMD" && cmd_received[1] == "3205" && cmd_received[2] == "ST") {
      long int  hh = cmd_received[3].toInt();
      long int mm = cmd_received[4].toInt();
      long int ss = cmd_received[5].toInt();
      Serial.println(hh);
      Serial.println(mm);
      Serial.println(ss);
      sensor_rtc(hh, mm, ss);
      Serial5.print(cmd_received[0]);
      Serial5.print(",");
      Serial5.print("3210");
      Serial5.print(",");
      Serial5.print(cmd_received[2]);
      Serial5.print(",");
      Serial5.print(cmd_received[3]);
      Serial5.print(",");
      Serial5.print(cmd_received[4]);
      Serial5.print(",");
      Serial5.print(cmd_received[5]);
      Serial5.println(",");
      Serial4.print(cmd_received[0]);
      Serial4.print(",");
      Serial4.print("3210");
      Serial4.print(",");
      Serial4.print(cmd_received[2]);
      Serial4.print(",");
      Serial4.print(cmd_received[3]);
      Serial4.print(",");
      Serial4.print(cmd_received[4]);
      Serial4.print(",");
      Serial4.print(cmd_received[5]);
      Serial4.print(",");
      cmd_echo = "SetTime";
    }
    XBEE_GCS.flush();
  }
}
int eeprom_write(int low, int high, int values)
{
  byte lowbyte = lowByte(values);
  byte highbyte = highByte(values);
  EEPROM.update(low, lowbyte);
  EEPROM.update(high, highbyte);
}
///////////////////////////*********************/////////////////////////////
int eeprom_read(int read_low, int read_high)
{
  byte  low = EEPROM.read(read_low);
  byte high = EEPROM.read(read_high);
  int recovery = low + (high << 8);
  return recovery;
}
float sensor_voltage()
{
  vout = (analogRead(A16) * 3.33) / 1024.0;
  vin = vout * 2 ;
  // volt = analogRead(A16) * (referans / 1023) * 11;
  //return analogRead(A16) * (3.3 / 1023) * 9;
}
void sent()
{
  if (cmd_controll_telemetry == 1) {
    if ( millis() - timer_telemetry >= 990)
    {
      if (altitude_operational == altitude_prev && software_state == 4)
      {
        altitude_change++;
      }
      timer_telemetry = millis();
      telemetry_();
      packet_count ++;
      eeprom_write(_packet_low, _packet_high, packet_count);
    }
  }
}
void telemetry_()
{
  Telemetry = "";
  Telemetry += team_ID;
  Telemetry += ",";
  if (hour() < 10)
    Telemetry += "0";
  Telemetry += hour();
  Telemetry += ":";
  if (minute() < 10)
    Telemetry += "0";
  Telemetry += minute();
  Telemetry += ":";
  if (second() < 10)
    Telemetry += "0";
  Telemetry += second();
  Telemetry += ",";
  Telemetry += packet_count;
  Telemetry += ",";
  Telemetry += 'C';
  Telemetry += ",";
  Telemetry += mode;
  Telemetry += ",";
  Telemetry += SP1_Released;
  Telemetry += ",";
  Telemetry += SP2_Released;
  Telemetry += ",";
  Telemetry += altitude_operational;
  Telemetry += ",";
  Telemetry +=  bmp.readTemperature();
  Telemetry += ",";
  Telemetry += vin;
  Telemetry += ",";
  Telemetry += "00:00:00";
  Telemetry += ",";
  Telemetry += "41.4495";
  Telemetry += ",";
  Telemetry += "31.7588";
  Telemetry += ",";
  Telemetry += "0.00";
  Telemetry += ",";
  Telemetry += "0";
  Telemetry += ",";
  Telemetry += software_state;
  Telemetry += ",";
  Telemetry += SP1_packet_count;
  Telemetry += ",";
  Telemetry += SP2_packet_count;
  Telemetry += ",";
  Telemetry += cmd_echo;
  Telemetry += ",";

  XBEE_GCS.println(Telemetry);
  myFile = SD.open("Flight_3205_C.csv", FILE_WRITE);
  if (software_state == 5)
  {  
    XBEE_GCS.println(Telemetry);
    cmd_controll_telemetry = 0 ;
  }
  Serial.println(Telemetry);
  myFile.println(Telemetry);
  myFile.close();
}
void sensor_bmp()
{
  if (cmd_controll_mode == 0)//Flight
  {
    altitude_operational = bmp.readAltitude(1013.25) - bmp_altitude_calibrate;
  }
  if (cmd_controll_mode == 1)//Simulation
  {
    mode = 'S';
    altitude_operational = altitude_simp - altitude_simp_calibrate;
  }
}
int sensor_rtc(int hh, int mm, int ss) {
  if (a == 0)
  {
    setTime(hh, mm, ss, 14, 6, 2021);
    a++;
  }
}
void EEPROM_clear() {
  for (int i = 0 ; i < 160 ; i++) {
    EEPROM.update(i, 0);
    delay(10);
  }
}
void software_states_()
{
  if ( altitude_operational  > 20 && software_state == 0 )
  {
    software_state = 1;
    eeprom_write(software_state_low, software_state_high, software_state);
  }
  else if (( software_state == 1 && altitude_operational - altitude_prev <= -10 ) && ( altitude_operational > 580))
  {
    software_state = 2;
    eeprom_write(software_state_low, software_state_high, software_state);
  }
  else if  (altitude_operational  < 510 && software_state == 2)
  {
    if (Kt == 0) {

      release_trigger(2);
    }
    Serial5.println("CMD,3210,SP1,ON,");
    software_state = 3;
    eeprom_write(software_state_low, software_state_high, software_state);
  }
  else if  (altitude_operational < 410 && software_state == 3 )
  {
    if (Kt == 1) {

      release_trigger(3);
    }
    Serial4.println("CMD,3210,SP2,ON,");
    software_state = 4;
    eeprom_write(software_state_low, software_state_high, software_state);
  }
  else if  (altitude_change > 11 || ( altitude_operational < 20 && software_state == 4)  )
  {
    software_state = 5;
    eeprom_write(software_state_low, software_state_high, software_state);
    buzzer();
  }
  /////////////PEAK CONTROL ///////////////
  if ( packet_count - packet_controll == 1)
  {
    altitude_prev = altitude_operational ;
  }
  packet_controll = packet_count;
  if (software_state == 5)
  {
    if (millis() - sp1_relayed >= 300000) {
      sp1_stop = 1;
    }

    if (millis() - sp2_relayed >= 300000) {
      sp2_stop = 1;
    }
  }
}
void buzzer()
{
  digitalWrite(pin_buzzer, HIGH);
}
void cmd_controll_cbn()
{
  EEPROM_clear();
  packet_count = 1;
  SP1_packet_count = 0 ;
  SP2_packet_count = 0 ;
  SP1_Released = 'N';
  SP2_Released = 'N';
  software_state = 0;
  if (cmd_controll_calibration == 1 && cmd_controll_mode == 1)
  {
    Serial.print("SIMP:");
    Serial.println(altitude_simp);
    altitude_simp_calibrate = altitude_simp;
    cmd_controll_calibration = 0;
    eeprom_write(altitude_simp_calibrate_low, altitude_simp_calibrate_high, altitude_simp_calibrate);
  }
  if (cmd_controll_calibration == 1 && cmd_controll_mode == 0)
  {
    bmp_altitude_calibrate = bmp.readAltitude(1013.25);
    cmd_controll_calibration = 0;
    eeprom_write(bmp_calibrate_low, bmp_calibrate_high, bmp_altitude_calibrate);
  }
  eeprom_write(_packet_low, _packet_high, packet_count);
  eeprom_write(SP1_packet_count_low, SP1_packet_count_high, SP1_packet_count);
  eeprom_write(SP2_packet_count_low, SP2_packet_count_high, SP2_packet_count);
  eeprom_write(bmp_calibrate_low, bmp_calibrate_high, bmp_altitude_calibrate);
  eeprom_write(software_state_low, software_state_high, software_state);
  //SD.remove("Flight_3205_SP1.csv");
  //SD.remove("Flight_3205_C.csv");
  //SD.remove("Flight_3205_SP2.csv");
  digitalWrite(pin_buzzer, HIGH);
  digitalWrite(pin_led, HIGH);
  delay(1000);
  digitalWrite(pin_buzzer, LOW);
  digitalWrite(pin_led, LOW);
  //Serial.println("dasdasdasdasd");
}
int release_trigger(int pin)
{ if (pin == 2)
  {
    analogWrite (pin_fishline_sp1, 140);
    digitalWrite(pin_buzzer, HIGH);
    trig = millis();
    Kt++;
  }
  if (pin == 3)
  {
    analogWrite (pin_fishline_sp2, 140);
    digitalWrite(pin_buzzer, HIGH);
    trig = millis();
    Kt++;
  }
}
void release_trigger_shutdown()
{
  if (software_state == 3)
  {
    if (millis() - trig >= 980)
    {
      analogWrite (pin_fishline_sp1, 0);
      digitalWrite(pin_buzzer, LOW);
      if (f == 0) {
        Serial4.println("CMD,3210,SP1,ON,");
        f = 1;
      }
      SP1_Released = 'R';
      sp1_relayed = millis();
      trig = 0;
      eeprom_write(software_state_low, software_state_high, software_state);
    }
  }

  if (software_state == 4)
  { if (millis() - trig > 980)
    {
      analogWrite (pin_fishline_sp2, 0);
      digitalWrite(pin_buzzer , LOW);
      if (f == 1) {
        Serial4.println("CMD,3210,SP2,ON,");
        f = 2;
      }
      trig = millis();
      SP2_Released = 'R';
      sp2_relayed = millis();
      eeprom_write(software_state_low, software_state_high, software_state);
    }
  }
}


void relay_sp2()
{
  if (Serial4.available() > 0)
  {
    Serial4.setTimeout(10);
    String gelen2 = Serial4.readString();
    XBEE_GCS.print(gelen2);
    myFile = SD.open("Flight_3205_SP2.csv", FILE_WRITE);
    myFile.print(gelen2);
    myFile.close();
    Serial.println(gelen2);
    SP2_packet_count++;
  }
}
void relay_sp1()
{
  if (Serial5.available() > 0)
  {
    Serial5.setTimeout(10);
    String gelen = Serial5.readString();
    XBEE_GCS.print(gelen);
    myFile = SD.open("Flight_3205_SP1.csv", FILE_WRITE);
    myFile.print(gelen);
    myFile.close();
    Serial.println(gelen);
    SP1_packet_count++;
  }
}
void reset_() {
  packet_count = eeprom_read(_packet_low, _packet_high);
  cmd_controll_mode = eeprom_read(cmd_controll_mode_low, cmd_controll_mode_high);
  cmd_controll_telemetry = eeprom_read(cmd_controll_telemetry_low, cmd_controll_telemetry_high);
  bmp_altitude_calibrate = eeprom_read(bmp_calibrate_low, bmp_calibrate_high);
  SP1_packet_count = eeprom_read(SP1_packet_count_low, SP1_packet_count_high);
  SP2_packet_count = eeprom_read(SP2_packet_count_low, SP2_packet_count_high);
  altitude_simp_calibrate = eeprom_read(altitude_simp_calibrate_low, altitude_simp_calibrate_high);
  altitude_simp = eeprom_read(simp_altitude_low, simp_altitude_high);
  software_state = eeprom_read(software_state_low, software_state_high);
}
void relay_payloads()
{
  if (sp1_stop == 0)
  {
    relay_sp1();
  }
  if (sp2_stop == 0) {
    relay_sp2();
  }
}
void camera_stabilization()
{
  //Serial.println(yaw());

  gyrooperational = yaw();
  if (prevgyro - gyrooperational >  200 ) {

    counter++ ;
  }
  if (prevgyro - gyrooperational < -200) {
    counter-- ;
  }
  careful = ((counter * 360) + gyrooperational);
  /*Serial.print("counter degeri=");
    Serial.println(counter);
    Serial.print("prev degeri=");
    Serial.println(prevgyro);
    Serial.print("anlik degeri=");
    Serial.println(gyrooperational);
    Serial.print("a degeri=");
    Serial.println(a);*/
  //delay(10);
  timer_hsr = millis();
  if (timer_hsr - prev_timer_hsr >= 10)
  {
    prev_timer_hsr = timer_hsr;
  }
  prevgyro = gyrooperational;

  setpoint = (-careful) / 3; //1200.0; // //(analogRead(A0)*0.3515625)/3;
  /*Serial.print("setpoint = ");
    Serial.println(setpoint);// modify to fit motor and encoder characteristics, potmeter connected to A0*/
  input = encoderPos;   // data from encoder
  //Serial.print("encoder = ");
  //Serial.println(encoderPos);                      // monitor motor position
  myPID.Compute();                                    // calculate new output
  /*Serial.print("output = ");
    Serial.println(outputt);*/
  pwmOut(outputt);                                     // drive L298N H-Bridge module
}

float yaw()
{
  unsigned long tStart = micros();
  sensors_event_t orientationData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

  // velocity of sensor in the direction it's facing
  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    //enough iterations have passed that we can print the latest data
    //Serial.print("Heading: ");
    //Serial.println(orientationData.orientation.x);
    printCount = 0;
  }
  else {
    printCount = printCount + 1;
  }
  while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
  {
    //poll until the next sample is ready
  }

  return orientationData.orientation.x;
}

void pwmOut(int out) {                                // to H-Bridge board
  if (out > 0) {
    analogWrite(M1, 0);                             // drive motor CW
    analogWrite(M2, out);
  }
  else if (out < 0) {
    analogWrite(M1, abs(out));
    analogWrite(M2, 0);                        // drive motor CCW
  }
  else {
    analogWrite(M1, 0);
    analogWrite(M2, 0);
  }
}

void encoder()
{ // pulse and direction, direct port reading to save cycles
  if (digitalRead(encodPinB1) == HIGH) {
    encoderPos++;             // if(digitalRead(encodPinB1)==HIGH)   count ++;
  }
  else {
    encoderPos--;             // if(digitalRead(encodPinB1)==LOW)   count --;
  }
}

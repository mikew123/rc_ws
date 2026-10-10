/* RCX6-tof-ctrl.ino
* Mike Williamson 8/31/2025
* Based on example VL53L8CX_HelloWorld_I2C.ino modified for RP2040
*
* Send results over serial port in JSON: {"tof_ab":{"dist":[[row0],[row1]
*  ,row2],[row3],[row4],[row5],[row6],[row7]]}} integer mm
*
*/

/*
TOF Controller (RP2040)
Uses both cores of the RP2040 microcontroller to manage
Time-of-Flight (TOF) sensor data.
- Core 1: Polls and processes data from the front cluster
  of three TOF sensors, formats, and sends JSON messages
over USB serial to the host computer.
- Core 2: Polls and processes data from the rear cluster
  of TOF sensors, buffers distance measurements, and passes
them to core 1 for transmission.
Sensor data validity is filtered using the status message
from each TOF sensor, ensuring only reliable measurements
are sent. The controller uses the VL53L8CX TOF sensor
library for sensor interfacing and data acquisition.
This design enables efficient, parallel acquisition and
low-latency transfer of TOF sensor readings for real-time
processing.
*/



#include <Wire.h>
#include <Arduino_JSON.h>
#include <strings.h>
#include <vl53l8cx.h>

#include <atomic>
#include <algorithm>

// Front and Rear TOF devices on seperate I2C interface
#define DEV_I2C_F Wire
#define DEV_I2C_R Wire1

#define SerialPort Serial

#define SERIAL_HZ 1000000
//#define I2C_HZ 1000000
#define I2C_HZ 600000
#define SCAN_RATE_HZ 10

#define I2C_DEFAULT 0x29

#define I2C_ADDR_FC 0x2A
#define I2C_ADDR_FL 0x2B
#define I2C_ADDR_FR 0x2C

// Uses different I2C bus
#define I2C_ADDR_RC 0x2D
#define I2C_ADDR_RL 0x2E
#define I2C_ADDR_RR 0x2F

// RP2040 pin assignments
#define SDA_F_PIN 0
#define SCL_F_PIN 1
#define PWRN_F_PIN 2 // 3.3V Front TOF regulator enable pin

#define SYNC_PIN 5 

#define LPN_FC_PIN 6
#define LPN_FL_PIN 7
#define LPN_FR_PIN 8

#define SDA_R_PIN 14
#define SCL_R_PIN 15

#define LPN_RC_PIN 26
//#define LPN_RL_PIN 27
//#define LPN_RR_PIN 28
#define LPN_RR_PIN 27
#define LPN_RL_PIN 28

#define PWRN_R_PIN 29 // 3.3V Rear TOF regulator enable pin

// TOF config parameters
#define RESOLUTION 64
#define SHARPEN 5


// create CENTER, LEFT, RIGHT TOF instances
VL53L8CX tof_fc(&DEV_I2C_F, -1, -1);
VL53L8CX tof_fl(&DEV_I2C_F, -1, -1);
VL53L8CX tof_fr(&DEV_I2C_F, -1, -1);

VL53L8CX tof_rc(&DEV_I2C_R, -1, -1);
VL53L8CX tof_rl(&DEV_I2C_R, -1, -1);
VL53L8CX tof_rr(&DEV_I2C_R, -1, -1);

void sendJson_F(String tof, uint32_t stamp, VL53L8CX_ResultsData *results);
void sendJson_R(const char* tof, uint32_t stamp, VL53L8CX_ResultsData *results);

void getData_F(String tof);

void setI2CAddresses_F();
void configTofDevices_F();
void startMeasurements_F();

VL53L8CX_ResultsData results_F;
uint8_t newDataReady_F = 0;
uint8_t status_F;

void getData_R(String tof);

void setI2CAddresses_R();
void configTofDevices_R();
void startMeasurements_R();

VL53L8CX_ResultsData results_R;
uint8_t newDataReady_R = 0;
uint8_t status_R;

// print  string queue
// serial TX string buffer size
#define SERTXSTR_SIZE 10

char g_serialStrings[SERTXSTR_SIZE][5000];
volatile int g_serialStringWrIdx = 0;
volatile int g_serialStringRdIdx = 0;
volatile std::atomic<int16_t> g_serialStringCnt = 0;

void setI2CAddresses_F() {
  // Cycle power to I2C devices to set default I2C
  pinMode(PWRN_F_PIN, OUTPUT);
  digitalWrite(PWRN_F_PIN, LOW);
  delay(100);
  digitalWrite(PWRN_F_PIN, HIGH);

  // LPn pins disable I2C interface when LOW
  pinMode(LPN_FC_PIN, OUTPUT);
  pinMode(LPN_FL_PIN, OUTPUT);
  pinMode(LPN_FR_PIN, OUTPUT);
  // Disable the 3 TOF devices on the I2C bus
  digitalWrite(LPN_FC_PIN, LOW);
  digitalWrite(LPN_FL_PIN, LOW);
  digitalWrite(LPN_FR_PIN, LOW);

  digitalWrite(LPN_FC_PIN, HIGH);
  delay(10);
  tof_fc.set_i2c_address(I2C_ADDR_FC <<1);
  digitalWrite(LPN_FL_PIN, HIGH);
  delay(10);
  tof_fl.set_i2c_address(I2C_ADDR_FL <<1);
  digitalWrite(LPN_FR_PIN, HIGH);
  delay(10);
  tof_fr.set_i2c_address(I2C_ADDR_FR <<1);

}


void configTofDevice_F(const char* tof) {
  rp2040.wdt_reset();

  int status;
  do{
    // Configure VL53L8CX component.
    // Serial.printf("VL53L8CX %s begin\n", tof);
    if(!strncmp(tof,"tof_fc",6)) tof_fc.begin();
    if(!strncmp(tof,"tof_fl",6)) tof_fl.begin();
    if(!strncmp(tof,"tof_fr",6)) tof_fr.begin();

    // Serial.printf("VL53L8CX %s init\n", tof);
    if(!strncmp(tof,"tof_fc",6)) status = tof_fc.init();
    if(!strncmp(tof,"tof_fl",6)) status = tof_fl.init();
    if(!strncmp(tof,"tof_fr",6)) status = tof_fr.init();

    if(status == 0) {
      Serial.printf("VL53L8CX %s intialized OK\n", tof);
    } else {
      Serial.printf("VL53L8CX %s failed to initialize - reboot: status=0x%02X\n", tof, status);
      Serial.println(status,HEX);
      Serial.flush();
      delay(1000);
      rp2040.reboot();
    }
  }while(status!=0);
}

void configTofDevices_F() {
  configTofDevice_F("tof_fc");
  configTofDevice_F("tof_fl");
  configTofDevice_F("tof_fr");
}

void startMeasurement_F(const char* tof){
  rp2040.wdt_reset();
  int status = -1;
  // Start Measurements
  if(!strncmp(tof,"tof_fc",6)) {
    status = tof_fc.set_ranging_frequency_hz(SCAN_RATE_HZ);
    status |= tof_fc.set_resolution(RESOLUTION);
    status |= tof_fc.set_sharpener_percent(SHARPEN);
    status |= tof_fc.start_ranging();
  }
  if(!strncmp(tof,"tof_fl",6)) {
    status = tof_fl.set_ranging_frequency_hz(SCAN_RATE_HZ);
    status |= tof_fl.set_resolution(RESOLUTION);
    status |= tof_fl.set_sharpener_percent(SHARPEN);
    status |= tof_fl.start_ranging();
  }
  if(!strncmp(tof,"tof_fr",6)) {
    status = tof_fr.set_ranging_frequency_hz(SCAN_RATE_HZ);
    status |= tof_fr.set_resolution(RESOLUTION);
    status |= tof_fr.set_sharpener_percent(SHARPEN);
    status |= tof_fr.start_ranging();
  }
  Serial.printf("VL53L8CX %s range started status = 0x%02X\n", tof, status);
  if(status != 0) {
    delay(1000);
    Serial.flush();
    rp2040.reboot();
  }
}

void startMeasurements_F(){
  startMeasurement_F("tof_fc");
  startMeasurement_F("tof_fl");
  startMeasurement_F("tof_fr");
}


//****************************************************************
// REAR TOF devices on 2nd CPU so I made seperate code 
void setI2CAddresses_R() {
 // Cycle power to I2C devices to set default I2C
 pinMode(PWRN_R_PIN, OUTPUT);
 digitalWrite(PWRN_R_PIN, LOW);
 delay(100);
 digitalWrite(PWRN_R_PIN, HIGH);

 // LPn pins disable I2C interface when LOW
 pinMode(LPN_RC_PIN, OUTPUT);
 pinMode(LPN_RL_PIN, OUTPUT);
 pinMode(LPN_RR_PIN, OUTPUT);
 // Disable the 3 TOF devices on the I2C bus
 digitalWrite(LPN_RC_PIN, LOW);
 digitalWrite(LPN_RL_PIN, LOW);
 digitalWrite(LPN_RR_PIN, LOW);

 digitalWrite(LPN_RC_PIN, HIGH);
 delay(10);
 tof_rc.set_i2c_address(I2C_ADDR_RC <<1);
 digitalWrite(LPN_RL_PIN, HIGH);
 delay(10);
 tof_rl.set_i2c_address(I2C_ADDR_RL <<1);
 digitalWrite(LPN_RR_PIN, HIGH);
 delay(10);
 tof_rr.set_i2c_address(I2C_ADDR_RR <<1);

}

void configTofDevice_R(const char* tof) {
 rp2040.wdt_reset();

 int status = 0;
 do{
   // Configure VL53L8CX component.
   // Serial.printf("VL53L8CX %s begin\n", tof);
   if(!strncmp(tof,"tof_rc",6)) tof_rc.begin();
   if(!strncmp(tof,"tof_rl",6)) tof_rl.begin();
   if(!strncmp(tof,"tof_rr",6)) tof_rr.begin();
   

   // Serial.printf("VL53L8CX %s init\n", tof);
    if(!strncmp(tof,"tof_rc",6)) status = tof_rc.init();
    if(!strncmp(tof,"tof_rl",6)) status = tof_rl.init();
    if(!strncmp(tof,"tof_rr",6)) status = tof_rr.init();

   if(status == 0) {
     Serial.printf("VL53L8CX %s intialized OK\n", tof);
   } else {
     Serial.printf("VL53L8CX %s failed to initialize - reboot: status=0x%02X\n", tof, status);
     Serial.println(status,HEX);
     rp2040.reboot();
   }
 }while(status!=0);
}

void configTofDevices_R() {
 configTofDevice_R("tof_rc");
 configTofDevice_R("tof_rl");
 configTofDevice_R("tof_rr");
}


void startMeasurement_R(const char* tof){
 int status = -1;
 // Start Measurements
 if(!strncmp(tof,"tof_rc",6)) {
   status = tof_rc.set_ranging_frequency_hz(SCAN_RATE_HZ);
   status |= tof_rc.set_resolution(RESOLUTION);
   status |= tof_rc.set_sharpener_percent(SHARPEN);
   status |= tof_rc.start_ranging();
 }
 if(!strncmp(tof,"tof_rl",6)) {
   status = tof_rl.set_ranging_frequency_hz(SCAN_RATE_HZ);
   status |= tof_rl.set_resolution(RESOLUTION);
   status |= tof_rl.set_sharpener_percent(SHARPEN);
   status |= tof_rl.start_ranging();
 }
 if(!strncmp(tof,"tof_rr",6)) {
   status = tof_rr.set_ranging_frequency_hz(SCAN_RATE_HZ);
   status |= tof_rr.set_resolution(RESOLUTION);
   status |= tof_rr.set_sharpener_percent(SHARPEN);
   status |= tof_rr.start_ranging();
 }
 Serial.printf("VL53L8CX %s range started status = 0x%02X\n", tof, status);
 if(status != 0) {
   delay(1000);
   Serial.flush();
   rp2040.reboot();
 }
}

void startMeasurements_R(){
 startMeasurement_R("tof_rc");
 startMeasurement_R("tof_rl");
 startMeasurement_R("tof_rr");
}


//*************************************************************************
// CPU core 0 setup and loop

// Setup core0 then core1
bool setup0_done = false;
bool setup1_done = false;

void setup() {
  // Initialize serial for output.
  SerialPort.begin(SERIAL_HZ);
  delay(1000);

  // enable the RP2040 watchdog timer for startup issues
  // Only 1 watchdog for both cores - not ideal an issue in 1 core may not be timedout
  rp2040.wdt_begin(83000); // 8.3 sec max timer

  // Initialize I2C bus.
  DEV_I2C_F.setSDA(SDA_F_PIN);
  DEV_I2C_F.setSCL(SCL_F_PIN);
  DEV_I2C_F.setClock(I2C_HZ);
  DEV_I2C_F.begin();

  setI2CAddresses_F();
  configTofDevices_F();
  startMeasurements_F();

  // enable the RP2040 watchdog timer that is reset when TOF data is received
  // rp2040.wdt_begin(1000); // 1 second timer

  setup0_done = true;

  // Waiting for 2nd core to finish setup()
  while(!setup1_done) delay(10);

  // enable the RP2040 watchdog timer that is reset when TOF data is received
  rp2040.wdt_begin(1000); // 1 second timer
}

void loop(void) {
  getJsonMsgs();
  getData_F("tof_fc");
  getData_F("tof_fl");
  getData_F("tof_fr");
  serialTx(); // messages from 2nd core
}


//*************************************************************************
// CPU core 1 setup and loop
// The 2nd core is used for one of the TOF sensor clusters
void setup1() {
  // Wait for main setup to get ready for 2nd core
  while(!setup0_done) delay(10);

  // Initialize I2C bus.
 DEV_I2C_R.setSDA(SDA_R_PIN);
 DEV_I2C_R.setSCL(SCL_R_PIN);
 DEV_I2C_R.setClock(I2C_HZ);
 DEV_I2C_R.begin();

 setI2CAddresses_R();
 configTofDevices_R();
 startMeasurements_R();

  setup1_done = true;
}

void loop1() {
  delay(10);
 getData_R("tof_rc");
 getData_R("tof_rl");
 getData_R("tof_rr");
}

// send queue strings from 2nd core to USB serial port
void serialTx() {
  if(g_serialStringCnt == 0) return;
  rp2040.wdt_reset();
  Serial.println(g_serialStrings[g_serialStringRdIdx]);
  g_serialStringRdIdx++;
  if(g_serialStringRdIdx>=SERTXSTR_SIZE) g_serialStringRdIdx = 0;

  //************* ATOMIC VARIABLE PROTECTION ***************
  g_serialStringCnt--;
  //*************************************************
}

// 2nd core uses this queue to send strings over serial interface
void sendSerialTxString(char* str) {
  if(g_serialStringCnt >= SERTXSTR_SIZE) return;
//  g_serialStrings[g_serialStringWrIdx] = str;
  strcpy(g_serialStrings[g_serialStringWrIdx], str);
  g_serialStringWrIdx++;
  if(g_serialStringWrIdx>=SERTXSTR_SIZE) g_serialStringWrIdx = 0;

  //************* ATOMIC VARIABLE PROTECTION ***************
  g_serialStringCnt++;
  //*************************************************

}

/********************************************************
* USB serial JSON code
*********************************************************/
void jsonParse(const char *jsonStr) ;

// retrieve JSON messages from USB serial port
void getJsonMsgs() {
  if (Serial.available()== 0) return;

  // read the incoming string and parse it
  String incomingString = Serial.readStringUntil('\n');
  jsonParse(incomingString.c_str());

}

void jsonParse(const char *jsonStr) {
  Serial.println(jsonStr);

  JSONVar myObject = JSON.parse(jsonStr);

  if (JSON.typeof(myObject) != "object") {
    Serial.println("Parsing JSON string input failed! Not a valid object");
  }

  if (myObject.hasOwnProperty("reset")) {
    bool reset = (String) myObject["reset"];
    Serial.print("reset = ");
    Serial.println(reset);
    if(reset) {
      // reset the RP2040, the print line after it should not occur
      rp2040.reboot();
      Serial.println("This is after the RP2040 hard reset, it should NOT occur");
    }
  }
}

/********************************************************
* I2C TOF code
*********************************************************/

// Front sensors
void getData_F(const char* tof) {
  if(!strncmp(tof,"tof_fc",6)) {
    status_F = tof_fc.check_data_ready(&newDataReady_F);
    if ((!status_F) && (newDataReady_F != 0)) {
      status_F = tof_fc.get_ranging_data(&results_F);
      uint32_t stamp = millis();
      sendJson_F(tof, stamp, &results_F);
    }
  }
  if(!strncmp(tof,"tof_fl",6)) {
    status_F = tof_fl.check_data_ready(&newDataReady_F);
    if ((!status_F) && (newDataReady_F != 0)) {
      status_F = tof_fl.get_ranging_data(&results_F);
      uint32_t stamp = millis();
      sendJson_F(tof, stamp, &results_F);
    }
  }
  if(!strncmp(tof,"tof_fr",6)) {
    status_F = tof_fr.check_data_ready(&newDataReady_F);
    if ((!status_F) && (newDataReady_F != 0)) {
      status_F = tof_fr.get_ranging_data(&results_F);
      uint32_t stamp = millis();
      sendJson_F(tof, stamp, &results_F);
    }
  }
}

// Rear sensors
void getData_R(const char* tof) {

 if(!strncmp(tof,"tof_rc",6)) {
   status_R = tof_rc.check_data_ready(&newDataReady_R);
   if ((!status_R) && (newDataReady_R != 0)) {
     status_R = tof_rc.get_ranging_data(&results_R);
     uint32_t stamp = millis();
     sendJson_R(tof, stamp, &results_R);
   }
 }
 if(!strncmp(tof,"tof_rl",6)) {
   status_R = tof_rl.check_data_ready(&newDataReady_R);
   if ((!status_R) && (newDataReady_R != 0)) {
     status_R = tof_rl.get_ranging_data(&results_R);
     uint32_t stamp = millis();
     sendJson_R(tof, stamp, &results_R);
   }
 }
 if(!strncmp(tof,"tof_rr",6)) {
   status_R = tof_rr.check_data_ready(&newDataReady_R);
   if ((!status_R) && (newDataReady_R != 0)) {
     status_R = tof_rr.get_ranging_data(&results_R);
     uint32_t stamp = millis();
     sendJson_R(tof, stamp, &results_R);
   }
 }
}

int getDist(int idx, VL53L8CX_ResultsData *results) {
  int dist = results->distance_mm[idx];
  int status = results->target_status[idx];
  if(dist<0 || status!=5) dist = -1;
  
  return(dist);
}

void sendJson_F(String tof, uint32_t stamp, VL53L8CX_ResultsData *results) {
  int row;
  int col;
  int rowIdx;
  int colIdx;
  int idx;
  int dist;


  // reset watchdog for hung I2C bus
  // TODO: be able to detect either bus hung
  rp2040.wdt_reset();

  // TODO: pre-process for valid data
  // Output JSON
  Serial.print("{\"");
  Serial.print(tof);
  Serial.print("\":{\"stamp\":");
  Serial.print(stamp);
  Serial.print(",\"dist\":[");
  for (row = 0; row < 8; row++) {
    rowIdx = (7-row)*1;
    Serial.print("[");
    for (col = 0; col < 8; col++) {
      colIdx = (7-col)*8;
      idx = rowIdx + colIdx;
      dist = getDist(idx, results);
      Serial.print(dist);
      if (col < 7) Serial.print(",");
    }
    Serial.print("]");
    if (row < 7) Serial.print(",");
  }
  Serial.println("]}}");
}

// Custom int to string conversion for core2
char* citoa(int num, char* str, int base)
{
    int i = 0;
    bool isNegative = false;

    /* Handle 0 explicitly, otherwise empty string is
     * printed for 0 */
    if (num == 0) {
        str[i++] = '0';
        str[i] = '\0';
        return str;
    }

    // In standard itoa(), negative numbers are handled
    // only with base 10. Otherwise numbers are
    // considered unsigned.
    if (num < 0 && base == 10) {
        isNegative = true;
        num = -num;
    }

    // Process individual digits
    while (num != 0) {
        int rem = num % base;
        str[i++] = (rem > 9) ? (rem - 10) + 'a' : rem + '0';
        num = num / base;
    }

    // If number is negative, append '-'
    if (isNegative)
        str[i++] = '-';

    str[i] = '\0'; // Append string terminator

    // Reverse the string
    std::reverse(str, &str[i]);

    return str;
}

// char string array for tof data
char str_R[1000];

char str[100];

void sendJson_R(const char* tof, uint32_t stamp, VL53L8CX_ResultsData *results) {
  int row;
  int col;
  int rowIdx;
  int colIdx;
  int idx;
  int dist;

  // reset watchdog for hung I2C bus
  // TODO: be able to detect either bus hung
  // rp2040.wdt_reset();

  
  // TODO: pre-process for valid data
  // Output JSON
  strcpy(str_R, ""); // null string
  strcat(str_R, "{\"");
  strcat(str_R, tof);
  strcat(str_R, "\":{\"stamp\":");
  strcat(str_R, citoa(stamp,str,10));
  strcat(str_R, ",\"dist\":[");

  for (row = 0; row < 8; row++) {
    rowIdx = (7-row)*1;
    strcat(str_R, "[");
    for (col = 0; col < 8; col++) {
      colIdx = (7-col)*8;
      idx = rowIdx + colIdx;
      dist = getDist(idx, results);
      strcat(str_R, citoa(dist,str,10));
      if (col < 7) strcat(str_R, ",");
    }
    strcat(str_R, "]");
    if (row < 7) strcat(str_R, ",");
  }
  strcat(str_R, "]}}");

  sendSerialTxString(str_R);
}

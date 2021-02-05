/*  
 *   Created for the M5StickC
 *   This sketch sends rotation and other sensor data
 *   using UDP packets to an OSC receiver
 *   
 *    Copyright 2019 Seth Persigehl
 *    Website: http://persigehl.com/
 *    Twitter: @KK4TEE
 *    Email: seth.persigehl@gmail.com
 *    MIT License - This copyright notice must be included.
 *   
 *    Please see the referenced libraries for their own licences.
 *    This follows the M5 tutorials and references. Please refer to
 *    them for more information.
 *    https://m5stack.com/products/stick-c
 *    
 *    |==============================================================|
 *     Updated February 2nd 2021 by Connor Rawls
 *     Synthesis @ ASU, Arizona State University
 *    
 *     + Added USB serial functionality for configuration of network 
 *       parameters on the device.
 *     + Added Storage of network configuration parameters into the
 *       EEPROM of the device.
 *    |==============================================================|
 */

 
#include <M5StickC.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <EEPROM.h>

#define UUIDIDX 0
#define PWDIDX 32
#define IPIDX 64
#define PORTIDX 68
#define LPORTIDX 70


////////// User Variables //////////////////////////////////////////////////
// Be sure to set these for your own WiFi network
const char * networkName = "Network Name";
const char * networkPswd = "Network Password";

void* tempPWD;
void* tempUUID;


int nonTurboRefreshDelay = 40;

//IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
char * udpAddress = "192.168.0.232"; //<- Enter your computer's IP address here
int udpPort = 8001;
int localPort = 7500;
char* targetIP = udpAddress;
//Are we currently connected?
boolean connected = false;

bool dispMode = false;

//The udp library class
WiFiUDP udp;

////////// System Variables //////////////////////////////////////////////////
enum imu_type {
  unknown,
  SH200Q,
  MPU6886
};

imu_type imuType = MPU6886;
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
double fXg, fYg, fZg;
const float alpha = 0.5;
double pitch, roll, Xg, Yg, Zg;
float gRes, aRes;

int analogSensorValue;
int16_t temp = 0;
double vbat = 0.0;
int discharge,charge;
double bat_p = 0.0;
bool isCharging;
bool isDischarging;

bool turboModeActive = false;
int screenBrightness = 15; // Range 0->15; Start at max brightness

///////////// WiFi Functions //////////////////////////////////////////////////
void connectToWiFi(const char * ssid, const char * pwd){
  USE_SERIAL.print("Connecting to WiFi network: ");
  USE_SERIAL.println(ssid);

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
 
  //Initiate connection
  WiFi.begin(ssid, pwd);

  USE_SERIAL.println("Waiting for WIFI connection...");
}

void reconnectToWiFi(const char* ssid, char * pwd) {
    USE_SERIAL.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
 
  //Initiate connection
  WiFi.begin(ssid, pwd);

  USE_SERIAL.println("Waiting for WIFI connection...");
}


//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set
          USE_SERIAL.print("WiFi connected! IP address: ");
          USE_SERIAL.println(WiFi.localIP()); 
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),localPort);
          connected = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          //USE_SERIAL.println("WiFi lost connection");
          connected = false;
          break;
      case SYSTEM_EVENT_WIFI_READY:
          break;
      default:
          break;
    }
}

void DecideGyroType(){
  // The M5StickC can have different types of IMUs
  // We need to detect which is installed and use it
  // For this we will test the accellerometer to see if
  // it reports all zeros. Since we are (probably) not in
  // outer space, this should be a decent proxy.

  imuType = MPU6886;
  //accX = 0;
  //accY = 0;
  //accZ = 0;
 // USE_SERIAL.println();
 // USE_SERIAL.println("Resetting IMU");
 
  // Test for the SH200Q using a call to M5.IMU
  //Wire1.end(); // Doesn't exist in the old version of Wire used
  //pinMode(21, OUTPUT);
  //pinMode(22, OUTPUT);
 // M5.IMU.sh200i_Reset();
 // M5.IMU.sh200i_ADCReset();
  M5.IMU.Init();
  M5.IMU.getAccelAdc(&accX,&accY,&accZ);
  //USE_SERIAL.print("X: "); USE_SERIAL.print(accX);
  //USE_SERIAL.print(" Y: "); USE_SERIAL.print(accY);
  //USE_SERIAL.print(" Z: "); USE_SERIAL.print(accZ);
  //USE_SERIAL.println();
//  if ( !(abs(accX) == 0 && abs(accY) == 0 && abs(accZ) == 0) && accX != 16380){
    // Because of the way the TwoWire library works, if the IMU is an MPU6886 the
    // byte that represents accX will return 16380, indicating it's not an SH200Q
 //   imuType = SH200Q;
  //  return;
 // }

  // Test for the MPU6886 using a call to M5.MPU6886
 
  M5.MPU6886.Init();
  M5.MPU6886.getAccelAdc(&accX,&accY,&accZ);
  if ( !(abs(accX) == 0 && abs(accY) == 0 && abs(accZ) == 0)){
    imuType = MPU6886;
    return;
  }
}


/*======================EEPROM STORAGE RETRIEVAL FUNCTIONS========================*/

void readUUID() {
  char uid[32];

  int nsize = 0;

  EEPROM.begin(72);
  int i = 0;
  for(i = 0; i < 32; i++) {
    uid[i] = (char)EEPROM.read(UUIDIDX + i);
    if(uid[i] == 0) {
      nsize = i;
      break;
    }
  }

  nsize = i + 1;

  EEPROM.end();
  
  if(nsize == 0) {
    tempUUID = NULL;
    return;
  }
  
  tempUUID = malloc(sizeof(char) * nsize);
  memcpy(tempUUID, uid, nsize);
  
}

void readPWD() {
  char pwd[32];

  int nsize = 0;

  EEPROM.begin(72);

  int i = 0;
  
  for(i = 0; i < 32; i++){
    pwd[i] = (char)EEPROM.read(PWDIDX + i);
    if(pwd[i] == 0) {
      nsize = i;
      break;
    }
  }

  EEPROM.end();

  nsize = i + 1;
  
  if(nsize == 0) {
    tempPWD = NULL;
    return;
  }

  tempPWD = malloc(sizeof(char) * nsize);
  memcpy(tempPWD, pwd, nsize);
}

void readIP() {

  int parts[4];

  EEPROM.begin(72);
  
  for(int i = 0; i < 4; i++) {
    parts[i] = (uint8_t)EEPROM.read(IPIDX + i);
  }

  EEPROM.end();

  if(parts[0] + parts[1] + parts[2] + parts[3] == 0) {
    targetIP = (char*)malloc(sizeof(char) * 11);
    targetIP[10] = 0;
    char* tempIP = "127.0.0.1";
    memcpy(targetIP, tempIP, 10); 
    return;
  }
  
  String ipstring = String(parts[0]) + String(".") + String(parts[1]) + String(".") + String(parts[2]) + String(".") + String(parts[3]);

  int strsize = ipstring.length() + 2;

  char iptemp[strsize];
  iptemp[strsize - 1] = 0;

  ipstring.toCharArray(iptemp, strsize);

  targetIP = (char*)malloc(sizeof(char) * strsize);

  memcpy(targetIP, iptemp, strsize);
  
}

void readTargetPort() {
  char port[4];
  port[2] = 0;
  port[3] = 0;

  EEPROM.begin(72);

  port[0] = EEPROM.read(PORTIDX);
  port[1] = EEPROM.read(PORTIDX + 1);

  EEPROM.end();

  if(port[0] + port[1] == 0) {
    int tport = 8001;
    memcpy(&udpPort, &tport, 4);
    return;
  }

  memcpy(&udpPort, port, 4);
}

void readLocalPort() {
  char port[4];
  port[2] = 0;
  port[3] = 0;

  EEPROM.begin(72);

  port[0] = EEPROM.read(LPORTIDX);
  port[1] = EEPROM.read(LPORTIDX + 1);

  EEPROM.end();

  if(port[0] + port[1] == 0) {
    int tport = 7500;
    memcpy(&localPort, &tport, 4);
    return;
  }

  memcpy(&localPort, port, 4);
}

/////// General Functions //////////////////////////////////////////////////

void setup() {
 
  M5.begin();
  DecideGyroType();
  pinMode(M5_LED, OUTPUT);
  digitalWrite(M5_LED, HIGH);
  pinMode(M5_BUTTON_HOME, INPUT);
  pinMode(M5_BUTTON_RST, INPUT);

  M5.Axp.ScreenBreath(screenBrightness);
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(0, 0);

  //init sensor memory locations
  accX = 0;
  accY = 0;
  accZ = 0;
  gyroX = 0;
  gyroY = 0;
  gyroZ = 0;

  readUUID();
  readPWD();

  if(tempUUID != NULL && tempPWD != NULL) {
    connectToWiFi((char*)tempUUID, (char*)tempPWD);
  } else {
    connectToWiFi(networkName, networkPswd);
  }
   
  readIP();
  readTargetPort();
  readLocalPort();
  /*
  EEPROM.begin(73);

  uint8_t turbo = EEPROM.read(72);

  USE_SERIAL.printf("Read Turbo Value: %d \n", turbo);

  turboModeActive = (turbo > 0) ? true : false;

  EEPROM.end();
 */
}




void HandleButtons(){
  // POWER BUTTON
  // 0x01 long press(1s), 0x02 press
  if(M5.Axp.GetBtnPress() == 0x01) {
    //USE_SERIAL.print("Button press 0x01");
    //USE_SERIAL.println(" - LONG");
      M5.Lcd.fillScreen(RED);
      // Run action
      M5.Lcd.fillScreen(ORANGE);
      delay(400);
      M5.Lcd.fillScreen(BLACK);
  }
  if(M5.Axp.GetBtnPress() == 0x02) {
    //esp_restart();
    //USE_SERIAL.print("Button press 0x02");
    //USE_SERIAL.println(" - LONG");
      M5.Lcd.fillScreen(RED);
      // Run action
      M5.Lcd.fillScreen(BLUE);
      delay(400);
      M5.Lcd.fillScreen(BLACK);
  }

  // TOP SIDE BUTTON
  if(digitalRead(M5_BUTTON_RST) == LOW){
    //USE_SERIAL.print("Button press TOP SIDE BUTTON");
    M5.Lcd.fillScreen(WHITE);
    unsigned long pressTime  = millis();
    while(digitalRead(M5_BUTTON_RST) == LOW);
    if (millis() > pressTime + 500){
      //USE_SERIAL.println(" - LONG");
      //M5.Lcd.fillScreen(RED);
      // Run action
      M5.Lcd.fillScreen(BLUE);
      M5.Lcd.setCursor(0, 0, 1);
      M5.Lcd.printf("Reset IMU");
      DecideGyroType();
      delay(400);
      M5.Lcd.fillScreen(BLACK);
    }
    else{
      // Run action
      dispMode = !dispMode;
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setCursor(0, 0, 1);
    }
  }

  // CENTER FACE BUTTON
  if(digitalRead(M5_BUTTON_HOME) == LOW){
    //USE_SERIAL.print("Button press CENTER FACE BUTTON");
    M5.Lcd.fillScreen(WHITE);
    unsigned long pressTime  = millis();
    while(digitalRead(M5_BUTTON_HOME) == LOW);
    if (millis() > pressTime + 500){
      USE_SERIAL.println(" - LONG");
      M5.Lcd.fillScreen(RED);
      // Run action
      M5.Lcd.fillScreen(BLUE);
      delay(400);
      M5.Lcd.fillScreen(BLACK);
    }
    else{
      //USE_SERIAL.println(" - SHORT");
      M5.Lcd.fillScreen(RED);
      // Run action
      // Set the display's brightness
      screenBrightness += 7;
      if (screenBrightness >= 16){
        screenBrightness = 0;
      }
      M5.Axp.ScreenBreath(screenBrightness);
     
      M5.Lcd.fillScreen(ORANGE);
      delay(400);
      M5.Lcd.fillScreen(BLACK);
    }
  }
}


void HandleSensors(){

  if (imuType == SH200Q){
    M5.IMU.getGyroAdc(&gyroX,&gyroY,&gyroZ);
    M5.IMU.getAccelAdc(&accX,&accY,&accZ);
    M5.IMU.getTempAdc(&temp);
    aRes = M5.IMU.aRes;
    gRes = M5.IMU.gRes;
    //USE_SERIAL.println("IMU Here");
  }
  else if (imuType == MPU6886){
    M5.MPU6886.getGyroAdc(&gyroX,&gyroY,&gyroZ);
    M5.MPU6886.getAccelAdc(&accX,&accY,&accZ);
    M5.MPU6886.getTempAdc(&temp);
    aRes = M5.MPU6886.aRes;
    gRes = M5.MPU6886.gRes;
    //USE_SERIAL.println("Grabbing IMU Data");
  } else {
    //USE_SERIAL.println("IMU Not Set");
  }
 
  vbat = M5.Axp.GetVbatData() * 1.1 / 1000;
  isCharging    = M5.Axp.GetIchargeData() / 2;
  isDischarging = M5.Axp.GetIdischargeData() / 2;
  // analogSensorValue = analogRead(sensorIn);
 
  //Low Pass Filter
  fXg = accX * alpha + (fXg * (1.0 - alpha));
  fYg = accY * alpha + (fYg * (1.0 - alpha));
  fZg = accZ * alpha + (fZg * (1.0 - alpha));

  //Roll & Pitch Equations
  roll  = 11+ (atan2(-fYg, fZg)*180.0)/M_PI;
  pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI - 8;
}


void displayNetwork() {

  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("Target IP:\n %s:%i", targetIP, udpPort);
  
  IPAddress myIP = WiFi.localIP();
  M5.Lcd.setCursor(0,20);
  M5.Lcd.printf("My IP:\n %i.%i.%i.%i:%i", myIP[0], myIP[1], myIP[2], myIP[3], localPort);

  M5.Lcd.setCursor(0,40);
  M5.Lcd.print("Network Name:");
  M5.Lcd.setCursor(0,50);

  int i = 0;
  char* uuid = (char*)tempUUID;
  while(uuid[i] != 0) {
    M5.Lcd.print(uuid[i]);
    i++;
  }

  M5.Lcd.setCursor(0,60);
  if(connected) {
    M5.Lcd.print("Connected");
  } else {
    M5.Lcd.print("Not Connected");
  }
  
  
}

void HandleDisplay(){

  //USE_SERIAL.print("Pitch: "); USE_SERIAL.print(pitch); USE_SERIAL.print(" Roll: "); USE_SERIAL.print(roll);
 
  M5.Lcd.setCursor(0, 0);
  if (connected){
    M5.Lcd.print("OSC Send: ");
  }
  else {
    M5.Lcd.print("WiFiError ");
  }
  M5.Lcd.setCursor(60, 0);
  M5.Lcd.print(targetIP);

  M5.Lcd.setCursor(0, 10);
  IPAddress myIP = WiFi.localIP();
  M5.Lcd.printf("My IP: %i.%i.%i.%i:%i", myIP[0], myIP[1], myIP[2], myIP[3], localPort);
 
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.println("  X       Y      Z");
  M5.Lcd.setCursor(114, 20);
  if (imuType == SH200Q){
    M5.Lcd.print(" SH200Q");
  }
  else if (imuType == MPU6886){
    M5.Lcd.print("MPU6886");
  }
  else {
    M5.Lcd.print("UNKNOWN");
  }
 
  M5.Lcd.setCursor(0, 30);
  M5.Lcd.printf("%.2f   %.2f   %.2f    ", ((float) gyroX) * gRes, ((float) gyroY) * gRes,((float) gyroZ) * gRes);
  M5.Lcd.setCursor(144, 30);
  M5.Lcd.print("mg");
  M5.Lcd.setCursor(0, 40);
  M5.Lcd.printf("%.2f   %.2f   %.2f     ",((float) accX) * aRes,((float) accY) * aRes, ((float) accZ) * aRes);
  M5.Lcd.setCursor(140, 40);
  M5.Lcd.print("*/s");
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.print("Analog: ");
  M5.Lcd.print((int)analogSensorValue);
  M5.Lcd.print("   ");
  M5.Lcd.setCursor(75, 50);
  M5.Lcd.print(" / 4095");
  M5.Lcd.setCursor(0, 60);
  M5.Lcd.printf("Turbo Mode: %s", turboModeActive ? "enabled\t" : "disabled\t");
 // M5.Lcd.printf("vbat:%.3fV",vbat);
  //if (isCharging){
    //M5.Lcd.print(" Charging      ");
 // }
  //else if (isDischarging){
    //M5.Lcd.print(" Discharging   ");
  //}
  //else if (!isDischarging && !isCharging){
    //M5.Lcd.print(" External Power");
  //}
  M5.Lcd.setCursor(0, 70);
  //M5.Lcd.printf("Temperature : %.2f C",((float) temp) / 333.87 + 21.0);
  M5.Lcd.printf("Output UDP Port: %i", udpPort);
}


void HandleNetwork(){
  // Only send data when connected
  if(connected){

    //create and fill the messages
    OSCMessage oscAccXMsg("/acc/x"); // First argument is OSC address
    oscAccXMsg.add((float)accX); // Then append the data
    OSCMessage oscAccYMsg("/acc/y"); // First argument is OSC address
    oscAccYMsg.add((float)accY); // Then append the data
    OSCMessage oscAccZMsg("/acc/z"); // First argument is OSC address
    oscAccZMsg.add((float)accZ); // Then append the data
    OSCMessage accRes("/acc/res");
    accRes.add(aRes);
    
    //send the messages
    udp.beginPacket(targetIP,udpPort);
    oscAccXMsg.send(udp); // send the bytes to the SLIP stream
    udp.endPacket(); // mark the end of the OSC Packet
    udp.beginPacket(targetIP,udpPort);
    oscAccYMsg.send(udp); // send the bytes to the SLIP stream
    udp.endPacket(); // mark the end of the OSC Packet
    udp.beginPacket(targetIP,udpPort);
    oscAccZMsg.send(udp); // send the bytes to the SLIP stream
    udp.endPacket(); // mark the end of the OSC Packet
    udp.beginPacket(targetIP,udpPort);
    accRes.send(udp);
    udp.endPacket(); // mark the end of the OSC Packet
    oscAccXMsg.empty(); // free space occupied by message
    oscAccYMsg.empty(); // free space occupied by message   
    oscAccZMsg.empty(); // free space occupied by message
    accRes.empty();


   OSCMessage oscGyroXMsg("/gyro/x"); // First argument is OSC address
    oscGyroXMsg.add((float)gyroX); // Then append the data
    udp.beginPacket(targetIP,udpPort);
    oscGyroXMsg.send(udp); // send the bytes to the SLIP stream
    udp.endPacket(); // mark the end of the OSC Packet
    oscGyroXMsg.empty(); // free space occupied by message

    OSCMessage oscGyroYMsg("/gyro/y"); // First argument is OSC address
    oscGyroYMsg.add((float)gyroY); // Then append the data
    udp.beginPacket(targetIP,udpPort);
    oscGyroYMsg.send(udp); // send the bytes to the SLIP stream
    udp.endPacket(); // mark the end of the OSC Packet
    oscGyroYMsg.empty(); // free space occupied by message

  OSCMessage oscGyroZMsg("/gyro/z"); // First argument is OSC address
    oscGyroZMsg.add((float)gyroZ); // Then append the data
    udp.beginPacket(targetIP,udpPort);
    oscGyroZMsg.send(udp); // send the bytes to the SLIP stream
    udp.endPacket(); // mark the end of the OSC Packet
    oscGyroZMsg.empty(); // free space occupied by message

    OSCMessage gyroRes("/gyro/res");
    gyroRes.add(gRes);
    udp.beginPacket(targetIP,udpPort);
    gyroRes.send(udp);
    udp.endPacket(); // mark the end of the OSC Packet
    gyroRes.empty();

  //  OSCMessage oscM5AnalogMsg("/m5Analog"); // First argument is OSC address
   // oscM5AnalogMsg.add((int)analogSensorValue); // Then append the data
  //  udp.beginPacket(targetIP,udpPort);
 //   oscM5AnalogMsg.send(udp); // send the bytes to the SLIP stream
 //   udp.endPacket(); // mark the end of the OSC Packet
  //  oscM5AnalogMsg.empty(); // free space occupied by message

    //USE_SERIAL.print(" WiFi connected and OSC packets sent");
  }
  else {
    //USE_SERIAL.print(" WiFi disconnected");
  }
  //USE_SERIAL.println("");
}


void copyPassword(char* buf, int count, int idx){
  if(count > 4) {
    if(tempPWD != NULL) {
      free(tempPWD);
    }

    int bsize = count - idx + 1;
    
    tempPWD = malloc(sizeof(char) * bsize);

    char* tmp = buf;
    tmp += idx;
    USE_SERIAL.print("Data Size: ");
    USE_SERIAL.println(sizeof(char) * bsize);
    memcpy(tempPWD, tmp, (bsize - 1));
    tmp = (char*)tempPWD + (bsize - 1);
    *tmp = 0;
    USE_SERIAL.print("New Password: ");
    USE_SERIAL.print((char*)tempPWD);
    USE_SERIAL.println();
    
  }
  else {
    USE_SERIAL.println("e2");
  }
}

void copyUUID(char* buf, int count, int idx){
  if(count > 4) {
    if(tempUUID != NULL) {
      free(tempUUID);
    }
    int bsize = count - idx + 1;
    tempUUID = malloc(sizeof(char) * bsize);

    char* tmp = buf;
    tmp += idx;
    USE_SERIAL.print("Data Size: ");
    USE_SERIAL.println(sizeof(char) * bsize);
    memcpy(tempUUID, tmp, (bsize - 1));
    tmp = (char*)tempUUID + (bsize - 1);
    *tmp = 0;
    USE_SERIAL.print("New Network: ");
    USE_SERIAL.print((char*)tempUUID);
    USE_SERIAL.println();
    
  }
  else {
    USE_SERIAL.println("e2");
  }
}

void attemptToConnect() {
  if(tempPWD != NULL && tempUUID != NULL) {
    reconnectToWiFi((char*)tempUUID, (char*)tempPWD);
  }
}

void setIPAddress(char* buf, int count, int idx){
  if(count >= 11) {
  if(targetIP != udpAddress && targetIP != NULL) {
    free(targetIP);
  }
  
  int bsize = count - idx + 1;

  targetIP = (char*)malloc(sizeof(char) * bsize);
  char* tmp = buf + idx;
  memcpy(targetIP, tmp, bsize - 1);
  tmp = targetIP + (bsize - 1);
  *tmp = 0;

  USE_SERIAL.println(targetIP);
  M5.Lcd.fillScreen(BLACK);
  }
  else {
    USE_SERIAL.println("e5"); 
  }
}

void setPort(char* buf, int count, int idx) {
  if(count >= 5) {

    int carry = buf[idx];
    int rem = 0;
    if(count >= 6) {
      rem = buf[idx + 1]; 
    }

    udpPort = (carry * 255) + rem;
    M5.Lcd.fillScreen(BLACK);
  }
  else {
    USE_SERIAL.println("e6");
  }
}

void setTurboMode(char* buf, int count, int idx) {

  //no args required, just toggle it.
  turboModeActive = !turboModeActive;
   
  USE_SERIAL.print("Turbo Mode: ");
  USE_SERIAL.print(turboModeActive);
  USE_SERIAL.println();
  M5.Lcd.fillScreen(BLACK);

}

void setLocalPort(char* buf, int count, int idx) {
  if(count >= 5) {

    int carry = buf[idx];
    int rem = 0;
    if(count >= 6) {
      rem = buf[idx + 1]; 
    }

    localPort = (carry * 255) + rem;
    M5.Lcd.fillScreen(BLACK);
  }
  else {
    USE_SERIAL.println("e6");
  }
}

void storeToEEPROM() {
  if(tempPWD != NULL && tempUUID != NULL) {

    EEPROM.begin(73);
    
    int i = 0;
    int dataIdx = 0;
    char* myUUID = (char*)tempUUID;
    
    //store Network Name
    while(myUUID[i] != 0 && i < 32) {
      EEPROM.write(dataIdx + i, myUUID[i]);
      i++;
    }
    //Clear remaining data field
    while(i < 32) {
      EEPROM.write(dataIdx + i, 0);
      i++;
    }

    //store Network Password
    i = 0;
    dataIdx = 32;
    char* myPWD = (char*)tempPWD;
    
    while(myPWD[i] != 0 && i < 32) {
      EEPROM.write(dataIdx + i, myPWD[i]);
      i++;
    }

    while(i < 32) {
      EEPROM.write(dataIdx + i, 0);
      i++;
    }

    //store target IP address

    IPAddress tip;
    tip.fromString(targetIP);
    

    EEPROM.write(64, tip[0]);
    EEPROM.write(65, tip[1]);
    EEPROM.write(66, tip[2]);
    EEPROM.write(67, tip[3]);

    //store target port
    int* tempport = &udpPort;
    char* port = (char*)tempport;

    EEPROM.write(68, port[0]);
    EEPROM.write(69, port[1]);

    //store local port
    int* templport = &localPort;
    char* lport = (char*)templport;

    EEPROM.write(70, lport[0]);
    EEPROM.write(71, lport[1]);  

    uint8_t turbo = (turboModeActive) ? 1 : 0;
    USE_SERIAL.printf("Saving Turbo Mode: %d\n", turbo);
    EEPROM.write(72, turbo);

    EEPROM.commit();
    EEPROM.end();
    
  } else {
    USE_SERIAL.println("e10");
  }
}

void clearEEPROM() {
  EEPROM.begin(73);
  for(int i = 0; i < 73; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  EEPROM.end();
}


void printEEPROM() {
  int i = 0;
  EEPROM.begin(72);
  USE_SERIAL.print("UUID: ");
  while(EEPROM.read(UUIDIDX + i) != 0) {
    USE_SERIAL.print((char)EEPROM.read(UUIDIDX + i));
    i++;
  }

  USE_SERIAL.println();

  i = 0;
  USE_SERIAL.print("PWD: ");
  while(EEPROM.read(PWDIDX + i) != 0) {
    USE_SERIAL.print((char)EEPROM.read(PWDIDX + i));
    i++;
  }

  USE_SERIAL.println();
  EEPROM.end();
}

/////// Main Loop //////////////////////////////////////////////////
void loop() {
  HandleButtons();
  HandleSensors();
  if(dispMode) {
    displayNetwork();
  } else {
    HandleDisplay();
  }
  HandleNetwork();

  /*
   * Check the Serial Buffer for any incoming data.
   */
  if(USE_SERIAL.available() > 0) {
    int count = USE_SERIAL.available(); //get buffer size
    char* buf = (char*)malloc(sizeof(char) * count); //create a buffer
    USE_SERIAL.readBytesUntil(0, buf, count); //read data to buffer
    if(count >= 3) {
      int cmd = 0;
      int i = 0;
      while(i < 3){
        char c = buf[i];
        putchar(toupper(c));
        cmd += c;
        i++;
      }
  
      USE_SERIAL.println(cmd);
      switch(cmd) {
        case 235: //Incoming Password
          copyPassword(buf, count, 4);
          break;
        case 226: //Incoming Network Name
          copyUUID(buf, count, 4);
          break;
        case 229: //Life Check Ping
          USE_SERIAL.write(1);
          M5.Lcd.fillScreen(RED);
          break;
        case 233: //Attempt To Connect
          attemptToConnect();
          break;
        case 218: //Target IP Address
          setIPAddress(buf, count, 4);
          break;
        case 246: //Target Port
          setPort(buf, count, 4);
          break;
        case 238: //set local port
          setLocalPort(buf, count, 4);
          break;
        case 231: //Display Settings
          printEEPROM();
          break;
        case 234: //Save Settings to EEPROM
          storeToEEPROM();
          break;
        case 225: //Clear data from EEPROM
          clearEEPROM();
          break;
        case 232: //Turbo mode
          setTurboMode(buf, count, 4);
          break; 
        default: //Anything Else
          USE_SERIAL.print("e1");
          break;          
      }
    }
    else {
      USE_SERIAL.println("e0");
    }

    free(buf);
  } else {
   //this else block intentionally left blank
  }
   
  if (turboModeActive){
    // No delay
  }
  else{
    // Delay based on user preference
    delay(nonTurboRefreshDelay);
  }
}

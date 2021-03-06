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
 *     + Added output of angle data as OSC.
 *    |==============================================================|
 */

 
#include <M5StickC.h>
#include <WiFi.h>
#include <WiFiUdp.h>      //creating sockets for sending data
#include <OSCMessage.h>   //for sending data in a structured packet via UDP
#include <EEPROM.h>       //for storing configuration settings on device
#include <WebServer.h>

//EEPROM data offsets
#define UUIDIDX 0
#define PWDIDX 32
#define IPIDX 64
#define PORTIDX 68
#define LPORTIDX 70


////////// User Variables //////////////////////////////////////////////////
void* myPWD;
void* myUUID;

//Used when the device swaps into SoftAP mode to handle configuration
const char* defaultSSID = "M5Stick-C_AP-EEA8";
const char* defaultPWD = "";
IPAddress myServerIP(192, 168, 1, 1);
IPAddress serverGateway(192, 168, 1, 1);
IPAddress serverSubnet(255, 255, 255, 0);
WebServer myServer(80);
bool usingSoftAP = false;
int wifiConnectionLoops = 0;
const int maxConnectionLoops = 1000;

int nonTurboRefreshDelay = 40;

//Variables for handling normal networing environment
//default output port
int udpPort = 8001;
//default input port
int localPort = 7500;
//output IP address
char* targetIP;
//Are we currently connected?
boolean connected = false;
//toggle between data display and network display
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
float gX, gY, gZ;
float pitch, roll, yaw;
double fXg, fYg, fZg;
const float alpha = 0.5;
double Xg, Yg, Zg;
float gRes, aRes;
bool buttonState = false;

int analogSensorValue;
int16_t temp = 0;
double vbat = 0.0;
int discharge,charge;
double bat_p = 0.0;


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
          USE_SERIAL.println("WiFi connected! IP address: ");
          USE_SERIAL.println(WiFi.localIP()); 
          USE_SERIAL.println("att 1");
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),localPort);
          connected = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          //USE_SERIAL.println("WiFi lost connection");
          connected = false;
          USE_SERIAL.println("att 0");
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
  
  if(nsize == 1) {
    //fill in default value
    const char* defaultUUID = "No Network";
    myUUID = malloc(12);
    memcpy(myUUID, defaultUUID, 11);
    char* t = (char*)myUUID;
    t += 11;
    *t = 0;
    return;
  }
  
  myUUID = malloc(sizeof(char) * nsize);
  memcpy(myUUID, uid, nsize);
  
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
  
  if(nsize == 1) {
    const char* pwd = "default";
    myPWD = malloc(8);
    memcpy(myPWD, pwd, 8);
    return;
  }

  myPWD = malloc(sizeof(char) * nsize);
  memcpy(myPWD, pwd, nsize);
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
    const char* tempIP = "127.0.0.1";
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


/*=======================SOFT ACCESS POINT CONFIGURATION SERVER===================*/

bool validNumberString(String s) {
  const char* contents = s.c_str();
  const char* c = contents;
  for(int i = 0; i < s.length(); i++, c++) {
    if(*c > 57 || *c < 48) {
      return false;
    }
  }
  return true;
}

//creating HTML source for configuration pages
String createMenuHTML(int saveStatus, int eraseStatus) {
  String html = "<!DCOTYPE html>\n<html>\n";
  html += "<head>\n\t<style>\n\t\tbody  {background-color: powderblue;}\n\t\th1    {color: black}\n\t\tp    {color: black}\n\t\tbutton    {max-width: 100%}\n\t</style>\n</head>\n";
  html += "<body>\n";
  html += "\t<h1>M5Stick-C Configuration Menu</h1>\n";
  html += "\t<form method=\"get\" action=\"/configwifi\">\n\t\t<input class=\"button\" type=\"submit\" name=\"\" value=\"Open WiFi Menu\" style=\"font-size:150%; width:300px; height: 80px;\">\n\t</form>\n";
  html += "\t<form method=\"get\" action=\"/targetsettings\">\n\t\t<input class=\"button\" type=\"submit\" name=\"\" value=\"Open OSC Menu\" style=\"font-size:150%; width:300px; height: 80px;\">\n\t</form>\n";
  html += "\t<form method=\"post\" action=\"/savesettings\">\n\t\t<input class=\"button\" type=\"submit\" name=\"\" value=\"Save Settings\" style=\"font-size:150%; width:300px; height: 80px;\">\n\t</form>\n";
  html += "\t<form method=\"post\" action=\"/clearsettings\">\n\t\t<input class=\"button\" type=\"submit\" name=\"\" value=\"Erase Settings\" style=\"font-size:150%; width:300px; height: 80px;\">\n\t</form>\n";
  //html += "\t<form method=\"post\" action=\"/restartdevice\">\n\t\t<input class=\"button\" type=\"submit\" name=\"\" value=\"Reboot M5Stick-C\" style=\"font-size:150%; width:300px; height: 80px;\">\n\t</form>\n";
  if(saveStatus == 1) {
    html += "\t<p style=\"font-size:150%; color:green;\">Settings sucessfully saved</p>\n";
  } else if(saveStatus == 0) {
    html += "\t<p style=\"font-size:150%; color:tomato;\">Error saving settings</p>\n";
  }
  if(eraseStatus == 1) {
    html += "\t<p style=\"font-size:150%; color:green;\">Settings sucessfully erased</p>\n";
  } else if(eraseStatus == 0) {
    html += "\t<p style=\"font-size:150%; color:tomato;\">Error erasing settings</p>\n";
  }
  html += "</body>\n</html>";
  return html;
}

String createOSCMenuHTML(int o1, int o2, int o3, int o4, int port, int iStatus, int pStatus) {
  String html = "<!DOCTYPE html>\n<html>\n";
  html += "<head>\n\t<style>\n\t\tbody {background-color: powderblue;}\n\t\th1 {color: black; font-size:150%;}\n\t\tp {font-size:150%;}\n\t</style>\n</head>\n";
  html += "<body>\n";
  html += "\t<h1> M5Stick-C OSC Output Settings</h1>\n";
  html += "\t<form method=\"post\" action=\"/configosc\">\n\t\t<p>Target IP Address</p>\n\t\t<div style=\"display:inline-block\">\n\t\t\t<input class=\"number\" id=\"ip01\" name=\"ip01\" value=\"";
  html += o1;
  html += "\" min=\"1\" max=\"255\" style=\"font-size:150%; width:80px; height:80px;\">\n";
  html += "\t\t\t<input class=\"number\" id=\"ip02\" name=\"ip02\" value=\"";
  html += o2;
  html += "\" min=\"0\" max=\"255\" style=\"font-size:150%; width:80px; height:80px;\">\n";
  html += "\t\t\t<input class=\"number\" id=\"ip03\" name=\"ip03\" value=\"";
  html += o3;
  html += "\" min=\"0\" max=\"255\" style=\"font-size:150%; width:80px; height:80px;\">\n";
  html += "\t\t\t<input class=\"number\" id=\"ip04\" name=\"ip04\" value=\"";
  html += o4;
  html += "\" min=\"1\" max=\"255\" style=\"font-size:150%; width:80px; height:80px;\">\n\t\t</div>\n";
  html += "\t\t<p>Output Port</p>\n\t\t<input class=\"number\" id=\"port\" name=\"port\" min=\"1000\" max=\"65535\" value=\"";
  html += port;
  html += "\"style=\"font-size:150%; width:100px; height:80px;\">\n\t\t<div>\n\t\t\t<input class=\"button\" type=\"submit\" value=\"Update Output Settings\" name=\"\" style=\"font-size:150%; width:300px; height:80px;\">\n\t\t</div>\n\t</form>\n";
  html += "\t<form method=\"get\" action=\"/\">\n\t\t<input class=\"button\" type=\"submit\" value=\"Back to Main Menu\" name=\"\" style=\"font-size:150%; width:300px; height:80px;\">\n\t</form>\n";
  if(iStatus == 0) {
    html += "\t<p style=\"color:tomato;\">Bad Values for Target IP Address</p>\n";
  } else if(iStatus == 1) {
    html += "\t<p style=\"color:green;\">Target IP Address updated sucessfully</p>\n";
  }
  if(pStatus == 0) {
    html += "\t<p style=\"color:tomato;\">Bad value for output port</p>\n";
  } else if(pStatus == 1) {
    html += "\t<p style=\"color:green;\">Output port sucessfully updated</p>\n";
  }
  html += "</body>\n</html>";
  return html;
  
}

String createWiFiHTML(int status) {
  char* nname = (char*)myUUID;
  String networkname(nname);
  String html = "<!DOCTYPE html>\n<html>\n";
  html += "<head>\n    <meta charset = \"utf-8\">\n    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n    <title>M5Stick-C Configuration</title>\n</head>\n\n";
  html += "<body>\n    <div class=\"config-box\">\n        <form method=\"post\" action=\"/setwifi\">\n            <div class=\"textbox\">\n                <input type=\"text\" placeholder=\"Network Name\" name=\"networkName\" value=\"";
  html += networkname;
  html += "\" style=\"font-size:150%; width:300px; height: 80px;\">\n            </div>\n";
  html += "            <div class=\"textbox\">\n                <input type=\"password\" placeholder=\"Network Password\" name=\"networkPassword\" value=\"\" style=\"font-size:150%; width:300px; height: 80px;\">\n            </div>\n";
  if(status == 0) {
    html += "            <p style=\"color:tomato;\">Missing Fields!</p>\n";
  } else if(status == 1) {
    html += "            <p style=\"color:green;\">Updated network name and password</p>\n";
  }
  html += "            <input class=\"button\" type=\"submit\" name=\"\" value=\"Config WiFi\" style=\"font-size:150%; width:300px; height: 80px;\">\n";
  html += "        </form>\n    </div>\n\t<form method=\"get\" action=\"/\">\n\t\t<input class=\"button\" type=\"submit\" name=\"\" value=\"Back To Menu\" style=\"font-size:150%; width:300px; height: 80px;\">\n\t</form>\n";
  html += "</body>\n\n</html>";
  return html;
}

void initializeSoftAP() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(10, 25);
  M5.Lcd.print("Unable to connect");
  M5.Lcd.setCursor(0, 45);
  M5.Lcd.print("Creating my own WiFi...");
  WiFi.disconnect();
  WiFi.softAP(defaultSSID, defaultPWD);
  delay(2000);
  WiFi.softAPConfig(myServerIP, serverGateway, serverSubnet);
  char myuuid[17];
  memcpy(myuuid, defaultSSID, 17);
  copyUUID(myuuid, 17, 0);
  connected = true;
  usingSoftAP=true;
  wifiConnectionLoops = 0;
}

void initializeServer() {

  //handle a user accessing the root menu page
  myServer.on("/", HTTP_GET, [](){
    myServer.send(200, "text/html", createMenuHTML(-1, -1));
  });

  //handle a user asking to modify the WiFi connection settings
  myServer.on("/configwifi", HTTP_GET, [](){
    myServer.send(200, "text/html", createWiFiHTML(-1));
  });

  //handle input from a user that is attempting to modify the WiFi connection settings
  myServer.on("/setwifi", HTTP_POST, []() {
   if(myServer.arg("networkName") == "" || myServer.arg("networkPassword") == "") {
    myServer.send(200, "text/html", createWiFiHTML(0));
    return;
   }
    String uuid = myServer.arg("networkName");
    String pwd = myServer.arg("networkPassword");
    Serial.println(uuid);
    char uuidBuf[30];
    memcpy(uuidBuf, uuid.c_str(), ((uuid.length() < 30) ? uuid.length() : 30));
    for(int i = 0; i < ((uuid.length() < 30) ? uuid.length() : 30); i++) {
      int currChar = uuidBuf[i];
      //handle apostrophe characters
      if(currChar == 226 && uuidBuf[i + 1] == 128 && uuidBuf[i + 2] == 153)
      {
        uuidBuf[i] = 39;
        currChar = uuidBuf[i];
        for(int j = i + 1; j < ((uuid.length() < 30) ? uuid.length() : 30); j++) {
          if(j + 2 < ((uuid.length() < 30) ? uuid.length() : 30)) {
            uuidBuf[j] = uuidBuf[j + 2];
          } else {
            uuidBuf[j] = 0;
          }
        }
      }
      Serial.print(currChar);
      Serial.print(" ");
    }
  
    copyUUID(uuidBuf, ((uuid.length() < 30) ? uuid.length() : 30), 0);
    
    Serial.println();
    Serial.println(pwd);
    char pwdBuf[30];
    memcpy(pwdBuf, pwd.c_str(), ((pwd.length() < 30) ? pwd.length() : 30)); 
    copyPassword(pwdBuf, ((pwd.length() < 30) ? pwd.length() : 30), 0);
    myServer.send(200, "text/html", createWiFiHTML(1));
  });

  //handle user asking for the current settings to be saved
  myServer.on("/savesettings", HTTP_POST, []() {
    storeToEEPROM();
    myServer.send(200, "text/html", createMenuHTML(1, -1));
  });

  //handle user asking for the current settings to be erased
  myServer.on("/clearsettings", HTTP_POST, []() {
    clearEEPROM();
    myServer.send(200, "text/html", createMenuHTML(-1, 1));
  });

  //handle request for menu to modify OSC target settings
  myServer.on("/targetsettings", HTTP_GET, []() {
    IPAddress target;
    target.fromString(targetIP);
    myServer.send(200, "text/html", createOSCMenuHTML(target[0], target[1], target[2], target[3], udpPort, -1, -1));
  });

  myServer.on("/configosc", HTTP_POST, []() {
    int ip0 = myServer.arg("ip01").toInt();
    int ip1 = myServer.arg("ip02").toInt();
    int ip2 = myServer.arg("ip03").toInt();
    int ip3 = myServer.arg("ip04").toInt();
    int port = myServer.arg("port").toInt();

    //track if we could suvve
    int portStore = 0;
    int ipStore = 0;
    
    if(port > 1000 && port <= 65535 && validNumberString(myServer.arg("port"))) {
      udpPort = port;
      portStore = 1;
    } else {
      portStore = 0;
    }

    if(validNumberString(myServer.arg("ip01")) && validNumberString(myServer.arg("ip02")) && validNumberString(myServer.arg("ip03")) &&validNumberString(myServer.arg("ip04"))) {
      String ipaddr = "";
      ipaddr += ip0;
      ipaddr += ".";
      ipaddr += ip1;
      ipaddr += ".";
      ipaddr += ip2;
      ipaddr += ".";
      ipaddr += ip3;

      char ipstring[15];

      memcpy(ipstring, ipaddr.c_str(),ipaddr.length());

      setIPAddress(ipstring, ipaddr.length(), 0);
      ipStore = 1;
    } else {
      ipStore = 0;
    }


    IPAddress tIP;
    tIP.fromString(targetIP);
    myServer.send(200, "text/html", createOSCMenuHTML(tIP[0], tIP[1], tIP[2], tIP[3], udpPort, ipStore, portStore));
  });

  //handle user asking for device reset
  myServer.on("/restartdevice", HTTP_POST, []() {
    if(usingSoftAP) {
      WiFi.softAPdisconnect();
    }

    M5.axp.LightSleep(10);
  });
  
  myServer.begin();

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

  M5.Axp.EnableCoulombcounter();
  //init sensor memory locations
  accX = 0;
  accY = 0;
  accZ = 0;
  gyroX = 0;
  gyroY = 0;
  gyroZ = 0;
  gX = 0.0;
  gY = 0.0;
  gZ = 0.0;
  pitch = 0.0;
  roll = 0.0;
  yaw = 0.0;

  readUUID();
  readPWD();

  connectToWiFi((char*)myUUID, (char*)myPWD);
  initializeServer();
  readIP();
  readTargetPort();
  readLocalPort();
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

  buttonState = (digitalRead(M5_BUTTON_HOME) == LOW) ? true : false;
  // CENTER FACE BUTTON
  if(digitalRead(M5_BUTTON_HOME) == LOW){
   /* //USE_SERIAL.print("Button press CENTER FACE BUTTON");
    M5.Lcd.fillScreen(WHITE);
    unsigned long pressTime  = millis();
    while(digitalRead(M5_BUTTON_HOME) == LOW);
    if (millis() > pressTime + 500){
       screenBrightness += 2;
      if (screenBrightness >= 16){
        screenBrightness = 0;
      }
      M5.Axp.ScreenBreath(screenBrightness);
     
      M5.Lcd.fillScreen(ORANGE);
      M5.Lcd.fillScreen(BLACK);
    }
    else{
      buttonState = !buttonState;
    }
    */
    
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
    M5.MPU6886.getGyroData(&gX, &gY, &gZ);
    M5.MPU6886.getAccelAdc(&accX,&accY,&accZ);
    M5.MPU6886.getTempAdc(&temp);
    aRes = M5.MPU6886.aRes;
    gRes = M5.MPU6886.gRes;
    //USE_SERIAL.println("Grabbing IMU Data");
  } else {
    //USE_SERIAL.println("IMU Not Set");
  }
 
  vbat = M5.Axp.GetBatVoltage();
 
  //Low Pass Filter
  fXg = accX * alpha + (fXg * (1.0 - alpha));
  fYg = accY * alpha + (fYg * (1.0 - alpha));
  fZg = accZ * alpha + (fZg * (1.0 - alpha));

  //Roll & Pitch Equations
  //roll  = 11+ (atan2(-fYg, fZg)*180.0)/M_PI;
  //pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI - 8;

  M5.MPU6886.getAhrsData(&pitch, &roll, &yaw);
  
}


void displayNetwork() {

  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("Target IP:\n %s:%i", targetIP, udpPort);

  if(!usingSoftAP) {
    IPAddress myIP = WiFi.localIP();
    M5.Lcd.setCursor(0,20);
    M5.Lcd.printf("My IP:\n %i.%i.%i.%i", myIP[0], myIP[1], myIP[2], myIP[3]);
  } else {
    IPAddress myIP = WiFi.softAPIP();
    M5.Lcd.setCursor(0,20);
    M5.Lcd.printf("My IP:\n %i.%i.%i.%i", myIP[0], myIP[1], myIP[2], myIP[3]);
  }

  M5.Lcd.setCursor(0,40);
  M5.Lcd.print("Network Name:");
  M5.Lcd.setCursor(0,50);

  int i = 0;
  char* uuid = (char*)myUUID;
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
  M5.Lcd.printf("Output UDP Port: %i", udpPort);
  /*
   * IPAddress myIP = WiFi.localIP();
   * M5.Lcd.printf("My IP: %i.%i.%i.%i:%i", myIP[0], myIP[1], myIP[2], myIP[3], localPort);
  */
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
  //M5.Lcd.printf("%.2f   %.2f   %.2f    ", ((float) gyroX) * gRes, ((float) gyroY) * gRes,((float) gyroZ) * gRes);
  M5.Lcd.printf("%.2f   %.2f   %.2f    ", gX, gY, gZ);
  M5.Lcd.setCursor(140, 30);
  M5.Lcd.print("*/s");
  M5.Lcd.setCursor(0, 40);
  M5.Lcd.printf("%.2f   %.2f   %.2f     ",((float) accX) * aRes,((float) accY) * aRes, ((float) accZ) * aRes);
  M5.Lcd.setCursor(144, 40);
  M5.Lcd.print("mg");
  /*M5.Lcd.setCursor(0, 50);
  M5.Lcd.print("Analog: ");
  M5.Lcd.print((int)analogSensorValue);
  M5.Lcd.print("   ");
  M5.Lcd.setCursor(75, 50);
  M5.Lcd.print(" / 4095"); */
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.printf("%.2f   %.2f   %.2f    ", pitch, roll, yaw);
  M5.Lcd.setCursor(0, 60);
  M5.Lcd.printf("Turbo Mode: %s", turboModeActive ? "enabled\t" : "disabled\t");
 
  M5.Lcd.setCursor(0, 70);
  M5.Lcd.printf("Battery Charge:\t%.3fV",vbat);
 
  //M5.Lcd.printf("Temperature : %.2f C",((float) temp) / 333.87 + 21.0);
  
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

    OSCMessage rotation("/rotation");
    rotation.add(pitch);
    rotation.add(roll);
    rotation.add(yaw);
    udp.beginPacket(targetIP, udpPort);
    rotation.send(udp);
    udp.endPacket();
    rotation.empty();

    OSCMessage buttonmsg("/button");
    buttonmsg.add((buttonState) ? 1 : 0);
    udp.beginPacket(targetIP, udpPort);
    buttonmsg.send(udp);
    udp.endPacket();
    buttonmsg.empty();

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
  if(count - idx > 0) {
    if(myPWD != NULL) {
      free(myPWD);
    }

    int bsize = count - idx + 1;
    
    myPWD = malloc(sizeof(char) * bsize);

    char* tmp = buf;
    tmp += idx;
    memcpy(myPWD, tmp, (bsize - 1));
    tmp = (char*)myPWD + (bsize - 1);
    *tmp = 0;

    //send back operation result
    USE_SERIAL.print("pwd 1");
    
  }
  else {
    //send back operation result
    USE_SERIAL.println("pwd 0");
  }
}

void copyUUID(char* buf, int count, int idx){
  if(count > 4) {
    if(myUUID != NULL) {
      free(myUUID);
    }
    int bsize = count - idx + 1;
    myUUID = malloc(sizeof(char) * bsize);

    char* tmp = buf;
    tmp += idx;
    memcpy(myUUID, tmp, (bsize - 1));
    tmp = (char*)myUUID + (bsize - 1);
    *tmp = 0;
    //send back operation result
    USE_SERIAL.println("uuid 1");
  }
  else {
    //send back operation result
    USE_SERIAL.println("uuid 0");
  }
}

void attemptToConnect() {
  if(myPWD != NULL && myUUID != NULL) {
    reconnectToWiFi((char*)myUUID, (char*)myPWD);
  } else {
    USE_SERIAL.println("att 0");
  }
}

void setIPAddress(char* buf, int count, int idx){
  if(count >= 11) {
  if(targetIP != NULL) {
    free(targetIP);
  }
  
  int bsize = count - idx + 1;

  targetIP = (char*)malloc(sizeof(char) * bsize);
  char* tmp = buf + idx;
  memcpy(targetIP, tmp, bsize - 1);
  tmp = targetIP + (bsize - 1);
  *tmp = 0;

  USE_SERIAL.println("ipa 1");
  M5.Lcd.fillScreen(BLACK);
  }
  else {
    USE_SERIAL.println("ipa 0"); 
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
    USE_SERIAL.println("prt 1");
    M5.Lcd.fillScreen(BLACK);
  }
  else {
    USE_SERIAL.println("prt 0");
  }
}

void setTurboMode(char* buf, int count, int idx) {

  //no args required, just toggle it.
  turboModeActive = !turboModeActive;

  USE_SERIAL.printf("trb %i", ((turboModeActive) ? 1 : 0));
  
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

    USE_SERIAL.println("lpt 1");
    M5.Lcd.fillScreen(BLACK);
  }
  else {
    USE_SERIAL.println("lpt 0");
  }
}

void storeToEEPROM() {
  if(myPWD != NULL && myUUID != NULL) {

    EEPROM.begin(73);
    
    int i = 0;
    int dataIdx = 0;
    char* tUUID = (char*)myUUID;
    
    //store Network Name
    while(tUUID[i] != 0 && i < 32) {
      EEPROM.write(dataIdx + i, tUUID[i]);
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
    char* tPWD = (char*)myPWD;
    
    while(tPWD[i] != 0 && i < 32) {
      EEPROM.write(dataIdx + i, tPWD[i]);
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
    //USE_SERIAL.printf("Saving Turbo Mode: %d\n", turbo);
    EEPROM.write(72, turbo);

    EEPROM.commit();
    EEPROM.end();

    USE_SERIAL.println("sav 1");
    
  } else {
    USE_SERIAL.println("sav 0");
  }
}

void clearEEPROM() {
  EEPROM.begin(73);
  for(int i = 0; i < 73; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  EEPROM.end();
  USE_SERIAL.println("clr 1");
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
  myServer.handleClient();
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
      //sum up the first 3 characters of the input
      while(i < 3){
        char c = buf[i];
        putchar(toupper(c));
        cmd += c;
        i++;
      }

      //parse the sum of the first 3 characters to find the appropriate function
      //all functions that take input require a space between the command and
      //the data parameters
      //USE_SERIAL.println(cmd);
      switch(cmd) {
        case 235: //Incoming Password (PWD <Password String>)
          copyPassword(buf, count, 4);
          break;
        case 226: //Incoming Network Name (UID <Network Name String>)
          copyUUID(buf, count, 4);
          break;
        case 229: //Life Check Ping (PNG)
          USE_SERIAL.write(1);
          M5.Lcd.fillScreen(RED);
          break;
        case 233: //Attempt To Connect (ATT)
          attemptToConnect();
          break;
        case 218: //Target IP Address (IPA <IP Address String>)
          setIPAddress(buf, count, 4);
          break;
        case 246: //Target Port (PRT <Port# / 255><Port# % 255>)
          setPort(buf, count, 4);
          break;
        case 238: //set local port (LPT <Port# / 255><Port# % 255>)
          setLocalPort(buf, count, 4);
          break;
        case 231: //Display Settings (DSP)
          printEEPROM();
          break;
        case 234: //Save Settings to EEPROM (SAV)
          storeToEEPROM();
          break;
        case 225: //Clear data from EEPROM (CLR)
          clearEEPROM();
          break;
        case 232: //Turbo mode (TRB (0|1))
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
  
  if(!usingSoftAP && !connected) {
    wifiConnectionLoops++;
    //Serial.printf("Connection Loops: %i\n", wifiConnectionLoops);
  }

  if(wifiConnectionLoops >= maxConnectionLoops) {
    initializeSoftAP();
  }
  
  if (turboModeActive){
    // No delay
  }
  else{
    // Delay based on user preference
    delay(nonTurboRefreshDelay);
  }
}

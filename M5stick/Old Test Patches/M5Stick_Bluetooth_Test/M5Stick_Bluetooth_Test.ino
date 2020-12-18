
#include <M5StickC.h>
#include <BluetoothSerial.h>

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

BluetoothSerial SerialBT;
uint64_t chipid;
char chipname[256];
char strBuffer[64];
String receivedStr;

void setup() {
  chipid = ESP.getEfuseMac();
  sprintf( chipname, "M5StickC_%04X", (uint16_t)(chipid >> 32));
  M5.begin();
  M5.Sh200Q.Init();
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.printf("Bluetooth: %s\n", chipname);
  M5.Lcd.printf("Ver: %s %s\n", __DATE__, __TIME__);
  M5.Lcd.println();
  M5.MPU6886.Init();
  
  SerialBT.begin(chipname);
  
  pinMode(10,OUTPUT);
  digitalWrite(10,HIGH);



  Serial.println(chipname);
  SerialBT.println(chipname);
  M5.Lcd.setCursor(8*4, 8*3);
  M5.Lcd.printf("%s", "Synthesis Center");
}

void loop() {
  receiveString();
  delay(10);
  M5.MPU6886.getGyroData(&gyroX,&gyroY,&gyroZ);
  M5.MPU6886.getAccelData(&accX,&accY,&accZ);
  SerialBT.println(accX);
  Serial.println(accX);
  SerialBT.println(accY);
  Serial.println(accY);
  SerialBT.println(accZ);
  Serial.println(accZ);
  SerialBT.println(gyroX);
  Serial.println(gyroX);
  SerialBT.println(gyroY);
  Serial.println(gyroY);
  SerialBT.println(gyroZ);
  Serial.println(gyroZ);
  m5.update();

}

void receiveString() {

  if (SerialBT.available()>0) {
    int i=0;
    // loop here until there is nothing left in the serial port
    while (SerialBT.available()>0){
      delay(10);
      strBuffer[i]=SerialBT.read();
      i++;   
    }
    //- add null termination at the end
    strBuffer[i-2]=0; 
    Serial.print("I just received: ");
    Serial.println(strBuffer);

    SerialBT.print("I just received: ");
    SerialBT.println(strBuffer);
    
    processString();
  }
}

void processString() {
  receivedStr=strBuffer;
  if (receivedStr.equals("ON")) {
    digitalWrite(10,LOW);
    Serial.println("Turning LED ON!");
    SerialBT.println("Turning LED ON!");
  }
  if (receivedStr.equals("OFF")) {
    digitalWrite(10,HIGH);
    Serial.println("Turning LED OFF!");
    SerialBT.println("Turning LED OFF!");
  }
  else {
    Serial.print("I don't know this command: ");
    Serial.println(receivedStr);

    SerialBT.print("I don't know this command: ");
    SerialBT.println(receivedStr);
  }
  //- clear buffer
  for (int i=0;i<64;i++) strBuffer[i]=0;
}

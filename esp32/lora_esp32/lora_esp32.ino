#include<WiFi.h>
#include <FirebaseESP32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <stdio.h>
#include <EEPROM.h>

FirebaseData firebaseData;
WiFiClient client;
String  path = "/";
FirebaseJson json;
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define FIREBASE_HOST "https://pzemt-iot-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "JY9IoWAjx649T8XKBYH9jJAinAmfVhbtMxkGbBRN"

const char* ssid = "KS Chuong"; //Enter SSID
const char* password = "0946627518"; //Enter Password

//  M1t33h50,60   mach 1 : nhiet do dht 33, do am dht 50, do am dat 60
//  M2t20h70,45   mach 2 : nhiet do dht 20, do am dht 70, do am dat 45
#define lora Serial2
#define BNT1 32
#define BNT2 35
#define BNT3 33
#define BNT4 34

#define READ_BNT1     digitalRead(BNT1)
#define READ_BNT2     digitalRead(BNT2)
#define READ_BNT3     digitalRead(BNT3)
#define READ_BNT4     digitalRead(BNT4)

String data_m1 = "";;
String data_m2 = "";;
String temp1 = "";
String temp2 = "";
String hum1  = "";
String hum2  = "";
String hum_land1 = "";
String hum_land2 = "";

typedef struct
{
  uint8_t temp1;
  uint8_t temp2;
  uint8_t hum1;
  uint8_t hum2:
  uint8_t hum_land1;
  uint8_t hum_land2;
}Sensor;
Sensor sensor;


typedef struct
{
  uint8_t send_m1;
  uint8_t send_m2;

} Status;
Status status;

typedef struct
{
  uint8_t e1;
  uint8_t e2;
  uint8_t e3;
} Read1;
Read1 read1;

typedef struct
{
  uint8_t e1;
  uint8_t e2;
  uint8_t e3;
} Read2;
Read2 read2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  lora.begin(9600);
  pinMode(BNT1, INPUT_PULLUP);
  pinMode(BNT2, INPUT_PULLUP);
  pinMode(BNT3, INPUT_PULLUP);
  pinMode(BNT4, INPUT_PULLUP);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("*");
  }

  Serial.println("");
  Serial.println("WiFi connection Successful");
  Serial.print("The IP Address of ESP32 Module is: ");
  Serial.print(WiFi.localIP());


  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (lora.available())
  {
    String data = lora.readString();
    if (data[1] == '1')
    {
      data_m1 = "";
      for (int i = 1; i < 10; i ++)
      {
        data_m1[i - 1] = data[i];
      }
      status.send_m1 = 1;
    }
    else if (data[2] == '2')
    {
      data_m2 = "";
      for (int i = 1; i < 10; i ++)
      {
        data_m2[i - 1] = data[i];
      }
      status.send_m2 = 1;
    }
  }

  /************ tach chuoi lora 1 ***************/
  if (status.send_m1 == 1)
  {
    temp1 = "";
    hum1  = "";
    hum_land1 = "";
    for (int i = 0; i < data_m1.length(); i ++)
    {
      if (data_m1[i] == 't')
      {
        read1.e1 = 1;
        read1.e2 = 0;
        read1.e3 = 0;
      }
      else if (data_m1[i] == 'h')
      {
        read1.e1 = 0;
        read1.e2 = 1;
        read1.e3 = 0;
      }
      else if (data_m1[i] == ',')
      {
        read1.e1 = 0;
        read1.e2 = 0;
        read1.e3 = 1;
      }
      else
      {
        if (read1.e1 == 1) temp1 += data_m1[i];
        if (read1.e2 == 1) hum1  += data_m1[i];
        if (read1.e3 == 1) hum_land1 += data_m1[i];
      }
    }

    sensor.temp1 = temp1.toInt();
    sensor.hum1  = hum1.toInt();
    sensor.humland1 = hum_land1.toInt();

    Firebase.setString(firebaseData, path + "/dht11_t1", temp1);
    Firebase.setString(firebaseData, path + "/dht11_h1", hum1);
    Firebase.setString(firebaseData, path + "/hum_land1", hum_land1);

    status.send_m1 = 0 ;
  }


  /************ tach chuoi lora 2 ***************/
  if (status.send_m2 == 1)
  {
    temp2 = "";
    hum2  = "";
    hum_land2 = "";
    for (int i = 0; i < data_m2.length(); i ++)
    {
      if (data_m2[i] == 't')
      {
        read2.e1 = 1;
        read2.e2 = 0;
        read2.e3 = 0;
      }
      else if (data_m2[i] == 'h')
      {
        read2.e1 = 0;
        read2.e2 = 1;
        read2.e3 = 0;
      }
      else if (data_m2[i] == ',')
      {
        read2.e1 = 0;
        read2.e2 = 0;
        read2.e3 = 1;
      }
      else
      {
        if (read2.e1 == 1) temp2 += data_m2[i];
        if (read2.e2 == 1) hum2  += data_m2[i];
        if (read2.e3 == 1) hum_land2 += data_m2[i];
      }
    }

    sensor.temp2 = temp2.toInt();
    sensor.hum2  = hum2.toInt();
    sensor.humland2 = hum_land2.toInt();
    
    Firebase.setString(firebaseData, path + "/dht11_t2", temp2);
    Firebase.setString(firebaseData, path + "/dht11_2", hum2);
    Firebase.setString(firebaseData, path + "/hum_land2", hum_land2);

    status.send_m2 = 0 ;
  }


  /*********** doc du lieu tu ap ****************/

  if (Firebase.getString(firebaseData, path + "/BOM1"))
  {

  }
  if (Firebase.getString(firebaseData, path + "/BOM2"))
  {

  }

  /******** hien thi du lieu len lcd *****/
  lcd.setCursor(0, 0);
  lcd.print("T1:");
  lcd.setCursor(3,0);
  if(sensor.temp1 < 10)
  {
    lcd.print("0");
    lcd.print(sensor.temp1);
  }
  else lcd.print(sensor.temp1);

  lcd.setCursor(8, 0);
  lcd.print("H1:");
  lcd.setCursor(11, 0);
  if(sensor.hum1 < 10)
  {
    lcd.print("0");
    lcd.print(sensor.hum1);
  }
  else lcd.print(sensor.hum1);


  lcd.setCursor(0,1);
  lcd.print("T2:");
  lcd.setCursor(3,1);
  if(sensor.temp2 < 10)
  {
    lcd.print("0");
    lcd.print(sensor.temp2);
  }
  else lcd.print(sensor.temp2);


  lcd.setCursor(8,1)'
  lcd.print("H2:");
  lcd.setCursor(11, 1);
  if(sensor.hum2 < 10)
  {
    lcd.print("0");
    lcd.print(sensor.hum2);
  }
  else lcd.print(sensor.hum2);


}

//Navigation Controller Tabiji CubeSat
//Update 05 March 2021 Version 1.3 : Sora

#include <Wire.h> //Needed for I2C to GNSS
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
#include <Adafruit_Sensor.h>                        // Global use
#include <TimeLib.h>                                // TEENSY RTC
#include <Adafruit_BNO055.h>                        // BNO055
#include <utility/imumaths.h>                       // BNO055
#include <SD.h>                                     // SD Card
#include <SPI.h>                                    // SD Card

//Declare connection protocol
SFE_UBLOX_GNSS myGNSS;
File DataLog;
time_t RTCTime;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
//***************************//

//Declare program constants
long prealt; int Pre=1; int state;          //WhatIsDeltaH
bool   co = true; int statecount;           //WhatIsDeltaH
int    prestate; bool onlyone = true;       //WhatIsDeltaH
String RTC_HOUR,RTC_MINUTE,RTC_SECOND;      //TEENSY RTC
String F9R_LAT,F9R_LON,F9R_ALT;             //GPS F9R
String ACCx,ACCy,ACCz,OREx,OREy,OREz;       //BNO055    
unsigned long counter; int Ptime; bool RSQ = false;                
//***************************//

void setup() 
{
  Serial.begin(115200);
  Serial2.begin(115200); //NAV to MAIN 
  Serial5.begin(9600);   //SIM800
  Wire.begin();
  Wire.setClock(400000);
  bno.begin();
  myGNSS.begin();
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  setSyncProvider(getTeensy3Time);
  if(!SD.begin(10))
  {
    Serial.println("SD ERROR");
  }
  else
  {
    Serial.println("SD OK");
  }
  DataLog = SD.open("NAVLOG.csv", FILE_WRITE);
}

time_t getTeensy3Time() {
  return Teensy3Clock.get();
} 

static void rtc()
{
  RTC_HOUR = "";
  RTC_MINUTE = "";
  RTC_SECOND = "";
  int HOUR = hour();
  int MINUTE = minute();
  int SECOND = second();
  if(HOUR<10)
  {
    RTC_HOUR = "0";
  }
  if(MINUTE<10)
  {
    RTC_MINUTE = "0";
  }
  if(SECOND<10)
  {
    RTC_SECOND = "0";
  }
  RTC_HOUR += String(HOUR);
  RTC_MINUTE += String(MINUTE);
  RTC_SECOND += String(SECOND);
}

static void F9R()
{
  long l1 = myGNSS.getLatitude();
  long l2 = myGNSS.getLongitude();
  long a1 = myGNSS.getAltitude();
  double ALT = a1/1000.0;
  double LAT = l1/10000000.0;
  double LON = l2/10000000.0;
  F9R_LAT = String(LAT,7);
  F9R_LON = String(LON,7);
  F9R_ALT = String(ALT,2); // in Meters
}

static void Bno()
{
   sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
   bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
   bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
   printEvent(&accelerometerData);
   printEvent(&orientationData);
}

void NAVtoMAIN(String PAX)
{
  Serial2.println(PAX);
  Serial.println(PAX);
}

int WhatIsDeltaH(long Alt)
{
  if(onlyone==true) 
  {
    onlyone=false;
    prealt = Alt;
  }
    if(prealt > Alt) //Altitude decreaseing
    {
      prealt=Alt;
      state = -1;
      co = true;
    }
    else if(prealt < Alt) //Altitude increaseing
    {
      prealt=Alt;
      state = 1;
      co = true;
    }
    else
    {
      prealt=Alt;
      state = 0;
      co = true;
    }
    if(co==true)
    {
      if(prestate==state)
      {
        prestate = state;
        statecount++;
      }
      else
      {
        prestate = state;
        statecount=0;
        return 0;
      }
      if(statecount>=50)
      {
        return state;
      }
    }
}

static void SendSMS(String SMS_packet)
{
  Serial5.print("AT+CMGF=1\r");                   //Set the module to SMS mode
  delay(100);
  Serial5.print("AT+CMGS=\"0828903390\"\r");  //Your phone number don't forget to include your country code, example +212123456789"
  delay(500);
  Serial5.print(SMS_packet);       //This is the text to send to the phone number, don't make it too long or you have to modify the SoftwareSerial buffer
  delay(500);
  Serial5.print((char)26);// (required according to the datasheet)
  delay(500);
  Serial5.println();
  delay(500);
}

void printEvent(sensors_event_t* event) 
{
  float accx,accy,accz,orex,orey,orez;
   if (event->type == SENSOR_TYPE_ACCELEROMETER) 
   {
    accx = event->acceleration.x;
    accy = event->acceleration.y;
    accz = event->acceleration.z;
    ACCx = String(accx);
    ACCy = String(accy);
    ACCz = String(accz);
   }
   else if (event->type == SENSOR_TYPE_ORIENTATION) 
   {
    orex = event->orientation.x;
    orey = event->orientation.y;
    orez = event->orientation.z;
    OREx = String(orex);
    OREy = String(orey);
    OREz = String(orez);
   }
   return;
}
void serialEvent2()
{
  if(Serial2.available())
  {
    char IN = Serial2.read();
    if(IN=='S')
    {
      RSQ = true;
    }
    else
    {
      RSQ = false;
    }
  }
}
unsigned long ontimE,ontimE2;
void loop() 
{
  if(ontimE-millis()>25)
  {
    rtc();
    F9R();
    Bno();
    ontimE = millis();
    String core = String(counter++) + "," + RTC_HOUR + "," + RTC_MINUTE + "," + RTC_SECOND + "," + F9R_LAT + "," + 
                  F9R_LON + "," + F9R_ALT + "," + ACCx + "," + ACCy + "," + ACCz + "," + OREx + "," + OREy + "," + OREz;
   if(DataLog)
        {
          DataLog.println(core);
          DataLog.flush();  
        }    
  }
  if(RSQ==true)
  {
    String heart =  "*" + F9R_LAT + "," + F9R_LON + "," + F9R_ALT + "," + ACCx + "," + ACCy + "," 
                  + ACCz + "," + OREx + "," + OREy + "," + OREz + "#";
    NAVtoMAIN(heart);
    RSQ = false;
    Ptime = RTC_SECOND.toInt();
  }          
  double RAM = F9R_ALT.toFloat();
         RAM = RAM*10;
  if(WhatIsDeltaH(RAM)==-1&&false) // delete second argument to run properly
   { Serial.println(RTC_MINUTE);
    if(RTC_MINUTE.toInt()%6==0&&RTC_SECOND.toInt()==0)
    {
       String SMS_Massage = "Current Location of Tabiji chan at " + RTC_HOUR + ":" + RTC_MINUTE + ":" + RTC_SECOND
                            + " is : " + F9R_LAT + "," + F9R_LON + " Altitude : " + F9R_ALT + "Meters from sea level." 
                            + " Good luck finding me Onii-chan :)";
       SendSMS(SMS_Massage);
    }
   }
}

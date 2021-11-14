//Main Processor Tabiji CubeSat
//Update 04 March 2021 Version 1.6.1 : Sora

#include <SPI.h>                              // SD Card
#include <SD.h>                               // SD Card
#include <Wire.h>                             // Global use
#include <Adafruit_Sensor.h>                  // Global use
#include "Seeed_BME280.h"                     // BME280
#include "MAX31855.h"                         // MAX31855
#include <TimeLib.h>                          // TEENSY RTC
#include "SparkFun_Ublox_Arduino_Library.h"   // NEO-M9N
#include <ADC.h>                              // TEENSY ADC
#include <ADC_util.h>                         // TEENSY ADC
#include <TinyGPS++.h>                        // MAX M8Q
#include <Q2HX711.h>                          // MPS20N0040D
#include "Adafruit_PM25AQI.h"                 // PMS7003

#define LOUD_Buzzer 3
#define LED         4
#define Buzzer      2
#define Battery     A8
#define STOP_SW     23
#define LOGIC_A     3
#define LOGIC_B     4

//Declare program constants
long prealt; int Pre=1; int state; int Ptime;         //WhatIsDeltaH
bool   co = true; int statecount;           //WhatIsDeltaH
int    prestate; bool onlyone = true;       //WhatIsDeltaH
int    pacnum = 1,comparesec;                          //Readfrom NAV
unsigned long counter,counter1;             //counter 1
double Maxalt; int SaveAlttime; 
bool   WASB = false,LPD = false,ASKLP = false;
String SYS_STA ="0",SYS_QUA;                //System stage and system query
String BME_TEMP,BME_PRES,BME_HUMI,BME_ALTI; //BME280
float  BM_TEM;                              //BME280
String MAX_PROBE_TEMP,MAX_INTER_TEMP;       //MAX31855
String RTC_H,RTC_M,RTC_S;                   //TEENSY RTC
String M9N_LAT,M9N_LON,M9N_ALT,GPS_STA;     //GPS M9N
long lastTime;
String BAT_VOLT; float BATTper;             //BATTERY VOLTAGE       
String M8Q_LAT,M8Q_LON;                     //GPS M8Q    
String MPS_PRES,EXT_ALT;                    //MPS20N0040D    
String PMS_10,PMS_25,PMS_100;               //PMS7003  
String F9R_LAT,F9R_LON,F9R_ALT;             //GPS F9R
String ACCx,ACCy,ACCz,OREx,OREy,OREz;       //BNO055 
bool buttonstate=false; int buttoncounter=0;
bool buttonlogic,Bonce=true,finalbutton=false;
char passstage = 'F'; bool DoPD = false;
//***************************//

//Declare connection protocol
BME280 bme280;
MAX31855 tc(9, 5, 6);
File DataLog;
time_t RTCTime;
SFE_UBLOX_GPS myGPS;
ADC *adc = new ADC();
TinyGPSPlus gps;
Q2HX711 MPS20N0040D(0,1);
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
PM25_AQI_Data data;
//***************************//    

void RAISARA()
{
  randomSeed(analogRead(A6)/(millis()*3.15445864157432657615)*analogRead(A2));
  int ran = random(2,3);
  if(ran==2)
  {
    int duration = 300;
    tone(Buzzer,1108.7,duration/2);
    delay(duration/2.5);
    tone(Buzzer,1244.5,duration/2);
    delay(duration/2.5);
    tone(Buzzer,1318.5,duration);
    delay(duration);
    tone(Buzzer,1318.5,duration);
    delay(duration);
    tone(Buzzer,1244.5,duration*1.5);
    delay(duration*1.5);
    tone(Buzzer,1108.7,duration*1.5);
    delay(duration*1.5);
    tone(Buzzer,659.26,duration*5);
    delay(duration*5);
    lcdclear();
  }
}

void BEEP(int du,int freq)
{
  tone(Buzzer,freq,du);
  delay(du);
}

static void lcdclear()
{
  Wire.beginTransmission(9);
  Wire.print("*$$"); // lcdclear(); command
  Wire.endTransmission();
  delay(75);
}

void lcdprint(String Massage,int Col,int Line)
{
  Wire.beginTransmission(9);
  Wire.print("<"); // lcdclear(); command
  Wire.print(Massage); // lcdclear(); command
  Wire.print(">"); // lcdclear(); command
  Wire.print(Col); // lcdclear(); command
  Wire.print(","); // lcdclear(); command
  Wire.print(Line); // lcdclear(); command
  Wire.print("$$"); // lcdclear(); command
  Wire.endTransmission();
  delay(75);
}

void Ini(bool logic)
{
  if(logic==true)
  {
    lcdprint("   Process failed   ",0,3);
    BEEP(100,1500);
    delay(1200);
  }
  else
  {
    lcdprint(" Process completed ",0,3);
    BEEP(100,450);
    delay(1200);
  }
}

void setup() 
{
  delay(5000);
  pinMode(Buzzer,OUTPUT);
  pinMode(LED,OUTPUT);
  pinMode(LOUD_Buzzer,OUTPUT);
  pinMode(STOP_SW,INPUT);
  pinMode(LOGIC_A,INPUT);
  pinMode(LOGIC_B,INPUT);
  Wire.begin();          
  lcdprint("-SoRa System-",3,0);
  lcdprint("-TABIJI CLONE1-",2,1);
  delay(1000);
  lcdprint("-TABIJI CUBESAT-",2,0);
  delay(100);
  lcdprint("Initializing object:",0,1);
  delay(100);
  lcdprint("--Computer Serial---",0,2);
  delay(100);
  Serial.begin(115200); if(!Serial) Ini(true); else Ini(false);
  
  lcdprint("-Navigation Serial--",0,2);
  Serial2.begin(115200); if(!Serial2) Ini(true); else Ini(false); // NAV controller

  lcdprint("Communication Serial",0,2);
  Serial3.begin(115200); if(!Serial3) Ini(true); else Ini(false); // COM controller

  lcdprint("----MAX M8Q GPS2----",0,2);
  Serial4.begin(9600);   if(!Serial4) Ini(true); else Ini(false); // MAX M8Q

  lcdprint("---PMS7003 Serial---",0,2);
  Serial5.begin(9600);
  aqi.begin_UART(&Serial5);
  if(!Serial5) Ini(true); else Ini(false); // PMS7003

  lcdprint("-------BME280-------",0,2);
  if(!bme280.init()) Ini(true); else Ini(false);

  lcdprint("------MAX31855------",0,2);
  tc.begin();          Ini(false);// CS pin5 ; MISO pin6 ; SCK pin9 No way to check.

  lcdprint("-----Teensy RTC-----",0,2);
  setSyncProvider(getTeensy3Time); Ini(false); //No way to check.  Sync program time to Teensy RTC time 

  lcdprint("--UBlox NEOM9N GPS--",0,2);
  if(!myGPS.begin()) Ini(true); else Ini(false);
  myGPS.setUART1Output(0); //Disable the UART1 port output 
  myGPS.setUART2Output(0); //Disable Set the UART2 port output
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.setNavigationFrequency(7); //Set output to 10 times a second]
  myGPS.saveConfiguration();
  
  lcdprint("-----TEENSY ADC-----",0,2);
  adc->adc0->setAveraging(10); // set number of averages
  adc->adc0->setResolution(16); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed
  Ini(false); //No way to check.

  lcdprint("---SD Card reader---",0,2);
  if(!SD.begin(10)) Ini(true); else Ini(false);
  DataLog = SD.open("MAINLOG.csv", FILE_WRITE);

  lcdclear();
  lcdprint("TABIJI CUBESAT",3,1);
  delay(100);
  lcdprint(" Process completed ",0,2);
  delay(1200);
  lcdprint(" Welcome Onii-chan ",0,2);
  RAISARA();
  lcdclear();
  lcdprint("Tabiji CS System",2,0);
  SYS_QUA = "F";
}

time_t getTeensy3Time() {
  return Teensy3Clock.get();
} 

static void BME()
{
  float bmepres;
  //Serial.print("BME\t");
  BME_TEMP = String(bme280.getTemperature());  // in degree celcius
  bmepres  = bme280.getPressure();
  BME_HUMI = String(bme280.getHumidity());
  BME_ALTI = String(bme280.calcAltitude(bmepres));   // in meters
  BME_PRES = String(bmepres/100.00);  //in hPa
  //Serial.print("BME_OVER\t");
  rtc();
}

void MAX()
{
  //Serial.print("MAX\t");
  int status = tc.read();
   if (tc.getStatus())
  {
    //Serial.print("error:\t\t");
    if (tc.shortToGND())   Serial.println("SHORT TO GROUND");
    if (tc.shortToVCC())   Serial.println("SHORT TO VCC");
    if (tc.openCircuit())  Serial.println("OPEN CIRCUIT");
    if (tc.genericError()) Serial.println("GENERIC ERROR");
    if (tc.noRead())       Serial.println("NO READ");
    if (tc.noCommunication()) Serial.println("NO COMMUNICATION");
  }
  float ambientTemperature = tc.getInternal();
  
  
  float probeTemperature = tc.getTemperature();
  if (probeTemperature>-40&&probeTemperature<100){
    MAX_PROBE_TEMP = String(probeTemperature);
  }

  MAX_INTER_TEMP = String(ambientTemperature); // in degree celcius
     // in degree celcius
     pms();
}

static void rtc()
{
  //Serial.print("RTC\t");
  RTC_H = "";
  RTC_M = "";
  RTC_S = "";
  int HOUR = hour();
  int MINUTE = minute();
  int SECOND = second();
  comparesec = SECOND;
  if(HOUR<10)
  {
    RTC_H = "0";
  }
  if(MINUTE<10)
  {
    RTC_M = "0";
  }
  if(SECOND<10)
  {
    RTC_S = "0";
  }
  RTC_H += String(HOUR);
  RTC_M += String(MINUTE);
  RTC_S += String(SECOND);
  BAT();
}

static void M9N()
{
  long l1 = myGPS.getLatitude();
  long l2 = myGPS.getLongitude();
  long a1 = myGPS.getAltitude();
  if(myGPS.getSIV()>=6)
  {
    GPS_STA = "OK.";
  }
  else
  {
    GPS_STA = "ERR.";
  }
  double ALT = a1/1000.0;
  double LAT = l1/10000000.0;
  double LON = l2/10000000.0;
  M9N_LAT = String(LAT,7);
  M9N_LON = String(LON,7);
  M9N_ALT = String(ALT,2); // in Meters
}

static void BAT()
{
  //Serial.print("BAT\t");
  int value1 = adc->adc0->analogRead(Battery);
  if(adc->adc0->fail_flag != ADC_ERROR::CLEAR) 
  {
    //Serial.print("ADC0: "); 
    //Serial.println(getStringADCError(adc->adc0->fail_flag));
  }
  float batt = (((value1*0.0008056640625)*(5000.0+1000.0))/1000.0)-0.16;
  BATTper = 100*pow(1+exp(-3.32*(batt-15.02)),-1);
  BAT_VOLT = String(batt,2); // in volts
  MPS();
}

static void M8Q()
{
  float l3 = gps.location.lat();
  float l4 = gps.location.lng();
  M8Q_LAT = String(l3,6);
  M8Q_LON = String(l4,6);
}

static void MPS()
{
  //Serial.print("MPS\t");
  float avg_val = MPS20N0040D.read();
  double VR = (avg_val*0.0000001966953277587890625)*1000.0;
  double P = ((29.5/50.0)*(VR-22.6))-38.29;
  MPS_PRES = String(P,2);
  MAX();
}

static void pms()
{
  //Serial.print("PMS\t");
  if(Serial5.available())
  {
      aqi.read(&data);
      PMS_10 = String(data.pm10_env);
      PMS_25 = String(data.pm25_env);
      PMS_100 = String(data.pm100_env);
  }
   if(Serial4.available())
  {
    //Serial.print("M8Q\t");
    while(Serial4.available())
    {
      if (gps.encode(Serial4.read()))
      {
        M8Q();
      }
    }
  }
  //Serial.print("Wire\t");
      Wire.beginTransmission(8);
      Wire.write(passstage);
      Wire.write(finalbutton);
      Wire.endTransmission();
      //Serial.print("Wire over\n");
}

void Report()
{
  
  lcdprint("Batt ",0,1);
  if(BATTper!=100)
  {
    lcdprint(String(BATTper),4,1);
  }
  else
  {
    lcdprint("100.0",4,1);
  }
  lcdprint("%",9,1);
  lcdprint("Pkt ",11,1);
  lcdprint(String(counter),14,1);
  lcdprint("Temp",0,2);
  lcdprint(String(BME_TEMP),4,2);
  lcdprint("C",8,2);
  lcdprint("GPS=",12,2);
  lcdprint(String(GPS_STA),16,2);
  lcdprint("Humi",0,3);
  lcdprint(String(BME_HUMI),4,3);
  lcdprint("%",7,3);
  lcdprint("Alt",9,3);
  lcdprint(String(EXT_ALT),12,3);
  lcdprint("m",19,3);
  ////Serial.println(MAX_PROBE_TEMP);
}

void MAINtoCOM(String P8681,String P8682,String P4331,String P4332)
{
  Serial3.print(P8681);
  Serial3.print(P8682);
  Serial3.print(P4331);
  Serial3.print(P4332);
//Serial.println(P8681);
//  Serial.println(P8682);
//  Serial.println(P4331);
//  Serial.println(P4332);
  ////Serial.println(P433);
}

double GetAltitude(float PRESS, float TEMP) 
{
 //return ((pow((1013.25 / PRESS), 1/5.257) - 1.0) * (TEMP + 273.15)) / 0.0065;
 return ((8.3143*TEMP)/0.283808)*((log(1004.8315/PRESS))/0.4342944819);
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
        if(Alt>prealt)
        {
          Maxalt = Alt;
          SaveAlttime = RTC_S.toInt();
        }
        return 0;
      }
      if(statecount>=50)
      {
        if(Alt>prealt)
        {
          Maxalt = Alt;
          SaveAlttime = RTC_S.toInt();
        }
        return state;
      }
    }
}

void SYS_STA_QUA(float alt,int Alttime)
{
    int compare = WhatIsDeltaH(alt);
    if(alt<=1020&&alt>=980&&compare==1)
    {
      SYS_STA = "A";
    }
    else if(alt<=11020&&alt>=10980) 
    {
      SYS_STA = "B";
      WASB = true;
    }
    else if(alt<Maxalt&&SaveAlttime==Alttime)
    {
      SYS_STA = "C";
    }
    else if(alt<=11020&&alt>=10980&&WASB==true)
    {
      SYS_STA = "D";
      passstage = 'T';
    }
    else if(compare==0)
    {
      SYS_STA = "E";
    }
}
void loop() 
{
  BME();
  if(DataLog)
       {
        M9N();
         String Sora = String(counter1++) + "," + RTC_H + "," + RTC_M + "," + RTC_S + "," + M9N_LAT + "," + M9N_LON + "," 
         + M9N_ALT + "," + M8Q_LAT + "," + M8Q_LON + "," + MAX_PROBE_TEMP + ",";
         String Sora2 = PMS_100 + "," + PMS_25 + "," + PMS_10 + "," + BAT_VOLT + "," + MPS_PRES + "," + BME_TEMP + "," + BME_HUMI + ","
          + String(GetAltitude(MPS_PRES.toFloat(), MAX_PROBE_TEMP.toFloat()));
          DataLog.print(Sora);
          DataLog.println(Sora2);
          DataLog.flush();  
       }
  if(digitalRead(LOGIC_A)==true&&DoPD==false)
  {
    SYS_QUA = "R";  //Request powerdown
    if(digitalRead(LOGIC_B)==true)
    {
      SYS_QUA = "D"; // POWER DOWN
      DoPD = true;
    }
  }
  else if(DoPD==false)
  {
    SYS_QUA = "F";
  }
  if(digitalRead(LOGIC_A)==false&&DoPD==true)
  {
    SYS_QUA = "P";  //Request powerup
    if(digitalRead(LOGIC_B)==false)
    {
      SYS_QUA = "F"; // NORMAL MODE
      DoPD = false;
    }
  }
  else if(DoPD==true)
  {
    SYS_QUA = "D";
  }
  if(comparesec%2==0&&DoPD==false&&Ptime!=comparesec)
  {
    Serial2.print("S");
    bool mind=false;
  if(Serial2.available())
  {
   while(Serial2.available())
    {
      //F9R_LAT,F9R_LON,F9R_ALT,ACCx,ACCy,ACCz,OREx,OREy,OREz;
      char inChar = Serial2.read();
      ////Serial.print(inChar);
      if(inChar == '*')
      {
        mind = true;
        F9R_LAT = "" ; F9R_LON = "" ;F9R_ALT= "" ;ACCx= "" ;ACCy= "" ;ACCz= "" ;OREx= "" ;OREy= "" ;OREz = "";
        pacnum=1;
      }
      if(mind==true)
      {
        if(inChar == ',')
        {
          pacnum++;
        }
        else if(inChar == '#')
        {
          pacnum=0;
        }
        else if(inChar != '*'&&inChar!='#')
        {
          switch(pacnum)
          {
            case 1 : F9R_LAT += String(inChar); break;
            case 2 : F9R_LON += String(inChar); break;
            case 3 : F9R_ALT += String(inChar); break;
            case 4 : ACCx += String(inChar);    break;
            case 5 : ACCy += String(inChar);    break;
            case 6 : ACCz += String(inChar);    break;
            case 7 : OREx += String(inChar);    break;
            case 8 : OREy += String(inChar);    break;
            case 9 : OREz += String(inChar);    break;
          }
        }
      }
    }
  } 
    Ptime = comparesec;
     counter++;
      float extalt = GetAltitude(MPS_PRES.toFloat(), MAX_PROBE_TEMP.toFloat());
      SYS_STA_QUA(extalt,RTC_S.toInt());
      EXT_ALT = String(extalt);
      String PAC8681 = "*$PSG2$CH868$PC" + String(counter) + "$ST" + RTC_H + RTC_M + RTC_S + "$GA" + M9N_LAT + "$GB" +
                      M9N_LON + "$GR" + M9N_ALT + "$GC" + M8Q_LAT;
      String PAC8682 = "$GD" + M8Q_LON + "$ET" + MAX_PROBE_TEMP + "$KA" + PMS_25 + "$KB" + PMS_10 + "$BV" + BAT_VOLT + 
                        "$KC" + SYS_STA + "$SQ" + SYS_QUA + "$2GSP$&";
      String PAC4331 = "#$PSG2$CH433$PC" + String(counter) + "$ST" + RTC_H + RTC_M + RTC_S + "$GA" + F9R_LAT + "$GB" +
                      F9R_LON + "$ER" + EXT_ALT + "$GR" + F9R_ALT + "$IT" + BME_TEMP;
      String PAC4332 = "$ET" + MAX_PROBE_TEMP + "$IH" 
                      + BME_HUMI + "%^" +"$aX" + ACCx + "$aY" + ACCy + "$aZ" + ACCz + "$gX" + OREx + "$gY" + OREy + "$gZ" 
                      + OREz + "$EP" + MPS_PRES + "$SQ" + SYS_QUA + "$2GSP$@";
      Report();
      MAINtoCOM(PAC8681,PAC8682,PAC4331,PAC4332);
      ////Serial.println(MAX_PROBE_TEMP);
  }
  if(comparesec%10==0&&DoPD==true&&Ptime!=comparesec)
  {
    Serial2.print("S");
    bool mind=false;
  if(Serial2.available())
  {
   while(Serial2.available())
    {
      //F9R_LAT,F9R_LON,F9R_ALT,ACCx,ACCy,ACCz,OREx,OREy,OREz;
      char inChar = Serial2.read();
      ////Serial.print(inChar);
      if(inChar == '*')
      {
        mind = true;
        F9R_LAT = "" ; F9R_LON = "" ;F9R_ALT= "" ;ACCx= "" ;ACCy= "" ;ACCz= "" ;OREx= "" ;OREy= "" ;OREz = "";
        pacnum=1;
      }
      if(mind==true)
      {
        if(inChar == ',')
        {
          pacnum++;
        }
        else if(inChar == '#')
        {
          pacnum=0;
        }
        else if(inChar != '*'&&inChar!='#')
        {
          switch(pacnum)
          {
            case 1 : F9R_LAT += String(inChar); break;
            case 2 : F9R_LON += String(inChar); break;
            case 3 : F9R_ALT += String(inChar); break;
            case 4 : ACCx += String(inChar);    break;
            case 5 : ACCy += String(inChar);    break;
            case 6 : ACCz += String(inChar);    break;
            case 7 : OREx += String(inChar);    break;
            case 8 : OREy += String(inChar);    break;
            case 9 : OREz += String(inChar);    break;
          }
        }
      }
    }
  } 
      Ptime = comparesec;
      counter++;
      float extalt = GetAltitude(MPS_PRES.toFloat(), MAX_PROBE_TEMP.toFloat());
      SYS_STA_QUA(extalt,RTC_S.toInt());
      EXT_ALT = String(extalt);
      String PAC8681 = "*$PSG2$CH868$PC" + String(counter) + "$ST" + RTC_H + RTC_M + RTC_S + "$GA" + M9N_LAT + "$GB" +
                      M9N_LON + "$GR" + M9N_ALT + "$GC" + M8Q_LAT;
      String PAC8682 = "$GD" + M8Q_LON + "$ET" + MAX_PROBE_TEMP + "$KA" + PMS_25 + "$KB" + PMS_10 + "$BV" + BAT_VOLT + 
                        "$KC" + SYS_STA + "$SQ" + SYS_QUA + "$2GSP$&";
      String PAC4331 = "#$PSG2$CH433$PC" + String(counter) + "$ST" + RTC_H + RTC_M + RTC_S + "$GA" + F9R_LAT + "$GB" +
                      F9R_LON + "$ER" + EXT_ALT + "$GR" + F9R_ALT + "$IT" + BME_TEMP;
      String PAC4332 = "$ET" + MAX_PROBE_TEMP + "$IH" 
                      + BME_HUMI + "%^" +"$aX" + ACCx + "$aY" + ACCy + "$aZ" + ACCz + "$gX" + OREx + "$gY" + OREy + "$gZ" 
                      + OREz + "$EP" + MPS_PRES + "$SQ" + SYS_QUA + "$2GSP$@";
      Report();
      MAINtoCOM(PAC8681,PAC8682,PAC4331,PAC4332);
  }
  
  if(digitalRead(STOP_SW)!=buttonstate)
  {
    buttonstate=buttonlogic;
    buttoncounter++;
    if(buttoncounter>=12)
    {
      finalbutton=true;
    }
  }
}

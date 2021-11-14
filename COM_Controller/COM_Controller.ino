//Communication Controller Tabiji CubeSat
//Update 05 March 2021 Version 1.3.2 : Sora

#include <SD.h>                 // SD Card
#include <SPI.h>                // SD Card


//Declare program constants
String PAC433,PAC868,PAC433_2;
String BUF433,BUF868,BUF433_2;
String s;       //TEENSY RTC
int    T_S;      
bool   R433 = false;
bool   R868 = false;    
bool   R433_2 = false;
bool   Write = false;  
//***************************//

//Declare connection protocol
File DataLog;
time_t RTCTime;
//***************************//    

void setup() 
{
  pinMode(SS, OUTPUT);
  Serial.begin(9600);
  Serial3.begin(115200);  //MAIN to COM
  Serial4.begin(9600);    //LoRa 433 MHz
  Serial5.begin(9600);    //LoRa 868 MHz
  if(!SD.begin(10))
  {
    Serial.println("SD ERROR");
  }
  else
  {
    Serial.println("SD OK");
  }
  DataLog = SD.open("COMLOG.csv", FILE_WRITE);
}

void loop() 
{
  if(Serial3.available())
  {
    while(Serial3.available())
    {
      char inChar = Serial3.read();
      if(inChar == '*')
      {
        s = "";
        Write = true;
      }
      if(inChar == '@') Write = false;
      if(Write==true)
      {
        s += String(inChar);
      }
      else if(Write==false)
      {
        //Serial.println(s);
        String F868 = s.substring(s.indexOf("*")+1,s.indexOf("&"));
        String F4331 = s.substring(s.indexOf("#")+1,s.indexOf("%"));
        String F4332 = s.substring(s.indexOf("^")+1,s.indexOf("@"));
//        Serial.println(F868);
//        Serial.print(F4331);
//        Serial.println(F4332);
        Serial5.println(F868);
        Serial4.print(F4331);
        Serial4.println(F4332);
        if(DataLog)
        {
          DataLog.println(F868);
          DataLog.print(F4331);
          DataLog.println(F4332);
          DataLog.flush();  
        }
      }
    }
  }
}


#include <SoftwareSerial.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);
SoftwareSerial mySerial(8, 9); // RX, TX

void setup() {
  lcd.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd.backlight();
  Serial.begin(9600);
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  //mySerial.println("Hello, world?");
}
// format = *<MASSAGE>col,line$$ 

void loop() { // run over and over
  if (mySerial.available()) {
    String s = mySerial.readStringUntil('$');
    String Massage = s.substring(s.indexOf("<")+1,s.indexOf(">"));
    String Col = s.substring(s.indexOf(">")+1,s.indexOf(","));
    String Line = s.substring(s.indexOf(",")+1,s.indexOf("$")); 
    if(s.indexOf("*")==-1)
    {
      lcd.setCursor(Col.toInt(),Line.toInt());
      lcd.print(Massage);
    }
    else 
    {
      lcd.clear();
    }
  }
}



#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>


#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>




void setup() {
  analogReference (EXTERNAL);
  Serial.begin(9600);


  delay(900);  

 
}
int count = 0;
void loop()
{
  while (1){
    int adc = analogRead(A0);
    if (adc >=100){ Serial.println((String)adc);count = 0;}
    //else{count++;
    //if(count%960==0)Serial.println((String)adc);}
   
    }
}


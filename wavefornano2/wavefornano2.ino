

#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>


#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>




void setup() {
  analogReference (EXTERNAL);
  Serial.begin(9600);




  Serial.println(F("##########################################################################################"));
  Serial.println(F("### CosmicWatch: The Desktop Muon Detector"));
  Serial.println(F("### Questions? saxani@mit.edu"));
  Serial.println(F("### Comp_date Comp_time Event Ardn_time[ms] ADC[0-1023] SiPM[mV] Deadtime[ms] Temp[C] Name"));
  Serial.println(F("##########################################################################################"));


  delay(900);  
  //Timer1.initialize(TIMER_INTERVAL);             // Initialise timer 1
 // Timer1.attachInterrupt(timerIsr);              // attach the ISR routine
 
}
int count = 0;
void loop()
{
  while (1){
    int adc = analogRead(A0);
    if (adc >=100){ Serial.println((String)adc+" "+400);count = 0;}
    else{count++;
    if(count%960==0)Serial.println((String)adc+" "+400);}
   
    }
}



#include <Wire.h>
#include "LowPower.h"

#include <RCSwitch.h>
RCSwitch mySwitch = RCSwitch();
#include <Wire.h>
//-----------SSD1306 low memory------------
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

#define I2C_ADDRESS 0x3C
// Define proper RST_PIN if required.
#define RST_PIN -1
SSD1306AsciiWire oled;
//---------------------------------------------------
#include <BMx280I2C.h>
#define I2C_ADDRESS 0x76
BMx280I2C bmx280(I2C_ADDRESS);
//----------------------------------------------------
#include <Adafruit_LPS35HW.h>
Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();
#define ALTITUDE 1655.0 
//---------------------------------------------------
#include "DHT20.h"
DHT20 DHT;
//----------------Pins Allocatin-------------------
const int ledPin    = 7;
const int RF_GND    = 6;
const int buttonPin = 5;
const int trigPin   = 8;
const int echoPin   = 9;
const int RF_Tx     = 4;
const int RF_Rx     = -1;          
const int BOOST     = 10;

const int LCD_GND     = 17;
const int PIR_sense     = 3; 
const int Door_sense     = 2;
const int Moist_sense     = A7;
const int Lux_sense     = A6;
const int Vin_sense     = A2;
const int Vdd_sense     = A1;
const int Vbatt_sense     = A0;
//---------------------------------------------------
float temperature_int, pressure_int, altitude_int;            // Create the temperature, pressure and altitude variables
float abs_pressure=0,start_pressure=0;
char first_time=0;
double pressure_level=0;
int known_pressure=939;
int measured_pressure=0;
int water_level=0;
char PIRState=0;
char counter_test=0;
float Vin_sense_float=0;
float Vdd_sense_float=0;
float Vbatt_sense_float=0; 

unsigned long data_out=0;
unsigned char Vin_sense_value=0;
unsigned char Vdd_sense_value=0;
unsigned char Vbatt_sense_value=0; 

unsigned char temp=0,hum=0,wlc=0;
//-------------------------------------------------
void setup() 
{  
  
  Serial.begin(9550);           // considering error!
  Serial.println("ASCII");
  
  pinMode(ledPin, OUTPUT);
  pinMode(BOOST, OUTPUT);
  pinMode(RF_Tx, OUTPUT);
  pinMode(LCD_GND, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); 

  digitalWrite(ledPin, HIGH);
  
    // Enable PCIE2 Bit3 = 1 (Port D)
  //PCICR |= B00000100;
  // Select PCINT21 Bit5 = 1 (Pin D5)
  //PCMSK2 |= B00100000;
  
  pinMode(PIR_sense, INPUT);  
  delay(100);  

  digitalWrite(BOOST, HIGH);
  delay(5);
  digitalWrite(BOOST, LOW);
  digitalWrite(LCD_GND, LOW);
     
  analogReference(INTERNAL); 
    
  //digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);
  delay(500);
  digitalWrite(ledPin, HIGH);
  delay(300);
  digitalWrite(ledPin, LOW); 

  mySwitch.enableTransmit(4);
  Wire.begin();

  //unsigned int startup_RF=7445;         //98 & 98 on SEM4.0 to ensure RF started      
  //mySwitch.send(startup_RF, 24);
  //data_send();
  //lps35hw.begin_I2C();   
///*
  //oled.begin(&Adafruit128x64, I2C_ADDRESS);
  //oled.setFont(Adafruit5x7);
/*
  uint32_t m = micros();
  oled.clear();
  oled.println("WLC Sensor Node");
  oled.println(" ");
  oled.println();
  oled.set2X();
  oled.println("WLC Node");
  oled.set1X();
  oled.print("\nmicros: ");
  oled.print(micros() - m);
*/
  //DHT.begin(); 
    if (!bmx280.begin())
    {
         Serial.println("begin() failed");
    }
  bmx280.resetToDefaults(); 
  bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
  bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);
/*  
  if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID));
 
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     ///* Operating Mode. 
                  Adafruit_BMP280::SAMPLING_X2,     ///* Temp. oversampling 
                  Adafruit_BMP280::SAMPLING_X16,    ///* Pressure oversampling 
                  Adafruit_BMP280::FILTER_X16,      ///* Filtering. 
                  Adafruit_BMP280::STANDBY_MS_500); ///* Standby time. 
                 // */
   //attachInterrupt(1, wakeUp, HIGH);
}
//*************************************************************
void WLC()
{   
   digitalWrite(ledPin, HIGH);
   digitalWrite(BOOST, HIGH);  

   measured_pressure = lps35hw.readPressure();
   water_level = measured_pressure - known_pressure;
   Serial.print("WL: ");
   Serial.print(water_level);
   Serial.println("cm");
   
   digitalWrite(ledPin, LOW);
   delay(2);  
}
//*************************************************************
void test()
{
    mySwitch.send(Vbatt_sense_value, 24);
    delay(2);        
}
//------------------------------------------------------------
void wakeUp()
{
    detachInterrupt(1);
    digitalWrite(ledPin, HIGH);
    PIRState = digitalRead(PIR_sense); 
    mySwitch.send(9505, 31);
    delay(2);    
}
//------------------------------------------------------------
ISR(PCINT2_vect)
{
   //data_send();
   //detachInterrupt(0); 
  // data_send();
   PIRState = digitalRead(PIR_sense);     
}
//*/
//*****************************************************
void loop()
{    
  data_send();
  sleepSeconds(24);  
}
//*****************************************************
void data_send() 
{        
    data_out=0;
    digitalWrite(ledPin, HIGH);
    digitalWrite(BOOST, HIGH);
    delay(7);

    if (!bmx280.measure())
    {
        Serial.println("could not start?");
        return;
    }  
    do
    {
        ADC_data(Vin_sense); 
        ADC_data(Vbatt_sense);
        delay(2);
    } 
    while (!bmx280.hasValue());  

    float temp_float=bmx280.getTemperature();
    temp=(temp_float*10)/2;
    float wlc_int=(bmx280.getPressure()/100);    
    //wlc=(wlc_int-938);
    wlc++;
    
    data_out=(long)temp<<24; 
    data_out +=(long)wlc<<16;
    data_out +=(long)Vin_sense_value<<8;
    data_out +=(long)Vbatt_sense_value;           // Frame: Temp(0-255)|WLC(0-255|VIN(0-255)|VBATT(0-255)    

    Serial.println("--DBG--");
    Serial.println(Vin_sense_float);
    Serial.println(Vin_sense_value);
    Serial.println(Vbatt_sense_value);
    Serial.println(temp_float);
    Serial.println(temp);
    Serial.println(wlc_int);
    Serial.println(wlc);
    Serial.println(data_out);

    mySwitch.send(data_out, 32);
    delay(3);

    if(Vbatt_sense_float<3.4)
    {
      digitalWrite(BOOST, LOW);      
    }      
    
    if(Vbatt_sense_float<=3.6)
    {
      digitalWrite(BOOST, HIGH);      
    } 
    else digitalWrite(BOOST, LOW);    
    digitalWrite(ledPin, LOW);

   //delay(2000);
      
}
//****************************************************
unsigned int ADC_data(char channel)
{
    int ADC_hold;
    unsigned char i=0;
    unsigned int accumulator=0;
    float result=0;
    result=0,ADC_hold=0;

    while(i<=4)
    {
      ADC_hold= analogRead(channel);
      accumulator+=ADC_hold;
      i++;
      delayMicroseconds(500);
    }
    ADC_hold=accumulator/5;
    
    result = ADC_hold;
    result = result * 1.072;         

    if (channel==Vin_sense)
    {
      Vin_sense_float=result* 0.011018;
      Vin_sense_value=(Vin_sense_float*100)/2;      
    }
    else if (channel==Vbatt_sense)
    {
      Vbatt_sense_float =result*0.00435;
      Vbatt_sense_value=(Vbatt_sense_float*100)/2;
    }
    else if (channel==Vdd_sense)
    {
      Vdd_sense_float=result*0.0110775;
      Vdd_sense_value=(Vdd_sense_float*100)/2;
    }
     
}
//****************************************************
void sleepSeconds(int seconds)
{  
  for (int i = 0; i < (seconds/8); i++) 
  {          
     LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);      
  }
}
//***************************************************

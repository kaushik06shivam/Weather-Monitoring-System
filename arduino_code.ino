//BMP280 SENSOR
//vcc - 3.3v
//SCL - A5 (or)SCL pin
//SDA - A4 (or)SDA pin
//DHT11 SENSOR
//vcc - 3.3v
//DATA - Pin2 
//Rain Sensor - A1
 
#include <Adafruit_BMP280.h>
#include "DHT.h"
 
Adafruit_BMP280 bmp; // I2C Interface

#define DHTPIN 2     // DHT11 pin 
#define DHTTYPE DHT11   // DHT 11 sensor type

DHT dht(DHTPIN, DHTTYPE);

const int RainSensor = A1;  // Analog input pin for Rain sensor
const int Buzzer = 13; 
const int LED = 12; 

float RainVoltage = 0.0;  // variable to store water level sensor 
float rainlevel = 0.0;

void setup(void)
{
  pinMode(Buzzer, OUTPUT); // initialize pin 8 as an output.
  pinMode(LED, OUTPUT); 
  Serial.begin(9600);       // Start the Arduino hardware serial port at 9600 baud
  /* Serial.println(F("Waiting for serial data from user/bluetooth"));
  while (Serial.available() == 0) 
  {
    // Wait for User to Input Data
    // Enter any data in serial to proceed below
  }
  int data = Serial.parseInt();
  Serial.println(data);*/
  delay(5000);
  digitalWrite(LED, HIGH);
  
  // Initialize DHT sensor    
////  Serial.println(F("DHT11 intializing.."));
  dht.begin();
  delay(2000);
  
  // Initialize BMP280 sensor 
//  Serial.println(F("BMP280 intializing.."));
//  if (!bmp.begin(0x76)) 
//  {
//    Serial.println(F("Failed to Read BMP280 Sensor!"));
 // }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

 delay(1000);
 digitalWrite(LED, LOW);
 delay(1000);
}
 
void loop() 
{   
 // int i=0;
 // for (i=0; 1<=5000; 1++)
 // {
      readBMP280();
      delay(10);
      readDHT11();
      delay(10);
      readRainSensor();
      delay(10);
      readAirQuality();
}
 
///////////////////////////////////////////////////////////////////
    //Check for BMP280 Data
    void readBMP280()
    {      
    Serial.print("Temp:");
    Serial.print(bmp.readTemperature()); 
    Serial.print(" ");
    Serial.print("Press:");
    Serial.print(bmp.readPressure()/100); 
    Serial.print(" ");
    Serial.print("Alt:");
    Serial.print(bmp.readAltitude(1011.00)); 
    Serial.print(" ");
   // Serial.println();
    }

/////////////////////////////////////////////////////////////////////
    //Check for DHT11 Data
    void readDHT11()
    {
    float h = dht.readHumidity();
    Serial.print("Graph:");
    Serial.print(h);  
    Serial.print(" ");   
    }
    
 //////////////////////////////////////////////////
  //Read Rain Drop Sensor
  void readRainSensor()
  {
   int val1 = analogRead(RainSensor); //Read value from Water Level Sensor
   delay(10);
   RainVoltage = (val1 * 5.0) / 1024.0; 
   rainlevel = ((5-RainVoltage) * 10);
   Serial.print("Rain:");
   Serial.print(rainlevel);  
   Serial.print(" ");  
   }

  /////////////////////////////////////////////////
  //Air Quality Sensor
  void readAirQuality()
  {
   float AQvoltage = 0.0;
   int analogvalue = analogRead(A2); //Read Sensor Voltage
   AQvoltage = (analogvalue * 5.0) / 1024.0; 
   Serial.print("AirQ:");
   Serial.println(AQvoltage);
  }
   
  //Serial Data format
  //Graph:160.59&74.17&234.76
  //Graph Tag: Graph:
  //Chart Type: Line Graph
  //Value Seperator : &
  //Termination Character : $
 

  

/*
 Test of Pmod TC1

*************************************************************************

  Description: Pmod_TC1
  The ambient temperature (in and ° C) is displayed in the serial monitor.


  Material
  1. Arduino Uno
  2. Pmod TC1 (download library
  https://github.com/adafruit/Adafruit-MAX31855-library)
  

  Wiring
  Module <----------> Arduino
  VCC     to          3V3
  GND     to          GND
  SCK     to          13
  MISO    to          12
  CS      to          10

  a logic level converter must be used!!!

  SSR     to          3
  
  

************************************************************************/

#define CS 10 //chip select pin
#define RelayPin 3 //SSR pin

// Call of libraries
#include <SPI.h>
#include <PID_v1.h>
#include <Adafruit_MAX31855.h>

Adafruit_MAX31855 thermocouple(CS); // Creation of the object

struct Point {
  unsigned long time_ms;
  double temp;
} P1, P2, P3, P4, P5, P6, P7;

double Setpoint, Input, Output;
int WindowSize = 5000;
unsigned long windowStartTime, windowTime, now, startTime;
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);


void setup()
{
  P1.temp = 30;
  P1.time_ms = 0;
  P2.temp = 100;
  P2.time_ms = 30000;
  P3.temp = 150;
  P3.time_ms = 120000;
  P4.temp = 183;
  P4.time_ms = 150000;
  P5.temp = 235;
  P5.time_ms = 210000;
  P6.temp = 183;
  P6.time_ms = 240000;
  P7.temp = 30;
  P7.time_ms = 250000;

  Setpoint = P1.temp;
  
  pinMode(RelayPin, OUTPUT);
  windowStartTime = millis();

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);


  Serial.begin(9600); // initialization of serial communication
  thermocouple.begin();  //initialize senzor
  startTime = millis();
  pinMode(RelayPin, OUTPUT);
}

unsigned int precision = 100;

double x = 183156;
void loop()
{
  now = millis();
  Serial.println(Setpoint);
  Input = thermocouple.readCelsius();
  
  windowTime = windowStartTime + Output;
  if(updateSetpoint(startTime, &Setpoint)) {
    myPID.Compute();
    windowTime = windowStartTime + Output;
  }
  else
  {
    Output = 0;
  }
  if (windowTime >= WindowSize)
  {
    windowStartTime = millis();
  }
  if (now <= windowStartTime + windowTime)
  {
    digitalWrite(RelayPin, HIGH);
  }
  else
  {
    digitalWrite(RelayPin, LOW);
  }
}

bool updateSetpoint(unsigned long startTime, double *Setpoint)
{
  if (now < startTime + P2.time_ms)
  {
    *Setpoint = (P2.temp - P1.temp) / (P2.time_ms - P1.time_ms) * (now - (startTime + P1.time_ms)) + P1.temp;
  }
  else if (now < startTime + P3.time_ms)
  {
    *Setpoint = (P3.temp - P2.temp) / (P3.time_ms - P2.time_ms) * (now - (startTime + P2.time_ms)) + P2.temp;
  }
  else if (now < startTime + P4.time_ms)
  {
    *Setpoint = (P4.temp - P3.temp) / (P4.time_ms - P3.time_ms) * (now - (startTime + P3.time_ms)) + P3.temp;
  }
  else if (now < startTime + P5.time_ms)
  {
    *Setpoint = (P5.temp - P4.temp) / (P5.time_ms - P4.time_ms) * (now - (startTime + P4.time_ms)) + P4.temp;
  }
  else if (now < startTime + P6.time_ms)
  {
    *Setpoint = (P6.temp - P5.temp) / (P6.time_ms - P5.time_ms) * (now - (startTime + P5.time_ms)) + P5.temp;
  }
  else if (now < startTime + P7.time_ms)
  {
    *Setpoint = (P7.temp - P6.temp) / (P7.time_ms - P6.time_ms) * (now - (startTime + P6.time_ms)) + P6.temp;
  }
  else
  {
    *Setpoint = P7.temp;
    return false;
  }
  return true;
}
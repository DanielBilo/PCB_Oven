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
#define numChars 20 //number of characters to read

// Call of libraries
#include <SPI.h>
#include <PID_v1.h>
#include <Adafruit_MAX31855.h>

Adafruit_MAX31855 thermocouple(CS); // Creation of the object

int i = 0;
struct Point {
  unsigned long time_ms;
  double temp;
} P1, P2, P3, P4, P5, P6, P7, P8, P9;

bool newData = false;
bool running = false;
char serialReceived[20];
double Setpoint, Input, Output;
int WindowSize = 5000;
unsigned long windowStartTime, windowTime, now, startTime;
PID myPID(&Input, &Output, &Setpoint, 80, 5, 10, DIRECT);

void recvWithEndMarker(bool *flag, char *receivedChars) {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    while (Serial.available() > 0 && *flag == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            *flag = true;
        }
    }
}


void setup()
{
  P1.temp = 30;
  P1.time_ms = 0;

  P2.temp = 100;
  P2.time_ms = 90000;

  P3.temp = 100;
  P3.time_ms = 130000;

  P4.temp = 150;
  P4.time_ms = 220000;

  P5.temp = 183;
  P5.time_ms = 280000;

  P6.temp = 235;
  P7.time_ms = 340000;

  P7.temp = 235;
  P7.time_ms = 310000;

  P8.temp = 183;
  P8.time_ms = 340000;
  
  P9.temp = 30;
  P9.time_ms = 370000;




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

void loop()
{
  recvWithEndMarker(&newData, serialReceived);

  if (newData) {
    newData = false;
    if (serialReceived[0] == 's') {
      Serial.println("Detected Start");
      running = true;
      startTime = millis();
      windowStartTime = millis();
    }
    else if (serialReceived[0] == 'e') {
      Serial.println("Detected End");
      running = false;
      Setpoint = P1.temp;
    }
  }
  if(running)
  {
    now = millis();
    if(i >= 4000)
    {
      i = 0;
      Serial.println(Input);
      Serial.println(Setpoint);
      Serial.println(Output);
    }
    i++;
    Input = thermocouple.readCelsius();

    if(updateSetpoint(startTime, &Setpoint)) {
      myPID.Compute();
      windowTime = windowStartTime + Output;
    }
    else
    {
      running = false;
      Output = 0;
    }
    if (now >= windowStartTime + WindowSize)
    {
      windowStartTime = millis();
    }
    if ((now > windowStartTime) && (now < windowTime))
    {
      digitalWrite(RelayPin, HIGH);
    }
    else
    {
      digitalWrite(RelayPin, LOW);
    }
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
  else if (now < startTime + P8.time_ms)
  {
    *Setpoint = (P8.temp - P7.temp) / (P8.time_ms - P7.time_ms) * (now - (startTime + P7.time_ms)) + P7.temp;
  }
  else if (now < startTime + P9.time_ms)
  {
    *Setpoint = (P9.temp - P8.temp) / (P9.time_ms - P8.time_ms) * (now - (startTime + P8.time_ms)) + P8.temp;
  }
  else
  {
    *Setpoint = P9.temp;
    return false;
  }
  return true;
}
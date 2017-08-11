#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BLE_UART.h>
#include "Adafruit_MAX31855.h"
#include "Arduino.h"
#include <PID_v1.h>
#include <Vcc.h>

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2
#define ADAFRUITBLE_RST 9
#define CS1 4
#define DO1 7
#define CLK1 8
#define CS2 A1
#define DO2 A4
#define CLK2 A2

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
Adafruit_MAX31855 thermocouple1(CLK1, CS1, DO1);
Adafruit_MAX31855 thermocouple2(CLK2, CS2, DO2);

boolean fanon = LOW;

double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
String T="";                                                                              

String inString="";
float coretemp=36.5;

PID myPID1(&Input1, &Output1, &Setpoint1, 55.0, 4.5, 10, REVERSE);
PID myPID2(&Input2, &Output2, &Setpoint2, 55.0, 4.5, 10, REVERSE);

int peltier1 = 6;
int peltier2 = 5;
int fanpin = A5;

void setup()
{
    Serial.begin(9600);
    while(!Serial);
    pinMode(peltier1, OUTPUT);
    pinMode(peltier2, OUTPUT);
    pinMode(fanpin, OUTPUT);
    Setpoint1 = 16;
    Setpoint2 = 16;
    myPID1.SetMode(AUTOMATIC);
    myPID1.SetOutputLimits(0,255);
    myPID2.SetMode(AUTOMATIC);
    myPID2.SetOutputLimits(0,255);
    BTLEserial.begin();
}
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;
void loop()
{
   BTLEserial.pollACI();
   aci_evt_opcode_t status = BTLEserial.getState(); // Ask what is our current status
   if (status != laststatus) {
      if (status == ACI_EVT_DEVICE_STARTED) {Serial.println(F("* Advertising started"));}
      if (status == ACI_EVT_CONNECTED) {Serial.println(F("* Connected!"));}
      if (status == ACI_EVT_DISCONNECTED) {Serial.println(F("* Disconnected or advertising timed out"));}
      laststatus = status;
   }
  if (status == ACI_EVT_CONNECTED) {
    Input1 = double(thermocouple1.readCelsius()); //Temperature reading of 1st thermocouple  
    Input2 = double(thermocouple2.readCelsius()); //Temperature reading of 2nd thermocouple
    if (Input1>-1){
        myPID1.Compute();
        analogWrite(peltier1, Output1); //Output1
        Serial.print(F("T1= "));Serial.println(Input1);//Serial.print(F("Output: "));Serial.println(Output1);
     }
    else {}
    if (Input2>-1){
        myPID2.Compute();
        analogWrite(peltier2, Output2); //Output 2
        Serial.print(F(" T2= "));Serial.println(Input2);//Serial.print(F("Output: "));Serial.println(Output2);
     }
    else {}
    while (BTLEserial.available()) {
      int c = BTLEserial.read();
      if(isDigit(c)||isPunct(c)) inString+=char(c);
      }

      if (inString.toFloat()!=0.00){
          coretemp = inString.toFloat();
          T = String(Input1)+" " +String(Input2);
          
          if(T.length()>1){
            uint8_t sendbuffer1[12];
            T.getBytes(sendbuffer1, 12);
            char sendbuffersize1 = min(12, T.length());
            BTLEserial.write(sendbuffer1, sendbuffersize1);
          }
      }
      Serial.print(F("Coretemp: ")); Serial.println(coretemp);
      inString = " ";
      T = "";
       if (coretemp>39.0){
          Setpoint1 = 16;
          Setpoint2 = 16;
          fanon=HIGH;
        }
      else if (coretemp>37.5){
          Setpoint1 = 20;
          Setpoint2 = 20;
          fanon=HIGH;
        }
      else if (coretemp>36.5){
          Setpoint1 = 24;
          Setpoint2 = 24;
          fanon=HIGH;
        }                  
        else if(coretemp<36.5){
          analogWrite(peltier1, 0);
          analogWrite(peltier2, 0);
          fanon=LOW;
        }
        digitalWrite(fanpin, fanon);
  }
    else if (status == ACI_EVT_DISCONNECTED || status == ACI_EVT_DEVICE_STARTED){
          analogWrite(peltier1, 0);
          analogWrite(peltier2, 0);
          digitalWrite(fanpin, LOW);
  }    
}


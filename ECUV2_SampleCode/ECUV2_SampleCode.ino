/**********************************************************************/
// (c) by WWW.SMART-ELECTRO.NET
// 
// This source code is subject to the CreativeCommons Attribution 4.0 International (CC BY 4.0) license
//
// You can copy and redistribute the material in any medium or format.
// Remix, transform, and build upon the material for any purpose, even commercially as long as you
// don't forget to mentioned www.smart-electro.net in your work.
// More about license can be found at: http://http://creativecommons.org/licenses/by/4.0/deed.en
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
// 
// CAN bus library is based in SeeedStudio CANBUS SHIELD
//
// Version: V1.0
// Author: Simon Grabar <simon.grabar@gmail.com>
// Created: Aug 2015
//
/****************************************
 FUNCTIONALITY
/****************************************

1. after key (DI_X1_E1) is in first position we go into STARTUP_MODE
2. after key is in second position (DI_X1_F1) and no gas pedal is pressed (DI_X1_F2) we go into RUNNING_MODE
3. when there are CAN messages from charger we go into CHARGING_MODE
4. in RUNNING_MODE or CHARGING_MODE we turn on BMS and control it (Integrate current, max/min voltage check)
5. BT is always UP and running for displaying the values on MyEV app


CAN SPEC - 250kB/s, 11bit identifyer



*/

#include <SPI.h>
#include "mcp_can.h"
#include <SimpleTimer.h>
#include <Bounce2.h>
#include <PID_v1.h>

//  SETTINGS  //
#define RatedBpCapacity 9000 //Wh
#define RatedBatVoltage 1300 //0.1V
#define NO_OF_BMS_MASTERS 5 //one index
#define CUTOFF_CELL_PWM 85 //cca 600ma
#define MAX_CELL_PWM 85 //cca 600ma
#define SETOPOINT_CELL_PWM 50 //cca 500ma
#define MAX_CHARGER_CURRENT 280 //20A (x0.1)
#define BALANCE_CHARGER_CURRENT 3 //0.3A (x0.1)
#define CHARGER_VOLTAGE 1236 //(x0.1)
#define BATTERY_CUT_OFF_VOLTAGE 3.00 //
#define BATTERY_LOW_VOLTAGE 3.3 //

//gauge settings
#define SOC_GAUGE_0 16
#define SOC_GAUGE_50 38
#define SOC_GAUGE_100 75
#define TEMP_GAUGE_0 16
#define TEMP_GAUGE_50 30
#define TEMP_GAUGE_100 63
#define TEMP_GAUGE_MAXTEMP 60
// END SETTINGS //






//Konector port definitions 
#define DI_X1_F1 35   //key pos 2  
#define DI_X1_E1 41   //Key pos 1
#define DI_X1_F2 39   //gas switch  
#define DI_X1_D3 A2   //fw
#define DI_X1_D2 A1   //reverse
#define DI_X1_D1 A0

#define DO_X1_J1 6    //temp gauge
#define DO_X1_K3 7    //soc gauge
#define DO_X1_K2 8    //BMS power supply
#define DO_X1_K1 9    //FW/RW  

#define DO_X1_H2 10   //Interlock
#define DO_X1_G1 11   //ign lamp
#define DO_X1_J3 12   // power gauge - tachometer
#define DO_X1_J2 13   //orange lamp

#define DI_X1_E3 A3    //current limiting pot

//sequence aliases
#define SLEEP_MODE 0
#define STARTUP_MODE 1
#define RUNNING_MODE 2
#define CHARGING_MODE 3
#define FULL_MODE 4

#define BMS_NORMAL 0
#define BMS_ONEFULL 1
#define BMS_ALLFULL 2
#define BMS_EMPTY 3
#define BMS_CUTOFF 4
#define BMS_OVERVOLTAGE 5
#define BMS_ERROR 6

// Instantiate a Bounce objects
Bounce KeyPos1 = Bounce(); 
Bounce KeyPos2 = Bounce(); 
Bounce PowPedalSw = Bounce(); 
Bounce FwSwitch = Bounce(); 
Bounce RwSwitch = Bounce(); 


#define set_bit(ADDRESS,BIT) (ADDRESS |= (1L<<BIT))
#define clear_bit(ADDRESS,BIT) (ADDRESS &= ~(1L<<BIT))
#define toggle_bit(ADDRESS,BIT) (ADDRESS ^= (1L<<BIT))
#define test_bit(ADDRESS,BIT) (ADDRESS & (1L<<BIT))

//MyVehicle mobile phone app struct definition
struct AvailableData {
  byte SOC;     //battery capacity in circle on "Main"
  float ttc;     //time to charge "Main"
  byte rd;      //remaining distance "Main"
  float amp;     //current on "MAIN"
  float vol;     //voltega on "Battery" 
  byte camp;    //temperature on "Battery"
  float temp;    //current on "battery"    
  float maxV;    //chart point "Main"
  float minV;    //chart point "Main"
  float avgV;    //chart point "Main"
  byte mode;    //header 3-charging, 2-driving, 1-off,

  float pw[10];     //chart point "Main"
  /*byte pw2;     //chart point "Main"
  byte pw3;     //chart point "Main"
  byte pw4;     //chart point "Main"
  byte pw5;     //chart point "Main"
  byte pw6;     //chart point "Main"
  byte pw7;     //chart point "Main"
  byte pw8;     //chart point "Main"
  byte pw9;     //chart point "Main"
  byte pw10;    //chart point "Main"
  byte pw11;    //chart point "Main"
*/  
  unsigned int CellVoltage[30]; //voltages of invididual cells from BMS
  unsigned char CellBalance[30]; //voltages of invididual cells from BMS

};
AvailableData MyVehicle;
  
  
//PID settings for charger controll
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,0.5,1,10, DIRECT); //PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction) 

//CAN variables
unsigned long CAN_ID;
unsigned char len = 0;
unsigned char buf[8], can_tx[8];

//Battery related
float BpCapacity=RatedBpCapacity*1000; //mWh
unsigned short CurrentFromCharger,MotorCurrent,ACcurrentFromCharger;
unsigned short VoltageFromCharger, MaxTemperature[NO_OF_BMS_MASTERS]={0}, MaxTemp=0;
unsigned char chargerStatus,BmsStatus[NO_OF_BMS_MASTERS],BmsStatus0[NO_OF_BMS_MASTERS*6],Bms_MasterStatus,bmsOverCurrentTimer;
float BmsVoltage[NO_OF_BMS_MASTERS*6]={0},CurrentToCharger=0;
byte MaxPwm=0;
byte MaxPwmId=0;


//inverter related 
float MechanicalPower = 0;
unsigned int VoltageFromInverter,InverterTemperature,MotorTemperature, canTxTimer1, canTxTimer2,canTxTimer3 ;
int CurrentFromInverter;



//power gauge (tachometer)
#define TACHO_PULSES_PER_ROTATION 4  //6000rpm /60s *2clinders 
unsigned long currentTime = micros();
unsigned long previousTime = micros();
char Toggle=0;
unsigned int tachoStartup=0;
unsigned char tachoStartupDir=0;
unsigned char GaugeStatrupDelay=0; //timer for turning on the gauges at begining

//other
unsigned char State=SLEEP_MODE;
unsigned char ChargerRxTimeout=255,InverterRxTimeout,BmsRxTimeout[NO_OF_BMS_MASTERS], send2serial,OrangeLampBlink;
char BMS_Master_Id,toggleLed=0;

unsigned short CurrentLimitPot=0; //0-100%
char FilterIndex=0;
char FwRwTimerDelay=0;


//timer
SimpleTimer timer;




/****************************************/

void setup() {
  //debug port
  Serial.begin(19200);

  //Bluetooth port
  Serial2.begin(115200);

  //PID setpiont
  Setpoint = SETOPOINT_CELL_PWM;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);   
  myPID.SetOutputLimits(0, MAX_CHARGER_CURRENT/2);
  
  //Pin init
  //DI
  pinMode(DI_X1_F1, INPUT);  
  pinMode(DI_X1_E1, INPUT);   
  pinMode(DI_X1_F2, INPUT);    
  pinMode(DI_X1_D3, INPUT);  
  pinMode(DI_X1_D2, INPUT);  
  pinMode(DI_X1_D1, INPUT); 
   
  //DO
  pinMode(DO_X1_J1, OUTPUT);  
  pinMode(DO_X1_K3, OUTPUT);
  pinMode(DO_X1_K2, OUTPUT);
  pinMode(DO_X1_K1, OUTPUT);
  pinMode(DO_X1_H2, OUTPUT);  
  pinMode(DO_X1_G1, OUTPUT);
  pinMode(DO_X1_J3, OUTPUT);
  pinMode(DO_X1_J2, OUTPUT);
  
  //analog inputs 
  analogReadResolution(10); 
  pinMode(DI_X1_E3, INPUT);  


  // After setting up the button, setup the Bounce instance :
  KeyPos1.attach(DI_X1_E1);
  KeyPos1.interval(50); // interval in ms
  KeyPos2.attach(DI_X1_F1);
  KeyPos2.interval(50); // interval in ms
  PowPedalSw.attach(DI_X1_F2);
  PowPedalSw.interval(50); // interval in ms
  FwSwitch.attach(DI_X1_D3);
  FwSwitch.interval(50); // interval in ms
  RwSwitch.attach(DI_X1_D2);
  RwSwitch.interval(50); // interval in ms












  
  //timer
  timer.setInterval(100, repeatMe);
  

  START_INIT:
  if(CAN_OK == CAN.begin(CAN_250KBPS))                   // init can bus : baudrate = 500k
  {
    Serial.println("CAN BUS Shield init ok!");
  }
  else
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println("Init CAN BUS Shield again");
    delay(2000);
    goto START_INIT;
  }

  //charger in timeout
  ChargerRxTimeout=255;

  //SOC is unknown so we set it to 20% of nominal
  BpCapacity*=0.75;


  //default BT values
  MyVehicle.ttc=30;  //ok
  MyVehicle.SOC=50; //ok
  MyVehicle.rd=00;  //
  MyVehicle.amp=99;  //ok
  MyVehicle.vol=100; //ok
  MyVehicle.temp=26;  // ok
  MyVehicle.maxV=2.22; // ok
  MyVehicle.minV=1.11;  // ok
  MyVehicle.avgV=1.23;  // ok
  MyVehicle.mode=1;    //
  MyVehicle.camp=25;  //ok

  //not asigned yet. TODO
  /*MyVehicle.pw1=10;
  MyVehicle.pw2=10;
  MyVehicle.pw3=10;
  MyVehicle.pw4=15;
  MyVehicle.pw5=15;
  MyVehicle.pw6=15;
  MyVehicle.pw7=15;
  MyVehicle.pw8=18;
  MyVehicle.pw9=18;
  MyVehicle.pw10=18;
  MyVehicle.pw11=1;*/
  
   Serial.print("****************   RESET  *****************");
}


/*********************************************************/
// a function to be executed periodically every 100ms
/*********************************************************/
void repeatMe(){    
   
    
    //we only increase counters to certain number otherwise they owerflow
    if (ChargerRxTimeout < 255) ChargerRxTimeout++;
    if (InverterRxTimeout<255) InverterRxTimeout++;
    if (BmsRxTimeout[0]<255) BmsRxTimeout[0]++;
    if (BmsRxTimeout[1]<255) BmsRxTimeout[1]++;
    if (BmsRxTimeout[2]<255) BmsRxTimeout[2]++;
    if (BmsRxTimeout[3]<255) BmsRxTimeout[3]++;
    if (BmsRxTimeout[4]<255) BmsRxTimeout[4]++;
    if (OrangeLampBlink<255)OrangeLampBlink++;
    if (GaugeStatrupDelay<255) GaugeStatrupDelay++;
    if (FwRwTimerDelay<250) FwRwTimerDelay++;
    if (bmsOverCurrentTimer<250) bmsOverCurrentTimer++;

    
    //triggers for CAN
    canTxTimer1=0;
    canTxTimer2=0;
    canTxTimer3=0;
 
    //current integration
    StateOfCHarge();
        
    //send to BT or serial every 1s
    send2serial++;
    if (send2serial>=25){
      send2serial=0; 
    }
    
}



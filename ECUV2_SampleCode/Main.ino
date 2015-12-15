void loop() {



  timer.run();
  // Update the Bounce instance :
  KeyPos1.update();
  KeyPos2.update();
  PowPedalSw.update();
  FwSwitch.update();
  RwSwitch.update();

  
  //check CAN messages
   /***********************************************/
    // Pool for new CAN message
    /***********************************************/
    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
      CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
      CAN_ID=CAN.getCanId();
     // Serial.println(CAN_ID);
      if(CAN_ID){
        switch (CAN_ID) { 
            //Status message from from Eltek charger 
             case 0x308: 
                
              //Serial.println("received charger message");
                
                ChargerRxTimeout=0;
                
            break; 
                
            //Status message from from Eltek charger 
            case 0x305: //charger status msg
                
              // Serial.println("eltek status1");
                
                ChargerRxTimeout=0;
              
                CurrentFromCharger= (int)((buf[4] << 8) + buf[3]); 
                ACcurrentFromCharger= (int)((buf[2] << 8) + buf[1]); 
                
                VoltageFromCharger= (int)((buf[6] << 8) + buf[5]);
                chargerStatus=buf[0];
                
            break;       
                
             //status message from BMS
            case 0x18ff0010:
            case 0x18ff0110:
            case 0x18ff0210:
            case 0x18ff0310:
            case 0x18ff0410:
                
                BMS_Master_Id=((CAN_ID >> (8)) & 0xff);
                BmsRxTimeout[BMS_Master_Id]=0;
                processBmsStatus();


                break; 
                
             //voltage message from BMS
            case 0x18ff0011: 
            case 0x18ff0111:
            case 0x18ff0211:
            case 0x18ff0311:
            case 0x18ff0411: 

                processBmsVoltage();

                break;
            
             //balance message from BMS
            case 0x18ff0012: 
            case 0x18ff0112: 
            case 0x18ff0212: 
            case 0x18ff0312:  
            case 0x18ff0412:

                BMS_Master_Id=((CAN_ID >> (8)) & 0xff);
                processBmsBalance();

                break; 

            //status message from inverter
            case 0x259: 
                Serial.println("Inverter Status");
                InverterRxTimeout=0;
                VoltageFromInverter =(int)((buf[1] << 8) + buf[0]); //0.01V
                CurrentFromInverter =(int)((buf[3] << 8) + buf[2]);//0.0A
                break; 
            
             //temperature message from inverter
            case 0x458: 
                
                InverterRxTimeout=0;
                InverterTemperature =(int)((buf[0] << 8) + buf[1]);
                MotorTemperature =(int)((buf[2] << 8) + buf[3]);
                break;
            //diag    
            case 0x123: 
                Serial.println("Set PID tune");
                myPID.SetTunings((float)buf[0]/10,(float)buf[2]/10,(float)buf[2]/10);  //.SetTunings(consKp, consKi, consKd);
                break; 

      
        }  
    }  
    }
  
  //Main state machine
  
  switch (State){ 
    case SLEEP_MODE:
      SleepMode();
      
      //if key is in first position we go into startup mode
      //if (digitalRead(DI_X1_E1)==0){
      if (KeyPos1.read()==0){
        State = STARTUP_MODE;
        tachoStartupDir=0; //at begining we turn the tacho to 100%
        GaugeStatrupDelay=0;
         Serial.println("state change: SLEEP->STARTUP; Key 1 position");
      }
      
      //if we are getting CAN messages from charger go to CHARGING MODE
      if(ChargerRxTimeout < 30){
        State = CHARGING_MODE; 
        CurrentToCharger=0;
         Serial.println("state change: SLEEP->CHARGING; Charger active");
      }
      
      //if key is in 2 position and no gas padal is pressed we go into running mode
      //if (digitalRead(DI_X1_F1)==0 && digitalRead(DI_X1_F2)){
      if (KeyPos2.read()==0 && PowPedalSw.read()){
        State = RUNNING_MODE ;
        Serial.print("state change: STURTUP->RUNNING;");
        Serial.print(KeyPos2.read());
        Serial.println(PowPedalSw.read());
      }
      
      
    break;  
    case STARTUP_MODE:
      StartupMode();

      
      //if key is in 2 position and no gas padal is pressed we go into running mode
      //if (digitalRead(DI_X1_F1)==0 && digitalRead(DI_X1_F2)){
      if (KeyPos2.read()==0 && PowPedalSw.read()){
        State = RUNNING_MODE ;
        Serial.print("state change: STURTUP->RUNNING;");
        Serial.print(KeyPos2.read());
        Serial.println(PowPedalSw.read());
      }

      //if key is put back to 0 we go to SLEEP
      //if (digitalRead(DI_X1_F1)==1 && digitalRead(DI_X1_E1) ==1){
     if (KeyPos1.read()==1 && KeyPos2.read()==1){
      
        State = SLEEP_MODE ;
        Serial.print("state change: STURTUP->SLEEP;"); //01
        Serial.print(KeyPos1.read());//pos1
        Serial.println(KeyPos2.read());//pos2
      }
      //if we are getting CAN messages from charger go to CHARGING MODE
      if(ChargerRxTimeout < 30 ){
        State = CHARGING_MODE; 
        CurrentToCharger=0;
         Serial.println("state change: STARTUP->CHARGING; Charger active");
      }
      
      
    break;  
    case RUNNING_MODE:

      
      RunningMode();
      
      //if key is not in first or second position we go into sleep mode
      //if (digitalRead(DI_X1_E1)==1 && digitalRead(DI_X1_F1)==1){
      if (KeyPos1.read()==1 && KeyPos2.read()==1){
        State = SLEEP_MODE;
        Serial.print("state change: RUNNING->SLEEP;");
        Serial.print(KeyPos1.read());//pos1
        Serial.println(KeyPos2.read());//pos2
        

        //reset myEv chart
        for(byte i = 0; i<=9; i++){MyVehicle.pw[i]=0;}
      }
     //if we are getting CAN messages from charger go to CHARGING MODE
      if(ChargerRxTimeout < 30){
        State = CHARGING_MODE; 
        CurrentToCharger=0;
         Serial.println("state change: RUNNING->CHARGING; Charger active");
      }

      
    break;  
    case CHARGING_MODE:
        
        ChargingMode();

        //check if messages from charger are valid 
        if(ChargerRxTimeout > 30){ // || chargerStatus==3){ //chargerStatus 3 = disconnected from mains
            chargerStatus==0; 
            State = SLEEP_MODE;
            Serial.println("state change: CHARGING->SLEEP;");
            //reset myEv chart
            for(byte i = 0; i<=9; i++){MyVehicle.pw[i]=0;}
         }
         
          //if all cells are full we go into sleep mode
          if(Bms_MasterStatus==BMS_ALLFULL) State = SLEEP_MODE;
     
          //check if messages from charger are valid 
          if(ChargerRxTimeout > 30){ // || chargerStatus==3){ //chargerStatus 3 = disconnected from mains
              chargerStatus==0; 
              State = SLEEP_MODE;
              Serial.println("state change: BALLANCING->SLEEP;");
              //reset myEv chart
              for(byte i = 0; i<=9; i++){MyVehicle.pw[i]=0;}
           }
           if(Bms_MasterStatus==BMS_ALLFULL){
                State = FULL_MODE;
                Serial.println("state change: CHARGING->FULL;"); 
           }

           
         //if maxPwm becomes greater than 0 we go into BALANCING_MODE
        /* if (MaxPwm > 0 && MaxPwm < 165) {
            State=BALLANCING_MODE; 
            Serial.println("state change: CHARGING->BALLANCING;"); 
         }*/
         
      break;    
      case FULL_MODE:
             SleepMode();
             //if key is in first position we go into startup mode
              //if (digitalRead(DI_X1_E1)==0){
              if (KeyPos1.read()==0 && ChargerRxTimeout > 31){
                State = STARTUP_MODE;
                tachoStartupDir=0; //at begining we turn the tacho to 100%
                GaugeStatrupDelay=0;
                 Serial.println("state change: FULL->STARTUP; Key 1 position");
              }
              
              
              //if key is in 2 position and no gas padal is pressed we go into running mode
              //if (digitalRead(DI_X1_F1)==0 && digitalRead(DI_X1_F2)){
              if (KeyPos2.read()==0 && ChargerRxTimeout > 31){
                State = RUNNING_MODE ;
                Serial.print("state change: FULL->RUNNING;");
                Serial.print(KeyPos2.read());
                Serial.println(PowPedalSw.read());
              }  

          
          /*BallancingMode();
          
          //if all cells are full we go into sleep mode
          if(Bms_MasterStatus==BMS_ALLFULL) State = SLEEP_MODE;
     
          //check if messages from charger are valid 
          if(ChargerRxTimeout > 30 || Bms_MasterStatus==BMS_ALLFULL){ // || chargerStatus==3){ //chargerStatus 3 = disconnected from mains
              chargerStatus==0; 
              State = SLEEP_MODE;
              Serial.println("state change: BALLANCING->SLEEP;");
              //reset myEv chart
              for(byte i = 0; i<=9; i++){MyVehicle.pw[i]=0;}
           }
      */
      break;
    
    }


    
  
    //send to serial or BT
    if (send2serial==0) {
        send2serial++;

        //time 2 charge calculation in hours
        //missing kwh - charger power 

        
        sendToTerminal(); 
        sendToMyVehicleApp();
    }
  
}
/*********************************************************/
// at startup we turn the power gauge and light IGN and Orange lamp 
/*********************************************************/
void SleepMode(){
    digitalWrite(DO_X1_K2,0); //BMS OFF
    digitalWrite(DO_X1_H2,0); //driving interllock
    digitalWrite(DO_X1_K1,0); //fwRw relay in off 

    //temp and SOC gauge off
    digitalWrite(DO_X1_J1,0); //temp
    digitalWrite(DO_X1_K3,0); //soc
    
    //temp and SOC gauge 
    SocGaugeControl(0);
    TempGaugeControl(0); 
  
    //turn off dashboard lights 
    digitalWrite(DO_X1_G1,0);
    digitalWrite(DO_X1_J2,0);
    
    //MyEv app current display
    MyVehicle.amp=0;
    MyVehicle.temp=MyVehicle.amp;
    MyVehicle.mode=3; //1-charging, 2-driving, 1-off,

    for (int i=0;i<(NO_OF_BMS_MASTERS*6);i++) {
        BmsVoltage[i]=0;
    }

  
}
/*********************************************************/
// at startup we turn the power gauge and light IGN and Orange lamp 
/*********************************************************/
void StartupMode(){

  digitalWrite(DO_X1_K2,1); //BMS on
  digitalWrite(DO_X1_H2,0); //driving interllock
  digitalWrite(DO_X1_K1,0); //fwRw relay in off 

  //turn on dashboard lights 
  digitalWrite(DO_X1_G1,1);
  digitalWrite(DO_X1_J2,1);


  
  //Control of power gauge (tachometer)
  if(tachoStartupDir == 0 ) tachoStartup++;
  if(tachoStartupDir == 1 ) tachoStartup--;
  if(tachoStartup >= 80000) tachoStartupDir=1;
  if(tachoStartupDir==1 && tachoStartup <=0 ) tachoStartupDir=3; //do nothing any more
  TachoControl(tachoStartup/10); //0-6000
  
  


  //temp and SOC gauge 
  SocGaugeControl(MyVehicle.SOC);
  TempGaugeControl(MaxTemp); 

      //MyEv app current display
    MyVehicle.amp=0;
    MyVehicle.temp=MyVehicle.amp;
  
   MyVehicle.mode=3; //1-charging, 2-driving, 1-off,
}


/*********************************************************/
// CHARGING MODE subsequence
/*********************************************************/
void ChargingMode() {    

      digitalWrite(DO_X1_K2,1); //BMS On
      digitalWrite(DO_X1_H2,0); //driving interllock
      digitalWrite(DO_X1_K1,0); //fwRw relay in off 
    
      //SOC and temp gauges
      TempGaugeControl(MaxTemp);
      SocGaugeControl(MyVehicle.SOC);
     
      //show charging current on tacho
      TachoControl(ACcurrentFromCharger*10);
    
      //MyEv app current display
      MyVehicle.amp=CurrentFromCharger/10;
      MyVehicle.temp=MyVehicle.amp;
      MyVehicle.mode=1; //1-charging, 2-driving, 1-off,
      if (send2serial==0) floatingAvarage(ACcurrentFromCharger/10);//power chart showing charging current from 0-15A

          
      //Current limiting by Pot
      CurrentToCharger=(analogRead(DI_X1_E3)*MAX_CHARGER_CURRENT)/1024;

      if(MaxPwm >1 ){
          //pid control for current
          Input = MaxPwm;
          myPID.Compute();
          CurrentToCharger=Output;
          Serial.print("  ct1:");
          Serial.print(CurrentToCharger);              
          //Serial.print("  err:");
          //Serial.print(Error);    
           Serial.print("  PID:");
          Serial.print(Output);          
          Serial.print("  MaxPwm:");
          Serial.print(String(MaxPwm));          
          Serial.print("  MaxPwmId:");
          Serial.println(String(MaxPwmId));  
            
              //CurrentToCharger=100;
              //if(MaxPwm >40) CurrentToCharger=35;
      } 
      
       ///  CHARGER COMMUNICATION   100ms ///
      if(canTxTimer1==0){
          canTxTimer1=1;  
          
          



         

          
          can_tx[5]=lowByte((short)CurrentToCharger); //CHARGER_MAXDCCURRLIMIT_LSB 
          can_tx[6]=highByte((short)CurrentToCharger); //CHARGER_MAXDCCURRLIMIT_MSB 

        

          
          /*if(Bms_MasterStatus==BMS_ONEFULL || Bms_MasterStatus==BMS_ALLFULL){
              can_tx[5]=lowByte((short)BALANCE_CHARGER_CURRENT); //CHARGER_MAXDCCURRLIMIT_LSB 
              can_tx[6]=highByte((short)BALANCE_CHARGER_CURRENT); //CHARGER_MAXDCCURRLIMIT_MSB
          }else{
              can_tx[5]=lowByte((short)MAX_CHARGER_CURRENT); //CHARGER_MAXDCCURRLIMIT_LSB 
              can_tx[6]=highByte((short)MAX_CHARGER_CURRENT); //CHARGER_MAXDCCURRLIMIT_MSB
          }*/
            
    
    
    
      
          //lower the voltage and turn on red light when error with BMS
          if(Bms_MasterStatus==BMS_ERROR){
              can_tx[3]=lowByte((short)(CHARGER_VOLTAGE*0.95)); //CHARGER_MAXDCVOLTLIMIT_LSB
              can_tx[4]=highByte((short)(CHARGER_VOLTAGE*0.95)); //CHARGER_MAXDCVOLTLIMIT_MSB
              digitalWrite(DO_X1_G1,1);
          }else{
              can_tx[3]=lowByte((short)CHARGER_VOLTAGE); //CHARGER_MAXDCVOLTLIMIT_LSB
              can_tx[4]=highByte((short)CHARGER_VOLTAGE); //CHARGER_MAXDCVOLTLIMIT_MSB
              digitalWrite(DO_X1_G1,0);  
          }
    
          //if there is overvoltage we disable charger for 3sec
          if(Bms_MasterStatus==BMS_OVERVOLTAGE || MaxPwm >= CUTOFF_CELL_PWM){
              bmsOverCurrentTimer=0;
              can_tx[0]=0x0;//disable charger 
              can_tx[5]=0; //CHARGER_MAXDCCURRLIMIT_LSB 
              can_tx[6]=0; //CHARGER_MAXDCCURRLIMIT_MSB
          }
          if(bmsOverCurrentTimer > 30){
              can_tx[0]=0x1;//CHARGER_ENABLE   
          }else{
              can_tx[0]=0x0;//CHARGER_ENABLE 
          }
          can_tx[1]=lowByte(1000); //CHARGER_POWER_REFERENCE_LSB 100% Power
          can_tx[2]=highByte(1000); //CHARGER_POWER_REFERENCE_MSB
    
    
          can_tx[7]=0x00; //reserved

          //if there is overvoltage we disable charger for 3sec
          if(Bms_MasterStatus==BMS_OVERVOLTAGE || MaxPwm > CUTOFF_CELL_PWM){
              bmsOverCurrentTimer=0;
              can_tx[5]=0; //CHARGER_MAXDCCURRLIMIT_LSB 
              can_tx[6]=0; //CHARGER_MAXDCCURRLIMIT_MSB
          }
          if(bmsOverCurrentTimer > 30){
              can_tx[0]=0x1;//CHARGER_ENABLE   
          }else{
              can_tx[0]=0x0;//CHARGER_ENABLE 
          }
          
        
          CAN.sendMsgBuf(0x2ff, 0, 7, can_tx);   // send data:  id, standrad flame, data len = 8, stmp: data buf
    }
}
/*********************************************************/
// BALLANCING MODE subsequence
/*********************************************************/
/*void BallancingMode() { 
      digitalWrite(DO_X1_K2,1); //BMS On
      digitalWrite(DO_X1_H2,0); //driving interllock
      digitalWrite(DO_X1_K1,0); //fwRw relay in off 
    
      //SOC and temp gauges
      TempGaugeControl(MaxTemp); //todo read from charger
      SocGaugeControl(MyVehicle.SOC);
     
      //show charging current on tacho
      TachoControl(CurrentFromCharger*10);
    
      //MyEv app current display
      MyVehicle.amp=CurrentFromCharger/10;
      MyVehicle.temp=MyVehicle.amp;
      MyVehicle.mode=1; //1-charging, 2-driving, 1-off,
      if (send2serial==0) floatingAvarage(ACcurrentFromCharger/10);//power chart showing charging current from 0-15A
         
          //PID controller
          Input = MaxPwm;
          myPID.Compute();
          CurrentToCharger=Output;
          
       ///  CHARGER COMMUNICATION   100ms ///
      if(canTxTimer1==0){
          canTxTimer1=1;  
      
      

   
          can_tx[5]=lowByte((short)CurrentToCharger); //CHARGER_MAXDCCURRLIMIT_LSB 
          can_tx[6]=highByte((short)CurrentToCharger); //CHARGER_MAXDCCURRLIMIT_MSB 
          
         
          //lower the voltage and turn on red light when error with BMS
          if(Bms_MasterStatus==BMS_ERROR){
              can_tx[3]=lowByte((short)(CHARGER_VOLTAGE*0.9)); //CHARGER_MAXDCVOLTLIMIT_LSB
              can_tx[4]=highByte((short)(CHARGER_VOLTAGE*0.9)); //CHARGER_MAXDCVOLTLIMIT_MSB
              digitalWrite(DO_X1_G1,1);
          }else{
              can_tx[3]=lowByte((short)CHARGER_VOLTAGE); //CHARGER_MAXDCVOLTLIMIT_LSB
              can_tx[4]=highByte((short)CHARGER_VOLTAGE); //CHARGER_MAXDCVOLTLIMIT_MSB
              digitalWrite(DO_X1_G1,0);  
          }

          
          //if there is overvoltage we disable charger for 3sec
          if(Bms_MasterStatus==BMS_OVERVOLTAGE || MaxPwm > CUTOFF_CELL_PWM){
              bmsOverCurrentTimer=0;
              can_tx[5]=0; //CHARGER_MAXDCCURRLIMIT_LSB 
              can_tx[6]=0; //CHARGER_MAXDCCURRLIMIT_MSB
          }
          if(bmsOverCurrentTimer > 30){
              can_tx[0]=0x1;//CHARGER_ENABLE   
          }else{
              can_tx[0]=0x0;//CHARGER_ENABLE 
          }
          
          can_tx[1]=lowByte(1000); //CHARGER_POWER_REFERENCE_LSB 100% Power
          can_tx[2]=highByte(1000); //CHARGER_POWER_REFERENCE_MSB
    
    
          can_tx[7]=0x00; //reserved
        
          CAN.sendMsgBuf(0x2ff, 0, 7, can_tx);   // send data:  id, standrad flame, data len = 8, stmp: data buf



          if (MaxPwm >= MAX_CELL_PWM) CurrentToCharger = BALANCE_CHARGER_CURRENT;
          if (CurrentToCharger>=MAX_CHARGER_CURRENT) CurrentToCharger=MAX_CHARGER_CURRENT;
          if (CurrentToCharger<0 || MaxPwm > CUTOFF_CELL_PWM) CurrentToCharger=0;        
              Serial.print("  ct1:");
            Serial.print(CurrentToCharger);              
          //Serial.print("  err:");
          //Serial.print(Error);    
                  Serial.print("  PID:");
          Serial.print(Output);          
          Serial.print("  MaxPwm:");
          Serial.print(String(MaxPwm));          
          Serial.print("  MaxPwmId:");
          Serial.println(String(MaxPwmId));   
      }


  


     
      
      //when balancing starts start lowerring current so maximum PWM on cells never reaches MAX_CELL_PWM otherwise to MAX_CHARGER_CURRENT
      //Sipmple P controller to stop charging in cca 10s
      




 





 
  
}
*/
/*********************************************************/
// RUNNING MODE subsequence
/*********************************************************/
void RunningMode() {    

    

    //SOC and temp gauges
    TempGaugeControl(MaxTemp); //todo read from inverter
    SocGaugeControl(MyVehicle.SOC);
    
    //dashboard lights
    //if there is no communication with inverter we turn on ign light
    if(InverterRxTimeout > 30){
         //todo error handling
         digitalWrite(DO_X1_G1,1); 
    } else digitalWrite(DO_X1_G1,0); 


    //orange lamp blinks according to FW/RW/NEWTRAL switch
    //-Newtral 1s blink
    //-rewerse constant on
    //-forward off
    
    
    if(FwSwitch.read()==0){
        digitalWrite(DO_X1_K1,0); //fw/rw relay output    
        digitalWrite(DO_X1_H2,1); //driving interllock
        digitalWrite(DO_X1_J2,0); //FW->off
        FwRwTimerDelay=0; //indicates that we are in Forward state
    }
    else if (RwSwitch.read()==0){     
        digitalWrite(DO_X1_K1,1); //fw/rw relay output
        digitalWrite(DO_X1_H2,1); //driving interllock
        digitalWrite(DO_X1_J2,1); //RW->on
        FwRwTimerDelay=0; //indicates that we are in Forward state
        
    }
    else{
        if(FwRwTimerDelay >= 2 ) digitalWrite(DO_X1_H2,0); //driving interllock
        if(OrangeLampBlink>=5){
            OrangeLampBlink=0;
            toggleLed = !toggleLed;
            digitalWrite(DO_X1_J2,toggleLed); //netutral  
        }
    }

    //BMS power On
    digitalWrite(DO_X1_K2,1);

    //show power on tachometer
    TachoControl((VoltageFromInverter*CurrentFromInverter)/1000); //dived by 10k!
    
    //read power pedal ADC and compensate it with 12V measurements (A4)
    /*PowerPedal=(analogRead(DI_X1_E3)*100)/analogRead(A4); //0-100%
    //Max measured is 94%. We scale this to 0-32767
    PowerRequest=(((short)PowerPedal*0xFFFF)/94);
    */

    //MyEv app current display
    MyVehicle.amp=(float)CurrentFromInverter/10;
    MyVehicle.temp=MyVehicle.amp;
    MyVehicle.mode=2; //1-charging, 2-driving, 1-off,
    if (send2serial==0) floatingAvarage((VoltageFromInverter*CurrentFromInverter)/10000);//power chart showing power from 0-150kw
    
    //if battery full stop regen TODO!
    
    //can mesage to inverter TODO!
    //lsb first Msb second byte
    if(canTxTimer2==0){
        canTxTimer2=1;
   
        can_tx[0]=0xFF; //all controll bits todo 
        can_tx[1]=0xFF; 
        can_tx[2]=0x00;
        can_tx[3]=0x00;
        can_tx[4]=0x00;
        can_tx[5]=0x00;
        can_tx[6]=0x00;
        can_tx[7]=0x00;
        CAN.sendMsgBuf(0x210, 0, 8, can_tx);   // send data:  id, 0 - standrad flame, data len = 8, stmp: data buf
    }
   
         
  
}




/*******************************************************************/
// Function that calculates the SOC
/*******************************************************************/
void StateOfCHarge(){

    if (State == CHARGING_MODE){
      //BMS is sending current in 0.1A resolution. Each 100ms we add 1/36000 of reported current to capacity. 
      
     // if(Bms_MasterStatus==BMS_ALLFULL) BpCapacity=RatedBpCapacity*1000; //mWh
      BpCapacity+=CurrentFromCharger/360;  //mWh
      
    }
    if (State == RUNNING_MODE){
      //MOTOR is sending current in 0.1A resolution. Each 100ms we add 1/36000 of reported current to capacity. 
      BpCapacity-=CurrentFromInverter/3600;  //mWh TODO check current and direction!
    }

    //If all batteryes are full we reset the capacity to nominal
    if (State == FULL_MODE){
      BpCapacity=RatedBpCapacity*1000; //mWh
    }
    if(Bms_MasterStatus==BMS_ALLFULL) BpCapacity=RatedBpCapacity*1000; //mWh

    //SOC in %
    MyVehicle.SOC=BpCapacity/(RatedBpCapacity*10);


    //time to charge for MyEV app
    //ratedCapacity-remainingCapacity / chargerPower
    MyVehicle.ttc=(((RatedBpCapacity*1000)-BpCapacity) / (MAX_CHARGER_CURRENT*CHARGER_VOLTAGE*10)); //CHARGER_VOLTAGE / CURRNENT [mV]
    
    //MyVehicle.vol=
}
 
/*******************************************************************/
// Function that process voltage message from BMS
/*******************************************************************/ 
void processBmsVoltage(){



    //we only consider voltage if status message is OK                 
    BMS_Master_Id=((CAN_ID >> (8)) & 0xff);
    if(BmsStatus0[(BMS_Master_Id*6)  ]) BmsVoltage[(BMS_Master_Id*6)]=((float)buf[0]+200)/103.4;
    if(BmsStatus0[(BMS_Master_Id*6)+1]) BmsVoltage[(BMS_Master_Id*6)+1]=((float)buf[1]+200)/103.4;
    if(BmsStatus0[(BMS_Master_Id*6)+2]) BmsVoltage[(BMS_Master_Id*6)+2]=((float)buf[2]+200)/103.4;
    if(BmsStatus0[(BMS_Master_Id*6)+3]) BmsVoltage[(BMS_Master_Id*6)+3]=((float)buf[3]+200)/103.4;
    if(BmsStatus0[(BMS_Master_Id*6)+4]) BmsVoltage[(BMS_Master_Id*6)+4]=((float)buf[4]+200)/103.4;
    if(BmsStatus0[(BMS_Master_Id*6)+5]) BmsVoltage[(BMS_Master_Id*6)+5]=((float)buf[5]+200)/103.4; 
    
//Serial.println();
//    Serial.print("Proces BMS master ID:");
//    Serial.print(BMS_Master_Id);
//    Serial.print(" V0:");
 //   Serial.print(BmsVoltage[0]);    
 //   Serial.print(" buf0:");
//    Serial.println(buf[0]);

    //overall voltage
    //add voltahes from all masters
    MyVehicle.vol=0;
    MyVehicle.maxV=0; //reset values
    MyVehicle.minV=10; //reset values
    for (int i=0;i<(NO_OF_BMS_MASTERS*6);i++) {
        MyVehicle.vol+=BmsVoltage[i];
        if(BmsVoltage[i] > MyVehicle.maxV) MyVehicle.maxV=BmsVoltage[i];
        if(BmsVoltage[i] < MyVehicle.minV) MyVehicle.minV=BmsVoltage[i];
    }
    MyVehicle.avgV=MyVehicle.vol/(NO_OF_BMS_MASTERS*6);



    
  
  
}
/*******************************************************************/
// Function that process Stratus message from BMS
/*******************************************************************/ 
void processBmsStatus(){

  //status byte 0
  BmsStatus0[(BMS_Master_Id*6)+0]=test_bit(buf[0],0);
  BmsStatus0[(BMS_Master_Id*6)+1]=test_bit(buf[0],1);
  BmsStatus0[(BMS_Master_Id*6)+2]=test_bit(buf[0],2);
  BmsStatus0[(BMS_Master_Id*6)+3]=test_bit(buf[0],3);
  BmsStatus0[(BMS_Master_Id*6)+4]=test_bit(buf[0],4);
  BmsStatus0[(BMS_Master_Id*6)+5]=test_bit(buf[0],5);
  
  BmsStatus[BMS_Master_Id]=buf[1];
  
  char allfull=0;
  char onefull=0;
  char bmsempty=0;
  char bmscutoff=0;
  char bmsovervoltage=0;
  char bmserror=0;

  //check all BMS units
  for(char i=0;i<=4;i++){
    //if all cells are full
    if(test_bit(BmsStatus[i],0)) allfull++;
    if(test_bit(BmsStatus[i],1)) onefull++;
  
    if(test_bit(BmsStatus[i],2)) bmsempty++;
    if(test_bit(BmsStatus[i],3)) bmscutoff++;
  
    if(test_bit(BmsStatus[i],4)) bmsovervoltage++;
    
    if (BmsRxTimeout[i] > 30) bmserror++; //timout with communication

 
  }
   Bms_MasterStatus=BMS_NORMAL;
  if (onefull > 0) Bms_MasterStatus= BMS_ONEFULL;   
  if (allfull==5) Bms_MasterStatus= BMS_ALLFULL;   
  if (bmsempty > 0) Bms_MasterStatus= BMS_EMPTY;   
 // TODO fox BMS!!! if (bmscutoff > 0) Bms_MasterStatus= BMS_CUTOFF;   
  if (bmsovervoltage >0 ) Bms_MasterStatus= BMS_OVERVOLTAGE;   
  if (bmserror >0 ) Bms_MasterStatus= BMS_ERROR;   

  MaxTemperature[BMS_Master_Id]=0;
  
  //temperaure - we only care about max temp for display
  for(char i=2;i<=5;i++){
      if (buf[i] > MaxTemperature[BMS_Master_Id] && buf[i] < 200 ) MaxTemperature[BMS_Master_Id]=buf[i];
  }
  MaxTemp=0;
  for(char i=0;i<=NO_OF_BMS_MASTERS;i++){
      if ( MaxTemperature[i] >MaxTemp ) MaxTemp=MaxTemperature[BMS_Master_Id];
  }
  
  MyVehicle.camp=MaxTemp;
  
  
  
  
}
/*******************************************************************/
// Function that process balance message from BMS
/*******************************************************************/ 
void processBmsBalance(){
  char i;
  //only if readout was OK

 
    if(BmsStatus0[(BMS_Master_Id*6)+0])  MyVehicle.CellBalance[((BMS_Master_Id)*6)+0]= buf[0]; 
    if(BmsStatus0[(BMS_Master_Id*6)+1]) MyVehicle.CellBalance[((BMS_Master_Id)*6)+1]= buf[1]; 
    if(BmsStatus0[(BMS_Master_Id*6)+2]) MyVehicle.CellBalance[((BMS_Master_Id)*6)+2]= buf[2]; 
    if(BmsStatus0[(BMS_Master_Id*6)+3])  MyVehicle.CellBalance[((BMS_Master_Id)*6)+3]= buf[3]; 
    if(BmsStatus0[(BMS_Master_Id*6)+4]) MyVehicle.CellBalance[((BMS_Master_Id)*6)+4]= buf[4]; 
    if(BmsStatus0[(BMS_Master_Id*6)+5]) MyVehicle.CellBalance[((BMS_Master_Id)*6)+5]= buf[5]; 

    //Max PWM for requred for balancing current
    MaxPwm=0;
    MaxPwmId=0;
    for (int i=0;i<(NO_OF_BMS_MASTERS*6);i++) {
        if(MyVehicle.CellBalance[i]>MaxPwm) {
          MaxPwm=MyVehicle.CellBalance[i]; 
          MaxPwmId=i+1;
        }
        
    }
  
  



  




}

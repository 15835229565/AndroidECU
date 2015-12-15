/*******************************************************************/
// Function that constructs XML for MyVehicle mobile phone app
/*******************************************************************/
void sendToMyVehicleApp(){
  
  //
  
  
  int i;
  //Serial2.write(27);  
  Serial2.print("  HE"); 
  Serial2.print("<head><donut v='"+String(MyVehicle.SOC)+"'/><ttc v='");
  Serial2.print(MyVehicle.ttc,2);
  Serial2.print("'/><rd v='"+String(MyVehicle.rd)+"'/><amp v='"+String(MyVehicle.amp,1)+"'/>");
  for( i = 9; i>=0; i--){    // print chart data
    if(MyVehicle.pw[i]>150) MyVehicle.pw[i]=150;
    Serial2.print("<pw  v='"+String(MyVehicle.pw[i],0)+"'/>");
  }
  
  //Serial.print("<pw  v='"+MyVehicle.pw1+"'/><pw  v='"+MyVehicle.pw2+"'/>");
  //Serial.print("<pw  v='"+String(MyVehicle.pw3)+"'/><pw  v='"+String(MyVehicle.pw4)+"'/><pw  v='"+String(MyVehicle.pw5)+"'/><pw  v='"+String(MyVehicle.pw6)+"'/><pw  v='"+String(MyVehicle.pw7)+"'/><pw  v='"+String(MyVehicle.pw8)+"'/>");
  //Serial.print("<pw  v='"+String(MyVehicle.pw9)+"'/><pw  v='"+String(MyVehicle.pw10)+"'/><pw  v='"+String(MyVehicle.pw11)+"'/>
  Serial2.print("<vol v='"+String(MyVehicle.vol,2)+"'/><camp v='"+String(MyVehicle.camp)+"'/><temp v='"+String(MyVehicle.temp,1)+"'/>");
  Serial2.print("<max v='"+String(MyVehicle.maxV,2)+"'/><min v='"+String(MyVehicle.minV,2)+"'/><avg v='"+String(MyVehicle.avgV,2)+"'/><mode v='"+String(MyVehicle.mode)+"'/><mode2 v='"+String(MyVehicle.mode)+"'/>");


  for( i = 0; i < ((NO_OF_BMS_MASTERS)*6); i++)    // print the CELL data
  {
 Serial2.print("<bt id='"+String(i+1)+"' v='"+String(BmsVoltage[i],2)+"'");// c='4'/>"); //1-orange, 2-blue , 3 green,o-red
   // Serial2.print("<bt id='"+String(i+1)+"' v='"+String(BmsVoltage[i],2)+"' c='1'/>");
    if(MyVehicle.CellBalance[i] >0 )Serial2.print(" c='3'/>");
    else if(BmsVoltage[i]< BATTERY_CUT_OFF_VOLTAGE) Serial2.print(" c='0'/>"); //1-orange, 2-blue , 3 green,o-red
    else Serial2.print(" c='2'/>"); //normal
  /*Serial.print("<bt id='"+String(BMS_MessageNo[i]+1)+"' v='"); 
     if(BMS_Voltage[i]>30000) {
     //if(CellStatus[i]==1) Serial.print("______*    ");
     Serial.print("0.0");
     }else {
     float x=(float)BmsVoltage[i]/1000;
     Serial.print(x,3);
     //if(CellStatus[i]==1) Serial.print("*    ");
     //else if(CellStatus[i]==3) Serial.print("Flt  ");
     }
     Serial.print("'/>");*/
     
  } 
    Serial2.print("<dht11 v='85' /><dht11 v='25' /></head>");
  Serial2.print("/HE");
  

 // Serial2.print("HE"); 
 // Serial2.print("<head> <donut v='70'/> <ttc v='75'/> <rd v='16'/> <amp v='30'/>");

  //Serial2.print("<pw  v='50'/><pw  v='32'/><pw  v='35'/><pw  v='102'/><pw  v='60'/><pw  v='50'/><pw  v='32'/><pw  v='35'/>");
  //Serial2.print("<pw  v='102'/><pw  v='60'/><pw  v='60'/>");
  //Serial2.print("<vol v='220'/><camp v='8.9'/> <temp v='189'/>");
 // Serial2.print("<max v='4'/><min v='2'/><avg v='3'/><mode v='1'/><mode2 v='1'/>");
  //Serial2.print("<bt id='1' v='3.0' c='3'/> <bt id='2' v='1.1' c='0'/><bt id='3' v='1.8' c='1'/><bt id='4' v='2.6' c='2'/> <bt id='5' v='1.0' c='0'/> <bt id='6' v='1.6' c='1'/><bt id='7' v='2.2' c='2'/><bt id='8' v='1.9' c='1'/><bt id='9' v='3.3' c='3'/><bt id='10' v='2.0' c='2'/><bt id='11' v='3.1' c='3'/><bt id='12' v='1.2' c='0'/><bt id='13' v='3.3' c='3'/><bt id='14' v='3.3' c='3'/>");
  //Serial2.print("<dht11 v='50' /><dht11 v='30' /></head>");
 // Serial2.print("/HE");
}

/*******************************************************************/
// Function that fills up array of 12 bytes with floating avarage
/*******************************************************************/
void floatingAvarage(byte data){
  float delta;

  if(FilterIndex >= 10){ 
      FilterIndex=0;
  }
  
  delta=0.7;
  MyVehicle.pw[0]=MyVehicle.pw[1]*(1-delta)+ data*delta;

//Serial.print(data);
  for(byte i=1;i<10;i++){
      delta=1/((float)i*2);
 //     Serial.print("dt=");
 //     Serial.print(String(delta,5));
      MyVehicle.pw[i]=MyVehicle.pw[i]*(1-delta)+ MyVehicle.pw[i-1]*delta;
  //    Serial.print(" ");
  //    Serial.print(String(i));
   //   Serial.print("=");
   //   Serial.print(MyVehicle.pw[i]);
  }
  //Serial.println();
  


}

/*******************************************************************/
// Function that sends RAW data to terminal
/*******************************************************************/ 
void sendToTerminal(){
  


      //  Serial.write(27);  
       // Serial.print("[2J"); // clear screen
       // Serial.write(27); // ESC
       // Serial.print("[H"); // cursor to home


        Serial.print("State:");
        Serial.print(State);


        Serial.print("  key2:");  
        Serial.print(digitalRead(DI_X1_F1));
        Serial.print("  Key1:");
        Serial.print(digitalRead(DI_X1_E1));
        Serial.print("  chargerStatus:");  
        Serial.print(chargerStatus);
        Serial.print("  CurrentToCharger:");
        Serial.print(String(CurrentToCharger,4));
        Serial.print("  MaxPwm:");
        Serial.print(MaxPwm);
        Serial.print("  SOC:");
        Serial.print(MyVehicle.SOC);
        Serial.print("  VC:");
        Serial.print(VoltageFromCharger);
        Serial.print("  IC:");
        Serial.print(CurrentFromCharger);
        Serial.print("  BMS Status:");
        Serial.print(Bms_MasterStatus);        
        Serial.print("  MaxTemp:");
        Serial.println(MaxTemp);
 
       
     /*   
        //Serial.println("*************");
        Serial.print("MaxV:");
      //  Serial.print(maxVoltage/1000,3);
        Serial.print("  MinV:");
     //   Serial.print(minVoltage/1000,3);
        Serial.print("  SumV:");
     //   Serial.print(sumVoltage/1000);
        Serial.print("  cI:");
        Serial.print(CurrentFromCharger);
        Serial.print("  cV:");
        Serial.print(VoltageFromCharger);
        Serial.print("  cS:");
        Serial.println(chargerStatus, BIN);
        Serial.print("bmsState:");
 */   //    Serial.print(bmsState);
    /*    if(bmsState==1){
            Serial.print("-Balancing");
        }
        else if(bmsState==2){ 
            Serial.print("-Full power");
        }       
        else Serial.print("-Stop charging");*/ 
/*        Serial.print("  batI:");
     //   Serial.print(BatteryCurrent);
        Serial.print("  batCap:");
    //    Serial.print(BatteryCapacity);
        Serial.print(" Ah");
        
  
        for(int i = 0; i<=63; i++)    // print the data
        {
               
          
            if (i% 4 == 0) Serial1.println(); //each 4rd battery we make new line
            if (i<=8) Serial1.print("0"); 
            
            Serial1.print(BMS_MessageNo[i]+1);
            Serial1.print("=");
            if(BMS_Voltage[i]>30000) {
              if(CellStatus[i]==1) Serial1.print("______*    ");
              else Serial1.print("______     ");
            }else {
              float x=(float)BMS_Voltage[i]/1000;
              Serial1.print(x,3);
              Serial1.print("V");
              if(CellStatus[i]==1) Serial1.print("*    ");
              else if(CellStatus[i]==3) Serial1.print("Flt  ");
              else Serial1.print("     ");

            }
            
            //we reset the value
            //voltage[i]=999;
            
        } 
       */ 

  
}



/*******************************************************************/
// Function that drives the tachometer
/*******************************************************************/ 
void TachoControl(int rpm){
    
    //the tachometer expects 1-4 pulses on every rotation (depending of tacho type). In order to control it we need to calculate time between pulses at certain RPM. 
    float dt;

    dt=(rpm/60)*TACHO_PULSES_PER_ROTATION;  // number of requred pulses per second
    dt=1000000/dt; // dt between pulses in micro seconds
   
    currentTime = micros();
    if (currentTime - previousTime >= dt) {
        previousTime = currentTime;

            Toggle = !Toggle;


        digitalWrite(DO_X1_J3, Toggle);
    }
  
}
/*******************************************************************/
// Function that drives the SOC gauge 
/*******************************************************************/ 
void SocGaugeControl(byte SOC){ //soc in %
  byte pwm;  
  //"curve fitting"
  if (SOC < 50){
      #define SOC_DELTA_50 SOC_GAUGE_50-SOC_GAUGE_0
      pwm = ((SOC*(byte)SOC_DELTA_50)/50)+SOC_GAUGE_0; 
 
  }else{
      #define SOC_DELTA_100 SOC_GAUGE_100-SOC_GAUGE_50 

      pwm = (((SOC-50)*(byte)SOC_DELTA_100)/50)+SOC_GAUGE_50; 
  }
  analogWrite(DO_X1_K3,pwm);     
}
/*******************************************************************/
// Function that drives the Temperature gauge 
/*******************************************************************/ 
void TempGaugeControl(byte Temp){ //temp in C
  //Temperature readings in degrees Celsius starting with 50C offset.
  //0 = -50C
  //255 =205C

  //int temperature = Temp-50;
  int temperature = Temp;
  if (temperature<0)temperature=0;

  Temp=(temperature*100)/TEMP_GAUGE_MAXTEMP;
  if(Temp>100)Temp=100;  
  
  
  //below 0 does not show on gauge
  
  byte pwm;
  //"curve fitting"
  if (Temp < 50){
      #define TEMP_DELTA_50 TEMP_GAUGE_50-TEMP_GAUGE_0 
      pwm = ((Temp*(byte)TEMP_DELTA_50)/50)+TEMP_GAUGE_0; 
 
  }else{
      #define TEMP_DELTA_100 TEMP_GAUGE_100-TEMP_GAUGE_50 
      pwm = (((Temp-50)*(byte)TEMP_DELTA_100)/50)+TEMP_GAUGE_50;  
  }
  analogWrite(DO_X1_J1,pwm);     
}

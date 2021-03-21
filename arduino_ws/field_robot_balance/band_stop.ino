/*
//https://www.norwegiancreations.com/2016/03/arduino-tutorial-simple-high-pass-band-pass-and-band-stop-filtering/

int EMA_a_low_pin = 0;          //pin number to use the ADC
int EMA_a_high_pin = 1;          //pin number to use the ADC

//initialization of EMA alpha (cutoff-frequency)
float EMA_a_low = 0.05;
float EMA_a_high = 0.1;
 
int EMA_S_low = 0;          //initialization of EMA S
int EMA_S_high = 0;
 
int highpass = 0;
int bandpass = 0;
int bandstop = 0;

    float EMA_a = 0.05;      //initialization of EMA alpha
    int EMA_S = 0;          //initialization of EMA S

/*void getEmaA(){
  EMA_a_low = analogRead(EMA_a_low_pin);    //read the sensor value using ADC
  EMA_a_high = analogRead(EMA_a_high_pin);   
  
  EMA_a_low = 0.001 + 0.0002 * EMA_a_low;      //EMA_a_low soll zwischen 0.001 und 0.2 laufen  
  EMA_a_high = 0.4 + 0.5 * EMA_a_high/1023;   //EMA_a_low soll zwischen 0.4 und 0.9 laufen
}*/

/*void lowPassFilter(){
  imuValue = 10*imuValue;
    EMA_S = (EMA_a*imuValue) + ((1-EMA_a)*EMA_S);    //run the EMA  
      Serial.print(imuValue);
      Serial.print(" ");
      Serial.println(EMA_S);
}

void highPassFilter(){
  imuValue = 10*imuValue;
    EMA_S = (EMA_a*imuValue) + ((1-EMA_a)*EMA_S);    //run the EMA  
    highpass = imuValue - EMA_S;                   //calculate the high-pass signal
      Serial.print(imuValue);
      Serial.print(" ");
      Serial.println(highpass);
}*/

// exponential moving average (EMA)
/*void bandStopFilter(){
  imuValue = 100*imuValue;
    // wenn EMA_a_low = 0, dann
  EMA_S_low = (EMA_a_low*imuValue) + ((1-EMA_a_low)*EMA_S_low);          
  EMA_S_high = (EMA_a_high*imuValue) + ((1-EMA_a_high)*EMA_S_high);

  // Hoch- und Tiefpass testen
  int lowPass = imuValue - EMA_S_low;     
  int highPass = imuValue - EMA_S_high;     

  bandpass = EMA_S_high - EMA_S_low;        //find the band-pass 
  bandstop = imuValue - bandpass;        //find the band-stop signal

  imuValue = bandstop;
  imuValue = imuValue/100;
}*/

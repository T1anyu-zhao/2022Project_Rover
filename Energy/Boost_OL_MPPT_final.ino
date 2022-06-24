/*
 * Modified from Program (used in Power Lab) written by Yue Zhu (yue.zhu18@imperial.ac.) in July 2020.
 * pin6 is PWM output at 62.5kHz.
 * duty-cycle saturation is set as 1% - 99%
 * Control frequency is set as 1.25kHz. uk
*/

#include <Wire.h>
#include <INA219_WE.h>
#include <SPI.h>

INA219_WE ina219; // this is the instantiation of the library for the current sensor

float vb,vpd,iL,current_mA; // Measurement Variables
unsigned int sensorValue0,sensorValue3;  // ADC sample values declaration
float oc=0; //internal signals
float pwm_out = 0.85;//initial pwm_out set to 0.85
float Power_now = 0, Power_anc = 0, voltage_anc = 0;
float delta = 0.02;

boolean Boost_mode = 0;
boolean CL_mode = 0;

unsigned int loopTrigger;
unsigned int int_count = 0;

void setup() {

  //Basic pin setups
  
  noInterrupts(); //disable all interrupts
  pinMode(13, OUTPUT);  //Pin13 is used to time the loops of the controller
  pinMode(3, INPUT_PULLUP); //Pin3 is the input from the Buck/Boost switch
  pinMode(2, INPUT_PULLUP); // Pin 2 is the input from the CL/OL switch
  analogReference(EXTERNAL); // We are using an external analogue reference for the ADC

  // TimerA0 initialization for control-loop interrupt.
  
  TCA0.SINGLE.PER = 999; //
  TCA0.SINGLE.CMP1 = 999; //
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm; //16 prescaler, 1M.
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP1_bm; 

  // TimerB0 initialization for PWM output
  
  pinMode(6, OUTPUT);
  TCB0.CTRLA=TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //62.5kHz
  analogWrite(6,120); 

  interrupts();  //enable interrupts.
  Wire.begin(); // We need this for the i2c comms for the current sensor
  ina219.init(); // this initiates the current sensor
  Wire.setClock(700000); // set the comms speed for i2c

  Serial.begin(9600); // USB Communications
  
}

 void loop() {
  if(loopTrigger) { // This loop is triggered, it wont run unless there is an interrupt
    
    digitalWrite(13, HIGH);   // set pin 13. Pin13 shows the time consumed by each control cycle. It's used for debugging.
    
    // Sample all of the measurements and check which control mode we are in
    sampling();
    CL_mode = digitalRead(3); // input from the OL_CL switch
    Boost_mode = digitalRead(2); // input from the Buck_Boost switch

    if (Boost_mode){
      if (CL_mode) { //Closed Loop Boost
          pwm_modulate(1); // This disables the Boost as we are not using this mode
      }else{ // Open Loop Boost
          pwm_out = saturation(pwm_out, 0.99, 0.7); //duty_cycle saturation, maybe use this? 0.8 corresponds to 8V roughly
          pwm_modulate(pwm_out); // and send it out
          //Serial.println("pwm_out");
          //Serial.println(pwm_out);
      }
    }else{      
      if (CL_mode) { // Closed Loop Buck
          pwm_modulate(0); // This disables the Buck as we are not using this mode
      }else{ // Open Loop Buck
          pwm_modulate(0); // This disables the Buck as we are not using this mode
      }
    }
    // closed loop control path

    digitalWrite(13, LOW);   // reset pin13.
    int_count++;
    loopTrigger = 0;
  }
  if(int_count==1000){//slow loop
    //MPPT (boost)
          sampling();
          Power_now = vb * (-current_mA); //assume current ripple small? Do we need to get an average value for inductor/output current? How? Saving measurements to SD cards then take out?
          if (Power_now > Power_anc)
          { if (vb > voltage_anc)
                pwm_out = pwm_out - delta;
            else
                pwm_out = pwm_out + delta;
          }
          else
          {
            if (vb > voltage_anc)
                pwm_out = pwm_out + delta;
            else
                pwm_out = pwm_out - delta;
           }
            Power_anc = Power_now;
            voltage_anc = vb;
            int_count = 0;
            Serial.println("vb");
            Serial.println(vb);
            Serial.println("Power_now");
            Serial.println(Power_now);
            Serial.println("iL");
            Serial.println(iL);
            Serial.println("current_mA");
            Serial.println(current_mA);
            Serial.println("pwm");
            Serial.println(pwm_out);
    }
}


// Timer A CMP1 interrupt. Every 800us the program enters this interrupt. 
// This, clears the incoming interrupt flag and triggers the main loop.

ISR(TCA0_CMP1_vect){
  TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_CMP1_bm; //clear interrupt flag
  loopTrigger = 1;
}

// This subroutine processes all of the analogue samples, creating the required values for the main loop

void sampling(){

  // Make the initial sampling operations for the circuit measurements
  
  sensorValue0 = analogRead(A0); //sample Vb
  //sensorValue2 = analogRead(A2); //sample Vref
  sensorValue3 = analogRead(A3); //sample Vpd
  current_mA = ina219.getCurrent_mA(); // sample the inductor current (via the sensor chip)

  // Process the values so they are a bit more usable/readable
  // The analogRead process gives a value between 0 and 1023 
  // representing a voltage between 0 and the analogue reference which is 4.096V
  
  vb = sensorValue0 * (4.096 / 1023.0); // Convert the Vb sensor reading to volts
  //vref = sensorValue2 * (4.096 / 1023.0); // Convert the Vref sensor reading to volts
  vpd = sensorValue3 * (4.096 / 1023.0); // Convert the Vpd sensor reading to volts

  // The inductor current is in mA from the sensor so we need to convert to amps.
  // We want to treat it as an input current in the Boost, so its also inverted
  // For open loop control the duty cycle reference is calculated from the sensor
  // differently from the Vref, this time scaled between zero and 1.
  // The boost duty cycle needs to be saturated with a 0.33 minimum to prevent high output voltages
  // duty ref is not used here
  
  if (Boost_mode == 1){
    iL = -current_mA/1000.0;
    //dutyref = saturation(sensorValue2 * (1.0 / 1023.0),0.99,0.33);
  }else{
    iL = current_mA/1000.0;
    //dutyref = sensorValue2 * (1.0 / 1023.0);
  }
  
}

float saturation( float sat_input, float uplim, float lowlim){ // Saturatio function
  if (sat_input > uplim) sat_input=uplim;
  else if (sat_input < lowlim ) sat_input=lowlim;
  else;
  return sat_input;
}

void pwm_modulate(float pwm_input){ // PWM function
  analogWrite(6,(int)(255-pwm_input*255)); 
}


/*end of the program.*/

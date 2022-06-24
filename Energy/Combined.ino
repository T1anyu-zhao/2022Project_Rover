/*
 * Program written by Yue Zhu (yue.zhu18@imperial.ac.uk) in July 2020.
 * pin6 is PWM output at 62.5kHz.
 * duty-cycle saturation is set as 2% - 98%
 * Control frequency is set as 1.25kHz. 
*/

#include <Wire.h>
#include <INA219_WE.h>
#include <SPI.h>
#include <SD.h>

INA219_WE ina219; // this is the instantiation of the library for the current sensor

Sd2Card card;
SdVolume volume;
SdFile root;

float open_loop, closed_loop; // Duty Cycles
float pwm_out, pwm_out boost;
float vpd,vb,vref,iL,dutyref,current_mA,vin,current_mA_boost, vb_boost; // Measurement Variables
unsigned int sensorValue0,sensorValue1,sensorValue2,sensorValue3,sensorValue4;  // ADC sample values declaration
float ev=0,cv=0,ei=0,oc=0; //internal signals
float Ts=0.0008; //1.25 kHz control frequency. It's better to design the control period as integral multiple of switching period.
float kpv=0.05024,kiv=15.78,kdv=0; // voltage pid.
float u0v,u1v,delta_uv,e0v,e1v,e2v; // u->output; e->error; 0->this time; 1->last time; 2->last last time
float kpi=0.02512,kii=39.4,kdi=0; // current pid.
float u0i,u1i,delta_ui,e0i,e1i,e2i; // Internal values for the current controller
float uv_max=4, uv_min=0; //anti-windup limitation
float ui_max=1, ui_min=0; //anti-windup limitation
float current_limit = 2.0;
float gain = 0.081;
float offset = 0;
boolean Boost_mode = 0;
boolean CL_mode = 0;

float Power_now = 0, Power_anc = 0, voltage_anc = 0;
float delta = 0.02;

unsigned int int_count = 0;
String dataString;
float current_measure;


unsigned int loopTrigger;
unsigned int com_count=0;   // a variables to count the interrupts. Used for program debugging.

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
  
  pinMode(6, OUTPUT);//buck pwm
  pinMode(5, OUTPUT);//boost pwm
  pinMode(9, OUTPUT); //relay
  TCB0.CTRLA=TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //62.5kHz
  analogWrite(6,120); 
  analogWrite(5,120);

  interrupts();  //enable interrupts.
  Wire.begin(); // We need this for the i2c comms for the current sensor
  ina219.init(); // this initiates the current sensor
  Wire.setClock(700000); // set the comms speed for i2c

  Serial.begin(9600);
  Serial.println("serial");

  digitalWrite(9, LOW); //relay off by default

  //open_loop = 0.8;
  
}

 void loop() {
  if(loopTrigger) { // This loop is triggered, it wont run unless there is an interrupt

    //Serial.println("enter loop");
    
    digitalWrite(13, HIGH);   // set pin 13. Pin13 shows the time consumed by each control cycle. It's used for debugging.
    
    // Sample all of the measurements and check which control mode we are in
    sampling();
    CL_mode = digitalRead(3); // input from the OL_CL switch
    Boost_mode = digitalRead(2); // input from the Buck_Boost switch

    //MPPT boost
    pwm_out_boost = saturation(pwm_out, 0.99, 0.7); //duty_cycle saturation, maybe use this? 0.8 corresponds to 8V roughly
    pwm_modulate(pwm_out_boost); // and send it out
          // Open Loop Buck, using this only
          //Serial.println("enter open loop");
          //current_limit = (1.6988*vpd - 7.6843);
    Serial.println(vpd); //2.27 and 2.28 when actually 5V output and input
    Serial.println(vb);
    Serial.println(vin);
    pwm_out = gain*vin + offset;
    pwm_out = saturation(pwm_out,0.99,0.01);
    pwm_modulate(pwm_out); // and send it out
    Serial.println("pwm_out");
    Serial.println(pwm_out);
    Serial.println("current_mA");
    Serial.println(current_mA);
          //maybe first time open loop saturated at dutyref
          //second time saturated at 0.99?
          
    }
    // closed loop control path

    digitalWrite(13, LOW);   // reset pin13.
    loopTrigger = 0;
    int_count++;
  }

  if (int_count == 1000) { // SLOW LOOP (1Hz) (for MPPT and relay operation)
    Serial.println("!!!!enter slow loop");

    sampling();
    Power_now = vb_boost * (-current_mA_boost); //assume current ripple small? Do we need to get an average value for inductor/output current? How? Saving measurements to SD cards then take out?
    if (Power_now > Power_anc)
          { if (vb_boost > voltage_anc)
                pwm_out_boost = pwm_out_boost + delta;
            else
                pwm_out_boost = pwm_out_boost - delta;
          }
     else
          {
            if (vb_boost > voltage_anc)
                pwm_out_boost = pwm_out_boost - delta;
            else
                pwm_out_boost = pwm_out_boost + delta;
           }
     Power_anc = Power_now;
     voltage_anc = vb_boost;

            Serial.println("vb_boost");
            Serial.println(vb_boost);
            Serial.println("Power_now");
            Serial.println(Power_now);
            Serial.println("iL");
            Serial.println(iL_boost);
            Serial.println("current_mA");
            Serial.println(current_mA_boost);
            Serial.println("pwm");
            Serial.println(pwm_out_boost);

    if (vb < 3.5 || vb > 4.5) { //Checking for Error states (low or high input voltage) vb 1.45 approximately correspond to 5.2 V output, 1.51 correspond to 4.80, 1.24 approximately 4.508 V (sunny weather) (cloudy) 4.674V to 1.09, should be 1.2 and 1.45 here, roughly 1.33 and 1.60 when connected to battery
          digitalWrite(7,true); //turn on the red LED
          digitalWrite(9, HIGH); //relay off
          Serial.println("voltage out of range");
          Serial.println(vb);
          //Serial.println(vpd);
          //pwm_out = 0; // no PWM?
      }
    else{ //charging
          digitalWrite(9,false); //turn off the red LED
          digitalWrite(9, LOW); //relay on
          Serial.println("charging");
          Serial.println(vb);
      }
    
    dataString = String(vb) + "," + String(current_measure); //build a datastring for the CSV file
    Serial.println(dataString); // send it to serial as well in case a computer is connected
    File dataFile = SD.open("SD_Test.csv", FILE_WRITE); // open our CSV file
    if (dataFile){ //If we succeeded (usually this fails if the SD card is out)
      dataFile.println(dataString); // print the data
    } else {
      Serial.println("File not open"); //otherwise print an error
    }
    dataFile.close(); // close the file
    int_count = 0; // reset the interrupt count so we dont come back here for 1000ms
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
  sensorValue2 = 2.8/(4.096/1023.0); //Vref, set to 2.6 better
  sensorValue3 = analogRead(A3); //sample Vpd (input voltage?)
  sensorValue4 = analogRead(A7); //buck input voltage
  current_mA = ina219.getCurrent_mA(); // sample the inductor current (via the sensor chip)

  // Process the values so they are a bit more usable/readable
  // The analogRead process gives a value between 0 and 1023 
  // representing a voltage between 0 and the analogue reference which is 4.096V
  
  vb = sensorValue0 * (4.096 / 1023.0); // Convert the Vb sensor reading to volts
  //vref = sensorValue2 * (4.096 / 1023.0); // Convert the Vref sensor reading to volts
  vref = 5.0; //can we defined vref to be certain constant like that?
  vpd = sensorValue3 * (4.096 / 1023.0); // Convert the Vpd sensor reading to volts, maybe change the above to 5.0 as well? but maybe connecting to a potential divider, check 
  vin = 2*sensorValue3 * (4.096 / 1023.0);//maximum 5 V? potential divider to convert maximally 8V to 5V

  // The inductor current is in mA from the sensor so we need to convert to amps.
  // We want to treat it as an input current in the Boost, so its also inverted
  // For open loop control the duty cycle reference is calculated from the sensor
  // differently from the Vref, this time scaled between zero and 1.
  // The boost duty cycle needs to be saturated with a 0.33 minimum to prevent high output voltages
  
  //if (Boost_mode == 1){
    //iL = -current_mA/1000.0;
    //dutyref = saturation(sensorValue2 * (1.0 / 1023.0),0.99,0.33);
  //}
  //else{
    iL = current_mA/1000.0;
    dutyref = sensorValue2 * (1.0 / 1023.0);
    dutyref = saturation(dutyref,0.99,0.02);
  //}
  
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

void pwm_modulate_boost(float pwm_input){ // PWM function
  analogWrite(5,(int)(255-pwm_input*255)); 
}

// This is a PID controller for the voltage

float pidv( float pid_input){
  float e_integration;
  e0v = pid_input;
  e_integration = e0v;
 
  //anti-windup, if last-time pid output reaches the limitation, this time there won't be any intergrations.
  if(u1v >= uv_max) {
    e_integration = 0;
  } else if (u1v <= uv_min) {
    e_integration = 0;
  }

  delta_uv = kpv*(e0v-e1v) + kiv*Ts*e_integration + kdv/Ts*(e0v-2*e1v+e2v); //incremental PID programming avoids integrations.there is another PID program called positional PID.
  u0v = u1v + delta_uv;  //this time's control output

  //output limitation
  saturation(u0v,uv_max,uv_min);
  
  u1v = u0v; //update last time's control output
  e2v = e1v; //update last last time's error
  e1v = e0v; // update last time's error
  return u0v;
}

// This is a PID controller for the current

float pidi(float pid_input){
  float e_integration;
  e0i = pid_input;
  e_integration=e0i;
  
  //anti-windup
  if(u1i >= ui_max){
    e_integration = 0;
  } else if (u1i <= ui_min) {
    e_integration = 0;
  }
  
  delta_ui = kpi*(e0i-e1i) + kii*Ts*e_integration + kdi/Ts*(e0i-2*e1i+e2i); //incremental PID programming avoids integrations.
  u0i = u1i + delta_ui;  //this time's control output

  //output limitation
  saturation(u0i,ui_max,ui_min);
  
  u1i = u0i; //update last time's control output
  e2i = e1i; //update last last time's error
  e1i = e0i; // update last time's error
  return u0i;
}


/*end of the program.*/

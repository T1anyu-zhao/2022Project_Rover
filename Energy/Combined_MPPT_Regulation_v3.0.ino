#include <Wire.h>
#include <INA219_WE.h>
#include <SPI.h>
#include <SD.h>

INA219_WE ina219; // this is the instantiation of the library for the current sensor

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

//****Regulator
const int chipSelect = 10; //hardwired chip select for the SD card
unsigned int loop_trigger;
unsigned int int_count = 0; // a variables to count the interrupts. Used for program debugging.
float Ts = 0.001; //1 kHz control frequency.
float current_measure;
float pwm_out;
boolean input_switch;
int state_num=0,next_state;
String dataString;

//****MPPT
//const int chipSelect = 10; //hardwired chip select for the SD card
//unsigned int loop_trigger_m;
unsigned int int_count_m = 0; // a variables to count the interrupts. Used for program debugging.
float Ts_m = 0.001; //1 kHz control frequency.
float current_measure_m;
float pwm_out_m;
boolean input_switch_m;
String dataString_m;

//****Regulator
int pwm_out = 0;           // SYSTEM PARAMETER -

int pwm_out_m = 0;           // SYSTEM PARAMETER -

//****Regulator
float vpd,vref,iL; // Measurement Variables
unsigned int sensorValue0,sensorValue1,sensorValue2,sensorValue3;  // ADC sample values declaration

//****MPPT
float vpd_m,vref_m,iL_m; // Measurement Variables
unsigned int sensorValue4,sensorValue5,sensorValue6,sensorValue7;  // ADC sample values declaration

//****Regulator
float vb = 0;
//float output_current = 0;


//****MPPT
float vb_m = 0;
float output_current_m = 0;
float Power_now = 0, Power_anc = 0, voltage_anc = 0;
float delta = 1;


void setup() {
  //Some General Setup Stuff

  Wire.begin(); // We need this for the i2c comms for the current sensor
  Wire.setClock(700000); // set the comms speed for i2c
  ina219.init(); // this initiates the current sensor
  Serial.begin(9600); // USB Communications


  //Check for the SD Card
  Serial.println("\nInitializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("* is a card inserted?");
    while (true) {} //It will stick here FOREVER if no SD is in on boot
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  if (SD.exists("SD_Test.csv")) { // Wipe the datalog when starting
    SD.remove("SD_Test.csv");
  }

  
  noInterrupts(); //disable all interrupts
  analogReference(EXTERNAL); // We are using an external analogue reference for the ADC

  //SMPS Pins
  pinMode(13, OUTPUT); // Using the LED on Pin D13 to indicate status //****for Regulator
  pinMode(12, OUTPUT); // ****for MPPT part 
  
  pinMode(2, INPUT_PULLUP); // Pin 2 is the input from the CL/OL switch //****for Regulator //****pin D2
  pinMode(3, INPUT_PULLUP); //****for MPPT part 
    
  pinMode(5, OUTPUT); // This is the PWM Pin //****for Regulator //****pin ~D5
  pinMode(6, OUTPUT); //****for MPPT part //****pin ~D6
  
  //LEDs on pin 7 and 8
  pinMode(7, OUTPUT); //error led //****for Regulator //****pin D7
  pinMode(4, OUTPUT); //****for MPPT part //****pin D4
  
  //pinMode(8, OUTPUT); //some digital output //****for Regulator //****pin D8
  //pinMode(11, OUTPUT); //****for MPPT part //****pin D11

  pinMode(9, OUTPUT); //relay //****for Regulator 

  //Analogue input, the battery voltage (also port B voltage)
  pinMode(A0, INPUT); //****for Regulator
  pinMode(A1, INPUT); //****for MPPT part //****this pin cannot be used for digital

  // TimerA0 initialization for 1kHz control-loop interrupt.
  TCA0.SINGLE.PER = 999; //
  TCA0.SINGLE.CMP1 = 999; //
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm; //16 prescaler, 1M.
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP1_bm;

  // TimerB0 initialization for PWM output
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //62.5kHz

  interrupts();  //enable interrupts.
  analogWrite(6, 120); //just a default state to start with

}

void loop() {
  if (loop_trigger == 1){ // FAST LOOP (1kHZ) (for changing duty cycle)
      state_num = next_state; //state transition

      // Make the initial sampling operations for the circuit measurements
  
      sensorValue0 = analogRead(A0); //sample Vb (output voltage)
      sensorValue4 = analogRead(A1); //****MPPT part
      
      //sensorValue2 = analogRead(A2); //sample Vref (desired output voltage)
      //sensorValue5 = analogRead(A5); //****MPPT part
      
      sensorValue2 = 5.0/(4.096/1023.0); //set Vref, intermediate voltage, roughly equal steps, can be changed
      sensorValue6 = 10.0/(4.096/1023.0); //****MPPT part
      
      sensorValue3 = analogRead(A3); //sample Vpd 
      sensorValue7 = analogRead(A4); //****MPPT part
      
      // Process the values so they are a bit more usable/readable
      // The analogRead process gives a value between 0 and 1023 
      // representing a voltage between 0 and the analogue reference which is 4.096V
  
      vb = sensorValue0 * (4.096 / 1023.0); // Convert the Vb sensor reading to volts (output voltage)
      vref = sensorValue2 * (4.096 / 1023.0); // Convert the Vref sensor reading to volts
      vpd = sensorValue3 * (4.096 / 1023.0); // Convert the Vpd sensor reading to volts

      //****for MPPT part
      vb_m = sensorValue4 * (4.096 / 1023.0); // Convert the Vb sensor reading to volts (output voltage)
      vref_m = sensorValue5 * (4.096 / 1023.0); // Convert the Vref sensor reading to volts
      vpd_m = sensorValue7 * (4.096 / 1023.0); // Convert the Vpd sensor reading to volts

      // The inductor current is in mA from the sensor so we need to convert to amps.
      // For open loop control the duty cycle reference is calculated from the sensor
      // differently from the Vref, this time scaled between zero and 1.

      pwm_out = sensorValue2 * (1.0 / 1023.0);  //****for Regulator

      pwm_out_m = sensorValue5 * (1.0 / 1023.0);  //****for MPPT part

      //Regulator
      if (vb < 4.50 || vb > 5.2) { //Checking for Error states (low or high input voltage)
          state_num = 2; //go directly to jail
          next_state = 2; // stay in jail
          digitalWrite(7, true); //turn on the red LED
          digitalWrite(9, HIGH); //relay off
          //pwm_out = 0; // no PWM?
      }
      
      current_measure = (ina219.getCurrent_mA()); // sample the inductor current (via the sensor chip)
      iL = current_measure/1000.0; //inductor current in Amperes
      
      //closed-loop buck
      current_limit = 2; // Buck has a higher current limit
      ev = vref - vb;  //voltage error at this time
      cv = pidv(ev);  //voltage pid
      cv = saturation(cv, current_limit, 0); //current demand saturation
      ei = cv-iL; //current error, not needed average inductor current?
      pwm_out = pidi(ei);  //current pid
      pwm_out = saturation(pwm_out,0.99,0.01);  //duty_cycle saturation
      analogWrite(5, (int)(255 - pwm_out * 255)); // write it out (inverting for the Buck here)
      int_count++; //count how many interrupts since this was last reset to zero
      loop_trigger = 0; //reset the trigger and move on with life



      //****MPPT part
      if (voltageOutput_m < 4.50) { //Checking for Error states (input voltage lower than battery minimum charging voltage)
          digitalWrite(4, true); //turn on the red LED
          pwm_out_m = 0; // no PWM
      }
      current_measure_m = (ina219.getCurrent_mA()); // sample the inductor current (via the sensor chip)
      iL_m = current_measure_m/1000.0; //inductor current in Amperes
      pwm_out_m = saturation(pwm_out_m, 0.99, 0.01); //duty_cycle saturation
      analogWrite(6, (int)(255 - pwm_out_m * 255)); // write it out (inverting for the Buck here)
      output_current_sum_m = output_current_m + iL_m; //summing current for averaging
      int_count_m++; //count how many interrupts since this was last reset to zero
      loop_trigger_m = 0; //reset the trigger and move on with life
  }
  
  if (int_count == 1000) { // SLOW LOOP (1Hz) (for MPPT and relay operation)
    input_switch = digitalRead(2); //get the OL/CL switch status
    switch (state_num) { // STATE MACHINE
      case 0:{ // Start state (no current, no LEDs)
        pwm_out = 0;
        if (input_switch == 1 || vb < 4.50 || vb > 5.2) { // if switch and voltages satisfy requirements, move to state 1
          next_state = 1;
          digitalWrite(9, LOW); //relay on
        } else { // otherwise stay put
          next_state = 0;
          digitalWrite(9, HIGH); //relay off
        }
        break;
      }
      case 1:{ // Charging State
        if (vb < 4.50 || vb > 5.2) { // if below 4.5V or above 5.2V go to state 2
          next_state = 2;
          digitalWrite(9, HIGH); //relay off          
        } else { // otherwise stay in the same state
          next_state = 1;
          digitalWrite(9, LOW); //relay on  
        }
        if(input_switch == 0){ // UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(9, HIGH); //relay off  
        }
        break;
      }
      case 2: { // ERROR (and charge termination) state RED led and no current 
        pwm_out = 0;
        next_state = 2; // Always stay here
        digitalWrite(7, true);
        digitalWrite(9, HIGH); //relay off
        if(input_switch == 0){ //UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(7,false);
        }
        break;
      }

      default :{ // Should not end up here ....
        Serial.println("Boop");
        pwm_out = 0;
        next_state = 2; // So if we are here, we go to error
        digitalWrite(7,true);
        digitalWrite(9, HIGH); //relay off
      }
      
    }
    
    dataString = String(state_num) + "," + String(vb) + "," + String(current_measure); //build a datastring for the CSV file
    Serial.println(dataString); // send it to serial as well in case a computer is connected
    File dataFile = SD.open("SD_Test.csv", FILE_WRITE); // open our CSV file
    if (dataFile){ //If we succeeded (usually this fails if the SD card is out)
      dataFile.println(dataString); // print the data
    } else {
      Serial.println("File not open"); //otherwise print an error
    }
    dataFile.close(); // close the file
    int_count = 0; // reset the interrupt count so we dont come back here for 1000ms



    //****MPPT
    input_switch_m = digitalRead(3); //get the OL/CL switch status
    
    output_current_m  = output_current_sum_m/1000; //averaging current
    
    //MPPT
    Power_now = vb_m * output_current_m; //assume current ripple small? Do we need to get an average value for inductor/output current? How? Saving measurements to SD cards then take out?
    if (Power_now > Power_anc)
    { if (vb_m > voltage_anc)
        pwm_out_m = pwm_out_m - delta;
      else
        pwm_out_m = pwm_out_m + delta;
    }
    else
    {
      if (vb_m > voltage_anc)
        pwm_out_m = pwm_out_m + delta;
      else
        pwm_out_m = pwm_out_m - delta;
    }
    Power_anc = Power_now;
    voltage_anc = vb_m;
    pwm_out_m = saturation(pwm_out_m, 0.99, 0.01); //duty_cycle saturation

    analogWrite(6, pwm_out_m);
    
    dataString_m = String(vb_m) + "," + String(output_current_m); //build a datastring for the CSV file
    Serial.println(dataString_m); // send it to serial as well in case a computer is connected
    
    //****Do we have to change sth here?
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

// Timer A CMP1 interrupt. Every 1000us the program enters this interrupt. This is the fast 1kHz loop
ISR(TCA0_CMP1_vect) {
  loop_trigger = 1; //trigger the loop when we are back in normal flow
  TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_CMP1_bm; //clear interrupt flag
}

float saturation( float sat_input, float uplim, float lowlim) { // Saturation function
  if (sat_input > uplim) sat_input = uplim;
  else if (sat_input < lowlim ) sat_input = lowlim;
  else;
  return sat_input;
}

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

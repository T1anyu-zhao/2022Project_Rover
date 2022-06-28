/*
 * Modified from Program (used in Power Lab) written by Yue Zhu (yue.zhu18@imperial.ac.uk) in July 2020.
 * pin6 is PWM output at 62.5kHz.
 * duty-cycle saturation is set as 1% - 99%
 * Control frequency is set as 1.25kHz. 
*/

#include <Wire.h>
#include <INA219_WE.h>
#include <SPI.h>
//#include <SD.h>

INA219_WE ina219; // this is the instantiation of the library for the current sensor

//Sd2Card card;
//SdVolume volume;
//SdFile root;

float pwm_out;
float vpd,vb,iL,current_mA,vin; // Measurement Variables
unsigned int sensorValue0,sensorValue3;  // ADC sample values declaration
float Ts=0.0008; //1.25 kHz control frequency. It's better to design the control period as integral multiple of switching period.
float gain = 0.075;
float gain_charging = 0.085;
float offset = 0;
boolean Boost_mode = 0;
boolean CL_mode = 0;

unsigned int int_count = 0;
String dataString;


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
  
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT); //relay
  TCB0.CTRLA=TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //62.5kHz
  analogWrite(6,120); 

  interrupts();  //enable interrupts.
  Wire.begin(); // We need this for the i2c comms for the current sensor
  ina219.init(); // this initiates the current sensor
  Wire.setClock(700000); // set the comms speed for i2c

  Serial.begin(9600);
  Serial.println("serial");

  digitalWrite(9, LOW); //relay on by default
  
}

 void loop() {
  if(loopTrigger) { // This loop is triggered, it wont run unless there is an interrupt

    //Serial.println("enter loop");
    
    digitalWrite(13, HIGH);   // set pin 13. Pin13 shows the time consumed by each control cycle. It's used for debugging.
    
    // Sample all of the measurements and check which control mode we are in
    sampling();
    CL_mode = digitalRead(3); // input from the OL_CL switch
    Boost_mode = digitalRead(2); // input from the Buck_Boost switch

    if (!Boost_mode){//somehow !Boost_mode corresponds to Boost mode
      if (CL_mode) { //Closed Loop Boost
          pwm_modulate(1); // This disables the Boost as we are not using this mode
          //Serial.println("enter loop");
      }else{ // Open Loop Boost
          pwm_modulate(1); // This disables the Boost as we are not using this mode
          //Serial.println("enter loop");
      }
    }else{      
      if (!CL_mode) { // Closed Loop Buck, not used
          //Serial.println("enter closed loop");
          pwm_modulate(0);
          
      }else{ // Open Loop Buck, using this only
          //Serial.println("enter open loop");
          Serial.println(vpd); 
          Serial.println(vb);
          Serial.println(vin);
          if(current_mA < 10){
              pwm_out = gain*vin + offset;
          }
          else{
              pwm_out = gain_charging*vin + offset;
          }
          pwm_out = saturation(pwm_out,0.99,0.01);
          pwm_modulate(pwm_out); // and send it out
          Serial.println("pwm_out");
          Serial.println(pwm_out);
          Serial.println("current_mA");
          Serial.println(current_mA);
      }
    }
    // closed loop control path

    digitalWrite(13, LOW);   // reset pin13.
    loopTrigger = 0;
    int_count++;
  }

  if (int_count == 1000) { // SLOW LOOP (1Hz) (for MPPT and relay operation)
    Serial.println("!!!!enter slow loop");

    if (vb < 3.5 || vb > 4.2) { //Checking for Error states (low or high battery input voltage) 
          digitalWrite(7,true); //turn on the red LED
          digitalWrite(9, HIGH); //relay off
          Serial.println("voltage out of range");
          Serial.println(vb);
          //Serial.println(vpd);
      }
    else{ //charging
          digitalWrite(7,false); //turn off the red LED
          digitalWrite(9, LOW); //relay on
          Serial.println("charging");
          Serial.println(vb);
      }
    
    //dataString = String(vb) + "," + String(current_mA); //build a datastring for the CSV file
    //Serial.println(dataString); // send it to serial as well in case a computer is connected
    //File dataFile = SD.open("SD_Test.csv", FILE_WRITE); // open our CSV file
    //if (dataFile){ //If we succeeded (usually this fails if the SD card is out)
      //dataFile.println(dataString); // print the data
    //} else {
      //Serial.println("File not open"); //otherwise print an error
    //}
    //dataFile.close(); // close the file
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
  sensorValue3 = analogRead(A3); //sample Vpd (buck input voltage through potential divider)
  current_mA = ina219.getCurrent_mA(); // sample the inductor current (via the sensor chip)

  // Process the values so they are a bit more usable/readable
  // The analogRead process gives a value between 0 and 1023 
  // representing a voltage between 0 and the analogue reference which is 4.096V
  
  vb = sensorValue0 * (4.096 / 1023.0); // Convert the Vb sensor reading to volts
  //vref = sensorValue2 * (4.096 / 1023.0); // Convert the Vref sensor reading to volts
  vpd = sensorValue3 * (4.096 / 1023.0); // Convert the buck input voltage sensor reading to volts, connected through a potential divider
  vin = vpd/0.3708;//converting to actual buck input voltage

  // The inductor current is in mA from the sensor so we need to convert to amps.
  iL = current_mA/1000.0;
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

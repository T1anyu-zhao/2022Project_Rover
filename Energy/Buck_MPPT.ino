#include <Wire.h>
#include <INA219_WE.h>
#include <SPI.h>
#include <SD.h>

INA219_WE ina219; // this is the instantiation of the library for the current sensor

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

const int chipSelect = 10; //hardwired chip select for the SD card
unsigned int loop_trigger;
unsigned int int_count = 0; // a variables to count the interrupts. Used for program debugging.
float Ts = 0.001; //1 kHz control frequency.
float current_measure;
float pwm_out=0;
boolean input_switch;
String dataString;

float vpd,vref,iL; // Measurement Variables
unsigned int sensorValue0,sensorValue1,sensorValue2,sensorValue3;  // ADC sample values declaration


float vb = 0;
float output_current = 0;
float output_current_sum = 0;
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
  pinMode(13, OUTPUT); // Using the LED on Pin D13 to indicate status
  pinMode(2, INPUT_PULLUP); // Pin 2 is the input from the CL/OL switch
  pinMode(6, OUTPUT); // This is the PWM Pin
  
  //LEDs on pin 7 and 8
  pinMode(7, OUTPUT); //error led
  pinMode(8, OUTPUT); //some digital output

  //Analogue input, the battery voltage (also port B voltage)
  pinMode(A0, INPUT);

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
      //assume to operate in Buck mode, maybe add code to identify boost mode (incorrect mode) later

      // Make the initial sampling operations for the circuit measurements
  
      sensorValue0 = analogRead(A0); //sample Vb (output voltage)
      //sensorValue2 = analogRead(A2); //sample Vref (desired output voltage)
      sensorValue2 = 3.3/(4.096/1023.0); //set Vref, intermediate voltage, roughly equal steps, can be changed
      sensorValue3 = analogRead(A3); //sample Vpd 
      
      // Process the values so they are a bit more usable/readable
      // The analogRead process gives a value between 0 and 1023 
      // representing a voltage between 0 and the analogue reference which is 4.096V
  
      vb = sensorValue0 * (4.096 / 1023.0); // Convert the Vb sensor reading to volts (output voltage)
      //vref = sensorValue2 * (4.096 / 1023.0); // Convert the Vref sensor reading to volts
      //vref = sensorValue2 * (4.096 / 1023.0); // Convert the Vref sensor reading to volts
      vpd = sensorValue3 * (4.096 / 1023.0); // Convert the Vpd sensor reading to volts

      // The inductor current is in mA from the sensor so we need to convert to amps.
      // For open loop control the duty cycle reference is calculated from the sensor
      // differently from the Vref, this time scaled between zero and 1.

     // pwm_out = sensorValue2 * (1.0 / 1023.0);  
  
      //if (vb < 3.3) { //Checking for Error states (input voltage lower than battery minimum charging voltage)
          //digitalWrite(7,true); //turn on the red LED
          //pwm_out = 0; // no PWM
      //}
      current_measure = (ina219.getCurrent_mA()); // sample the inductor current (via the sensor chip)
      iL = current_measure/1000.0; //inductor current in Amperes
      //pwm_out = saturation(pwm_out, 0.99, 0.01); //duty_cycle saturation
      //analogWrite(6, (int)(255 - pwm_out * 255)); // write it out (inverting for the Buck here)
      output_current_sum = output_current + iL; //summing current for averaging
      int_count++; //count how many interrupts since this was last reset to zero
      loop_trigger = 0; //reset the trigger and move on with life
  }

  if (int_count == 1000) { // SLOW LOOP (1Hz) (for MPPT and relay operation)
    input_switch = digitalRead(2); //get the OL/CL switch status
    
    output_current  = output_current_sum/1000; //averaging current
    
    //MPPT
    Power_now = vb * output_current; //assume current ripple small? Do we need to get an average value for inductor/output current? How? Saving measurements to SD cards then take out?
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
    pwm_out = saturation(pwm_out, 0.99, 0.01); //duty_cycle saturation

    analogWrite(6, (int)(255 - pwm_out * 255));
    
    dataString = String(vb) + "," + String(output_current); //build a datastring for the CSV file
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

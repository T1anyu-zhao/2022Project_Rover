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
float pwm_out;
float V_in;
boolean input_switch;
int state_num=0,next_state;
String dataString;

int pwm_out                   = 0,           // SYSTEM PARAMETER -

float voltageValue = 0;
float currentValue = 0;
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

float currentOutput = currentmeasure;

//POWER COMPUTATION - Through computation
float powerOutput     = voltageOutput*currentOutput;
float powerInput      = powerOutput/Efficiency;
float currentInput = powerInput/voltageInput;
float currentCharging         = 30.0000,     //   USER PARAMETER - Maximum Charging Current (A - Output)

//STATE OF CHARGE - Battery Percentage
float batteryPercent  = ((voltageOutput-voltageBatteryMin)/(voltageBatteryMax-voltageBatteryMin))*101;
float batteryPercent  = constrain(batteryPercent,0,100);

void loop() {
  if (loop_trigger == 1){ // FAST LOOP (1kHZ) (for changing duty cycle)
      state_num = next_state; //state transition
      V_in = analogRead(A0)*4.096/1.03; //check the battery voltage (1.03 is a correction for measurement error, you need to check this works for you)
      if (voltageOutput < 4500 || VoltageOutput > 5200) { //Checking for Error states (low or high input voltage)
          digitalWrite(7,true); //turn on the red LED
          pwm_out = 0; // no PWM
      }
      current_measure = (ina219.getCurrent_mA()); // sample the inductor current (via the sensor chip)
      pwm_out = saturation(pwm_out, 0.99, 0.01); //duty_cycle saturation
      analogWrite(6, (int)(255 - pwm_out * 255)); // write it out (inverting for the Buck here)
      int_count++; //count how many interrupts since this was last reset to zero
      loop_trigger = 0; //reset the trigger and move on with life
  }
  
  if (int_count == 1000) { // SLOW LOOP (1Hz) (for MPPT and relay operation)
    input_switch = digitalRead(2); //get the OL/CL switch status
    
    //MPPT
    Power_now = voltageValue * currentValue;
    if (Power_now > Power_anc)
    { if (voltageValue > voltage_anc)
        pwm = pwm - delta;
      else
        pwm = pwm + delta;
    }
    else
    {
      if (voltageValue > voltage_anc)
        pwm = pwm + delta;
      else
        pwm = pwm - delta;
    }
    Power_anc = Power_now;
    voltage_anc = voltageValue;
    pwm_out = saturation(pwm_out, 0.99, 0.01); //duty_cycle saturation

    analogWrite(6, pwm);
    
    dataString = String(state_num) + "," + String(V_in) + "," + String(current_measure); //build a datastring for the CSV file
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

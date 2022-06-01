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

float PPWM_margin             = 99.5000,    //  CALIB PARAMETER - Minimum Operating Duty Cycle for Predictive PWM (%)
float PWM_MaxDC               = 97.0000,    //  CALIB PARAMETER - Maximum Operating Duty Cycle (%) 90%-97% is good
int pwmMax                = 0,           // SYSTEM PARAMETER -?
int pwmMaxLimited         = 0,           // SYSTEM PARAMETER -?
int pwm_out                   = 0,           // SYSTEM PARAMETER -
int PPWM                  = 0,           // SYSTEM PARAMETER -
int pwmChannel            = 0,           // SYSTEM PARAMETER -

float voltageBatteryMin = 4500;
float voltageBatteryMax = 5200;
float powerInputPrev;
float voltageInputPrev;

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
  pinMode(9, OUTPUT); //relay?

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

//STATE OF CHARGE - Battery Percentage
float batteryPercent  = ((voltageOutput-voltageBatteryMin)/(voltageBatteryMax-voltageBatteryMin))*101;
float batteryPercent  = constrain(batteryPercent,0,100);

void loop() {
  if (loop_trigger == 1){ // FAST LOOP (1kHZ) (for changing duty cycle)
      state_num = next_state; //state transition
      V_in = analogRead(A0)*4.096/1.03; //check the battery voltage (1.03 is a correction for measurement error, you need to check this works for you)
      if (V_in < 4500 || V_in > 5200) { //Checking for Error states (low or high input voltage)
          state_num = 5; //go directly to jail
          next_state = 5; // stay in jail
          digitalWrite(7,true); //turn on the red LED
          digitalWrite(9, HIGH); //relay off
          pwm_out = 0; // no PWM
      }
      current_measure = (ina219.getCurrent_mA()); // sample the inductor current (via the sensor chip)
      switch (state_num){
        case 1:{
           Buck_2A(); //open loop buck to keep constant current (2A)
          }
        case 2:{
           Buck_OLCL(); //closd loop buck to keep constant voltage (ser Vref to 5V)
          }
      }
      //pwm_out = saturation(pwm_out, 0.99, 0.01); //duty_cycle saturation
      //analogWrite(6, (int)(255 - pwm_out * 255)); // write it out (inverting for the Buck here)
      int_count++; //count how many interrupts since this was last reset to zero
      loop_trigger = 0; //reset the trigger and move on with life
  }
  
  if (int_count == 1000) { // SLOW LOOP (1Hz) (for MPPT and relay operation)
    input_switch = digitalRead(2); //get the OL/CL switch status
    //MPPT
    if(currentOutput>currentCharging){pwm_out--;}                                      //Current Is Above → Decrease Duty Cycle
    else if(voltageOutput>voltageBatteryMax){pwm_out--;}                               //Voltage Is Above → Decrease Duty Cycle   
    else{                                                                          //MPPT ALGORITHM
      if(powerInput>powerInputPrev && voltageInput>voltageInputPrev)     {pwm_out--;}  //  ↑P ↑V ; →MPP  //D--
      else if(powerInput>powerInputPrev && voltageInput<voltageInputPrev){pwm_out++;}  //  ↑P ↓V ; MPP←  //D++
      else if(powerInput<powerInputPrev && voltageInput>voltageInputPrev){pwm_out++;}  //  ↓P ↑V ; MPP→  //D++
      else if(powerInput<powerInputPrev && voltageInput<voltageInputPrev){pwm_out--;}  //  ↓P ↓V ; ←MPP  //D--
      else if(voltageOutput<voltageBatteryMax)                           {pwm_out++;}  //  MP MV ; MPP Reached - 
      powerInputPrev   = powerInput;                                               //Store Previous Recorded Power
      voltageInputPrev = voltageInput;                                             //Store Previous Recorded Voltage        
    }   
      PWM_Modulation();                                                              //Set PWM signal to Buck PWM GPIO  
    switch (state_num) { // STATE MACHINE
      case 0:{ // Start state (no current, no LEDs)
        pwm_out = 0;
        if (input_switch == 1) { // if switch, move to state 1
          next_state = 1;
          digitalWrite(8,true);
          digitalWrite(9, LOW); //relay on
        } else { // otherwise stay put
          next_state = 0;
          digitalWrite(8,false);
          digitalWrite(9, HIGH); //relay off
        }
        break;
      }
      case 1:{ // PWM State 1 CC charging
        pwm_out = 0.5;
        if (V_in < 5000) { // if below 5V we go back to this state
          next_state = 1;
          digitalWrite(8,true);
          digitalWrite(9, LOW); //relay on          
        } else { // otherwise go to state 2
          next_state = 2;
          digitalWrite(8,false);
          digitalWrite(9, LOW); //relay on  
        }
        if(input_switch == 0){ // UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
          digitalWrite(9, HIGH); //relay off  
        }
        break;
      }
      case 2:{ // PWM State 2 CV charging
        pwm_out = 0.75;
        if (V_in >= 5000 && V_in <= 5200) { // If v in is above 5V and below 5.2V then we stay here
          next_state = 2;
          digitalWrite(8,false);
          digitalWrite(9, LOW); //relay on
        } 
        else { // Or move to PWM state 5
          next_state = 5;
          digitalWrite(8,false);
          digitalWrite(9, HIGH); //relay off
        }
        if(input_switch == 0){ // UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
          digitalWrite(9, HIGH); //relay off
        }
        break;        
      }
      case 5: { // ERROR (and charge termination) state RED led and no current 
        pwm_out = 0;
        next_state = 5; // Always stay here
        digitalWrite(7,true);
        digitalWrite(8,false);
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
        next_state = 5; // So if we are here, we go to error
        digitalWrite(7,true);
        digitalWrite(9, HIGH); //relay off
      }
      
    }
    
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

void predictivePWM(){                                                                //PREDICTIVE PWM ALGORITHM 
  if(voltageInput<=0){PPWM=0;}                                                       //Prevents Indefinite Answer when voltageInput is zero
  else{PPWM =(PPWM_margin*pwmMax*voltageOutput)/(100.00*voltageInput);}              //Compute for predictive PWM Floor and store in variable
  PPWM = constrain(PPWM,0,pwmMaxLimited);
}   

void PWM_Modulation(){
  if(output_Mode==0){pwm_out = constrain(pwm_out,0,pwmMaxLimited);}                          //PSU MODE PWM = PWM OVERFLOW PROTECTION (limit floor to 0% and ceiling to maximim allowable duty cycle)
  else{
    predictivePWM();                                                                 //Runs and computes for predictive pwm floor
    pwm_out = constrain(pwm_out,PPWM,pwmMaxLimited);                                         //CHARGER MODE PWM - limit floor to PPWM and ceiling to maximim allowable duty cycle)                                       
  } 
  ledcWrite(pwmChannel,PWM);                                                         //Set PWM duty cycle and write to GPIO when buck is enabled
  //buck_Enable();                                                                     //Turn on MPPT buck (IR2104)
}

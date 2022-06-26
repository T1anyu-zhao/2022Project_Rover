#include <iostream>
#include <iterator>
#include <list>
#include <Arduino.h>
#include <Robojax_L298N_DC_motor.h>
#include "SPI.h"
using namespace std;

// motor 1 right wheel
#define CHA 0
#define ENA 4 // this pin must be PWM enabled pin if Arduino board is used
#define IN1 15
#define IN2 2
// motor 2 settings left wheel
#define IN3 16
#define IN4 21
#define ENB 22 // this pin must be PWM enabled pin if Arduino board is used
#define CHB 1
const int CCW = 2; // do not change
const int CW = 1;  // do not change
#define motor1 1   // do not change
#define motor2 2   // do not change

// these pins may be different on different boards

#define PIN_SS 5
#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_SCK 18

#define PIN_MOUSECAM_RESET 17
#define PIN_MOUSECAM_CS 5

#define ADNS3080_PIXELS_X 30
#define ADNS3080_PIXELS_Y 30

#define ADNS3080_PRODUCT_ID 0x00
#define ADNS3080_REVISION_ID 0x01
#define ADNS3080_MOTION 0x02
#define ADNS3080_DELTA_X 0x03
#define ADNS3080_DELTA_Y 0x04
#define ADNS3080_SQUAL 0x05
#define ADNS3080_PIXEL_SUM 0x06
#define ADNS3080_MAXIMUM_PIXEL 0x07
#define ADNS3080_CONFIGURATION_BITS 0x0a
#define ADNS3080_EXTENDED_CONFIG 0x0b
#define ADNS3080_DATA_OUT_LOWER 0x0c
#define ADNS3080_DATA_OUT_UPPER 0x0d
#define ADNS3080_SHUTTER_LOWER 0x0e
#define ADNS3080_SHUTTER_UPPER 0x0f
#define ADNS3080_FRAME_PERIOD_LOWER 0x10
#define ADNS3080_FRAME_PERIOD_UPPER 0x11
#define ADNS3080_MOTION_CLEAR 0x12
#define ADNS3080_FRAME_CAPTURE 0x13
#define ADNS3080_SROM_ENABLE 0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER 0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER 0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER 0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER 0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER 0x1d
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER 0x1e
#define ADNS3080_SROM_ID 0x1f
#define ADNS3080_OBSERVATION 0x3d
#define ADNS3080_INVERSE_PRODUCT_ID 0x3f
#define ADNS3080_PIXEL_BURST 0x40
#define ADNS3080_MOTION_BURST 0x50
#define ADNS3080_SROM_LOAD 0x60

#define ADNS3080_PRODUCT_ID_VAL 0x17

float total_x = 0;
float total_y = 0;

float total_x1 = 0;
float total_y1 = 0;

int x = 0;
int y = 0;

int a = 0;
int b = 0;

float distance_x = 0;
float distance_y = 0;

volatile byte movementflag = 0;
volatile int xydat[2];

Robojax_L298N_DC_motor robot(IN1, IN2, ENA, CHA, IN3, IN4, ENB, CHB);
// for two motors with debug information
// Robojax_L298N_DC_motor robot(IN1, IN2, ENA, CHA, IN3, IN4, ENB, CHB, true);

boolean reachDestination = false;
int destination_x;
int destination_y;
int destination_x_prev;
int destination_y_prev;

// Values for PID
float kp = 0.35;
float ki = 0.5;
float kd = 0;
float controlSignal = 0;
float prevT = 0;       // for calculating delta t
float prevE = 0;       // for calculating the derivative (edot)
float eIntegral = 0;   // integral error
float currT = 0;       // time in the moment of calculation
float deltaT = 0.0008; // time difference
float error = 0;       // error
float edot = 0;        // derivative (de/dt)
char mode = 'C';
char motion = 'S';
int desire_x, desire_y;
float desire_angle;
float dy_mm = 0, dx_mm = 0;
float current_x = 0, current_y = 0;
int constant = 0;
float current_angle = 0;
int r = 130;
const float pi = 3.14159265359;
float error_angle = 0;

// Values for timer
long lastTrigger = 0;
boolean startTimer = false;

// Values for the default path
int g = 1, h = 1, j = 1, k = 1;
int i = 1;
bool go_straight_1 = false, go_straight_2 = false, go_straight_3 = false, go_straight_4 = true;
int length_r = 200, width_r = 200;
int length_c = 3600, width_c = 2400;
int increment = 0;
int p = 0;

// Values for detection
float angle_o, d_o, d_o_v;
int op_from_v;
bool update_destination_xy_1;

// Values for the coordinate_list
bool check_done;//also in mode 'C'
int follow = 1;
typedef list<float> Coordinate_list;
Coordinate_list coordinate_list;
Coordinate_list::iterator it;
float x1, x2, y_1, y2;
float del_x;
float OH, base;
float final_destination_x, final_destination_y;
bool set_line_end = true;
bool send_coordinate;

const int STATE_DELAY = 1000;
int randomState = 0;

enum States
{
    STOP,
    ANGLE,
    DISTANCE
};
States State;

////FUNCTIONS////
void IRAM_ATTR detectsCLK()
{
    Serial.println("SCLK rising");
    // digitalWrite(20, HIGH); // pin20 for debug only
    // bool startTimer = true;
    lastTrigger = micros();
}

int convTwosComp(int b)
{
    // Convert from 2's complement
    // only negative number will go through this
    if (b & 0x80)
    {
        b = -1 * ((b ^ 0xff) + 1);
    }
    return b;
}

float convertTodegree(float angle_radians)
{
    return angle_radians * (180 / 3.14159265359);
}

int tdistance = 0;

void mousecam_reset()
// reset the sensor to restore it to normal motion
{
    digitalWrite(PIN_MOUSECAM_RESET, HIGH);
    delay(1); // reset pulse >10us
    digitalWrite(PIN_MOUSECAM_RESET, LOW);
    delay(35); // 35ms from reset to functional
}

void mousecam_write_reg(int reg, int val)
{
    digitalWrite(PIN_MOUSECAM_CS, LOW);
    SPI.transfer(reg | 0x80); // MSB is '1' to indicate data direction
    SPI.transfer(val);
    digitalWrite(PIN_MOUSECAM_CS, HIGH);
    delayMicroseconds(50);
}

int mousecam_read_reg(int reg)
{
    digitalWrite(PIN_MOUSECAM_CS, LOW);
    SPI.transfer(reg);
    delayMicroseconds(75);
    int ret = SPI.transfer(0xff); // send a dummy byte to the EEPROM for the purpose of shifting the data out
    digitalWrite(PIN_MOUSECAM_CS, HIGH);
    delayMicroseconds(1);
    return ret;
}

int mousecam_init()
{
    pinMode(PIN_MOUSECAM_RESET, OUTPUT);
    pinMode(PIN_MOUSECAM_CS, OUTPUT);

    digitalWrite(PIN_MOUSECAM_CS, HIGH);

    mousecam_reset();

    int pid = mousecam_read_reg(ADNS3080_PRODUCT_ID);
    if (pid != ADNS3080_PRODUCT_ID_VAL)
    {
        return -1;
    }

    // turn on sensitive mode
    mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);
    return 1;
}

struct MD
{
    byte motion;
    char dx, dy;
    byte squal;
    word shutter;
    byte max_pix;
};

void mousecam_read_motion(struct MD *p)
{
    digitalWrite(PIN_MOUSECAM_CS, LOW);
    SPI.transfer(ADNS3080_MOTION_BURST);
    delayMicroseconds(75);
    p->motion = SPI.transfer(0xff);
    p->dx = SPI.transfer(0xff);
    p->dy = SPI.transfer(0xff);
    p->squal = SPI.transfer(0xff);
    p->shutter = SPI.transfer(0xff) << 8;
    p->shutter |= SPI.transfer(0xff);
    p->max_pix = SPI.transfer(0xff);
    digitalWrite(PIN_MOUSECAM_CS, HIGH);
    delayMicroseconds(5);
}

// pdata must point to an array of size ADNS3080_PIXELS_X x ADNS3080_PIXELS_Y
// you must call mousecam_reset() after this if you want to go back to normal operation
int mousecam_frame_capture(byte *pdata)
{
    mousecam_write_reg(ADNS3080_FRAME_CAPTURE, 0x83);
    // write 0x83 to this register will cause the next available complete 1 2/3 frames of pixel values to be stored to SROM RAM
    // write to this register is required before using the Frame Capture brust mode to read the pixel values

    digitalWrite(PIN_MOUSECAM_CS, LOW);

    SPI.transfer(ADNS3080_PIXEL_BURST); // used for high-speed access to all the pixel values from one and 2/3 complete frame
    delayMicroseconds(50);

    int pix;
    byte started = 0;
    int count;
    int timeout = 0;
    int ret = 0;
    for (count = 0; count < ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y;)
    {
        pix = SPI.transfer(0xff);
        delayMicroseconds(10);
        if (started == 0)
        {
            if (pix & 0x40) // the first pixel of the first frame has bit 6 set to 1 as a start-of-frame marker
                started = 1;
            else
            {
                timeout++;
                if (timeout == 100)
                {
                    ret = -1;
                    break;
                }
            }
        }
        if (started == 1)
        {
            pdata[count++] = (pix & 0x3f) << 2; // scale to normal grayscale byte range
        }
    }

    digitalWrite(PIN_MOUSECAM_CS, HIGH);
    delayMicroseconds(14);

    return ret;
}

float calculate_desried_angle(float destinationx, float destinationy)
{
    float desire_a;
    float change_x = destinationx - current_x;
    float change_y = destinationy - current_y;
    Serial.println("Change_x = " + String(change_x));
    Serial.println("Change_y = " + String(change_y));
    Serial.println("Current_x = " + String(current_x));
    Serial.println("Current_y = " + String(current_y));
    if (change_x > 0 && change_y > 0)
    {
        desire_a = atan(change_x / change_y); // 1st quadrant
    }
    else if (change_x < 0 && change_y > 0)
    {
        desire_a = atan(-change_x / change_y); // 2nd quadrant
    }
    else if (change_x < 0 && change_y < 0)
    {
        desire_a = atan(-change_y / change_x) - 90 * (pi / 180); // 3rd quadrant
    }
    else if (change_x > 0 && change_y < 0)
    {
        desire_a = atan(-change_y / change_x) + 90 * (pi / 180); // 4th quadrant
    }
    else if (change_x > 0 && change_y == 0)
    {
        Serial.println("Rotate right 90");
        desire_a = pi / 2; // turn right 90 degree
    }
    else if (change_x < 0 && change_y == 0)
    {
        Serial.println("Rotate left 90");
        desire_a = -pi / 2; // turn left 90 degree
    }
    else if (change_x == 0 && change_y > 0)
    {
        desire_a = 0; // go straight
    }
    else if (change_x == 0 && change_y < 0)
    {
        desire_a = pi; // go straight
    }
    Serial.println("Desire angle = " + String(convertTodegree(desire_a)));

    return desire_a;
}

float calculatePID(float error)
{
    // Determining the elapsed time
    /*
    currT = micros();                     // current time
    deltaT = (currT - prevT) / 1000000.0; // time difference in seconds
    prevT = currT;                        // save the current time for the next iteration to get the time difference
    Serial.println("Delt T = " + String(deltaT));
    */
    //---
    edot = (error - prevE) / deltaT; // edot = de/dt - derivative term

    eIntegral = eIntegral + (error * deltaT); // integral term - Newton-Leibniz, notice, this is a running sum!

    controlSignal = (kp * error) + (kd * edot) + (ki * eIntegral); // final sum, proportional term also calculated here

    prevE = error; // save the error for the next iteration to get the difference (for edot)
    Serial.println("Control signal = " + String(controlSignal));
    Serial.println("\n ");
    return controlSignal;
}

bool distance_control(int desired_x, int desired_y)
{
    float error_distance = sqrt(pow((current_x - desired_x), 2) + pow((current_y - desired_y), 2));
    float control_distance = calculatePID(error_distance);
    bool forward = (((current_y + error_distance * cos(current_angle) < desired_y + 5) && (current_y + error_distance * cos(current_angle) > desired_y - 5))) ? true : false;
    int speed = 0;
    // Determine speed and direction based on the value of the control signal
    // direction
    if (error_distance > 30 && !forward) // move backward
    {
        Serial.println("Move backward");
        constant = -1; // move backwards, when calculate current position should minus the distance it traveld
        if (error_distance > 1000)
        { //
            speed = fabs(control_distance) * 0.03;
            if (speed < 15)
            {
                speed = 15;
            }
            robot.rotate(motor1, speed, CW);
            robot.rotate(motor2, speed, CW);
        }
        else if (error_distance > 500)
        {
            speed = fabs(control_distance) * 0.2;
            if (speed < 15)
            {
                speed = 15;
            }
            robot.rotate(motor1, speed, CW);
            robot.rotate(motor2, speed, CW);
        }
        else if (error_distance > 50)
        { // move backward with 40 speed when error distance in range 100-50
            robot.rotate(motor1, 30, CW);
            robot.rotate(motor2, 30, CW);
        }
        else // move backward with 20 speed when error distance less than 50
        {
            robot.rotate(motor1, 18, CW);
            robot.rotate(motor2, 18, CW);
        }
    }
    else if (error_distance > 30 && forward) // move forward
    {
        Serial.println("Move forward");
        constant = 1; // move backwards, when calculate current position should minus the distance it traveld
        if (error_distance > 900)
        { //>1000
            speed = fabs(control_distance) * 0.03;
            if (speed < 15)
            {
                speed = 15;
            }
            robot.rotate(motor1, speed, CCW);
            robot.rotate(motor2, speed, CCW);
        }
        else if (error_distance > 500)
        {
            speed = fabs(control_distance) * 0.2;
            if (speed < 15)
            {
                speed = 15;
            }
            robot.rotate(motor1, speed, CCW);
            robot.rotate(motor2, speed, CCW);
        }
        else if (error_distance > 100)
        {
            robot.rotate(motor1, 40, CCW);
            robot.rotate(motor2, 40, CCW);
        }

        else if (error_distance > 50)
        { // move backward with 40 speed when error distance in range 100-50
            robot.rotate(motor1, 30, CCW);
            robot.rotate(motor2, 30, CCW);
        }
        else // move backward with 20 speed when error distance less than 50
        {
            robot.rotate(motor1, 18, CCW);
            robot.rotate(motor2, 18, CCW);
        }
    }
    else // == 0, stop/break
    {
        robot.brake(1);
        robot.brake(2);
        return true; // distance_control done
        Serial.println("Distance_control done");
    }

    float moved_distance = sqrt(pow(dx_mm, 2) + pow(dy_mm, 2));
    current_y = current_y + (constant * moved_distance) * cos(current_angle);
    current_x = current_x + (constant * moved_distance) * sin(current_angle);
    Serial.println("Current_angle: " + String(convertTodegree(current_angle)));
    Serial.println("Current_y: " + String(current_y));
    Serial.println("Current_x: " + String(current_x));
    return false;
}

bool angle_control(float desired_angle, float dy, float dx, float *current_an)
{
    error_angle = desired_angle - convertTodegree(current_angle);
    float control_angle = calculatePID((error_angle));
    Serial.println("Error angle: " + String(error_angle));
    Serial.println("Desire angle in angle control: " + String(desired_angle));
    int speed = 0;
    // Determine speed and direction based on the value of the control signal
    // direction
    if (error_angle >= 2) // turn right
    {
        constant = 1;
        Serial.println("Rotate right");
        if (abs(error_angle) > 90)
        {
            speed = fabs(control_angle) * 0.5;
            if (speed < 15)
            {
                speed = 15;
            }
            robot.rotate(motor1, speed, CCW);
            robot.rotate(motor2, speed, CW);
        }
        else if (abs(error_angle) > 10)
        { // move backward with 40 speed when error distance in range 100-50
            robot.rotate(motor1, 20, CCW);
            robot.rotate(motor2, 20, CW);
        }
        else // move backward with 20 speed when error distance less than 50
        {
            robot.rotate(motor1, 15, CCW);
            robot.rotate(motor2, 15, CW);
        }
    }
    else if (error_angle <= -2) // turn left
    {
        constant = -1;
        Serial.println("Rotate left");
        if (abs(error_angle) > 90) // greater than 90
        {
            speed = fabs(control_angle) * 0.5;
            if (speed < 15)
            {
                speed = 15;
            }
            robot.rotate(motor1, speed, CW);
            robot.rotate(motor2, speed, CCW);
        }
        else if (abs(error_angle) > 10) // 10-30
        {                               // move backward with 40 speed when error distance in range 100-50
            robot.rotate(motor1, 20, CW);
            robot.rotate(motor2, 20, CCW);
        }
        else // <5
        {
            robot.rotate(motor1, 15, CW);
            robot.rotate(motor2, 15, CCW);
        }
    }
    else // stop/break
    {
        // robot.brake(1);
        // robot.brake(2);
        return true; // angle_control done
        Serial.println("Angle_control done");
    }

    float moved_distance = sqrt(pow(dx, 2) + pow(dy, 2));
    float moved_angle = asin(moved_distance / (2 * r)) * 2;
    float angle = *current_an;
    angle = angle + constant * moved_angle;
    angle = (angle > 3.14159265359) ? (angle - 3.14159265359) : angle;
    angle = (angle < -3.14159265359) ? (angle + 3.14159265359) : angle;
    *current_an = angle;
    Serial.println("Current angle: " + String(convertTodegree(angle))); // easier to read when convert to degree
    return false;
}

void rotate360()
{
    float desired_angle = pi / 2;
    bool turn_done;
    while (desire_angle < 2 * pi)
    {
        turn_done = angle_control(convertTodegree(desire_angle), dy_mm, dx_mm, &current_angle);
        desired_angle = desired_angle + pi;
    }
}

void rotate_v(bool found, bool detect_d_o, float &send_x, float &send_y){
  //this function is called when the obstacle is detected
  //but no reroute is required
  if(found){
    float d_o = detect_d_o;
    send_coordinate = 1;
    send_x = current_x + sin(current_angle);
    send_y = current_y + cos(current_angle);
    send_coordinate = 0;
    mode = 'C';
    //the destination is given by either the default path or the deflection
    //rotate back to the original driving angle
  }
  else{
    if(-88 < angle_o && angle_o < -2){
      mode = 'V';
      motion = 'L';
    }
    else if(88 > angle_o && angle_o > 2){
      mode= 'V';
      motion = 'R';
    }
  }
}

bool check_detect(int vision_output){

  //would be extended to check whether the obstacle has been marked
  if(vision_output = 0){
    return false;
  }

  else{
    return true;
  }
}


bool check_block(int binary_output, float& angle_o, float& d_o_v){

  switch(binary_output){
    case 0:
    //check_divert_done = true;
    return false;

    case 1:
    update_destination_xy_1 = false;

    //d_o and angle_o would be input
    d_o = d_o_v + length_r;
    //d_o_v must be larger than 30cm
 
    float del_x = sin(angle_o)*d_o_v;
    float del_y = cos(angle_o)*d_o_v;

    if(del_x < width_r) { // block the way
      return true;

    }
  }
  return false;
}

bool generate_coordinates(){

  if(check_block(op_from_v, angle_o, d_o_v)){

    //float d_o;//  =; distance from the rover to the obstacle detected, given from the Vision
    //float angle_o;// =; angle between the current 'go straight' path and the line connected the rover and the obstacle

    del_x = sin(angle_o + current_angle)*d_o;
    OH = cos(angle_o)*d_o;

    float safe_d = 200;
    //desire angle becomes: convertTodegree(asin(change_x / change_y)) 
    float safe_angle = asin(safe_d / d_o);
    
    float divert_angle, desire_angle;
    if(current_angle < 0 && !go_straight_3 && (i != p)){// divert from left hand side
      divert_angle = (-1)*angle_o + safe_angle; // must be positive according to the setting scheme
      desire_angle = current_angle - divert_angle;
    }

    else{
      divert_angle = angle_o + safe_angle;
      desire_angle = current_angle + divert_angle;
    }
 
    //float desire_angle = current_angle + divert_angle;
    
    float magnitude = OH / cos(divert_angle);
    float base = 2*OH;
    
    //add one intermediate point
    float x1 = current_x + magnitude * sin(desire_angle);
    float y_1 = current_y + magnitude * cos(desire_angle);

    
    //add second point to finish divesion
    //float x2 = current_x + base * sin(current_angle);
    //float y2 = current_y + base * cos(current_angle);

    return true;
    
    //int order_position;// which pair of coordinates 
  }
  
  return false;
}

void set_destination(){//执行设置目的地以及要求行动

  for(int count_pair; coordinate_list.size()/2>0; ){
    it = coordinate_list.begin();
    destination_x = *it;
    advance(it,follow);
    destination_y = *it;

    it = coordinate_list.end();
    final_destination_y = *--it;
    final_destination_x = *--it;
  }
  //mode = 'C';
}

void edit_list(){

  if(check_done){
    it = coordinate_list.begin();
    coordinate_list.erase(it);
    advance(it,1);
    coordinate_list.erase(it);
    check_done = false;

    //for debug
    std::cout<<"coordinate_list.begin()----coordinate_list.end():" << endl;
    for (it; it != coordinate_list.end(); ++it){
      std::cout << *it << " ";
    }
    std::cout << endl;

    return;
  
  }

  float d2destination = sqrt(pow(destination_x-current_x,2) + pow(destination_y-current_y,2));

  if(generate_coordinates){

    if(current_angle*(destination_x-final_destination_x)<0){
      coordinate_list.push_front(y_1);
      coordinate_list.push_front(x1);
      //add only one coordinate
      
      it = coordinate_list.end();
      coordinate_list.erase(it);
      coordinate_list.erase(it);
      //last coordinate deleted!
      //then the rover get back to the normal path gradually
      if(coordinate_list.empty()){
        reachDestination = true;
      }
      return;

    }

    if((go_straight_1 || go_straight_3) && del_x < 30){
      coordinate_list.push_front(y_1);
      coordinate_list.push_front(x1);
    }

    if(2*OH > d2destination){
      it = coordinate_list.begin();
      coordinate_list.erase(it);
      advance(it,follow);
      coordinate_list.erase(it);
    }

    coordinate_list.push_front(y2);
    coordinate_list.push_front(x2);
    coordinate_list.push_front(y_1);
    coordinate_list.push_front(x1);
  }
  return;
}




///////Set up///////
void setup()
{
    Serial.begin(9600);

    pinMode(PIN_SS, OUTPUT);
    pinMode(PIN_MISO, INPUT);
    pinMode(PIN_MOSI, OUTPUT);
    pinMode(PIN_SCK, OUTPUT);

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV32);
    SPI.setDataMode(SPI_MODE3);
    SPI.setBitOrder(MSBFIRST);

    robot.begin();

    // pinMode(PIN_SCK, INPUT_PULLUP);

    // attachInterrupt(digitalPinToInterrupt(PIN_SCK), detectsCLK, RISING);

    if (mousecam_init() == -1)
    {
        Serial.println("Mouse cam failed to init");
        while (1)
            ;
    }
}

char asciiart(int k)
{
    static char foo[] = "WX86*3I>!;~:,`. ";
    return foo[k >> 4];
}

byte frame[ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y];

/////Loop//////////
void loop()
{  

  //--------------------------------set destination-----------------------------------
  /*main algorithm
  set the destination as the end of each line section
  check_detect() ---->  ()  
  check_block()  ---->  rotate_v for send the coordinate
  edit_list()
  ---->let the iterator points to the first coordinate in the front of the list

  */

#if 0
/*
    if(movementflag){

    tdistance = tdistance + convTwosComp(xydat[0]);
    Serial.println("Distance = " + String(tdistance));
    movementflag=0;
    delay(3);
    }

  */
  // if enabled this section grabs frames and outputs them as ascii art

  if(mousecam_frame_capture(frame)==0)
  {
    int i,j,k;
    for(i=0, k=0; i<ADNS3080_PIXELS_Y; i++)
    {
      for(j=0; j<ADNS3080_PIXELS_X; j++, k++)
      {
        Serial.print(asciiart(frame[k]));
        Serial.print(' ');
      }
      Serial.println();
    }
  }
  Serial.println();
  delay(250);

#else
    //mode_C;
    if(set_line_end){

    if(!go_straight_1 && go_straight_4){

        cout << "go_straight_1------------------" << endl;
        //cout << "destination(x,y) is: (" << destination_x << ", " << destination_y << ")" << endl;

        if(g == 1){
          cout << "g = " << g << endl;
          destination_x = increment;
          destination_y = length_c - length_r - 500;
 
          cout << "destination(x,y) is: (" << destination_x << ", " << destination_y << ")" << endl;

          coordinate_list.push_front(destination_x);
          coordinate_list.push_back(destination_y);

          it = coordinate_list.begin();
          destination_x = *it;
          advance(it,follow);
          destination_y = *it;

          cout << "using iterator destination(x,y) is: (" << destination_x << ", " << destination_y << ")" << endl;
          //g++;
            go_straight_1 = true;
            reachDestination = false;
            coordinate_list.clear();
          
        }
    

        if(reachDestination){
            go_straight_1 = true;
            reachDestination = false;
            // break;
            coordinate_list.clear();
            delay(250);
        }


    }

    else if(go_straight_1 && !go_straight_2){

        cout << "go_straight_2------------------" << endl;
            
        if(h == i){
          increment = increment + 300;
          destination_x = increment;
          destination_y = length_c-500-length_r;

          coordinate_list.push_front(destination_x);
          coordinate_list.push_back(destination_y);

          it = coordinate_list.begin();
          destination_x = *it;
          advance(it,follow);
          destination_y = *it;

          cout << "using iterator destination(x,y) is: (" << destination_x << ", " << destination_y << ")" << endl;
          //g++;
            go_straight_1 = true;
            reachDestination = false;
            coordinate_list.clear();
        }


        cout << "destination(x,y) is: (" << destination_x << ", " << destination_y << ")" << endl;
        //mode = 'C';
        if(reachDestination){
          go_straight_2 = true;
          reachDestination = false;
          coordinate_list.clear();
        }
          h++;
        }

    else if(go_straight_2 && !go_straight_3){

        cout << "go_straight_3------------------" << endl;
            
          if(j == 1){
            destination_x = increment;
            destination_y = 500+length_r; 
          }
          //mode = 'C';
          cout << "destination(x,y) is: (" << destination_x << ", " << destination_y << ")" << endl;
          if(reachDestination){
            go_straight_3 = true;
            reachDestination = false;
            coordinate_list.clear();
            delay(250);
          }
          j++;
      }

    else if(go_straight_3 && !go_straight_4){

        cout << "go_straight_4------------------" << endl;

        if(i == p && j == 1){
          destination_x = 0;
          destination_y = 0; //origin
        }
          //mode = 'C';
          cout << "destination(x,y) is: (" << destination_x << ", " << destination_y << ")" << endl;
          if(reachDestination){
            go_straight_4 = true;
            cout << "Detection completed." << endl;
            reachDestination = true;
            robot.brake(1);
            robot.brake(2);
            delay(2500);
          }
        

          if(g == 1){
            increment = increment + 300;
            destination_x = increment;
            destination_y = 500+length_r;
            //mode = 'C';
            cout << "destination(x,y) is: (" << destination_x << ", " << destination_y << ")" << endl;
            if(reachDestination){
              go_straight_4 = true;
              reachDestination = false;
              go_straight_1 = false;
              coordinate_list.clear();
              delay(250);
              h = g = j =1;
              i++; 
            } 
            g++;
            //return;
            }
      }
  }
  
  //check_detect(op_from_v);
  //check_block(op_from_v, angle_o, d_o_v);
  //bool generate_done = generate_coordinates;
  //if(generate_done){
   // edit_list;
  //}
  //set_destination;
  //at the end of loop, the destination has been determined.
  cout << " we set destination(x,y) is: (" << destination_x << ", " << destination_y << ")" << endl; 
    
    Serial.println("Another loop");
    // if enabled this section produces a bar graph of the surface quality that can be used to focus the camera
    // also drawn is the average pixel value 0-63 and the shutter speed and the motion dx,dy.

    int val = mousecam_read_reg(ADNS3080_PIXEL_SUM); // find the avrage pixel value
    MD md;
    mousecam_read_motion(&md);
    for (int i = 0; i < md.squal / 4; i++) // number of features = SQUAL register value *4
        Serial.print('*');
    Serial.print(' ');
    Serial.print((val * 100) / 351); // calculate average pixel
    Serial.print(' ');
    Serial.print(md.shutter);
    Serial.print(" (");
    Serial.print((int)md.dx);
    Serial.print(',');
    Serial.print((int)md.dy);
    Serial.println(')');

    // Serial.println(md.max_pix);// maximum = 63
    delay(100);

    distance_x = md.dx; // convTwosComp(md.dx);
    distance_y = md.dy; // convTwosComp(md.dy);

    total_x1 = total_x1 + distance_x;
    total_y1 = total_y1 + distance_y;

    total_x = (total_x1 / 157) * 10;
    total_y = (total_y1 / 157) * 10;

    dx_mm = (distance_x / 157) * 10; // convert distance to mm
    dy_mm = (distance_y / 157) * 10;

    Serial.print('\n');

    Serial.println("Distance_x = " + String(total_x));

    Serial.println("Distance_y = " + String(total_y));

    Serial.println("dx = " + String(dx_mm));

    Serial.println("dy = " + String(dy_mm));
    Serial.print('\n');

    delay(250);

    Serial.println("mode = " + String(mode));

    
    if (mode == 'V')
    {
        if (motion == 'F')
        {
            constant = 1;
            robot.rotate(motor1, 25, CCW);
            robot.rotate(motor2, 25, CCW);
        }
        else if (motion == 'B')
        {
            constant = -1;
            robot.rotate(motor1, 25, CW);
            robot.rotate(motor2, 25, CW);
        }
        else if (motion == 'L')
        {
            constant = -1;
            robot.rotate(motor1, 25, CW);
            robot.rotate(motor2, 25, CCW);
        }
        else if (motion == 'R')
        {
            constant = 1;
            robot.rotate(motor1, 25, CCW);
            robot.rotate(motor2, 25, CW);
        }
        else if (motion == 'S')
        {
            robot.brake(1);
            robot.brake(2);
        }
    }

    if (motion == 'F' || motion == 'B')
    {
        float moved_distance = sqrt(pow(dx_mm, 2) + pow(dy_mm, 2));
        current_y = current_y + (constant * moved_distance) * cos(current_angle);
        current_x = current_x + (constant * moved_distance) * sin(current_angle);
        Serial.println("Current_y: " + String(current_y));
        Serial.println("Current_x: " + String(current_x));
        Serial.print('\n');
    }
    else if (motion == 'L' || motion == 'R')
    {
        float moved_distance = sqrt(pow(dx_mm, 2) + pow(dy_mm, 2));
        float moved_angle = asin(moved_distance / (2 * r)) * 2 * (180 / 3.14159265359);
        current_angle = current_angle + constant * moved_angle;
        current_angle = (current_angle > 3.14159265359) ? (current_angle - 3.14159265359) : current_angle;
        current_angle = (current_angle < -3.14159265359) ? (current_angle + 3.14159265359) : current_angle;
        Serial.println("Current angle: " + String(current_angle));
        Serial.print('\n');
    }

    Serial.println("Destination_x = " + String(destination_x));

    Serial.println("Destination_y = " + String(destination_y));

    

    if (mode == 'C')
    {
      
      if (reachDestination)
        {
            robot.brake(1);
            robot.brake(2);
        }
      else{
        reachDestination = (destination_x != current_x || destination_y != current_y) ? false : true;
        float desire_angle;
        bool turn_done;
        // rotate
        desire_angle = calculate_desried_angle(destination_x, destination_y);
        turn_done = angle_control(convertTodegree(desire_angle), dy_mm, dx_mm, &current_angle);

        // move forward/backward
        while (turn_done && !reachDestination)
        {
            Serial.println("Turn done");
            int val = mousecam_read_reg(ADNS3080_PIXEL_SUM); // find the avrage pixel value
            MD md;
            mousecam_read_motion(&md);
            for (int i = 0; i < md.squal / 4; i++) // number of features = SQUAL register value *4
                Serial.print('*');
            Serial.print(' ');
            Serial.print((val * 100) / 351); // calculate average pixel
            Serial.print(' ');
            Serial.print(md.shutter);
            Serial.print(" (");
            Serial.print((int)md.dx);
            Serial.print(',');
            Serial.print((int)md.dy);
            Serial.println(')');

            // Serial.println(md.max_pix);// maximum = 63
            delay(100);

            distance_x = md.dx; // convTwosComp(md.dx);
            distance_y = md.dy; // convTwosComp(md.dy);

            total_x1 = total_x1 + distance_x;
            total_y1 = total_y1 + distance_y;

            total_x = (total_x1 / 157) * 10;
            total_y = (total_y1 / 157) * 10;

            dx_mm = (distance_x / 157) * 10; // convert distance to mm
            dy_mm = (distance_y / 157) * 10;

            Serial.print('\n');

            Serial.println("Distance_x = " + String(total_x));

            Serial.println("Distance_y = " + String(total_y));

            Serial.println("dx = " + String(dx_mm));

            Serial.println("dy = " + String(dy_mm));
            Serial.print('\n');

            delay(250);
            reachDestination = distance_control(destination_x, destination_y);
        
      
        }
      }

        Serial.print('\n');
        Serial.println("Final Current_y: " + String(current_y));
        Serial.println("Final Current_x: " + String(current_x));
        Serial.println("Final Current angle: " + String(convertTodegree(current_angle)));
    }
    
#endif
}

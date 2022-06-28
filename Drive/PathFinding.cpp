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
#define IN3 14
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

#define PIN_MOUSECAM_RESET 26
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

//int a = 0;
//int b = 0;

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
bool go_straight_1 = false, go_straight_2 = false, go_straight_3 = false, go_straight_4 = false;
bool go_straight_5 = false, go_straight_6 = false, go_straight_7 = true;
int length_r = 200, width_r = 200;
int length_c = 1000, width_c = 2400;
int increment = 100;
int p = 0;

//Values for breaking bits
int null, lg, dg, rd, pk, yw, be, bg;
int dis_lg, dis_dg, dis_rd, dis_pk, dis_yw, dis_be,dis_bg; 
int deg_lg, deg_dg, deg_rd, deg_pk, deg_yw, deg_be,deg_bg; 

// Values for vision detection
float angle_o, d_o, d_o_v;
int op_from_v;
int colour,  pre_colour;
int dir;
bool arrive;

// Values for radar detection
float a, b, c;
float ax, ay;
int angle_45 = 45;
bool turn_done_45;
//y1 = ax;
//y2 = bx + c;


// Values for the coordinate_list
bool check_done;//also in mode 'C'
int follow = 1;
typedef list<float> Coordinate_list;
Coordinate_list coordinate_list;
Coordinate_list::iterator it;
float a1, a2, b1, b2;
float del_x;
float OH, base;
float final_destination_x, final_destination_y;
bool set_line_end = true;
bool send_coordinate, send_x, send_y;

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
    cout << "error distance is: " << error_distance << endl;

    float control_distance = calculatePID(error_distance);
    bool forward = (((current_y + error_distance * cos(current_angle) < desired_y + 5) && (current_y + error_distance * cos(current_angle) > desired_y - 5))) ? true : false;
    int speed = 0;
    // Determine speed and direction based on the value of the control signal
    // direction
    if (error_distance > 20 && !forward) // move backward
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
            robot.rotate(motor1, 25, CW);
            robot.rotate(motor2, 25, CW);
        }
        else // move backward with 20 speed when error distance less than 50
        {
            robot.rotate(motor1, 18, CW);
            robot.rotate(motor2, 18, CW);
        }
    }
    else if (error_distance > 20 && forward) // move forward
    {
        Serial.println("Move forward");
        constant = 1; // move backwards, when calculate current position should minus the distance it traveld
        if (error_distance > 1000)
        { //>1000
            speed = fabs(control_distance) * 0.05;
            if (speed < 15)
            {
                speed = 15;
            }
            robot.rotate(motor1, speed, CCW);
            robot.rotate(motor2, speed, CCW);
        }
        else if (error_distance > 500)
        {
            speed = fabs(control_distance) * 0.5;
            if (speed < 15)
            {
                speed = 15;
            }
            robot.rotate(motor1, speed, CCW);
            robot.rotate(motor2, speed, CCW);
        }

        else if (error_distance > 50)
        { // move backward with 40 speed when error distance in range 100-50
            robot.rotate(motor1, 25, CCW);
            robot.rotate(motor2, 25, CCW);
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

bool rotate_constant(float desired_angle, float dy, float dx, float *current_an)
{
  error_angle = desired_angle - convertTodegree(current_angle);
  Serial.println("Error angle: " + String(error_angle));
  Serial.println("Desire angle in angle control: " + String(desired_angle));
  int speed = 15;
  // Determine speed and direction based on the value of the control signal
  // direction
  if (error_angle >= 5) // turn right
  {
    constant = 1;
    Serial.println("Rotate right");
    robot.rotate(motor1, speed, CCW);
    robot.rotate(motor2, speed, CW);
  }
  else if (error_angle <= -5) // turn left
  {
    constant = -1;
    Serial.println("Rotate left");
    robot.rotate(motor1, speed, CW);
    robot.rotate(motor2, speed, CCW);
  }
  else // stop/break
  {
    // robot.brake(1);
    // robot.brake(2);
    return true; // angle_control done
    Serial.println("Angle_rotate done");
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

  float min_reading = 1.5;
  float orientation;
  //bool turn_done_45;
  float turn_angle;// k * 45,
  float now_angle;

  for(k; k < 10; ){

    if(k == 1){
      now_angle = current_angle;
    }
    if(k==9){

      return;
    }
    //if (angle_45<=360){

    while(!turn_done_45)
    {//////////////////////////////////////////////////////////
      turn_angle = now_angle + k*45;

      Serial.println("turn_angle = " + String(turn_angle));
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
      turn_done_45 = rotate_constant(turn_angle, dy_mm, dx_mm, &current_angle);
    }
    ///////////////////////////////////////
    if(turn_done_45){
        //if(min_reading < radar_reading){
          //orientation = current_angle;
        //}
      turn_done_45 = false;
      k++;
      delay(100);
    }
    
  //}
  }
 
}
void go_straight(float angle_o, float d_o_v){

    d_o = d_o_v + 200;

    del_x = sin((angle_o + current_angle)*2*pi/360)*d_o;
    cout << "del_x: " << del_x << endl;
    OH = cos(angle_o*2*pi/360)*d_o;
    cout << "OH: " << OH << endl;

    float safe_d = 200;
    //desire angle becomes: convertTodegree(asin(change_x / change_y)) 
    float safe_angle = convertTodegree(asin(safe_d / d_o));
    
    float divert_angle, desire_angle;
    //if(current_angle < 0 && !go_straight_3 && (i != p)){// divert from left hand side
      //divert_angle = (-1)*angle_o + safe_angle; // must be positive according to the setting scheme
      //desire_angle = current_angle - divert_angle;
    //}

    //else{
      divert_angle = angle_o + safe_angle;
      desire_angle = current_angle + divert_angle;
      cout << "desire_angle: " << desire_angle << endl;
    //}
 
    //float desire_angle = current_angle + divert_angle;
    
    float magnitude = OH / cos(divert_angle*2*pi/360);
    cout << "magnitude: " << magnitude << endl;

    float a1 = current_x + magnitude * sin(desire_angle*2*pi/360);
    float a2 = current_y + magnitude * cos(desire_angle*2*pi/360);
    cout << "a(" << a1 << ", " << a2 << ")" << endl;


      while(!arrive)
      {//////////////////////////////////////////////////////////
        //next_x = now_x + magnitude*current_angle;

        //Serial.println("turn_angle = " + String(turn_angle));
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
        arrive = distance_control(a1, a2);
    }
    
  
}

void turn45(float angle_o)
{ 
  if(angle_o <= 0){
    dir = -1;
  }
  else{
    dir = 1;
  }

  //bool turn_done_45;
  float turn_angle;// k * 45,
  float now_angle;

  for(h; h < 3; ){

    if(h == 1){
      now_angle = current_angle;
    }
    if(h==2){

      return;
    }
    //if (angle_45<=360){

    while(!turn_done_45)
    {//////////////////////////////////////////////////////////
      turn_angle = now_angle + dir*45;

      Serial.println("turn_angle = " + String(turn_angle));
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
      turn_done_45 = rotate_constant(turn_angle, dy_mm, dx_mm, &current_angle);
    }
    ///////////////////////////////////////
    if(turn_done_45){
        //if(min_reading < radar_reading){
          //orientation = current_angle;
        //}
      turn_done_45 = false;
      h++;
      delay(100);
    }
  }
 
}

/*
float rotate360(float& radar_reading)
{   
    float min_reading = 1.6;
    float orientation;
    float desired_angle = pi / 2;
    bool turn_done;
    while (desire_angle < 2 * pi)
    {   
        if(min_reading < radar_reading){
          orientation = current_angle;
        }
        turn_done = angle_control(convertTodegree(desire_angle), dy_mm, dx_mm, &current_angle);
        cout << "-------read radar output-------" << endl;
        delay(100);
        desired_angle = desired_angle + pi;
    }
    return orientation;
}
*/
void rotate_v(bool center, bool detect_d_o, float &send_x, float &send_y){
  //this function is called when the obstacle is detected
  //but no reroute is required
  if(center){
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

int min_dis(int v[7][2]){
  int m, min;
  min = v[0][1];
  for(m = 0; m < 7;m++){
    if(v[m][1]<min){
      min = v[m][1];
    }
  }
  return m;
}

//deal with the input pack, identify whether it has been marked;
bool check_mark(int vision_colour){

  return true;
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

//
bool check_block(float angle_o, float d_o_v){


  //d_o and angle_o would be input
  d_o = d_o_v + length_r;
  //d_o_v must be larger than 30cm
  //angle_o = angle_o*2*pi/360;
 
  del_x = sin(angle_o*2*pi/360)*d_o_v;
  //del_y = cos(angle_o)*d_o_v;
  cout << del_x << "   "  << endl;

  if(del_x < width_r) { // block the way
    return true;

  }
  else{
    return false;
  }

}


void generate_coordinates(){

  //if(check_block(angle_o, d_o_v)){

    //float d_o;//  =; distance from the rover to the obstacle detected, given from the Vision
    //float angle_o;// =; angle between the current 'go straight' path and the line connected the rover and the obstacle

    del_x = sin((angle_o + current_angle)*2*pi/360)*d_o;
    cout << "del_x: " << del_x << endl;
    OH = cos(angle_o*2*pi/360)*d_o;
    cout << "OH: " << OH << endl;

    float safe_d = 200;
    //desire angle becomes: convertTodegree(asin(change_x / change_y)) 
    float safe_angle = convertTodegree(asin(safe_d / d_o));
    
    float divert_angle, desire_angle;
    if(current_angle < 0 && !go_straight_3 && (i != p)){// divert from left hand side
      divert_angle = (-1)*angle_o + safe_angle; // must be positive according to the setting scheme
      desire_angle = current_angle - divert_angle;
    }

    else{
      divert_angle = angle_o + safe_angle;
      desire_angle = current_angle + divert_angle;
      cout << "desire_angle: " << desire_angle << endl;
    }
 
    //float desire_angle = current_angle + divert_angle;
    
    float magnitude = OH / cos(divert_angle*2*pi/360);
    cout << "magnitude: " << magnitude << endl;
    float base = 2*OH;
    
    //add one intermediate point
    float a1 = current_x + magnitude * sin(desire_angle*2*pi/360);
    float a2 = current_y + magnitude * cos(desire_angle*2*pi/360);
    cout << "a(" << a1 << ", " << a2 << ")" << endl;
   
    //add second point to finish divesion
    float b1 = current_x + base * sin(current_angle*2*pi/360);
    float b2 = current_y + base * cos(current_angle*2*pi/360);
    cout << "b(" << b1 << ", " << b2 << ")" << endl;
    cout << "destination (" << destination_x << ", " << destination_y << ")" << endl;
    cout << "-----------------" << endl;

    coordinate_list.push_front(b2);
    coordinate_list.push_front(b1);
    coordinate_list.push_front(a2);
    coordinate_list.push_front(a1);
    
    //int order_position;// which pair of coordinates 
  //}
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

  /*if(check_done){
    cout << "check_done finished" << endl;
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
  
  }*/

  float d2destination = sqrt(pow(destination_x-current_x,2) + pow(destination_y-current_y,2));
  cout << "distance btwn current position and the destination is: " << d2destination << endl;

  /*if(generate_coordinates){

    if(current_angle*(destination_x-b1)<0 || (b2-current_y)>(destination_y-current_y)){

      if(coordinate_list.empty()){
        reachDestination = true;
        return;
      }

      //erase the current destination
      it = coordinate_list.begin();
      coordinate_list.erase(it);
      it = coordinate_list.begin();
      coordinate_list.erase(it);
      
      // only insert one point in the front of the list
      //assembles B1 ad C1 in the example
      coordinate_list.push_front(a2);
      coordinate_list.push_front(a1);

      //last coordinate deleted!
      //then the rover get back to the normal path gradually
      
      return;

    }*/

    if(2*OH > d2destination){
      it = coordinate_list.begin();
      coordinate_list.erase(it);
      advance(it,follow);
      coordinate_list.erase(it);
    }

    cout << "Edit the list." << endl;

    coordinate_list.push_front(b2);
    coordinate_list.push_front(b1);
    coordinate_list.push_front(a2);
    coordinate_list.push_front(a1);
    cout << "push successfully" << endl;
  //}
  return;
}




///////Set up///////
void setup()
{
    Serial.begin(9600);
    Serial2.begin(115200);

    pinMode(PIN_SS, OUTPUT);
    pinMode(PIN_MISO, INPUT);
    pinMode(PIN_MOSI, OUTPUT);
    pinMode(PIN_SCK, OUTPUT);
    //pinMode(33, INPUT);

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

  //for radar detection
  
  /*int radar = analogRead(33);
  //float radar_reading = radar * 3.3/1023;
  //cout << "radar reading: " << radar_reading << endl;


  if(radar_reading>1.5){

    cout << "radar detection" << endl;

    if(radar_reading>1.8){
      send_coordinate = 1;
      send_x = current_x;
      send_y = current_y;
      cout << "send coordinate: (" << send_x << ", " << send_y << ")" << endl;
    }

    else{
        if(a == 0){
        a = rotate360(radar_reading);
        ax = current_x;
        ay = current_y;
      }
      else{
        b = rotate360(radar_reading);
        c = (current_y-ay) - (current_x-ax)*tan(current_angle);
      }

      if(a != 0 && b != 0){
        send_coordinate = 1;
        send_x = ax + c/(a-b);
        send_y = ay + a*send_x;
        cout << "send coordinate: (" << send_x << ", " << send_y << ")" << endl;
        a = 0;
        b = 0;
      }
    }

    send_coordinate = 0;

  }*/
  

  if(Serial2.available() > 0){
    null = Serial2.read();

    if(null == 0){
      lg = Serial2.read();
      dg = Serial2.read();
      rd = Serial2.read();
    }

    else{
      pk = null;
      yw = Serial2.read();
      be = Serial2.read();
      bg = Serial2.read();
    }
  }

  dis_lg = (lg / 16)*5;
  deg_lg = ((lg % 16)-1)*5-20;

  dis_dg = (dg / 16)*5;
  deg_dg = ((dg % 16)-1)*5-20;

  dis_rd = (rd / 16)*5;
  deg_rd = ((rd % 16)-1)*5-20;

  dis_pk = (pk / 16)*5;
  deg_pk = ((pk % 16)-1)*5-20;

  dis_yw = (yw / 16)*5;
  deg_yw = ((yw % 16)-1)*5-20;

  dis_be = (be / 16)*5;
  deg_be = ((be % 16)-1)*5-20;

  dis_bg = (bg / 16)*5;
  deg_bg = ((bg % 16)-1)*5-20;

  if(lg != 1){
    int dis_lg = (lg/16)*5;
    int deg_lg = ((lg%16)-1)*5 -20;
  }

  int v[7][2] = {
    {dis_lg, deg_lg},
    {dis_dg, deg_dg},
    {dis_rd, deg_rd},
    {dis_pk, deg_pk},
    {dis_yw, deg_yw},
    {dis_be, deg_be},
    {dis_bg, deg_bg},
  };

  colour = min_dis(v);
  if(colour != pre_colour){
    d_o_v = v[min_dis(v)+1][1];
    angle_o = v[min_dis(v)+1][2];
  }
  pre_colour = colour;
  
  cout << "the colour index is: " << colour << endl;
  cout << "d_o_v is: " << d_o_v << endl;
  cout << "angle_o is: " << angle_o << endl;
  


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

      cout << "set line end point." << endl;
      //length_c = 750;
      for(int  o = 0; o < 1; o++){
        coordinate_list.push_back(o*increment);
        coordinate_list.push_back(length_c-700);
        coordinate_list.push_back((o+1)*increment);
        coordinate_list.push_back(length_c-700);
        coordinate_list.push_back((o+1)*increment);
        coordinate_list.push_back(0);
        coordinate_list.push_back((o+2)*increment);
        coordinate_list.push_back(0);
        coordinate_list.push_back((o+2)*increment);
        coordinate_list.push_back(length_c-700);
        coordinate_list.push_back((o+3)*increment);
        coordinate_list.push_back(length_c-700);
        coordinate_list.push_back((o+3)*increment);
        coordinate_list.push_back(0);
        coordinate_list.push_back(0);
        coordinate_list.push_back(0);
        cout << "loop finished" << endl;  

      }

      it = coordinate_list.begin();
      destination_x = *it;
      advance(it,1);
      destination_y = *it;

      if(check_block(angle_o, d_o_v)){

          if(d_o_v > 250 && d_o_v < 350){
            cout << "the obstacle blocks the loop" << endl;
            turn45(angle_o);
            go_straight(angle_o, d_o_v);
            //a1,a2,b1,b2 are updated;
            //edit_list();
          }
      }

      /*
      coordinate_list.push_back(0);
      coordinate_list.push_back(50);
      coordinate_list.push_back(30);
      coordinate_list.push_back(50);
      coordinate_list.push_back(30);
      coordinate_list.push_back(0);
      coordinate_list.push_back(60);
      coordinate_list.push_back(0);
      coordinate_list.push_back(60);
      coordinate_list.push_back(50);
      coordinate_list.push_back(90);
      coordinate_list.push_back(50);
      coordinate_list.push_back(90);
      coordinate_list.push_back(0);
      coordinate_list.push_back(0);
      coordinate_list.push_back(0);*/

      /*for(g; g<2; g++){
        float angle_o = 10*2*3.1415926/360;
        float d_o_v = 400;

        cout << check_block(angle_o, d_o_v) << endl;

        if(check_block(angle_o, d_o_v)){

          if(d_o_v > 250 && d_o_v < 500){
            cout << "the obstacle blocks the loop" << endl;
            generate_coordinates();
            //a1,a2,b1,b2 are updated;
            //edit_list();
          }
        
        }
        else if(!check_block(angle_o,d_o_v)){
          cout << "the obstacle does not block the loop" << endl;
          rotate_v;
        }
    }*/


      it = coordinate_list.begin();
      destination_x = *it;
      advance(it,1);
      destination_y = *it;

      cout << "In the list ----------destination(x,y) is: (" << destination_x << ", " << destination_y << ")" << endl;
  
      it = coordinate_list.end();
      float final_destination_y = *--it;
      float final_destination_x = *--it;
      cout << "final desination is:" << final_destination_x << ", " << final_destination_y << endl;

  }

  cout << " ------- destination(x,y) is: (" << destination_x << ", " << destination_y << ")" << endl;
    
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
    {  cout << "j = " << j << endl;
      
      if (reachDestination)
        {
            robot.brake(1);
            robot.brake(2);
            delay(50);
            cout<< "reach destination." << endl;

            //delete the first 2 elements
            it = coordinate_list.begin();
            coordinate_list.erase(it);
            it = coordinate_list.begin();
            coordinate_list.erase(it);
            reachDestination = false;
            j++;
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
              
            //if (check_block){
            
              //break;
           // }
            //else{
                    reachDestination = distance_control(destination_x, destination_y);
                //}
            
      
        }
      }

        Serial.print('\n');
        Serial.println("Final Current_y: " + String(current_y));
        Serial.println("Final Current_x: " + String(current_x));
        Serial.println("Final Current angle: " + String(convertTodegree(current_angle)));

    }
    
#endif
}

#include "motors.h"
#include "encoders.h"
#include "pid.h"
//#include "linesensor.h"
#include "bumpsensor.h"
#include "kinematics.h"
#include <math.h>

#define BUZZER_PIN 6
#define INITIAL_STATE 0
#define JOIN_THE_LINE 1
#define FOLLOW_THE_LINE 2
#define DRIVE_10CM 3
#define ROTATE_180DEGREE 4
#define DRIVE_FORWARDS 5
#define COME_BACK 6
#define TAKE_RIGHT 7
#define DRIVE_BACK 8
#define TAKE_LEFT 9
#define END 10

Motors_c motors;

//LineSensor_c linesensor;
BumpSensor_c bumpsensor;

Kinematics_c kinematics_origin;
Kinematics_c kinematics_traverse_line;
Kinematics_c kinematics_reference;

PID_c spd_pid_left; // speed PID controller left
PID_c spd_pid_right; //speed PID controller right
PID_c heading; //
PID_c reference; // PID for angle calculations
PID_c right_turn;
PID_c rotate;

unsigned long update_ts; // timestamp
unsigned long pid_test_ts;

unsigned long pass_time;
unsigned long diff_time;
unsigned long program_time;

float avg_e1_spd; // low pass filter speed
float avg_e0_spd;

long count_e1_last; // for difference in encoder counts
long count_e0_last;

long right_encoder;
long left_encoder;

long currentLeft, currentRight; //for coming back

float demand_wheels;
float demand_heading;
float demand_theta;
float theta_diff;
int state = 0;
float pwm_left;
float pwm_right;
float drive_distance;
float angle_rotated;
bool a = true;
bool b = true;
bool c = true;
bool d = true;
bool e = true;
bool f = true;
int n = -11;

float pwm_heading;
float e_line;
float current_theta;

// put your setup code here, to run once:
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  // set up motors
  motors.initialise();

  kinematics_origin.initialise();
  kinematics_traverse_line.initialise();
  kinematics_reference.initialise();

  // Encoder setup
  setupEncoder0();
  setupEncoder1();

  //linesensor.initialise();
  //linesensor.caliberation();
  //Set up kp, ki, kd
  spd_pid_left.initialise(0.6, 0.001, 50);
  spd_pid_right.initialise(0.6, 0.001, 50);
  heading.initialise(0.1, 0, 0);
  right_turn.initialise(1, 0, 0);
  rotate.initialise(5, 0, 0);

  avg_e1_spd = 0.0;

  demand_wheels = 0.36;
  demand_heading = 0;
  drive_distance = 0;
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  update_ts = millis();
  //update_elapsed_distance = millis();
  //drive_time = 0;

  //set initial value for our last e1 count within the loop
  count_e1_last = count_e1;
  count_e0_last = count_e0;

  motors.setMotorPower(0, 0);
  e_line = 0;

  //Reset PID before use it.
  spd_pid_left.reset();
  spd_pid_right.reset();
  heading.reset();
  right_turn.reset();

  pid_test_ts = millis();

  state = INITIAL_STATE;
  bumpsensor.initialise();
}

// put your main code here, to run repeatedly:

void loop() {
  program_time = millis();
  bumpsensor.readBumpSensor();
  if ((bumpsensor.real_sensor_read[0] != 1 && linesensor.real_sensor_read[1] != 1)) {
    calculate_speed();
    //e_line = bumpsensor.e_line;
    pwm_heading = heading.update(demand_heading, e_line);
    pwm_left = spd_pid_left.update(0.45 + pwm_heading, avg_e1_spd);
    pwm_right = spd_pid_right.update(0.45 - pwm_heading, avg_e0_spd);
    motors.setMotorPower(pwm_left / 0.018, pwm_right / 0.018);
    //linesensor.getLineError();
  }
  else {
    come_back();


  }

}

void updatestatus() {
  bumpsensor.getLineError();
  kinematics_origin.kinematics_cal();
  Serial.println(kinematics_origin.X_M);

}


void come_back() {
  while (d == true) {
    while (a == true) {
      motors.setMotorPower(0, 0);
      delay(1000);
      a = false;
    }
    kinematics_origin.kinematics_cal();
    if (kinematics_origin.R_M < 0 && b == true) {
      right_encoder = count_e0 + 486;
      left_encoder = count_e1 - 486;
      while (count_e0 < right_encoder && count_e1 > left_encoder) {
        motors.setMotorPower(-20, +20);
      }
      b = false;
      while (e == true) {
        motors.setMotorPower(0, 0);
        delay(1000);
        e = false;
      }
    }
    else if (kinematics_origin.R_M > 0 && c == true) {
      right_encoder = count_e0 - 470;
      left_encoder = count_e1 + 470;
      while (count_e0 > right_encoder && count_e1 < left_encoder) {
        motors.setMotorPower(+25, -25);
      }
      c = false;
      while (f == true) {
        motors.setMotorPower(0, 0);
        delay(1000);
        f = false;
      }
    }

    demand_theta = 0;
    current_theta = 0;
    pass_time = millis();
    diff_time = millis() - pass_time;
    currentLeft = count_e0;
    while ((count_e0 - currentLeft) <= 500 ) {
      motors.setMotorPower(25, 25);
      diff_time = millis() - pass_time;


      void calculate_speed() {
        unsigned long elapsed; // calculate difference in time

        //last time, current time.
        elapsed = millis() - update_ts;

        // calculate speed estimate
        if (elapsed > 20) { //every 20 ms
          update_ts = millis();

          long diff_e1; // difference in encoder counts
          float e1_speed; // speed calculator for e1.
          long diff_e0;
          float e0_speed;

          // calculate difference in e1.
          diff_e1 = count_e1 - count_e1_last;
          count_e1_last = count_e1;

          diff_e0 = count_e0 - count_e0_last;
          count_e0_last = count_e0;

          //encoder counts per ms
          e1_speed = (float)diff_e1;
          e1_speed /= (float)elapsed; //actual elapsed ms.

          e0_speed = (float)diff_e0;
          e0_speed /= (float)elapsed; //actual elapsed ms.

          avg_e1_spd = ( avg_e1_spd * 0.9 ) + ( e1_speed * 0.1 );
          avg_e0_spd = ( avg_e0_spd * 0.9 ) + ( e0_speed * 0.1 );
        }
      }

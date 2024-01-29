
#include "Wire.h"
#include <MPU6050_light.h>
#include <math.h>

//distance sensor
const int trigPin = 7;
const int echoPin = 13;

//variables for PID control
// float target = 0;
float error = 0;
float integral = 0;
float derivative = 0;
float last_error = 0;
float angle = 0;
// float 

//the 'k' values are the ones you need to fine tune before your program will work. Note that these are arbitrary values that you just need to experiment with one at a time.
float Kp = 0.7;
float Ki = 0.0;
float Kd = 0.4;

// IMU and Angles
MPU6050 mpu(Wire);
unsigned long timer = 0;

volatile int target_angle = 0;

//ENCODERS
const int ENCODER_PIN_A = 3;
const int ENCODER_PIN_B = 2;
volatile int count_pulses_A = 0;
volatile int count_pulses_B = 0;
const float stepcount = 20.0;
const float wheeldiameter = 6.4;
const int pointing_threshold = 1;
const int rotation_threshold = 0.00;
const int start_distance = 25;
const int forward_distance = 50;
// const int turn_distance = 14*3.14159/4;
//FIX -- maybe don't need?
const int turn_distance = 500;

//MOTORS

int STBY = 10; //standby

//Motor A
int PWMA = 6; //Speed control 
int AIN1 = 9; //Direction
int AIN2 = 8; //Direction

//Motor B
int PWMB = 5; //Speed control
int BIN1 = 11; //Direction
int BIN2 = 12; //Direction

//orig speeds
int orig_forward_speed = 75;
int orig_turn_speed = 100;

//current speeds
int forward_speed=75;
int turn_speed=90;

// State machine
enum robot_states{
  START,
  IDLE,
  FORWARD,
  RIGHT,
  LEFT,
  BACKWARD,
  WAIT,
};

//declare current state (start)
robot_states current_state = START;

//TIME
unsigned long target_time = 40000;
//current turn time (milliseconds)
unsigned long turn_time = 910;
//current move time (milliseconds)
unsigned long move_time = 3500;
//current start time (milliseconds)
//unsigned long start_time = 1600;
//holds number of turns that has happened so far
int turn_count = 0;
//holds number of moves that has happened so far
int move_count = 0;
//holds time where action began
unsigned long old_time = 0;
//holds total number of turns
int total_turn_count = 0;
//holds total number of moves
int total_move_count = 0;
//holds extra time that is not in the move
unsigned long extra_time = 0;
//holds target action time
unsigned long target_action_time = 0;

//CHANGE HEREE!!!!
//HERRRREEE
robot_states state_order[] = {LEFT, FORWARD, RIGHT, FORWARD, BACKWARD, IDLE};

//int distance_order[] = {-1, 50};

//bool using_distances = false;

int state_order_idx = -1;

bool started_new_state = true;

int status;

int CMtoSteps(float cm) { //converts centimeters to encoder steps
  int result;  // Final calculation result
  float circumference = wheeldiameter * 3.14159; // Calculate wheel circumference in cm
  float cm_step = circumference / stepcount;  // CM per Step
  
  float f_result = cm / cm_step;  // Calculate result as a float
  result = (int) f_result; // Convert to an integer (note this is NOT rounded)
  
  return result;  // End and return result
}

void DCMotorAEncoder(){
  count_pulses_A++;
  }

void DCMotorBEncoder(){
  count_pulses_B++;
}

void setup() {
  Serial.begin(9600);
  Serial.println("hi");
  Wire.begin();
  Serial.println("targettime4");
  Serial.println(turn_time);
  
  //distance sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);

  status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));

  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done! Starting Robot! \n");

  //ENCODERS
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A),DCMotorAEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B),DCMotorBEncoder,RISING);

  //MOTORS
  pinMode(STBY, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  Serial.println("target time 2");
  Serial.println(target_time);

  //count total number of turns, moves
  for (int i = 0; i < sizeof(state_order)/sizeof(state_order[1]); i++) {
    if (state_order[i] == RIGHT || state_order[i] == LEFT){
      total_turn_count++;
    }else if(state_order[i] == FORWARD || state_order[i] == BACKWARD){
      total_move_count++;
    }
  }
  Serial.println("blah");
  Serial.println(total_turn_count);
  Serial.println(total_move_count);
  Serial.println("target time 3");
  Serial.println(target_time);
}

void reset(){
  count_pulses_A = 0;  //  reset counter A to zero
  count_pulses_B = 0;  //  reset counter B to zero

  old_time = millis();

  started_new_state=false;

  error = 0;
  integral = 0;
  derivative = 0;
  last_error = 0;
  angle = 0;
  digitalWrite(STBY, HIGH); //disable standby
}

void new_state(){
  digitalWrite(STBY, LOW); //enable standby
  state_order_idx++;
  started_new_state=true;
  current_state = state_order[state_order_idx];
}

void move(bool is_forward){
  float angle_output;
  if (status==0){
    angle_output = mpu.getAngleZ();
  }else{
    angle_output = 0;
  }
  error = target_angle - angle_output;// proportional
  integral = integral + error; //integral
  derivative = error - last_error; //derivative
  last_error=error;
  angle = (error * Kp) + (integral*Ki) + (derivative * Kd);
  int motorspeeda = forward_speed + angle;
  int motorspeedb = forward_speed - angle;
  if (is_forward==false){
    motorspeeda = forward_speed - angle;
    motorspeedb = forward_speed + angle;
  }

  if(error!=0){//motor A or B is going too fast or too slow
    analogWrite(PWMA, motorspeeda);
    analogWrite(PWMB, motorspeedb);
  }else{//motors are moving at good pace, continue
      analogWrite(PWMA, forward_speed);
      analogWrite(PWMB, forward_speed);       
  }
}

void loop() {
  if (status==0){
    mpu.update();
  }
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;

  if (current_state == WAIT){
    unsigned long time_spent_so_far = millis()-old_time;
    //Serial.print(started_new_state);
   //Serial.print("WAITING, extra time: ");
  //  Serial.print(extra_time);
  //  Serial.print(" time spent so far: ");
  //  Serial.println(time_spent_so_far);
  //  Serial.println(started_new_state);
  //  Serial.println(time_spent_so_far>extra_time);
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    // Set Motor A backwards
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    
    // Set Motor B forward
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    if (started_new_state){
      Serial.println("WAITING, extra time: ");
      Serial.println(extra_time);
      reset();
      digitalWrite(STBY, LOW); //enable standby
    } else if (time_spent_so_far>extra_time || time_spent_so_far>2000){

      new_state();
    }else{

    }
  }
  else if (current_state==START){
      int steps = CMtoSteps(start_distance);      
      if (started_new_state){
        reset();
        Serial.println("Start");
        target_time += millis();
        //start_time = millis()-old_time;
        // Set Motor A forward
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
      
        // Set Motor B forward
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
      }else if (count_pulses_A>steps || count_pulses_B>steps || millis()-old_time>10000){ // finished!
        //Serial.print("start_time");
        //Serial.println(start_time);
        new_state();
        //Serial.println("target time 1");
        //Serial.println(target_time);        
      }else{ //continue moving forward!
        //Serial.println(count_pulses_A);
        //Serial.println(count_pulses_B);
        move(true);
      }
  } else if (current_state==RIGHT){
      int steps = CMtoSteps(turn_distance);
      float angle_output;
      if (status==0){
        angle_output = mpu.getAngleZ();
      }else{
        angle_output = 0;
      }
      //Serial.println(angle_output);
      if (started_new_state){
        Serial.println("Right");
        reset();
        target_action_time = (target_time-old_time)*turn_time/((total_turn_count-turn_count)*turn_time+(total_move_count-move_count)*move_time);
        turn_speed = min(100, max(orig_turn_speed*(turn_time/target_action_time), 60));
        target_angle-=90;
        // Set Motor A backwards
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        
        // Set Motor B forward
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
      }else if (angle_output<=target_angle || (count_pulses_A>steps || count_pulses_B>steps)){ // finished!
        unsigned long time_spent_so_far = millis()-old_time;
          Serial.println(target_action_time);
          Serial.println(time_spent_so_far);
        turn_count++;
        if (time_spent_so_far<target_action_time){
          current_state = WAIT;
          extra_time = target_action_time-time_spent_so_far;
          started_new_state=true;
          Serial.println(extra_time);
        }else{
          new_state();
        }
      }else{ //continue moving forward!
            analogWrite(PWMA, max(60, turn_speed*abs(target_angle-angle_output)/90));
            analogWrite(PWMB, max(60, turn_speed*abs(target_angle-angle_output)/90));       
      }
  } else if (current_state==LEFT){
      int steps = CMtoSteps(turn_distance);

      float angle_output;
      if (status==0){
        angle_output = mpu.getAngleZ();
      }else{
        angle_output = 0;
      }  
      if (started_new_state){
        reset();
        Serial.println("Left");
        target_angle+=90;
        target_action_time = (target_time-old_time)*turn_time/((total_turn_count-turn_count)*turn_time+(total_move_count-move_count)*move_time);
        turn_speed = min(100, max(orig_turn_speed*(turn_time/target_action_time), 60));
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
      
        // Set Motor B backwards
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
      }else if (angle_output>=target_angle || (count_pulses_A>steps || count_pulses_B>steps)){ // finished!
        unsigned long time_spent_so_far = millis()-old_time;
        turn_count++;
          Serial.println(target_action_time);
          Serial.println(time_spent_so_far);
        if (time_spent_so_far<target_action_time){
          current_state = WAIT;
          extra_time = target_action_time-time_spent_so_far;
          started_new_state=true;
          Serial.println(extra_time);
        }else{
          new_state();
        }
      }else{ //continue moving forward!
            analogWrite(PWMA, max(60, turn_speed*abs(target_angle-angle_output)/90));
            analogWrite(PWMB, max(60, turn_speed*abs(target_angle-angle_output)/90));     
      }
  } else if (current_state==FORWARD){
      int steps = CMtoSteps(forward_distance);
      if (started_new_state){
        reset();
        Serial.println("FORWARD");
        target_action_time = (target_time-old_time)*move_time/((total_turn_count-turn_count)*turn_time+(total_move_count-move_count)*move_time);
        forward_speed = min(100, max(orig_forward_speed*(move_time/target_action_time), 70));
        //Serial.println("Forward");
        //Serial.println(target_angle);
        // Set Motor A forward
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
      
        // Set Motor B forward
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
      }else if (count_pulses_A>steps && count_pulses_B>steps){ // finished! not using distances
        // Serial.println("hi!");
        unsigned long time_spent_so_far = millis()-old_time;
          Serial.println(target_action_time);
          Serial.println(time_spent_so_far);
        move_count++;

        if (time_spent_so_far<target_action_time){
          current_state = WAIT;
          extra_time = target_action_time-time_spent_so_far;
          started_new_state=true;
          Serial.println(extra_time);
        }else{
          new_state();
        }
      }else{ //continue moving forward!
        //Serial.println(count_pulses_A);
        //Serial.println(count_pulses_B);
        move(true);
      }
  } else if (current_state==BACKWARD){
      int steps = CMtoSteps(forward_distance);      
      if (started_new_state){
        Serial.println("backward");
        reset();
        target_action_time = (target_time-old_time)*move_time/((total_turn_count-turn_count)*turn_time+(total_move_count-move_count)*move_time);
        forward_speed = min(100, max(orig_forward_speed*(move_time/target_action_time), 70));

        //Serial.println("Forward");
        //Serial.println(target_angle);
        // Set Motor A backward
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
      
        // Set Motor B backward
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
      }else if (count_pulses_A>steps && count_pulses_B>steps){ // finished!
        unsigned long time_spent_so_far = millis()-old_time;
          Serial.println(target_action_time);
          Serial.println(time_spent_so_far);
        move_count++;

        if (time_spent_so_far<target_action_time){
          current_state = WAIT;
          extra_time = target_action_time-time_spent_so_far;
          started_new_state=true;
          Serial.println(extra_time);
        }else{
          new_state();
        }
      }else{ //continue moving forward!
        move(false);
      }
  }
}

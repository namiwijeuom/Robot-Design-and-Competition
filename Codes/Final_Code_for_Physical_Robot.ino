#include <SharpIR.h>

//To measure previous milliseconds before a particular interval
unsigned long previousMillis = 0;
//Interval to measure to stay in all black situation
const long interval = 500;

unsigned long explorationFinishedTime = 0;
unsigned long startedTime = 0;

// Define model and input pins:
#define IRPinLeft A6
#define IRPinFront A7
#define IRmodel 1080
#define indicatorLEDPin 13
#define collisionSensorLeft 34
#define collisionSensorFront 35
#define collisionSensorNorthWest 39


// IR line detection sensors
#define LINE_DETECTION_SENSOR_CENTER A1
#define LINE_DETECTION_SENSOR_LEFT_CORNER A9
#define LINE_DETECTION_SENSOR_RIGHT_CORNER A8
#define LINE_DETECTION_SENSOR_LEFT_MIDDLE A0
#define LINE_DETECTION_SENSOR_RIGHT_MIDDLE A2
#define LINE_DETECTION_SENSOR_REAR A10

const int LINE_DETECTION_SENSOR_CENTER_BLACK_VALUE = 900;
const int LINE_DETECTION_SENSOR_LEFT_CORNER_BLACK_VALUE = 750;
const int LINE_DETECTION_SENSOR_RIGHT_CORNER_BLACK_VALUE = 800;
const int LINE_DETECTION_SENSOR_LEFT_MIDDLE_BLACK_VALUE = 700;
const int LINE_DETECTION_SENSOR_RIGHT_MIDDLE_BLACK_VALUE = 750;
const int LINE_DETECTION_SENSOR_REAR_BLACK_VALUE = 900;

const int LINE_DETECTION_SENSOR_CENTER_WHITE_VALUE = 550;
const int LINE_DETECTION_SENSOR_LEFT_CORNER_WHITE_VALUE = 150;
const int LINE_DETECTION_SENSOR_RIGHT_CORNER_WHITE_VALUE = 400;
const int LINE_DETECTION_SENSOR_LEFT_MIDDLE_WHITE_VALUE = 150;
const int LINE_DETECTION_SENSOR_RIGHT_MIDDLE_WHITE_VALUE = 200;
const int LINE_DETECTION_SENSOR_REAR_WHITE_VALUE = 700;

bool is_line_detection_sensor_center_on_white = false;
bool is_line_detection_sensor_left_corner_on_white = false;
bool is_line_detection_sensor_right_corner_on_white = false;
bool is_line_detection_sensor_left_middle_on_white = false;
bool is_line_detection_sensor_right_middle_on_white = false;
bool is_line_detection_sensor_rear_on_white = false;


// Create variable to store the distance:
int left_distance_cm;
int front_distance_cm;
bool goingToCollideLeft = false;
bool goingToCollideFront = false;
bool goingToCollideNorthWest = false;

// wall detection IR sensors other constants
const int MAX_DIST = 15;     // Max distance to measure in cm
const int MIN_DIST = 5;      // Min distance to measure in cm
const int TARGET_DIST = 8;  // Target distance from the wall in cm
const int FRONT_COLLISION_AVOIDANCE_DISTANCE = 10;
const int Kp_FRONT_COLLISION_AVOIDANCE = 60;
const int CONTROL_OUTPUT_LOWER_THRESHOLD = -100;
const int CONTROL_OUTPUT_UPPER_THRESHOLD = 100;


// To change
const int PID_TURN_TEST_TIME = 50;
const int Kp = 500;  // Proportional gain for control
const int Kd = 400;  // Derivative gain for control
const int Ki = 0;    // Integral gain for control
const int TURN_SPEED = 60;
const int TURN_MOTORS_BY_DIFFERENT_SPEEDS_MOTOR_SPEEDS_LOWER_THRESHOLD = 40;
const int TURN_MOTORS_BY_DIFFERENT_SPEEDS_MOTOR_SPEEDS_UPPER_THRESHOLD = 70;

// // GOOD
// const int PID_TURN_TEST_TIME = 50;
// const int Kp = 500;  // Proportional gain for control
// const int Kd = 400;  // Derivative gain for control
// const int Ki = 0;    // Integral gain for control
// const int TURN_SPEED = 60;
// const int TURN_MOTORS_BY_DIFFERENT_SPEEDS_MOTOR_SPEEDS_LOWER_THRESHOLD = 40;
// const int TURN_MOTORS_BY_DIFFERENT_SPEEDS_MOTOR_SPEEDS_UPPER_THRESHOLD = 70;

// Create a new instance of the SharpIR class:
SharpIR leftIRSensor = SharpIR(IRPinLeft, IRmodel);
SharpIR frontIRSensor = SharpIR(IRPinFront, IRmodel);

// Variables for control loop
float last_error = 0;
float error = 0;
float integral = 0;
float derivative = 0;

// Right Motor connections
int RIGHT_MOTOR_EN = 12;
int RIGHT_MOTOR_IN1 = 27;
int RIGHT_MOTOR_IN2 = 26;
// Left Motor connections
int LEFT_MOTOR_EN = 11;
int LEFT_MOTOR_IN1 = 22;
int LEFT_MOTOR_IN2 = 23;


//Line Following Variables
int const LINE_FOLLOWING_FORWARD_SPEED = 65;
int const LINE_FOLLOWING_FORWARD_TIME = 150;
int const LINE_FOLLOWING_CORRECTION_LEAN_RIGHT_SPEED = 60;
int const LINE_FOLLOWING_CORRECTION_LEAN_RIGHT_TIME = 100;
int const LINE_FOLLOWING_CORRECTION_LEAN_LEFT_SPEED = 60;
int const LINE_FOLLOWING_CORRECTION_LEAN_LEFT_TIME = 100;
int const LINE_FOLLOWING_TURN_RIGHT_SPEED = 60;
int const LINE_FOLLOWING_TURN_RIGHT_TIME = 600;
int const LINE_FOLLOWING_TURN_LEFT_SPEED = 60;
int const LINE_FOLLOWING_TURN_LEFT_TIME = 600;
int const LINE_FOLLOWING_FORWARD_AFTER_TURN_SPEED = 60;
int const LINE_FOLLOWING_FORWARD_AFTER_TURN_TIME = 200;
int const LINE_FOLLOWING_MOVE_FORWARD_A_BIT_SPEED = 70;
int const LINE_FOLLOWING_MOVE_FORWARD_A_BIT_TIME = 40;
int const LINE_FOLLOWING_MOVE_BACKWARD_AT_DEAD_END_SPEED = 60;
int const LINE_FOLLOWING_MOVE_BACKWARD_AT_DEAD_END_TIME = 200;
bool completedExploreRun = false;
bool lineFollowingDone = false;


// Other Variables
int testSpeed = 150;




const int LEFT_MOTOR_OFFSET_ADJUSTMENT = 0;
const int RIGHT_MOTOR_OFFSET_ADJUSTMENT = 0;


void setup() {
  Serial.begin(9600);
  pinMode(collisionSensorLeft, INPUT);
  pinMode(collisionSensorFront, INPUT);
  pinMode(collisionSensorNorthWest, INPUT);

  // Set all the motor control pins to outputs
  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_EN, OUTPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  // Indicator LED
  pinMode(indicatorLEDPin, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);

  // pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  pinMode(LINE_DETECTION_SENSOR_CENTER, INPUT);
  pinMode(LINE_DETECTION_SENSOR_LEFT_CORNER, INPUT);
  pinMode(LINE_DETECTION_SENSOR_RIGHT_CORNER, INPUT);
  pinMode(LINE_DETECTION_SENSOR_LEFT_MIDDLE, INPUT);
  pinMode(LINE_DETECTION_SENSOR_RIGHT_MIDDLE, INPUT);
  pinMode(LINE_DETECTION_SENSOR_REAR, INPUT);

  startedTime = millis();
}


void loop() {

  while (!lineFollowingDone) {
     takeLineFollowingReadings();
     followLine();
   }

  for (int i = 0; i < 2500; i++) {
    takeWallReadings();
    int control_output = wallFollowingAdjustPathPID();
    turnMotorsByDifferentSpeeds(control_output, PID_TURN_TEST_TIME);
  }
  delay(500);
}


int wallFollowingAdjustPathPID() {
  error = left_distance_cm - TARGET_DIST;  // Calculate error for control loop
  derivative = error - last_error;         // Calculate derivative term for control loop
  integral = integral + error;
  float control_output = (Kp * error) + (Kd * derivative) + (Ki * integral);  // Calculate control output


  control_output = control_output / 100;  // undo the effect of the factor of 100 in Kp, Ki and Kd

  // check the front sensor and see its going towards a wall
  if (front_distance_cm < FRONT_COLLISION_AVOIDANCE_DISTANCE) {
    control_output += (front_distance_cm - FRONT_COLLISION_AVOIDANCE_DISTANCE) * Kp_FRONT_COLLISION_AVOIDANCE;
  }



  if (control_output > CONTROL_OUTPUT_UPPER_THRESHOLD) control_output = CONTROL_OUTPUT_UPPER_THRESHOLD;
  if (control_output < CONTROL_OUTPUT_LOWER_THRESHOLD) control_output = CONTROL_OUTPUT_LOWER_THRESHOLD;


  last_error = error;  // Update last error for next iteration

  // Serial.print("Control output : ");
  // Serial.print(control_output);

  return control_output;
}



void turnMotorsByDifferentSpeeds(int control_output, int t) {  // Function to Turn accordingly for PID controller
  // Serial.print("Control output : ");
  // Serial.print(control_output);

  int left_turn_speed = TURN_SPEED - control_output;
  int right_turn_speed = TURN_SPEED + control_output;

  left_turn_speed = left_turn_speed + LEFT_MOTOR_OFFSET_ADJUSTMENT;
  right_turn_speed = right_turn_speed + RIGHT_MOTOR_OFFSET_ADJUSTMENT;

  if (left_turn_speed < TURN_MOTORS_BY_DIFFERENT_SPEEDS_MOTOR_SPEEDS_LOWER_THRESHOLD) {
    left_turn_speed = 0;
  }
  if (left_turn_speed > TURN_MOTORS_BY_DIFFERENT_SPEEDS_MOTOR_SPEEDS_UPPER_THRESHOLD) {
    left_turn_speed = TURN_MOTORS_BY_DIFFERENT_SPEEDS_MOTOR_SPEEDS_UPPER_THRESHOLD + LEFT_MOTOR_OFFSET_ADJUSTMENT;
  }
  if (right_turn_speed < TURN_MOTORS_BY_DIFFERENT_SPEEDS_MOTOR_SPEEDS_LOWER_THRESHOLD) {
    right_turn_speed = 0;
  }
  if (right_turn_speed > TURN_MOTORS_BY_DIFFERENT_SPEEDS_MOTOR_SPEEDS_UPPER_THRESHOLD) {
    right_turn_speed = TURN_MOTORS_BY_DIFFERENT_SPEEDS_MOTOR_SPEEDS_UPPER_THRESHOLD + RIGHT_MOTOR_OFFSET_ADJUSTMENT;
  }

  // collision avoidance emergency front or left, change the speeds with directions of the tires
  if (goingToCollideFront || goingToCollideLeft || goingToCollideNorthWest) {
  // if (goingToCollideNorthWest) {
      Serial.print("\tEMERGENCY\t, NW Sensor reading : ");
      Serial.print(goingToCollideNorthWest);
      Serial.print("\t");
    

    analogWrite(LEFT_MOTOR_EN, TURN_SPEED * 2);
    analogWrite(RIGHT_MOTOR_EN, TURN_SPEED * 2);

    // turning on motors for opposite directions
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    delay(t);

    Serial.print("    Left Motor : ");
    Serial.print(TURN_SPEED);
    Serial.print("    Right Motor : ");
    Serial.println(-TURN_SPEED);


  } else {
    analogWrite(LEFT_MOTOR_EN, left_turn_speed);
    analogWrite(RIGHT_MOTOR_EN, right_turn_speed);

    // turning on motors for forward direction
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    delay(t);

    Serial.print("    Left Motor : ");
    Serial.print(left_turn_speed);
    Serial.print("    Right Motor : ");
    Serial.println(right_turn_speed);
  }

  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
}


void takeLineFollowingReadings() {
  int line_detection_sensor_center_reading = analogRead(LINE_DETECTION_SENSOR_CENTER);
  int line_detection_sensor_left_middle_reading = analogRead(LINE_DETECTION_SENSOR_LEFT_MIDDLE);
  int line_detection_sensor_left_corner_reading = analogRead(LINE_DETECTION_SENSOR_LEFT_CORNER);
  int line_detection_sensor_right_middle_reading = analogRead(LINE_DETECTION_SENSOR_RIGHT_MIDDLE);
  int line_detection_sensor_right_corner_reading = analogRead(LINE_DETECTION_SENSOR_RIGHT_CORNER);
  int line_detection_sensor_rear_reading = analogRead(LINE_DETECTION_SENSOR_REAR);

  // Sensor readings
  Serial.print("LC : ");
  Serial.print(line_detection_sensor_left_corner_reading);
  Serial.print("  \t");

  Serial.print("LM : ");
  Serial.print(line_detection_sensor_left_middle_reading);
  Serial.print("  \t");

  Serial.print("C : ");
  Serial.print(line_detection_sensor_center_reading);
  Serial.print("  \t");

  Serial.print("RM : ");
  Serial.print(line_detection_sensor_right_middle_reading);
  Serial.print("  \t");

  Serial.print("RC : ");
  Serial.print(line_detection_sensor_right_corner_reading);
  Serial.print("  \t");

  Serial.print("R : ");
  Serial.print(line_detection_sensor_rear_reading);
  Serial.print("  \t");




  if (line_detection_sensor_center_reading
      < ((LINE_DETECTION_SENSOR_CENTER_BLACK_VALUE + LINE_DETECTION_SENSOR_CENTER_WHITE_VALUE) / 2)) {
    is_line_detection_sensor_center_on_white = true;
  } else {
    is_line_detection_sensor_center_on_white = false;
  }

  if (line_detection_sensor_left_middle_reading
      < ((LINE_DETECTION_SENSOR_LEFT_MIDDLE_BLACK_VALUE + LINE_DETECTION_SENSOR_LEFT_MIDDLE_WHITE_VALUE) / 2)) {
    is_line_detection_sensor_left_middle_on_white = true;
  } else {
    is_line_detection_sensor_left_middle_on_white = false;
  }

  if (line_detection_sensor_left_corner_reading
      < ((LINE_DETECTION_SENSOR_LEFT_CORNER_BLACK_VALUE + LINE_DETECTION_SENSOR_LEFT_CORNER_WHITE_VALUE) / 2)) {
    is_line_detection_sensor_left_corner_on_white = true;
  } else {
    is_line_detection_sensor_left_corner_on_white = false;
  }

  if (line_detection_sensor_right_middle_reading
      < ((LINE_DETECTION_SENSOR_RIGHT_MIDDLE_BLACK_VALUE + LINE_DETECTION_SENSOR_RIGHT_MIDDLE_WHITE_VALUE) / 2)) {
    is_line_detection_sensor_right_middle_on_white = true;
  } else {
    is_line_detection_sensor_right_middle_on_white = false;
  }

  if (line_detection_sensor_right_corner_reading
      < ((LINE_DETECTION_SENSOR_RIGHT_CORNER_BLACK_VALUE + LINE_DETECTION_SENSOR_RIGHT_CORNER_WHITE_VALUE) / 2)) {
    is_line_detection_sensor_right_corner_on_white = true;
  } else {
    is_line_detection_sensor_right_corner_on_white = false;
  }

  if (line_detection_sensor_rear_reading
      < ((LINE_DETECTION_SENSOR_REAR_BLACK_VALUE + LINE_DETECTION_SENSOR_REAR_WHITE_VALUE) / 2)) {
    is_line_detection_sensor_rear_on_white = true;
  } else {
    is_line_detection_sensor_rear_on_white = false;
  }

  // printing the color of the line underneath each sensor
  Serial.print("Left_corner : ");
  if (is_line_detection_sensor_left_corner_on_white) {
    Serial.print("W");
  } else {
    Serial.print("B");
  }
  Serial.print(" \tLeft_middle : ");
  if (is_line_detection_sensor_left_middle_on_white) {
    Serial.print("W");
  } else {
    Serial.print("B");
  }
  Serial.print(" \tCenter : ");
  if (is_line_detection_sensor_center_on_white) {
    Serial.print("W");
  } else {
    Serial.print("B");
  }
  Serial.print("\tRight_middle : ");
  if (is_line_detection_sensor_right_middle_on_white) {
    Serial.print("W");
  } else {
    Serial.print("B");
  }
  Serial.print("\tRight_corner : ");
  if (is_line_detection_sensor_right_corner_on_white) {
    Serial.print("W");
  } else {
    Serial.print("B");
  }
  Serial.print("\tRear : ");
  if (is_line_detection_sensor_rear_on_white) {
    Serial.print("W");
  } else {
    Serial.print("B");
  }
  Serial.println();
}


void followLine() {

  //move forward if only the center sensor is on the white line
  if ((is_line_detection_sensor_center_on_white) && !(is_line_detection_sensor_left_middle_on_white) && !(is_line_detection_sensor_right_middle_on_white) && !(is_line_detection_sensor_left_corner_on_white) && !(is_line_detection_sensor_right_corner_on_white)) {
    moveForward(LINE_FOLLOWING_FORWARD_SPEED, LINE_FOLLOWING_FORWARD_TIME);
    return;
  }

  //move forward if all the middle sensors are on the white line
  if ((is_line_detection_sensor_center_on_white) && (is_line_detection_sensor_left_middle_on_white) && (is_line_detection_sensor_right_middle_on_white) && !(is_line_detection_sensor_left_corner_on_white) && !(is_line_detection_sensor_right_corner_on_white)) {
    moveForward(LINE_FOLLOWING_FORWARD_SPEED, LINE_FOLLOWING_FORWARD_TIME);
    return;
  }

  // if only left_middle is white, then lean left and correct it.
  if (!(is_line_detection_sensor_center_on_white) && (is_line_detection_sensor_left_middle_on_white) && !(is_line_detection_sensor_right_middle_on_white) && !(is_line_detection_sensor_left_corner_on_white) && !(is_line_detection_sensor_right_corner_on_white)) {
    lineFollowingCorrectionToLeft(LINE_FOLLOWING_CORRECTION_LEAN_LEFT_SPEED, LINE_FOLLOWING_CORRECTION_LEAN_LEFT_TIME);
    return;
  }

  // if center and left_middle are white, then lean left and correct it.
  if ((is_line_detection_sensor_center_on_white) && (is_line_detection_sensor_left_middle_on_white) && !(is_line_detection_sensor_right_middle_on_white) && !(is_line_detection_sensor_left_corner_on_white) && !(is_line_detection_sensor_right_corner_on_white)) {
    moveForward(LINE_FOLLOWING_FORWARD_SPEED, LINE_FOLLOWING_FORWARD_TIME);
    return;
  }

  // if left corner, center and left_middle are white, then lean left and correct it.
  if ((is_line_detection_sensor_center_on_white) && (is_line_detection_sensor_left_middle_on_white) && !(is_line_detection_sensor_right_middle_on_white) && (is_line_detection_sensor_left_corner_on_white) && !(is_line_detection_sensor_right_corner_on_white)) {
    moveForward(LINE_FOLLOWING_FORWARD_SPEED, LINE_FOLLOWING_FORWARD_TIME);
    return;
  }

  // if only right_middle is white, then lean right and correct it.
  if (!(is_line_detection_sensor_center_on_white) && !(is_line_detection_sensor_left_middle_on_white) && (is_line_detection_sensor_right_middle_on_white) && !(is_line_detection_sensor_left_corner_on_white) && !(is_line_detection_sensor_right_corner_on_white)) {
    lineFollowingCorrectionToRight(LINE_FOLLOWING_CORRECTION_LEAN_RIGHT_SPEED, LINE_FOLLOWING_CORRECTION_LEAN_RIGHT_TIME);
    return;
  }

  // if center and right_middle are white, then lean right and correct it.
  if ((is_line_detection_sensor_center_on_white) && !(is_line_detection_sensor_left_middle_on_white) && (is_line_detection_sensor_right_middle_on_white) && !(is_line_detection_sensor_left_corner_on_white) && !(is_line_detection_sensor_right_corner_on_white)) {
    moveForward(LINE_FOLLOWING_FORWARD_SPEED, LINE_FOLLOWING_FORWARD_TIME);
    return;
  }

  // if right corner, center and right_middle are white, then lean right and correct it.
  if ((is_line_detection_sensor_center_on_white) && !(is_line_detection_sensor_left_middle_on_white) && (is_line_detection_sensor_right_middle_on_white) && !(is_line_detection_sensor_left_corner_on_white) && (is_line_detection_sensor_right_corner_on_white)) {
    moveForward(LINE_FOLLOWING_FORWARD_SPEED, LINE_FOLLOWING_FORWARD_TIME);
    return;
  }

  // if only right corner sensor is working turn right.
  if (!(is_line_detection_sensor_center_on_white) && !(is_line_detection_sensor_left_middle_on_white) && !(is_line_detection_sensor_right_middle_on_white) && !(is_line_detection_sensor_left_corner_on_white) && (is_line_detection_sensor_right_corner_on_white)) {
    turnRightOnSameCenter(LINE_FOLLOWING_TURN_RIGHT_SPEED, LINE_FOLLOWING_TURN_RIGHT_TIME);
    moveForward(LINE_FOLLOWING_FORWARD_AFTER_TURN_SPEED, LINE_FOLLOWING_FORWARD_AFTER_TURN_TIME);
    return;
  }

  // if only left corner sensor is working turn left.
  if (!(is_line_detection_sensor_center_on_white) && !(is_line_detection_sensor_left_middle_on_white) && !(is_line_detection_sensor_right_middle_on_white) && (is_line_detection_sensor_left_corner_on_white) && !(is_line_detection_sensor_right_corner_on_white)) {
    turnLeftOnSameCenter(LINE_FOLLOWING_TURN_LEFT_SPEED, LINE_FOLLOWING_TURN_LEFT_TIME);
    moveForward(LINE_FOLLOWING_FORWARD_AFTER_TURN_SPEED, LINE_FOLLOWING_FORWARD_AFTER_TURN_TIME);
    return;
  }

  // if 5 main sensors are BLACK, move forward a little bit.
  if (!(is_line_detection_sensor_center_on_white) && !(is_line_detection_sensor_left_middle_on_white) && !(is_line_detection_sensor_right_middle_on_white) && !(is_line_detection_sensor_left_corner_on_white) && !(is_line_detection_sensor_right_corner_on_white) && (is_line_detection_sensor_rear_on_white)) {
    moveForward(LINE_FOLLOWING_MOVE_FORWARD_A_BIT_SPEED, LINE_FOLLOWING_MOVE_FORWARD_A_BIT_TIME);
    return;
  }

  // if left middle and right middle are white forward a little bit.
  if (!(is_line_detection_sensor_center_on_white) && (is_line_detection_sensor_left_middle_on_white) && (is_line_detection_sensor_right_middle_on_white) && !(is_line_detection_sensor_left_corner_on_white) && !(is_line_detection_sensor_right_corner_on_white)) {
    moveForward(LINE_FOLLOWING_MOVE_FORWARD_A_BIT_SPEED, LINE_FOLLOWING_MOVE_FORWARD_A_BIT_TIME);
    return;
  }

  // if all the sensors are BLACK including the rear, DEAD-END.
  if (!(is_line_detection_sensor_center_on_white) && !(is_line_detection_sensor_left_middle_on_white) && !(is_line_detection_sensor_right_middle_on_white) && !(is_line_detection_sensor_left_corner_on_white) && !(is_line_detection_sensor_right_corner_on_white) && !(is_line_detection_sensor_rear_on_white)) {
    moveBackward(LINE_FOLLOWING_MOVE_BACKWARD_AT_DEAD_END_SPEED, LINE_FOLLOWING_MOVE_BACKWARD_AT_DEAD_END_TIME);
    turnLeftOnSameCenter(LINE_FOLLOWING_TURN_LEFT_SPEED, LINE_FOLLOWING_TURN_LEFT_TIME - 100);
    turnLeftOnSameCenter(LINE_FOLLOWING_TURN_LEFT_SPEED, LINE_FOLLOWING_TURN_LEFT_TIME - 100);
    delay(200);
    return;
  }


  // if left corner, center are white, left junction
  if ((is_line_detection_sensor_center_on_white) && (is_line_detection_sensor_left_corner_on_white) && !(is_line_detection_sensor_right_corner_on_white)) {
    turnLeftOnSameCenter(LINE_FOLLOWING_TURN_LEFT_SPEED, LINE_FOLLOWING_TURN_LEFT_TIME);
    moveForward(LINE_FOLLOWING_FORWARD_AFTER_TURN_SPEED, LINE_FOLLOWING_FORWARD_AFTER_TURN_TIME);
    return;
  }

  // // if right corner, center are white, right junction
  // if ((is_line_detection_sensor_center_on_white) && !(is_line_detection_sensor_left_corner_on_white) && (is_line_detection_sensor_right_corner_on_white)) {
  //   turnRightOnSameCenter(LINE_FOLLOWING_TURN_RIGHT_SPEED, LINE_FOLLOWING_TURN_RIGHT_TIME);
  //   moveForward(LINE_FOLLOWING_FORWARD_AFTER_TURN_SPEED, LINE_FOLLOWING_FORWARD_AFTER_TURN_TIME);
  //   return;
  // }



  // if right corner, center, left corner are white, T junction, turn left.
  if ((is_line_detection_sensor_center_on_white) && (is_line_detection_sensor_left_corner_on_white) && (is_line_detection_sensor_right_corner_on_white) && (!(is_line_detection_sensor_right_middle_on_white) || !(is_line_detection_sensor_left_middle_on_white))) {
    turnLeftOnSameCenter(LINE_FOLLOWING_TURN_LEFT_SPEED, LINE_FOLLOWING_TURN_LEFT_TIME);
    moveForward(LINE_FOLLOWING_FORWARD_AFTER_TURN_SPEED, LINE_FOLLOWING_FORWARD_AFTER_TURN_TIME);
    return;
  }


  // if all are white, then END
  if ((is_line_detection_sensor_center_on_white) && (is_line_detection_sensor_left_middle_on_white) && (is_line_detection_sensor_right_middle_on_white) && (is_line_detection_sensor_left_corner_on_white) && (is_line_detection_sensor_right_corner_on_white) && (is_line_detection_sensor_rear_on_white)) {
    stopAllMotors();
    if (!completedExploreRun) {
      if ((millis() - startedTime) < 10000) {
        startedTime = millis();
        moveForward(LINE_FOLLOWING_FORWARD_SPEED, LINE_FOLLOWING_FORWARD_TIME);
        return;
      }
      digitalWrite(indicatorLEDPin, HIGH);
      delay(1000);
      digitalWrite(indicatorLEDPin, LOW);
      delay(1000);
      digitalWrite(indicatorLEDPin, HIGH);
      delay(1000);
      digitalWrite(indicatorLEDPin, LOW);
      delay(1000);
      digitalWrite(indicatorLEDPin, HIGH);
      delay(1000);
      digitalWrite(indicatorLEDPin, LOW);
      delay(1000);
      completedExploreRun = true;
      explorationFinishedTime = millis();
      moveBackward(LINE_FOLLOWING_MOVE_BACKWARD_AT_DEAD_END_SPEED, LINE_FOLLOWING_MOVE_BACKWARD_AT_DEAD_END_TIME);
      turnLeftOnSameCenter(LINE_FOLLOWING_TURN_LEFT_SPEED, LINE_FOLLOWING_TURN_LEFT_TIME - 100);
      turnLeftOnSameCenter(LINE_FOLLOWING_TURN_LEFT_SPEED, LINE_FOLLOWING_TURN_LEFT_TIME - 100);
      delay(100);
      return;
    }
    if ((millis() - explorationFinishedTime) < 10000) {
      explorationFinishedTime = millis();
      moveBackward(LINE_FOLLOWING_FORWARD_SPEED, LINE_FOLLOWING_FORWARD_TIME);
      // lineFollowingCorrectionToRight(LINE_FOLLOWING_CORRECTION_LEAN_RIGHT_SPEED, LINE_FOLLOWING_CORRECTION_LEAN_RIGHT_TIME);
      return;
    } else {
      lineFollowingDone = true;
      moveForward(LINE_FOLLOWING_FORWARD_SPEED, LINE_FOLLOWING_FORWARD_TIME * 5);
      return;
    }
  }


  moveBackward(LINE_FOLLOWING_MOVE_FORWARD_A_BIT_SPEED, LINE_FOLLOWING_MOVE_FORWARD_A_BIT_TIME * 2);
  lineFollowingCorrectionToRight(LINE_FOLLOWING_MOVE_FORWARD_A_BIT_SPEED, 50);
}
void takeWallReadings() {
  left_distance_cm = leftIRSensor.distance();
  front_distance_cm = frontIRSensor.distance();
  Serial.print("Left Sensor : ");
  Serial.print(left_distance_cm);
  Serial.print("\t");
  Serial.print("Front Sensor : ");
  Serial.print(front_distance_cm);
  Serial.print("\t");
  if ((digitalRead(collisionSensorLeft) == LOW) && (left_distance_cm < 10)) {
    goingToCollideLeft = true;
  }
  if ((digitalRead(collisionSensorLeft) == HIGH) || (left_distance_cm > 12)) {
    goingToCollideLeft = false;
  }
  if ((digitalRead(collisionSensorFront) == LOW) && (front_distance_cm < 10)) {
    goingToCollideFront = true;
  }
  if (digitalRead(collisionSensorFront) == HIGH) {
    goingToCollideFront = false;
  }
  if (digitalRead(collisionSensorNorthWest) == LOW) {
    goingToCollideNorthWest = true;
  }
  if (digitalRead(collisionSensorNorthWest) == HIGH) {
    goingToCollideNorthWest = false;
  }
}


void moveForward(int speed, int t) {

  analogWrite(LEFT_MOTOR_EN, speed);
  analogWrite(RIGHT_MOTOR_EN, speed);

  // forward direction
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);

  delay(t);

  stopAllMotors();
}

void moveBackward(int speed, int t) {
  analogWrite(LEFT_MOTOR_EN, speed);
  analogWrite(RIGHT_MOTOR_EN, speed);

  // backward direction
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);

  delay(t);

  stopAllMotors();
}


void turnLeftOnSameCenter(int speed, int t) {
  // moveForward(speed, t/2);
  // stopAllMotors();
  analogWrite(LEFT_MOTOR_EN, speed);
  analogWrite(RIGHT_MOTOR_EN, speed);

  // right motor forward direction, left motor backward direction
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
  delay(t);

  stopAllMotors();
}

void turnRightOnSameCenter(int speed, int t) {
  // moveForward(speed, t/2);
  // stopAllMotors();
  analogWrite(LEFT_MOTOR_EN, speed);
  analogWrite(RIGHT_MOTOR_EN, speed);

  // right motor backward direction, left motor forward direction
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  delay(t);

  stopAllMotors();
}


void stopAllMotors() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
}


void lineFollowingCorrectionToRight(int speed, int t) {
  stopAllMotors();
  // moveBackward(speed, t);
  // delay(t);
  turnRightOnSameCenter(speed, t);
  // delay(t);
  moveForward(speed, t);
  // delay(t);
  // turnLeftOnSameCenter(speed, t);
}


void lineFollowingCorrectionToLeft(int speed, int t) {
  stopAllMotors();
  // moveBackward(speed, t);
  // delay(t);
  turnLeftOnSameCenter(speed, t);
  // delay(t);
  moveForward(speed, t);
  // delay(t);
  // turnRightOnSameCenter(speed, t);
}

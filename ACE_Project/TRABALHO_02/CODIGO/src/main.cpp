#include <Arduino.h>
#include <WiFi.h>

#include "pico/cyw43_arch.h"

#include <Wire.h>
#include <VL53L0X.h>

/*--------DEFINE--------*/
// Led
#define CYW43_WL_GPIO_LED_PIN 0

// Encoder pins
#define ENCODER_PIN_A 0  // GPIO pin connected to Channel A
#define ENCODER_PIN_B 1  // GPIO pin connected to Channel B

// Motor 1 pins
#define DIR1_PIN 16
#define PWM1_PIN 17
#define DIR2_PIN 18
#define PWM2_PIN 19

// Line sensors pin
#define NUM_SENSORS 5
#define THRESHOLD 700
#define MAX_SENSOR_VALUE 1023
#define MIN_SENSOR_VALUE 0
//#define SENSOR_D1 28  //Right sensor
//#define SENSOR_D4 27  //Center sensor
//#define SENSOR_D8 26  //Left sensor
#define MUX_S1 5   
#define MUX_S2 6
#define MUX_S3 7
#define MUX_S4 8
#define MUX_SIGNAL 28

// Distance sensor pins
#define DIST_SENSOR_SDA 20
#define DIST_SENSOR_SCL 21

// Maze
#define ROWS 6
#define COLUMNS 5

// Test
#define MOTOR_SPEED 100
#define COMPENSATION 0.05
#define DIST_OBJECT 0.14
#define TICKS_TURN_90 800 //"-" to go right and "+" to go left
/*----------------------*/

/*--------VARIABLES--------*/
VL53L0X tof;
float distance, prev_distance;

int LED_state;
unsigned long interval = 40 * 1000;
unsigned long currentMicros, previousMicros;
int loop_count;

bool start = true;

enum{
  FOLLOW_LINE,
  MAZE_SOLVER,
  DEBUG
};
int carMode = FOLLOW_LINE;
/*-------------------------*/

/*-------------ENCODERS------------*/
volatile int encPosition = 0; // Track the encoder position
int lastEncChannelA, lastEncChannelB = 0;
bool encDirection = false;

bool turning = false;
int targetEncPosition = 0;
/*---------------------------------*/

/*-------------SENSORS------------*/
int sensorValues[NUM_SENSORS] = {0};
int lastSensorValues[NUM_SENSORS] = {0};
int sensorMinValue = MAX_SENSOR_VALUE/2;
int sensorMaxValue = MAX_SENSOR_VALUE/2;
const int sensorWeights[NUM_SENSORS] = {-2, -1, 0, 1, 2}; //Change this according to the number of sensors
/*--------------------------------*/

/*-------------MOTORS------------*/
enum {
  FORWARD,
  REVERSE
};
enum{
  MOTOR1,
  MOTOR2
};
bool motorDir[2] = {REVERSE};
int motorSpeed[2] = {0};     // Value from 0 to 255
/*-------------------------------*/

/*-------------PID------------*/
float Kp = 1.4;   // Tune these values - 0.4 before
float Ki = 0.025;   // Start with 0 for integral, then increase if you need less steady-state error
float Kd = 0.025;//0.025;   // Tune for stability (helps reduce overshoot)

float setpoint = 3; // Middle sensor
float integral = 0.0;
float lastError = 0.0;
unsigned long lastTime = 0;
int correction = 0;

float pastError = 0.0; //Speed increment
/*----------------------------*/

/*--------------OBSTACLES-------------*/
enum{
  ALIGN,
  STOP,
  FRONT,
  RIGHT,
  END
};
int obstacleState = ALIGN;
unsigned long obstacleTimer = 0;
/*------------------------------------*/

/*--------------MAZE-------------*/
bool mazeGrid[ROWS][COLUMNS] = {true};   //If it's "true" then the intersection is free
bool visited[ROWS][COLUMNS] = {false};
int directions[4][2] = {
  {-1,  0},  // down
  { 1,  0},  // up
  { 0, -1},  // right
  { 0,  1}   // left
};
enum{
  NORTH,
  EAST,
  SOUTH,
  WEST
};
int currDir = NORTH;
int desiredDir = NORTH;

int endRow = ROWS - 1;
int endCol = COLUMNS - 1;
int currRow = 0;
int currCol = 0;

int parentRow[ROWS][COLUMNS];
int parentCol[ROWS][COLUMNS];
static int pathIndex = 0;

unsigned long mazeTimer = 0;
bool intersection = false;
bool atIntersection = false;
#define MAZE_ADJUST 325     //250

enum{
  MAZE_STOP,
  MAZE_EXECUTE_PATH,
  MAZE_CALCULATE_SHORTEST_PATH,
  MAZE_ALIGN_CAR,
  MAZE_TURN_CAR,
  MAZE_DETECT_OBSTACLE,
  MAZE_MOVE_CAR,
  MAZE_ADJUST_CAR,
  MAZE_END
};
int mazeState = MAZE_CALCULATE_SHORTEST_PATH;

// For BFS, we can store up to ROWS*COLUMNS cells in the queue
#define MAX_QUEUE (ROWS * COLUMNS)

// We define a small struct for row,col
struct Cell {
  int r;
  int c;
};

// The ring-buffer queue
Cell queueArray[MAX_QUEUE];
int queueFront = 0;
int queueBack  = 0;
int queueCount = 0;

// A simple path array to store the final route
#define MAX_PATH (ROWS * COLUMNS) 
int pathR[MAX_PATH];
int pathC[MAX_PATH];
int pathLength = 0;
/*-------------------------------*/

/*-------------MATH METHODS------------*/
template <typename T>
T abs(T value) {
  return (value < 0) ? -value : value;
}
/*-------------------------------------*/

void connectToWifi()
{
  // Connect to Wi-Fi
  WiFi.begin("iPhone do Ivo", "cabuflamanau");

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("Connecting to Wi-Fi...");
  }

  //DEBUG
  Serial.println("Connected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void encReader() {
  //Serial.println("Interrupt Triggered!");
  int encChannelA = digitalRead(ENCODER_PIN_A); // Read Channel A

  if((lastEncChannelA == LOW) && encChannelA==HIGH)
  {
    int encChannelB = digitalRead(ENCODER_PIN_B); // Read Channel B
    if(encChannelB == LOW && encDirection)
    {
      encDirection = false; //Reverse
    }
    else if(encChannelB == HIGH && !encDirection)
    {
      encDirection = true;  //Forward
    }
  }
  lastEncChannelA = encChannelA;

  if(encDirection)  encPosition++;
  else  encPosition--;

  // Debug
  /*
  if(encPosition % (960*2) == 0)
  {
    Serial.print("Number of revolutions: ");
    Serial.println(encPosition/(960*2));

    Serial.print("Travel distance: ");
    Serial.println( 2*3.1416 * 7 * (encPosition/(960*2)) );
  }*/
}

void motorControl()
{
  // Set motor direction
  digitalWrite(DIR1_PIN, motorDir[MOTOR1]);
  digitalWrite(DIR2_PIN, motorDir[MOTOR2]);

  // Set motor speed
  analogWrite(PWM1_PIN, motorSpeed[MOTOR1]);
  analogWrite(PWM2_PIN, motorSpeed[MOTOR2]);
}

void readLineSensors() {

  for (int i = 0; i < NUM_SENSORS; i++) {
    // Set the multiplexer to the current channel
    digitalWrite(MUX_S1, i & 0x01);        // Bit 0
    digitalWrite(MUX_S2, (i >> 1) & 0x01); // Bit 1
    digitalWrite(MUX_S3, (i >> 2) & 0x01); // Bit 2
    digitalWrite(MUX_S4, (i >> 3) & 0x01); // Bit 3

    delayMicroseconds(50); // Allow time for the signal to stabilize

    // Read the sensor value
    sensorValues[i] = analogRead(MUX_SIGNAL);

    if(sensorValues[i] > sensorMaxValue) sensorMaxValue = sensorValues[i];
    else if(sensorValues[i] < sensorMinValue) sensorMinValue = sensorValues[i];
  }

  if(carMode == FOLLOW_LINE){
    if(sensorValues[0]>THRESHOLD && sensorValues[1]>THRESHOLD && sensorValues[2]>THRESHOLD && sensorValues[3]>THRESHOLD && sensorValues[4]>THRESHOLD)
    {
      for (int i = 0; i < NUM_SENSORS; i++)
      {
        sensorValues[i] = lastSensorValues[i];
      }
    }
    else if (sensorValues[0]<THRESHOLD && sensorValues[1]<THRESHOLD && sensorValues[2]<THRESHOLD && sensorValues[3]<THRESHOLD && sensorValues[4]<THRESHOLD)
    {
      for (int i = 0; i < NUM_SENSORS; i++)
      {
        sensorValues[i] = lastSensorValues[i];
      }
    }
  }
  else{
    if( atIntersection && ((lastSensorValues[0]<THRESHOLD && sensorValues[0]>THRESHOLD) || (lastSensorValues[NUM_SENSORS-1]<THRESHOLD && sensorValues[NUM_SENSORS-1]>THRESHOLD)) ){
      Serial.println("Intersection found!");
      intersection = true;
    }
    if ( (sensorValues[0]<THRESHOLD && sensorValues[1]<THRESHOLD && sensorValues[2]<THRESHOLD) || (sensorValues[NUM_SENSORS-3]<THRESHOLD && sensorValues[NUM_SENSORS-2]<THRESHOLD && sensorValues[NUM_SENSORS-1]<THRESHOLD))
    {
      atIntersection = true;
    }
  }
  

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    lastSensorValues[i] = sensorValues[i];
  }
  
}

float calculateMediumSensor() {
  float weightedSum = 0;
  float totalInvertedValues = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    //float invertedValue = map(sensorValues[i], MIN_SENSOR_VALUE, MAX_SENSOR_VALUE, sensorMaxValue, sensorMinValue);
    float invertedValue = sensorMaxValue - sensorValues[i];    //Black gets a higher value than white 

    //Weighted sum and total inverted values
    weightedSum += invertedValue * sensorWeights[i];
    totalInvertedValues += invertedValue;
  }

  if (totalInvertedValues == 0) {
    return round((float)NUM_SENSORS/2); //Default to center position if no line is detected
  }

  return (weightedSum / totalInvertedValues) + round((float)NUM_SENSORS/2); //Average + Offsets posLine to the real position

}

void calculatePWMAndDir(float posLine) {
  // Compute error
  float error = posLine - setpoint;

  // Compute time difference for integral and derivative
  unsigned long now = micros();
  float dt = (lastTime == 0) ? 0.02 : (now - lastTime) / 1000000.0; // Default to 20ms if lastTime=0
  lastTime = now;

  // Calculate integral (sum of errors over time)
  integral += error * dt;

  // Calculate derivative (rate of change of error)
  float derivative = (error - lastError) / dt;

  // Adjust motor speed dynamically based on error
  float dynamicSpeed = MOTOR_SPEED;
  if (abs(error) < 0.5) {
    dynamicSpeed = MOTOR_SPEED * map(abs(error), 0.0, 0.5, 1.0 + pastError, 1.0); //1.5
  } else if (abs(error) > 0.5) {
    dynamicSpeed = MOTOR_SPEED * 0.8; //0.5
  }

  //PID
  float correction = (Kp * dynamicSpeed) * error + (Ki * dynamicSpeed) * integral + (Kd * dynamicSpeed) * derivative;

  // Store the current error as the last error for the next iteration
  lastError = error;

  // Adjust motor speeds based on correction
  if (error > 0 + COMPENSATION) {
    if (correction >= dynamicSpeed) {
      motorDir[MOTOR1] = REVERSE;
      correction -= dynamicSpeed;
    } else {
      motorDir[MOTOR1] = FORWARD;
    }
    motorSpeed[MOTOR1] = abs(round(correction));
    motorSpeed[MOTOR2] = dynamicSpeed;
    motorDir[MOTOR2] = FORWARD;

    pastError -= 0.1;
  } else if (error < 0 - COMPENSATION) {
    if (correction <= -dynamicSpeed) {
      motorDir[MOTOR2] = REVERSE;
      correction += dynamicSpeed;
    } else {
      motorDir[MOTOR2] = FORWARD;
    }
    motorSpeed[MOTOR2] = abs(round(correction));
    motorSpeed[MOTOR1] = dynamicSpeed;
    motorDir[MOTOR1] = FORWARD;

    pastError -= 0.1;
  } else {
    motorDir[MOTOR1] = motorDir[MOTOR2] = FORWARD;
    motorSpeed[MOTOR1] = motorSpeed[MOTOR2] = dynamicSpeed;

    pastError += 0.1;
  }

  // Ensure motor speeds are within valid range
  motorSpeed[MOTOR1] = constrain(motorSpeed[MOTOR1], 0, 255);
  motorSpeed[MOTOR2] = constrain(motorSpeed[MOTOR2], 0, 255); //25
  pastError = constrain(pastError, 0, 1.5);
}

bool alignRobot(){
  readLineSensors();
  float posLine = calculateMediumSensor();
  float error   = posLine - setpoint;

  //Align robot to center of line
  if (abs(error) <= 0.01)
  {
    motorSpeed[MOTOR1] = 0;
    motorSpeed[MOTOR2] = 0;

    return true;
  }
  else
  {
    int rotateSpeed = MOTOR_SPEED * 0.5;

    if (error > 0)
    {
      motorSpeed[MOTOR1] = rotateSpeed;
      motorDir[MOTOR1]   = REVERSE;
      
      motorSpeed[MOTOR2] = rotateSpeed;
      motorDir[MOTOR2]   = FORWARD;
    }
    else
    {
      motorSpeed[MOTOR1] = rotateSpeed;
      motorDir[MOTOR1]   = FORWARD;
      
      motorSpeed[MOTOR2] = rotateSpeed;
      motorDir[MOTOR2]   = REVERSE;
    }

    return false;
  }
}

void obstacleAvoidance(unsigned long currentMillis){
  switch (obstacleState)
  {
    case ALIGN:
      if (alignRobot())
      {
        obstacleState = STOP;
        obstacleTimer = currentMillis;
      }
      break;
      
    case STOP:
      motorSpeed[MOTOR1] = 0;
      motorSpeed[MOTOR2] = 0;
      if (currentMillis - obstacleTimer >= 500) {
        obstacleState = RIGHT;
        obstacleTimer = currentMillis;
      }
      break;

    case RIGHT:
      /*
      if (currentMillis - obstacleTimer >= 1000) {
        obstacleState = FRONT;
        obstacleTimer = currentMillis;
      } else {
        motorSpeed[MOTOR1] = MOTOR_SPEED/2;
        motorDir[MOTOR1]   = REVERSE;

        motorSpeed[MOTOR2] = MOTOR_SPEED/2;
        motorDir[MOTOR2]   = FORWARD;
      }*/
      if (!turning) {
        turning           = true;
        targetEncPosition = encPosition - TICKS_TURN_90; 
      }

      if (encPosition <= targetEncPosition) {
        turning = false;
        motorSpeed[MOTOR1] = 0;
        motorSpeed[MOTOR2] = 0;

        obstacleState = FRONT;
        obstacleTimer = currentMillis;
      }
      else {
        motorSpeed[MOTOR1] = MOTOR_SPEED/2;
        motorDir[MOTOR1]   = REVERSE;

        motorSpeed[MOTOR2] = MOTOR_SPEED/2;
        motorDir[MOTOR2]   = FORWARD;
      }
      break;

    case FRONT:
      /*
      if (currentMillis - obstacleTimer >= 3250) {
        obstacleState = END;
        obstacleTimer = currentMillis;
      } else {
        motorSpeed[MOTOR1] = MOTOR_SPEED;
        motorDir[MOTOR1]   = FORWARD;

        motorSpeed[MOTOR2] = MOTOR_SPEED/2;
        motorDir[MOTOR2]   = FORWARD;
      }*/
      if (!turning) {
        turning           = true;
        targetEncPosition = encPosition + 7800; 
      }

      if (encPosition >= targetEncPosition) {
        turning = false;

        obstacleState = END;
        obstacleTimer = currentMillis;
      }
      else {
        motorSpeed[MOTOR1] = MOTOR_SPEED;
        motorDir[MOTOR1]   = FORWARD;

        motorSpeed[MOTOR2] = MOTOR_SPEED/2;
        motorDir[MOTOR2]   = FORWARD;
      }
      break;

    case END:
      readLineSensors();
      if (currentMillis - obstacleTimer >= 1000) {
        start = true;
        obstacleState = STOP;
      } else {
        motorSpeed[MOTOR1] = MOTOR_SPEED/2;
        motorDir[MOTOR1]   = REVERSE;

        motorSpeed[MOTOR2] = MOTOR_SPEED/2;
        motorDir[MOTOR2]   = FORWARD;
      }
      break;
  }
}

/*------------------------MAZE_SOLVER------------------------*/
void clearQueue() {
  queueFront = 0;
  queueBack  = 0;
  queueCount = 0;
}

bool enqueue(int rr, int cc) {
  // If the queue is full, return false
  if (queueCount >= MAX_QUEUE) {
    return false;
  }
  // Place the cell in queueArray at 'queueBack'
  queueArray[queueBack].r = rr;
  queueArray[queueBack].c = cc;
  // Advance 'queueBack' in a circular way
  queueBack = (queueBack + 1) % MAX_QUEUE;
  // Increment the count
  queueCount++;
  return true;
}

bool dequeue(int *outR, int *outC) {
  if (queueCount <= 0) {
    // Queue is empty
    return false;
  }
  // The front cell
  *outR = queueArray[queueFront].r;
  *outC = queueArray[queueFront].c;
  // Advance front index
  queueFront = (queueFront + 1) % MAX_QUEUE;
  queueCount--;
  return true;
}

bool bfsComputePath(int startR, int startC, int goalR, int goalC) {
  // Reset visited + parent arrays
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLUMNS; c++) {
      visited[r][c]     = false;
      parentRow[r][c]   = -1;
      parentCol[r][c]   = -1;
    }
  }

  clearQueue();

  visited[startR][startC] = true;
  enqueue(startR, startC);

  // BFS loop
  while (queueCount > 0) {
    int r, c;
    if (!dequeue(&r, &c)) {
      // Should not happen if queueCount > 0, but just in case
      break;
    }

    // If at the end of maze
    if (r == goalR && c == goalC) {
      return true;
    }

    // Check the 4 possible directions
    for (int i = 0; i < 4; i++) {
      int nr = r + directions[i][0];
      int nc = c + directions[i][1];

      // Check bounds
      if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLUMNS) {
        // If the cell is free and not visited
        if (mazeGrid[nr][nc] && !visited[nr][nc]) {
          visited[nr][nc] = true;
          parentRow[nr][nc] = r;
          parentCol[nr][nc] = c;
          enqueue(nr, nc);
        }
      }
    }
  }

  // If we exit the loop without finding the goal, no path
  return false;
}

void reconstructPath(int startR, int startC, int goalR, int goalC) {
  pathLength = 0;

  // If the goal wasn't visited, no path
  if (!visited[goalR][goalC]) {
    return;
  }

  // We'll store the path in reverse order first in a temporary buffer
  // (goal -> start), then we'll flip it.
  int tempRow[MAX_PATH];
  int tempCol[MAX_PATH];
  int count = 0;

  // Start from goal
  int r = goalR;
  int c = goalC;

  while (!(r == startR && c == startC)) {
    tempRow[count] = r;
    tempCol[count] = c;
    count++;

    // Jump to the parent cell
    int pr = parentRow[r][c];
    int pc = parentCol[r][c];
    r = pr;
    c = pc;
  }
  // Finally add the start node
  tempRow[count] = startR;
  tempCol[count] = startC;
  count++;

  // Now 'count' is the number of cells in the path
  // Reverse them into pathR[], pathC[] to get start->goal order
  pathLength = count;
  int idx = 0;
  for (int i = count - 1; i >= 0; i--) {
    pathR[idx] = tempRow[i];
    pathC[idx] = tempCol[i];
    idx++;
  }
}


bool turn90(int side){ //"0" to go right and "1" to go left
  switch (side)
  {
  case 0:{
    if (!turning) {
      turning           = true;
      targetEncPosition = encPosition - TICKS_TURN_90;

      // Print encoder position
      noInterrupts();
      if(micros() % 10*interval == 0) {
        Serial.print("Encoder Position: ");
        Serial.println(encPosition);
        Serial.print("Target Encoder Position: ");
        Serial.println(targetEncPosition);
      }
      interrupts(); 

      motorSpeed[MOTOR1] = MOTOR_SPEED/2;
      motorDir[MOTOR1]   = REVERSE;

      motorSpeed[MOTOR2] = MOTOR_SPEED/2;
      motorDir[MOTOR2]   = FORWARD;

      motorControl();
    }

    if (encPosition <= targetEncPosition) {
      turning = false;
      motorSpeed[MOTOR1] = 0;
      motorSpeed[MOTOR2] = 0;

      motorControl();

      if(currDir == WEST) currDir = NORTH;
      else currDir++; 

      Serial.print("Current Direction: "); Serial.println(currDir);
      Serial.print("Desired Direction: "); Serial.println(desiredDir);

      return true;
    }
    break;
  }

  case 1:{
    if (!turning) {
      turning           = true;
      targetEncPosition = encPosition + TICKS_TURN_90; 

      // Print encoder position
      noInterrupts();
      if(micros() % 10*interval == 0) {
        Serial.print("Encoder Position: ");
        Serial.println(encPosition);
        Serial.print("Target Encoder Position: ");
        Serial.println(targetEncPosition);
      }
      interrupts(); 

      motorSpeed[MOTOR1] = MOTOR_SPEED/2;
      motorDir[MOTOR1]   = FORWARD;

      motorSpeed[MOTOR2] = MOTOR_SPEED/2;
      motorDir[MOTOR2]   = REVERSE;

      motorControl();
    }

    if (encPosition >= targetEncPosition) {
      turning = false;
      motorSpeed[MOTOR1] = 0;
      motorSpeed[MOTOR2] = 0;

      motorControl();

      if(currDir == NORTH) currDir = WEST;
      else currDir--; 

      Serial.print("Current Direction: "); Serial.println(currDir);
      Serial.print("Desired Direction: "); Serial.println(desiredDir);

      return true;
    }
    break;
  }
  
  default:
    break;
  }

  return false;
}

float mazeCalculateMediumSensor(){
  float weightedSum = 0.0f;
  float totalInvertedValues = 0.0f;

  int localWeights[5] = {0, -1, 0, 1, 0};

  for (int i = 1; i <= 3; i++) {
    float invertedValue = sensorMaxValue - sensorValues[i];
    weightedSum        += (invertedValue * localWeights[i]);
    totalInvertedValues += invertedValue;
  }

  if(atIntersection) return 3.0f;

  if (totalInvertedValues == 0.0f) {
    return 3.0f;
  }

  float ratio = (weightedSum / totalInvertedValues) + 3.0f;
  return ratio;

}

bool goToNextIntersection(int speed){
  readLineSensors();

  if( intersection ) {//millis() - mazeTimer >= 2000 ){
    switch (currDir)
    {
    case NORTH:
      currRow++;
      break;
      
    case SOUTH:
      currRow--;
      break;

    case EAST:
      currCol--;
      break;

    case WEST:
      currCol++;
      break;
    }

    motorDir[MOTOR1] = motorDir[MOTOR2] = FORWARD;
    motorSpeed[MOTOR1] = motorSpeed[MOTOR2] = 0;
    motorControl();

    intersection = false;
    atIntersection = false;

    return false;
  }
  else{
    float posLine = mazeCalculateMediumSensor();

    float error = posLine - setpoint;

    //if(abs(error) >= 0.3) error = 0;

    if (error > 0 + COMPENSATION) {
      motorSpeed[MOTOR1] = 0;
      motorDir[MOTOR1] = FORWARD;

      motorSpeed[MOTOR2] = speed;
      motorDir[MOTOR2] = FORWARD;
    } else if (error < 0 - COMPENSATION) {
      motorSpeed[MOTOR2] = 0;
      motorDir[MOTOR2] = FORWARD;

      motorSpeed[MOTOR1] = speed;
      motorDir[MOTOR1] = FORWARD;
    } else {
      motorDir[MOTOR1] = motorDir[MOTOR2] = FORWARD;
      motorSpeed[MOTOR1] = motorSpeed[MOTOR2] = speed;
    }

    motorControl();

    return true;
  }
}

bool detectObstacle(){
  if ( (prev_distance <= DIST_OBJECT * 2.5) && (distance <= DIST_OBJECT * 1.5) ){
    return true;
  }
  else return false;
}

/*-----------------------------------------------------------*/

void debug(float posLine)
{
  // Toggle builtin LED    
  loop_count++;
  if (loop_count > 5) {
    LED_state = !LED_state;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, LED_state);
    loop_count = 0;
  }

  // Print sensor's distance to object
  Serial.print("Dist: ");
  Serial.print(distance, 3);
  Serial.println();

  // Print encoder position
  noInterrupts();
  if(encPosition % 10 == 0)
  {
    //Serial.print("Encoder Position: ");
    Serial.print("Number of revolutions: ");
    Serial.println(encPosition/2000);
  }
  Serial.print("Encoder Position: ");
  Serial.println(encPosition);
  interrupts();

  //Print sensor values
  Serial.print("Sensor Values: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.println();

  //Print motor data
  Serial.println("-----------------");
  Serial.print("Line position: ");
  Serial.println(posLine);
  Serial.print("Speed Motor 1: ");
  Serial.println(motorSpeed[MOTOR1]);
  Serial.print("Speed Motor 2: ");
  Serial.println(motorSpeed[MOTOR2]);
  Serial.print("Direction Motor 1: ");
  Serial.println(motorDir[MOTOR1]);
  Serial.print("Direction Motor 2: ");
  Serial.println(motorDir[MOTOR2]);
  Serial.println("-----------------");
  
}

void setup() 
{
  Serial.begin(115200);

  /*-------------DISTANCE SENSOR-------------*/
  Wire.setSDA(DIST_SENSOR_SDA);
  Wire.setSCL(DIST_SENSOR_SCL);
  Wire.begin();

  tof.setTimeout(500);

  //DEBUG
  delay(5000);
  
  while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  }
  Serial.println(F("Initialized VL53L0X!"));

  // Reduce timing budget to 20 ms (default is about 33 ms)
  tof.setMeasurementTimingBudget(20000);

  // Start new distance measure
  tof.startReadRangeMillimeters();  
  /*-----------------------------------------*/

  // Wifi method
  //connectToWifi();

  /*-------------MOTORS-------------*/
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  /*--------------------------------*/

  /*-------------MUX-------------*/
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(MUX_S4, OUTPUT);
  pinMode(MUX_SIGNAL, INPUT);

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    lastSensorValues[i] = sensorValues[i] = THRESHOLD * 2;
  }

  for (int i = 4; i <= 14; i++)
  {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
  }
  
  /*-----------------------------*/

  /*-------------ENCODER-------------*/
  // Set encoder pins as inputs with internal pull-ups
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  // Attach interrupts to the encoder pins
  Serial.println("Attaching interrupt...");
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encReader, CHANGE);
  Serial.println("Interrupt attached.");

  Serial.println("Encoder test initialized.");
  /*---------------------------------*/

  /*-------------MAZE-------------*/
  for(int r = 0; r < ROWS; r++){
    for(int c = 0; c < COLUMNS; c++){
      mazeGrid[r][c] = true;
    }
  }
  /*------------------------------*/
  
}

void loop() 
{
  currentMicros = micros();
  unsigned long currentMillis = millis();

  // The Control Loop
  if (currentMicros - previousMicros >= interval) {
    previousMicros = currentMicros;

    
    if (tof.readRangeAvailable()) {
      prev_distance = distance;
      distance = tof.readRangeMillimeters() * 1e-3;

      if ( (prev_distance <= DIST_OBJECT * 1.2) && (distance <= DIST_OBJECT) && (start == true) && (carMode == FOLLOW_LINE)){
        start = false;
        obstacleState = ALIGN;
        obstacleTimer = currentMillis;
      }
    }
 
    // Start new distance measure
    tof.startReadRangeMillimeters(); 

    // Read from serial monitor
    /*
    if (Serial.available()) { // Check if data is available
      char input = Serial.read(); // Read the entire input as a String
      Serial.print("You entered: ");
      Serial.println(input);

      //interpretSerialInput(input);
    }*/

    switch (carMode)
    {
    case FOLLOW_LINE:{
      if(start)
      {
        readLineSensors();
        
        float posLine = calculateMediumSensor();
        calculatePWMAndDir(posLine);
        motorControl();

        // Debug
        if(micros() % 10*interval == 0) debug(posLine);
      }
      else
      {
        obstacleAvoidance(currentMillis);
        motorControl();

        // Print encoder position
        noInterrupts();
        if(micros() % 10*interval == 0) {
          if(encPosition % 10 == 0)
          {
            Serial.print("Number of revolutions: ");
            Serial.println(encPosition/2000);
          }
          Serial.print("Encoder Position: ");
          Serial.println(encPosition);
          Serial.print("Target Encoder Position: ");
          Serial.println(targetEncPosition);
          Serial.print("State: ");
          Serial.println(obstacleState);
        }
        interrupts();
      }
      break;
    }

    case MAZE_SOLVER:{

      switch (mazeState)
      {
      case MAZE_CALCULATE_SHORTEST_PATH:{
        Serial.println("Calculating shortest path...");
        if (bfsComputePath(currRow, currCol, endRow, endCol)) {
          reconstructPath(currRow, currCol, endRow, endCol);

          //DEBUG
          Serial.println("Shortest path found!");
          for (int i = 0; i < pathLength; i++) {
            Serial.print("(");
            Serial.print(pathR[i]);
            Serial.print(", ");
            Serial.print(pathC[i]);
            Serial.println(")");
          }
          //delay(1000);

          // Start executing this path from index 0
          pathIndex = 0;
          mazeState = MAZE_EXECUTE_PATH;
        }
        else {
          Serial.println("No path found!");
          mazeState = MAZE_END;
        }

        //DEBUG
        Serial.print("Maze State: ");
        Serial.println(mazeState);

        break;
      }

      case MAZE_EXECUTE_PATH: {
        Serial.println("|----------------------------------------|");
        if (pathIndex >= pathLength - 1) {
          mazeState = MAZE_END;
          Serial.println("Car got to the end of the maze!");
          Serial.print("Current Position: "); Serial.print(currRow); Serial.println(currCol);
          break;
        }
        else{
          mazeState = MAZE_ALIGN_CAR;
          //DEBUG
          Serial.print("Maze State: ");
          Serial.println(mazeState);
          Serial.print("Aligning car");
        }

        break;
      }

      case MAZE_ALIGN_CAR:{
        Serial.print(".");
        if (alignRobot()) {
          break;
        }

        int r  = pathR[pathIndex];
        int c  = pathC[pathIndex];
        int nr = pathR[pathIndex + 1];
        int nc = pathC[pathIndex + 1];

        // Set direction
        Serial.println("");
        Serial.println("Setting direction...");
        if      (nr == r - 1 && nc == c    ) desiredDir = SOUTH;
        else if (nr == r + 1 && nc == c    ) desiredDir = NORTH;
        else if (nr == r     && nc == c - 1) desiredDir = EAST;
        else if (nr == r     && nc == c + 1) desiredDir = WEST;
        Serial.print("Desired direction: "); Serial.println(desiredDir);
        Serial.print("Current direction: "); Serial.println(currDir);

        mazeState = MAZE_TURN_CAR;
        //DEBUG
        Serial.print("Maze State: ");
        Serial.println(mazeState);
        Serial.print("Turning car");
        
        break;
        }

      case MAZE_TURN_CAR:{
        if (currDir != desiredDir) {
          Serial.print(".");

          if ((desiredDir == currDir + 1) || ((currDir == WEST) && (desiredDir == NORTH))) {
            turn90(0); // turn right
          } else if((desiredDir == currDir - 1) || ((currDir == NORTH) && (desiredDir == WEST))){
            turn90(1); // turn left
          } else {
            turn90(0);
          }
          //turn90(0);

          break;
        }
        Serial.println("");
        mazeState = MAZE_DETECT_OBSTACLE;
        //DEBUG
        Serial.print("Maze State: ");
        Serial.println(mazeState);
        break;
      }

      case MAZE_DETECT_OBSTACLE:{
        Serial.println("Detecting obstacles...");
        if (detectObstacle()) {
          Serial.println("Obstacle detected!");

          switch (currDir)
          {
          case NORTH:
            mazeGrid[currRow + 1][currCol] = false;
            break;
            
          case SOUTH:
            mazeGrid[currRow - 1][currCol] = false;
            break;

          case WEST:
            mazeGrid[currRow][currCol + 1] = false;
            break;

          case EAST:
            mazeGrid[currRow][currCol - 1] = false;
            break;

          }

          mazeState = MAZE_STOP;
          //DEBUG
          Serial.print("Maze State: ");
          Serial.println(mazeState);
          break;
        }

        mazeTimer = millis();
        Serial.print("Maze Timer: "); Serial.println(mazeTimer);
        mazeState = MAZE_MOVE_CAR;
        //DEBUG
        Serial.print("Maze State: "); Serial.println(mazeState);
        Serial.print("Moving car");
        break;
      }

      case MAZE_MOVE_CAR:{
        Serial.print(".");
        if (goToNextIntersection(MOTOR_SPEED * 0.5)) {
          break;
        }

        Serial.println("");
        pathIndex++;

        mazeState = MAZE_ADJUST_CAR;

        noInterrupts();
        targetEncPosition = encPosition + MAZE_ADJUST;
        interrupts();
        
        //DEBUG
        Serial.print("Target Encoder Position: "); Serial.println(targetEncPosition);
        Serial.print("Encoder Position: "); Serial.println(encPosition);

        Serial.print("Maze State: ");
        Serial.println(mazeState);
        break;
      }

      case MAZE_ADJUST_CAR:{  

        if( encPosition <= targetEncPosition ){
          int speed = MOTOR_SPEED * 0.5;

          readLineSensors();
          float posLine = mazeCalculateMediumSensor();

          float error = posLine - setpoint;

          error = 0;

          if (error > 0 + COMPENSATION) {
            motorSpeed[MOTOR1] = 0;
            motorDir[MOTOR1] = FORWARD;

            motorSpeed[MOTOR2] = speed;
            motorDir[MOTOR2] = FORWARD;
          } else if (error < 0 - COMPENSATION) {
            motorSpeed[MOTOR2] = 0;
            motorDir[MOTOR2] = FORWARD;

            motorSpeed[MOTOR1] = speed;
            motorDir[MOTOR1] = FORWARD;
          } else {
            motorDir[MOTOR1] = motorDir[MOTOR2] = FORWARD;
            motorSpeed[MOTOR1] = motorSpeed[MOTOR2] = speed;
          }

          motorControl();

          break;
        }
        
        mazeState = MAZE_EXECUTE_PATH;
        //mazeState = MAZE_END;

        motorDir[MOTOR1] = motorDir[MOTOR2] = FORWARD;
        motorSpeed[MOTOR1] = motorSpeed[MOTOR2] = 0;
        motorControl();

        delay(1000); //Delay 1 sec

        break;
      }

      case MAZE_STOP:{
        motorSpeed[MOTOR1] = motorSpeed[MOTOR2] = 0;
        motorDir[MOTOR1] = motorDir[MOTOR2] = FORWARD;
        motorControl();

        mazeState = MAZE_CALCULATE_SHORTEST_PATH;
        //DEBUG
        Serial.print("Maze State: ");
        Serial.println(mazeState);
        break;
      }

      case MAZE_END:{
        motorSpeed[MOTOR1] = motorSpeed[MOTOR2] = 0;
        motorDir[MOTOR1] = motorDir[MOTOR2] = FORWARD;
        motorControl();
        break;
      }

      default:
        break;
      }

      break;
    }
    

    case DEBUG:{
      readLineSensors();
      if(micros() % 10*interval == 0) debug(0);

      break;
    }

    default:
      break;

    }
  
  }
}


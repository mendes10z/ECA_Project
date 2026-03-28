#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#define NUM_SENSORS 3
#define MOTOR_SPEED 100
#define THRESHOLD 400
#define MAX_SENSOR_VALUE 1000
#define MIN_SENSOR_VALUE 200

//int valueSensor[NUM_SENSORS] = {400,20,1000,20,1000};
int valueSensor[NUM_SENSORS] = {1000,1000,1000};

int PWM1, PWM2 = 0;
bool DIR1, DIR2 = 1;


// Weights for sensor positions
const int sensorWeights[NUM_SENSORS] = {-1, 0, 1};

float calculateMediumSensor() {
    float weightedSum = 0;
    float totalInvertedValues = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        float invertedValue = MAX_SENSOR_VALUE - valueSensor[i];    //Black gets a higher value than white 

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
    float speedValue = 0.0;
    //DEBUG
    //printf("NUM_SENSORS / 2: %.2f\n", round((float)NUM_SENSORS/2));
    //printf("NUM_SENSORS / 4: %.2f\n", round((float)NUM_SENSORS/4));
    //printf("3 * NUM_SENSORS / 4: %.2f\n", round(3*(float)NUM_SENSORS/4));
    if (posLine < round((float)NUM_SENSORS/2))
    {
        speedValue = (float)MOTOR_SPEED * (posLine - round((float)NUM_SENSORS/4));
        if(speedValue > 0) DIR1 = 1;
        else DIR1 = 0;
        PWM1 = abs(round(speedValue));
        PWM2 = MOTOR_SPEED;
        DIR2 = 1;
    } 
    else if(posLine > round((float)NUM_SENSORS/2))
    {
        speedValue = MOTOR_SPEED * (- posLine + round(3*(float)NUM_SENSORS/4));
        if(speedValue > 0) DIR2 = 1;
        else DIR2 = 0;
        PWM2 = abs(round(speedValue));
        PWM1 = MOTOR_SPEED;
        DIR1 = 1;
    }
    else 
    {
        DIR1 = DIR2 = 1;
        PWM1 = PWM2 = MOTOR_SPEED;
    }

    //DEBUG
    printf("Line Position: %.2f\n", posLine);
    printf("Speed Value: %.2f\n", speedValue);
}

int main()  {

    float posLine = calculateMediumSensor();
    calculatePWMAndDir(posLine);

    //DEBUG
    printf("PWM1: %d    DIR1: %d\n", PWM1, DIR1);
    printf("PWM2: %d    DIR2: %d", PWM2, DIR2);

    return 0;
}
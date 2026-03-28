// 2022 Paulo Costa
// Pico W LED access

#include <Arduino.h>
#include <WiFi.h>

#include "pico/cyw43_arch.h"

#include <Wire.h>
#include <VL53L0X.h>

/*--------DEFINE--------*/
#define CYW43_WL_GPIO_LED_PIN 0
// Encoder pins
#define ENCODER_PIN_A 2  // GPIO pin connected to Channel A
#define ENCODER_PIN_B 3  // GPIO pin connected to Channel B
/*----------------------*/

VL53L0X tof;
float distance, prev_distance;

int LED_state;
unsigned long interval = 40 * 1000;
unsigned long currentMicros, previousMicros;
int loop_count;

volatile int position = 0; // Track the encoder position

void connectToWifi()
{
  // Connect to Wi-Fi
  WiFi.begin("your-SSID", "your-password");

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("Connecting to Wi-Fi...");
  }

  //DEBUG
  Serial.println("Connected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void ola() {
  Serial.println("Interrupt Triggered!");
  int a = digitalRead(ENCODER_PIN_A); // Read Channel A
  int b = digitalRead(ENCODER_PIN_B); // Read Channel B

  // Determine the direction based on A and B channel states
  if (a == b) {
      position++;  // Clockwise rotation
  } else {
      position--;  // Counterclockwise rotation
  }
}

void setup() 
{
  Serial.begin(115200);

  Wire.setSDA(8);
  Wire.setSCL(9);

  Wire.begin();

  //tof.setTimeout(500);
  /*
  while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  }  */

  // Reduce timing budget to 20 ms (default is about 33 ms)
  //tof.setMeasurementTimingBudget(20000);

  // Start new distance measure
  //tof.startReadRangeMillimeters();  

  //WiFi.begin
  //connectToWifi();

  /*-------------ENCODER-------------*/
  // Set encoder pins as inputs with internal pull-ups
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  // Attach interrupts to the encoder pins
  Serial.println("Attaching interrupt...");
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), ola, CHANGE);
  Serial.println("Interrupt attached.");

  Serial.println("Encoder test initialized.");
  /*---------------------------------*/
}

/*
void loop() {
  int a = digitalRead(ENCODER_PIN_A); // Read Channel A
  int b = digitalRead(ENCODER_PIN_B); // Read Channel B

  Serial.print("Channel A: ");
  Serial.print(a);
  Serial.print(" | Channel B: ");
  Serial.println(b);

  delay(100); // Check pin state every 100 ms
}
*/

void loop() 
{
  currentMicros = micros();

  // THE Control Loop
  if (currentMicros - previousMicros >= interval) {
    previousMicros = currentMicros;

    /*
    if (tof.readRangeAvailable()) {
      prev_distance = distance;
      distance = tof.readRangeMillimeters() * 1e-3;
    }*/
 
    // Start new distance measure
    //tof.startReadRangeMillimeters(); 

    // Toggle builtin LED    
    loop_count++;
    if (loop_count > 5) {
      LED_state = !LED_state;
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, LED_state);
      loop_count = 0;
    }

    Serial.print(" Dist: ");
    Serial.print(distance, 3);
    Serial.println();


    // Print encoder position
    noInterrupts();
    Serial.print("Encoder Position: ");
    Serial.println(position);
    interrupts();
  }
}


#include <SPI.h>
#include "Adafruit_VL53L0X.h"

// Start Mode Choice
uint8_t Mode = 0;
// Global pump power level. MAX=255, MIN=0
uint8_t pump = 0;

// --- Pins ---
const uint8_t RANGE_SENSOR_PIN = 10;
const uint8_t PUMP_PIN = 5;
const uint8_t POTI_PIN = A0; // A0 is defined as Pin 14 on the Arduino Uno Board

// Distance Measurement
Adafruit_VL53L0X distanceSensor;
const uint8_t zeroPosition_mm = 243;
const uint8_t maxPosition_mm = 75;
const uint8_t fillLevelMax_mm = zeroPosition_mm - maxPosition_mm;


// Function to measure the distance and calculate a moving average.
// Returns the moving average of the measurements 
int readDistanceFiltered() {
  const uint8_t numSamples = 20;
  static uint8_t vals[numSamples] = {0}; // static is used to remember the values between the function calls

  VL53L0X_RangingMeasurementData_t distanceMeasurement;
  distanceSensor.rangingTest(&distanceMeasurement, false);

  // set distance value in cache with conditional operator
  // only sets value if the measurement is valid
  // defaults to last good measurement
  vals[numSamples-1] = 
    (distanceMeasurement.RangeStatus != 4) ? // valid measurement?
      distanceMeasurement.RangeMilliMeter : // if yes
      vals[numSamples-2]; // if no

  // FIFO (first-in-first-out) push and moving average calculation
  int sum = 0;
  for(int i=0; i < numSamples-1; i++){
    vals[i] = vals[i+1]; // shifts the array to the left
    sum += vals[i];
  }
  return sum / numSamples; 
}

// Function to read from the Potentiometer
// result is scaled to 100
uint8_t readPoti(){
  const float scalingFactor = 100.0;
  return round(scalingFactor * analogRead(POTI_PIN) / 1024);
}

// Function to control the pump by using a two point controller
// Mode = 1
uint8_t controlTwoPoint(uint8_t fillLevel_percent){

  const uint8_t threshold = 50;

  bool pumpOn;

  if (fillLevel_percent < threshold) pumpOn = true;
  else pumpOn = false;

  return pumpOn * 255;
}

// Function to control the pump by setting a distinct pump power 
// corresponding to a distinct fill level using look up tables
// Mode = 2
uint8_t controlLookUpTable(uint8_t fillLevel_percent){

  // separate the fill level and the power level in 3 segments
  // min: 0, max: 100!!
  const uint8_t fillLevelTable[3] = {
    70, 
    40, 
    0
  };
  // min: 0, max: 255!!
  const uint8_t pumpPowerTable[3] = {
    0, 
    100,
    255
  };

  for(int i = 0; i < 3; i++){
    // is the fill level higher than a threshold?
    if (fillLevel_percent > fillLevelTable[i]) {
      // set the corresponding pump power level
      return pumpPowerTable[i];
    }
  }
}

// Function to control the pump by using a Schmitt trigger
// to avoid fast switching
// Mode = 3
uint8_t controlSchmittTrigger(uint8_t fillLevel_percent){
  const uint8_t onThreshold = 60;
  const uint8_t offThreshold = 40;

  static bool pumpOn = false;

  if (fillLevel_percent < offThreshold) pumpOn = true;
  else if (fillLevel_percent > onThreshold) pumpOn = false;
  
  return pumpOn * 255;
  
}

// Function to control the pump by using a PID-Controller
// Mode = 4
uint8_t controlPID(uint8_t potiVal_percent, uint8_t fillLevel_percent){
  
  // generates time Step -> time since last execution of this function
  static unsigned long prevExecutionTime_ms = 0;
  unsigned long now_ms = millis();
  unsigned long timeStep_ms = now_ms - prevExecutionTime_ms;
  prevExecutionTime_ms = now_ms;

  if (timeStep_ms == 0) timeStep_ms = 1;
  float timeStep_s = timeStep_ms / 1000.0;

  const float Kp = 10;
  const float Ti = 10;
  const float Td = 0.0;

  static float prevError = 0.0;

  // proportional termin -> (w-x)
  float error = potiVal_percent - fillLevel_percent;

  // integral term -> accumulates over time
  static float integral = 0.0;
  integral += 1 / Ti * (error * timeStep_s);
  integral = constrain(integral, -300.0, 300.0);
  
  // derivative term -> gets bigger when error changes
  float derivative = Td * (error - prevError) / timeStep_s;

  prevError = error;

  return constrain((int)(Kp * (error + integral + derivative)), 0, 255);
} 

// Function to write to the pump
void writePoti(uint8_t value){
  digitalWrite(RANGE_SENSOR_PIN, LOW);
  SPI.transfer(0);
  SPI.transfer(value);
  digitalWrite(RANGE_SENSOR_PIN, HIGH);
}


void setup(){
  Serial.begin(9600);
  pinMode(PUMP_PIN, OUTPUT);

  distanceSensor.begin();
  SPI.begin();
}

void loop(){  
  uint8_t distance_mm = readDistanceFiltered();
  uint8_t fillLevel_mm = zeroPosition_mm - distance_mm;
  uint8_t fillLevel_percent = round(100 * (float)fillLevel_mm / (float)fillLevelMax_mm);
  uint8_t potiVal = readPoti();

  // reads from the serial input, if there is any
  if (Serial.available() > 0) {
    char incomingChar = Serial.read();

    if (incomingChar >= '0' && incomingChar <= '4') {
      Mode = incomingChar - '0';
      Serial.print("Mode: ");
      Serial.println(Mode);
      delay(1000);

    } else if (incomingChar == 10) {
      // ignore -> it's a new line
    } else {
      Serial.print("InputError. Mode: ");
      Serial.println(Mode);
      delay(1000);

    }
  }

  switch (Mode) {
    case 0:
      pump = constrain(potiVal*2.55, 0, 255);
      break;
    case 1:
      pump = controlTwoPoint(fillLevel_percent);
      break;
    case 2:
      pump = controlLookUpTable(fillLevel_percent);
      break;
    case 3:
      pump = controlSchmittTrigger(fillLevel_percent);
      break;
    case 4:
      pump = controlPID(potiVal, fillLevel_percent);
      break;
  }

  writePoti(pump);

  Serial.print(0); Serial.print(",");
  // value 2
  Serial.print(potiVal); Serial.print(",");
  // value 3
  Serial.print(fillLevel_percent); Serial.print(",");
  // value 4
  Serial.print(pump); Serial.print(",");
  Serial.println(255);

  delay(10);
}

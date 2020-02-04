#include <Wire.h>
#include <VL53L1X.h>

const int NUM_TOF = 3;

const uint8_t S_1_X = 12;
const uint8_t S_2_X = 8;
const uint8_t S_3_X = 4;

const uint8_t HAPTIC_1 = 5;
const uint8_t HAPTIC_2 = 9;
const uint8_t HAPTIC_3 = 11;

VL53L1X all_tof[NUM_TOF];
uint8_t xshut_pins[NUM_TOF] = { S_1_X, S_2_X, S_3_X };
uint8_t mem_addr[NUM_TOF] = { -1, 0x31, 0x33 };
uint8_t haptic_pins[NUM_TOF] = { HAPTIC_1, HAPTIC_2, HAPTIC_3 };

/* VL53L1X Configuration */
int TOF_TIMEOUT = 500;
VL53L1X::DistanceMode TOF_DIST_MODE = VL53L1X::Short;
int TOF_TIMING_BUDGET = 50000;

/* Exponential filter */
int N = 20;
float EMA_ALPHA = 0.70;
long ema_vals[NUM_TOF];

void ema_update(int sensor_id) {
  all_tof[sensor_id].read();
  ema_vals[sensor_id] = ema_vals[sensor_id] * (1 - EMA_ALPHA) + all_tof[sensor_id].ranging_data.range_mm * EMA_ALPHA;
}

void probeI2C() {
  Serial.println("Probing for I2C devices...");
  for(byte i = 0; i < 120; ++i) {
    Wire.beginTransmission(i);
    if(Wire.endTransmission() == 0) {
      Serial.print("Found address: ");
      Serial.print(i, DEC);
      Serial.print(" (0x");
      Serial.print(i, HEX);
      Serial.println(")");
      delay(5);
    }
  }
}

void turnOffToF(int sensor_id) {
  pinMode(xshut_pins[sensor_id], OUTPUT);
  digitalWrite(xshut_pins[sensor_id], LOW);
}

void initToF(int sensor_id) {
  digitalWrite(xshut_pins[sensor_id], HIGH);
  all_tof[sensor_id].init();
  delay(100);
  all_tof[sensor_id].setTimeout(TOF_TIMEOUT);
  all_tof[sensor_id].setDistanceMode(TOF_DIST_MODE);
  all_tof[sensor_id].setMeasurementTimingBudget(TOF_TIMING_BUDGET);

  if(mem_addr[sensor_id] > 0) {
    all_tof[sensor_id].setAddress(mem_addr[sensor_id]);
  }

  pinMode(haptic_pins[sensor_id], OUTPUT);  /* setup haptic output pins */
}

void haptic_feedback(int sensor_id) {
  long dist = ema_vals[sensor_id];

  bool isClose = dist < 300;
  bool isMedium = !isClose && dist < 600;
  bool isFar = !isMedium && dist < 900;
  long power;
  if(isClose) {
    power = 255;
  } else if(isMedium) {
    power = int(-0.45 * dist + 390);
  } else if(isFar) {
    power = int(-0.4 * dist + 360);
  } else {
    power = 0;
  }

  if(power < 90) {
    power = 0;
  }
  analogWrite(haptic_pins[sensor_id], power);
  Serial.print(ema_vals[sensor_id]);
  Serial.print(' ');
  Serial.print(power);
  Serial.print('\t');
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(500);
  
  /* turn off all sensors */
  for(int i = 0; i < NUM_TOF; ++i) {
    turnOffToF(i);
  }

  /* initialize sensors in backwards order */
  for(int i = NUM_TOF - 1; i >= 0; --i) {
    initToF(i);
  }

//  probeI2C();

  /* populate ema array with sensor reading */
  for(int i = 0; i < NUM_TOF; ++i) {
    all_tof[i].startContinuous(50);
    all_tof[i].read();
    ema_vals[i] = all_tof[i].ranging_data.range_mm;
  }
  
  Serial.println("Finished ToF setup!");

  /* Initialize EMA */
  for(int i = 0; i < N; ++i) {
    for(int j = 0; j < NUM_TOF; ++j) {
      ema_update(j);
    }
  }
  Serial.println("Finished ema initialization!");

  /* Notify user of startup */
  for(int i = 0; i < NUM_TOF; ++i) {
    analogWrite(haptic_pins[i], 150);
  }
  delay(250);
  for(int i = 0; i < NUM_TOF; ++i) {
    analogWrite(haptic_pins[i], 200);
  }
  delay(250);
  for(int i = 0; i < NUM_TOF; ++i) {
    analogWrite(haptic_pins[i], 250);
  }
  delay(250);
  for(int i = 0; i < NUM_TOF; ++i) {
    analogWrite(haptic_pins[i], 0);
  }
  
}

void loop() {
  for(int i = 0; i < NUM_TOF; ++i) {
      ema_update(i);
      haptic_feedback(i);
  }
  Serial.println();
}

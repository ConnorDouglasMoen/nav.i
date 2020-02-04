#include <Wire.h>
#include "VL53L1X.h"

#define TRIG_PIN 11
#define ECHO_PIN 12
#define BUFFER_SIZE 3
#define MOTOR_PWM_PIN 5
#define LED_PIN 13

long duration, cm;     // for sonar sensor
long sonar_avg = 0;
long sonar_buffer[BUFFER_SIZE];
int sonar_buf_count = 0;
int BUZZ_PIN = A0;

VL53L1X Distance_Sensor;
VL53L1X Distance_Sensor_2;

uint8_t TOF_1_XSHUT = 11;
uint8_t TOF_2_XSHUT = 10;

long dist_tof;
long dist_tof_2;

void setup() {
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(TOF_1_XSHUT, OUTPUT);
  pinMode(TOF_2_XSHUT, OUTPUT);
  
  // hardware shutdows on all ToFs
  digitalWrite(TOF_1_XSHUT, LOW);
  digitalWrite(TOF_2_XSHUT, LOW);
  
  /*
   * I2C setup
   */
  Wire.begin();
  Wire.beginTransmission(0x29);
  Wire.setClock(400000); // use 400 kHz I2C
  
  /*
   * ToF_2 setup
   */
  digitalWrite(TOF_2_XSHUT, HIGH);
  delay(150); 
  if (!Distance_Sensor_2.init())
  {
    Serial.println("Failed to initialize VL53L1X Distance_Sensor_2!");
    while (1);
  }
  delay(100);
  Distance_Sensor_2.setAddress(0x35);
  Distance_Sensor_2.setDistanceMode(VL53L1X::Short);
  Distance_Sensor_2.setMeasurementTimingBudget(50000);
  Distance_Sensor_2.startContinuous(50);
  Distance_Sensor_2.setTimeout(100);

  /*
   * ToF_1 setup
   */
  digitalWrite(TOF_1_XSHUT, HIGH);
  delay(150); 
  if (!Distance_Sensor.init())
  {
    Serial.println("Failed to initialize VL53L1X Distance_Sensor!");
    while (1);
  }
  delay(100);
  /*
   * Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
   * You can change these settings to adjust the performance of the sensor, but
   * the minimum timing budget is 20 ms for short distance mode
   */
  Distance_Sensor.setDistanceMode(VL53L1X::Short);
  Distance_Sensor.setMeasurementTimingBudget(50000);
  Distance_Sensor.startContinuous(50);
  Distance_Sensor.setTimeout(100);

  Serial.println("Printing addresses...");

//  /*
//   * Sonar setup
//   */
//  pinMode(TRIG_PIN, OUTPUT);
//  pinMode(ECHO_PIN, INPUT);
//
// /*
//  * Motor setup
//  */
// pinMode(MOTOR_PWM_PIN, OUTPUT);
//
// /*
//  * Buzzer setup      
//  */
//  pinMode(BUZZ_PIN, OUTPUT);
  
  for (byte i = 1; i < 120; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      delay (3);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println("Checking for sensor");

}

void loop()
{
  /* The sonar sensor is triggered by a HIGH pulse of 10 or more microseconds.
   * Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
   */
//  digitalWrite(TRIG_PIN, LOW);
//  delayMicroseconds(5);
//  digitalWrite(TRIG_PIN, HIGH);`
//  delayMicroseconds(10);
//  digitalWrite(TRIG_PIN, LOW);

  /* Read the signal from the sonar sensor: a HIGH pulse whose
   * duration is the time (in microseconds) from the sending 
   * of the ping to the reception of its echo off of an object.
   * Then, convert the time into a distance.
   */
//  pinMode(ECHO_PIN, INPUT);
//  duration = pulseIn(ECHO_PIN, HIGH);
//  cm = (duration/2.0) / 29.1;     // Divide by 29.1 or multiply by 0.0343
//  inches = (duration/2) / 74;   // Divide by 74 or multiply by 0.0135

  // calculate average sonar value over BUFFER_SIZE counts
//  if(sonar_buf_count != BUFFER_SIZE - 1) {
//    sonar_buf_count++;
//    sonar_buffer[sonar_buf_count] = cm;
//  } else {
//    sonar_avg = 0;
//    for(int i = 0; i < BUFFER_SIZE; ++i)
//      sonar_avg += sonar_buffer[i];
      
//    sonar_avg /= BUFFER_SIZE;
//    Serial.print("Sonar Distance(cm):");
//    Serial.print(cm);
//    Serial.println();
//    sonar_buf_count = 0;
//  }

  // Read from ToF sensor
  Distance_Sensor.read(true);

//  Serial.print("Distance(mm):");
//    Serial.print("\tStatus: ");
//    Serial.println(VL53L1X::rangeStatusToString(Distance_Sensor.ranging_data.range_status));

  if (Distance_Sensor.ranging_data.range_status == VL53L1X::RangeValid) { // not valid range
//    digitalWrite(LED_PIN, HIGH);
    dist_tof = Distance_Sensor.ranging_data.range_mm;
//    Serial.print(dist_tof);
//    Serial.print("\tStatus: ");
//    Serial.println(VL53L1X::rangeStatusToString(Distance_Sensor.ranging_data.range_status));
//    Serial.println();
  
//    bool isClose = dist_tof < 50 && sonar_avg < 5;
//    bool isNear = !isClose && (dist_tof < 100 && sonar_avg < 10);
//    bool isFar = !isClose && !isNear &&(dist_tof < 250 && sonar_avg < 25);
//    bool tooFar = !isClose && !isNear && !isFar;  
  
//    if ( dist_tof < 1500 ) {
//      Serial.print("dist_tof : ");
//      Serial.println(dist_tof);
//      long power = min(850 / (dist_tof/90), 255);
//      if(power == -1) {
//        analogWrite(MOTOR_PWM_PIN, 0);
//      }
  //    long power = min(255 / ((dist_tof + sonar_avg)/90), 255); // some basic scaling garbage, may need adjustment
//      else {
  //      Serial.print("adding power: ");
  //      Serial.println(power);
//        long freq = pow(power, 1.1) + 250;
//        Serial.print("freq: ");
//        Serial.println(freq);
        
//        tone(BUZZ_PIN, freq, 100);
//        analogWrite(MOTOR_PWM_PIN, power);
//      }
//    } else {
//      Serial.println("too far");
//      analogWrite(MOTOR_PWM_PIN, 0);
//    }
//  
//  } else {
//    analogWrite(MOTOR_PWM_PIN, 0);
//    digitalWrite(LED_PIN, LOW);
  }
//  delayMicroseconds(5);
}

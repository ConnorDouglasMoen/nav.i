#include <Wire.h>
#include <VL53L1X.h>

uint8_t S_1_X = 7;
VL53L1X sensor;  /* XSHUT is D7 (7) */

uint8_t S_2_X = 4;  /* XSHUT is D4 (4) */
VL53L1X sensor2; 

//uint8_t S_3_X = 8;  /* XSHUT is D8 (8) */
//VL53L1X sensor3; 

/*
int s = 20;   
int i = 0;
int sensorVals[s];
int sensorVals2[s];
int sensorVals3[s];
*/  
int N = 20;    
float alpha = 0.2;
int ema_vals[2];


void ema_update(int sensor1, int sensor2){
  ema_vals[0] = ema_vals[0] * (1-alpha) + sensor1 * alpha;
  ema_vals[1] = ema_vals[1] * (1-alpha) + sensor2 * alpha;
  Serial.print(" Sensor1: ");
  Serial.print(sensor1);
  Serial.print(" Sensor2: ");
  Serial.print(sensor2);
  Serial.println();

  Serial.print(" EMA1: ");
  Serial.print(ema_vals[0]);
  Serial.print(" EMA2: ");
  Serial.print(ema_vals[1]);
  Serial.println();
  Serial.print("-----------------------------");
  //ema_vals[2] = ema_vals[2] * alpha + sensor3 * (1-alpha);
}

void setup()
{
  Wire.begin();
  Serial.begin (115200);
  delay(500);

  /* Set XSHUT pins to LOW to reset all VL53L1X */
  pinMode(S_1_X, OUTPUT);
  pinMode(S_2_X, OUTPUT);
  //pinMode(S_3_X, OUTPUT);
  digitalWrite(S_1_X, LOW);
  digitalWrite(S_2_X, LOW);
  //digitalWrite(S_3_X, LOW);
  delay(150);

  //digitalWrite(S_3_X, HIGH);  /* turn on third ToF */
//  pinMode(S_3_X, INPUT);
  //Serial.println("sensor3 reset");
  //sensor3.init();
  //Serial.println("sensor3 initialized.");
  //delay(100);
  //sensor3.setAddress(0x35);
  //Serial.println("sensor3 address changed");
  //delay(150);
  
  digitalWrite(S_2_X, HIGH);  /* turn on second ToF */
//  pinMode(S_2_X, INPUT);
  Serial.println("sensor2 reset");
  sensor2.init();
  Serial.println("sensor2 initialized.");
  delay(100);
  sensor2.setAddress(0x31);
  Serial.println("sensor2 address changed");
  delay(150);
  
  digitalWrite(S_1_X, HIGH);
//  pinMode(S_1_X, INPUT);
  Serial.println("sensor1 reset");
  sensor.init();  /* set at default address of 0x29 */
  Serial.println("sensor1 initialized. Scanning for addresses...");

  
  sensor.setTimeout(500);
  sensor2.setTimeout(500);
  //sensor3.setTimeout(500);

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
      delay (1); 
    }
  }
  
  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);
  
  sensor2.setDistanceMode(VL53L1X::Short);
  sensor2.setMeasurementTimingBudget(50000);
  sensor2.startContinuous(50);

//  sensor3.setDistanceMode(VL53L1X::Short);
//  sensor3.setMeasurementTimingBudget(50000);
//  sensor3.startContinuous(50);

  Serial.println("reading initial sensor values to ema array");
  ema_vals[0] = sensor.read();
  ema_vals[1] = sensor2.read();
//  ema_vals[2] = sensor3.read();
  for (int i = 0; i < N; i++){
    ema_update(sensor.read(),sensor2.read());
    Serial.print(sensor.read());
    Serial.print(',');
    Serial.print(sensor2.read());
    Serial.print(',');
    Serial.println();
  }
  Serial.print(ema_vals[0]);
  Serial.print(ema_vals[1]);
  Serial.println("Finished ema initialization");  
}
/*
void average(int v1, int v2, int v3){
  int avg, avg1, avg2;
  int sum = 0;
  int sum1 = 0;
  int sum2 = 0;
  sensorVals[i] = v1
  sensorVals2[i] = v2
  sensorVals3[i] = v3
  i = (i + 1)%s;
  for (int j = 0; j < s; j++){
    sum = sum + sensorVals[j];
    sum = sum1 + sensorVals1[j];
    sum = sum2 + sensorVals2[j];
  }
  avg = sum/s;
  avg1 = sum1/s;
  avg2 = sum2/s;
}
*/


void loop()
{
//  Serial.println(" Sensor Values");
//  Serial.print(sensor.read());
//  Serial.print(',');
//  Serial.print(sensor2.read());
//  Serial.print(',');
//  //Serial.print(sensor3.read());
//  Serial.println();
  ema_update(sensor.read(),sensor2.read());
//  Serial.println(" EMA Values");
//  Serial.print(ema_vals[0]);
//  Serial.print(',');
//  Serial.print(ema_vals[1]);
//  Serial.print(',');
//  Serial.println(); 
}

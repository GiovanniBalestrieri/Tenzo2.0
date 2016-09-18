

#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <Wire.h>

ADXL345 acc;
ITG3200 gyro;

float vals[6];

void setup() {
  Serial.begin(115200);  
  Wire.begin();
  acc = ADXL345();
  gyro = ITG3200();
  
  // init ADXL345
  acc.init(0x53);  

  // init ITG3200
  gyro.init(ITG3200_ADDR_AD0_LOW);
  delay(500);
  // calibrate the ITG3200
  // takes 1000 samples every 2 ms and computes the mean.
  gyro.zeroCalibrate(1000,2);
  
  Serial.println("Setup Done");
}

void loop() {
  // put your main code here, to run repeatedly:
  getValues(vals);
  getAccel(vals);
  serialRoutine();
  delay(500);
}

void serialRoutine()
{
  Serial.print("\nAcc:\t");
  Serial.print(vals[0]);
  Serial.print("\t");
  Serial.print(vals[1]);
  Serial.print("\t");
  Serial.print(vals[2]);
  Serial.print("\t");
  Serial.print("\tGyro:");
  Serial.print(vals[3]);
  Serial.print("\t");
  Serial.print(vals[4]);
  Serial.print("\t");
  Serial.println(vals[5]);

}

void getValues(float * values) {  
  int accval[3];
  acc.readAccel(&accval[0], &accval[1], &accval[2]);
  values[0] = ((float) accval[0]);
  values[1] = ((float) accval[1]);
  values[2] = ((float) accval[2]);
  
  gyro.readGyro(&values[3]);
  
  //magn.getValues(&values[6]);
}


void getAccel(float * values) {  
  acc.get_Gxyz(values);
  //magn.getValues(&values[6]);
}

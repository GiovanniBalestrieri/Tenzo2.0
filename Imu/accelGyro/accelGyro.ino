

#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include "ComplementaryFilter.h"
#include <Wire.h>

ADXL345 acc;
ITG3200 gyro;
ComplementaryFilter cf;

float vals[6];
int rawAccel[3];
float accs[3];
float w[3];
float * attitude;

// comple filter gains
float kacc = 0.03;
float kgyro = 0.97;

float offsetAcc[3] = {0.4666,-0.1594, -0.7343};
int xMax=-246,xMin=273,yMax=-275,yMin=254,zMax=-300,zMin=208;
int offsetX,offsetY,offsetZ;

float gains[3] = {0.00376390,0.00376009,0.00349265};

void setup() {
  Serial.begin(115200);  
  Wire.begin();
  acc = ADXL345();
  gyro = ITG3200();
  cf = ComplementaryFilter(kacc,kgyro);
  
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
  cf.Compute(accs,w);
  serialRoutine();
  delay(1);
}

void printAcc()
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
  
  accs[0] = ((float) accval[0]);
  accs[1] = ((float) accval[1]);
  accs[2] = ((float) accval[2]);
  
  int gyroval[3];
  gyro.readGyroRawCal(&gyroval[0],&gyroval[1],&gyroval[2]);
  values[3] = ((float) gyroval[0]);
  values[4] = ((float) gyroval[1]);
  values[5] = ((float) gyroval[2]);
  
  w[0] =  values[3];
  w[1] =  values[4];
  w[2] =  values[5];
  //magn.getValues(&values[6]);
}

void printEstimates()
{
  float ro =  cf.getRollAcc(accs);
  float pi = cf.getPitchAcc(accs);
  Serial.print("\nAccEst:\tRoll");
  Serial.print(ro);
  Serial.print("\tPitch");
  Serial.print(pi);
  Serial.print("\nAttitude:\tRoll");
  Serial.print(cf.getRoll());
  Serial.print("\tPitch");
  Serial.print(cf.getPitch());
}

void serialRoutine()
{
  if (Serial.available()>0)
  {
    char t = Serial.read();
    
    if (t=='X')
    {
      Serial.println("\t\tCalibration towards XMAX");
      acc.readAccel(rawAccel);
      xMax = rawAccel[0];
      Serial.print("\nRAW:\t");
      Serial.println(rawAccel[0]);
    }
    if (t=='x')
    {
      Serial.println("\t\tCalibration towards Xmin");
      acc.readAccel(rawAccel);
      xMin = rawAccel[0];
      Serial.print("\nRAW:\t");
      Serial.println(rawAccel[0]);
    }
    if (t=='Y')
    {
      Serial.println("\t\tCalibration towards YMAX");
      acc.readAccel(rawAccel);
      yMax = rawAccel[1];
      Serial.print("\nRAW:\t");
      Serial.println(rawAccel[1]);
    }
    if (t=='y')
    {
      Serial.println("\t\tCalibration towards Ymin");
      acc.readAccel(rawAccel);
      yMin = rawAccel[1];
      Serial.print("\nRAW:\t");
      Serial.println(rawAccel[1]);
    }
    if (t=='Z')
    {
      Serial.println("\t\tCalibration towards ZMAX");
      acc.readAccel(rawAccel);
      zMax = rawAccel[2];
      Serial.print("\nRAW:\t");
      Serial.println(rawAccel[2]);
    }
    if (t=='z')
    {
      Serial.println("\t\tCalibration towards Zmin");
      acc.readAccel(rawAccel);
      zMin = rawAccel[2];
      Serial.print("\nRAW:\t");
      Serial.println(rawAccel[2]);
    }
    if (t == 'c')
    {
      Serial.println("\t\tUpdating offsets!");
      offsetX = (xMax+xMin)/2;
      offsetY = (yMax+yMin)/2;
      offsetZ = (zMax+zMin)/2;
      
      offsetAcc[0] = offsetX;
      offsetAcc[1] = offsetY;
      offsetAcc[2] = offsetZ;
    }
    if (t == 'a')
      printAcc();
      
    if (t == 'o')
      printEstimates();
    
    if (t == 'p')
    {
      Serial.println("\t\tPrinting offsets:");
      
      Serial.print("[ Max\tMin]\nX:");
      Serial.print(xMax);
      Serial.print("\t");
      Serial.print(xMin);
      Serial.print("\nY:");
      Serial.print(yMax);
      Serial.print("\t");
      Serial.print(yMin);
      Serial.print("\nZ:");
      Serial.print(zMax);
      Serial.print("\t");
      Serial.println(zMin);
      
      for (int o =0;o<3;o++)
      { 
        Serial.println(offsetAcc[o]);
      }
    }
  }
  
}

void getAccel(float * values) 
{
  acc.get_Gxyz(values);
  //accRoutine();  
  acc.readAccel(rawAccel);
  /*
  Serial.print("\nRAW:\t");
  Serial.print(rawAccel[0]);
  Serial.print("\t");
  Serial.print(rawAccel[1]);
  Serial.print("\t");
  Serial.println(rawAccel[2]);
  */
  int i;
  for (i=0;i<3;i++)
  {
    values[i] = (rawAccel[i]- offsetAcc[i])*gains[i] ;
  }
  //magn.getValues(&values[6]);
}

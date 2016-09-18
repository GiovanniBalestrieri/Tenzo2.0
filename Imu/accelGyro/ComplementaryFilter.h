/*
  ComplementaryFilter.h - Library implementing a complementary Filter based on Acceleration and angular velocities
  Created by Giovanni Balestrieri - UserK, September 18, 2016
  www.userk.co.uk
*/
#ifndef ComplementaryFilter_h
  #define ComplementaryFilter_h

#include "Arduino.h"

#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2

class ComplementaryFilter
{
  private:
    float  _kacc;
    float  _kgyro;
    float  _kmagn;
    boolean  _9dof;
    boolean  _6dof;
 
  public:
    ComplementaryFilter(float kacc,float kgyro);
    ComplementaryFilter();
    ComplementaryFilter(float kacc,float kgyro,float kmagn);
    void Compute(float*,float*);

    float getXRotation(float *);
    float getYRotation(float *); 
    float getRollAcc(float *);
    float getPitchAcc(float *);
    float getRoll();
    float getPitch();
    float toDegree(float);
    double dist(float,float);
    boolean setK(float,float);
    float getK();
    void reset();
    void pause();
    void restart();

    float roll;
    float pitch;
    float yaw;
    float angleXAcc;
    float angleYAcc;

    boolean cold;

    unsigned long temp;
    unsigned long dt;
};

#endif

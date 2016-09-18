/*
  ComplementaryFilter.h - Library implementing a complementary Filter based on Acceleration and angular velocities
  Created by Giovanni Balestrieri - UserK, September 18, 2016
  www.userk.co.uk
*/

#include "ComplementaryFilter.h"



/*
 *	Constructor: Set private gains
 * 	PARAM
 * 		@FLOAT Kacc,Kgyro	
 *
 */
ComplementaryFilter::ComplementaryFilter(float kacc,float kgyro)
{
	_kacc = kacc;
	_kgyro = kgyro;
	_6dof = true;
	_9dof = false;
	cold = true;
  Serial.println("[ OK ] Complementary Filter ");
}

/*
 *  Constructor: Initialize
 *  PARAM
 *    @FLOAT Kacc,Kgyro 
 *
 */
ComplementaryFilter::ComplementaryFilter()
{
  cold = true;
}

/*
 *	Constructor: Set private gains
 * 	PARAM
 * 		@FLOAT Kacc,Kgyro,Kmagn	
 *
 */
ComplementaryFilter::ComplementaryFilter(float kacc,float kgyro,float kmagn)
{
	_kacc = kacc;
	_kgyro = kgyro;
	_kmagn = kmagn;
	_9dof = true;
	_6dof = false;
	cold = true;
}


/*
 *	Computes value
 *  	PARAM
 * 		@FLOAT X,Y,Z	
 *	RETURNS
 *		@FLOAT Roll
 */
float* ComplementaryFilter::Compute(float * acc,float * gyro)
{
  if (cold)
  {
	temp = micros();
	cold = false;
  }
  dt = micros()-temp;

  angleXAcc = getRoll(acc);
  angleYAcc = getPitch(acc);

  roll = (roll + gyro[0]*(float)dt/1000000.0)*_kgyro + angleXAcc*_kacc;
  pitch = (pitch + gyro[1]*(float)dt/1000000.0)*_kgyro + angleYAcc*_kacc;

  // Filter Estimates with Median Filter
  /*
  if (filterEst)
  {
    medianEstX.in(estXAngle);
    estXAngle = medianEstX.out();
    medianEstY.in(estYAngle);
    estYAngle = medianEstY.out();
  }
  */
  temp=micros();  
  float orientation[2] = {roll,pitch};
  return orientation;
}

/* 
 *	Computes the distance between two points_ euclidean norm
 *  	PARAM 
 *  		@FLOAT point A
 *		@FLOAT point B
 *  	RETURNS
 *		@FLOAT distance
 */
double ComplementaryFilter::dist(float a,float b) {
   return sqrt( (a*a)+(b*b));
}


/* 
 *	Converts from radians to degree
 *  	PARAM 
 *  		@FLOAT radians
 *  	RETURNS
 *		@FLOAT degree
 */
float ComplementaryFilter::toDegree(float radians) {
	float deg = radians * (180.0 / M_PI);
	return deg;
}

/*
 *	Computes angular position along the X axis from acc data
 *  	PARAM
 * 		@FLOAT [X,Y,Z]
 *	RETURNS
 *		@FLOAT Roll
 */
float ComplementaryFilter::getXRotation(float acc[3])
{
	float radians = atan2(acc[Y_INDEX], (double) dist(acc[X_INDEX],acc[Z_INDEX]));
	return toDegree(radians);
}

/*
 *	Computes roll angle from a trigonometric approach
 *  	PARAM
 * 		@FLOAT [X,Y,Z]	
 *	RETURNS
 *		@FLOAT Roll
 */
float ComplementaryFilter::getRoll(float acc[])
{
	float roll = getXRotation(acc);
	return roll;
}

/*
 *	Computes angular position along the Y axis from acc data
 *  	PARAM
 * 		@FLOAT [X,Y,Z]	
 *	RETURNS
 *		@FLOAT Pitch
 */
float ComplementaryFilter::getYRotation(float acc[])
{
	float radians = atan2(acc[X_INDEX], (double) dist(acc[Y_INDEX],acc[Z_INDEX]));
	return toDegree(radians);
}

/*
 *	Computes pitch angle from a trigonometric approach
 *  	PARAM
 * 		@FLOAT [X,Y,Z]	
 *	RETURNS
 *		@FLOAT Roll
 */
float ComplementaryFilter::getPitch(float acc[])
{
	float pitch = getYRotation(acc);
	return pitch;
}


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
  roll = 0.0;
  pitch = 0.0;
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
  roll = 0.0;
  pitch = 0.0;
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
  roll = 0.0;
  pitch = 0.0;
}


/*
 *	Computes value
 *  	PARAM
 * 		@FLOAT X,Y,Z	
 *	RETURNS
 *		@FLOAT Roll
 */
void ComplementaryFilter::Compute(float * acc, float * gyro)
{  
  if (cold)
  {
  	temp = micros();
  	cold = false;
  }
  dt = micros()-temp;

  angleXAcc = getRollAcc(acc);
  angleYAcc = getPitchAcc(acc);

  roll = (roll + gyro[0]*(float)dt/1000000.0)*_kgyro + angleXAcc*_kacc;
  pitch = (pitch + gyro[1]*(float)dt/1000000.0)*_kgyro + angleYAcc*_kacc;

  //Serial.print("\nroll:\t");
  //Serial.print((roll + gyro[0]*(float)dt/1000000.0)*_kgyro); 
  //Serial.print("\t Acc component:\t");
  //Serial.println(angleXAcc*_kacc);
  //Serial.print("\t delta:\t");
  //Serial.println(dt);

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
}

/* 
 *  Computes the distance between two points_ euclidean norm
 *    PARAM 
 *      @FLOAT point A
 *    @FLOAT point B
 *    RETURNS
 *    @FLOAT distance
 */
double ComplementaryFilter::dist(float a,float b) {
   return sqrt( (a*a)+(b*b));
}


/* 
 *  Computes roll angle
 *
 *    RETURNS
 *    @FLOAT Roll angle
 */
float ComplementaryFilter::getRoll(){
   return roll;
}

/* 
 *  Computes the pitch angle
 *
 *    RETURNS
 *    @FLOAT Pitch angle
 */
float ComplementaryFilter::getPitch() {
   return pitch;
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
float ComplementaryFilter::getRollAcc(float acc[])
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
	float radians = atan2(-acc[X_INDEX], (double) dist(acc[Y_INDEX],acc[Z_INDEX]));
	return toDegree(radians);
}

/*
 *	Computes pitch angle from a trigonometric approach
 *  	PARAM
 * 		@FLOAT [X,Y,Z]	
 *	RETURNS
 *		@FLOAT Roll
 */
float ComplementaryFilter::getPitchAcc(float acc[])
{
	float pitch = getYRotation(acc);
	return pitch;
}


/*
 * LKF.h
 *
 *  Created on: Nov 11, 2023
 *      Author: win 10
 */

#ifndef LKF_H_
#define LKF_H_

#define LENGTH_REAR 10
#define LENGTH_CAR 20
#define SIZE 5
#define NUMBEROFMODLE 5
typedef struct{
	float FriPx;
	float FriPy;
	float FriVel;
	float FriHea;
	float FriStee;

	float NexPx;
	float NexPy;
	float NexVel;
	float NexHea;
	float NexStee;

	float CovPx;
	float CovPy;
	float CovVel;
	float CovHea;
	float CovStee;

	uint8_t Prediction_CovarianceNex[NUMBEROFMODLE][NUMBEROFMODLE];
	uint8_t Prediction_CovarianceFri[NUMBEROFMODLE][NUMBEROFMODLE];
	float Jacobian[NUMBEROFMODLE][NUMBEROFMODLE];
	 //inovation
}LKF;

typedef struct {
	float Measurement_Covariance;
	float Jacobian[NUMBEROFMODLE];
	float Jacobian_Covariance;
	float Inovation[NUMBEROFMODLE];
}Measurement_Inovation;

typedef struct {
	float Vel;
	float Stee;
	float Hea;
	float Time;
	float Acceleration;
}Input;

typedef struct{
	float AngleBeta;
	float AngleTheta;
}Angle;

typedef struct{
	float GPSPosition[2][2];
	float GPSCovariance[2][2];
	uint8_t GPS_Model[NUMBEROFMODLE][NUMBEROFMODLE];
}GPS;

#endif /* LKF_H_ */



/*
 * LKF.c
 *
 *  Created on: Nov 11, 2023
 *      Author: win 10
 */
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "LKF.h"
void transposeMatrix(uint8_t mat[SIZE][SIZE], uint8_t result[SIZE][SIZE]) {
    for (uint8_t i = 0; i < SIZE; i++) {
        for (uint8_t j = 0; j < SIZE; j++) {
            result[j][i] = mat[i][j];
        }
    }
}
void matrixMultiplication(uint8_t mat1[SIZE][SIZE], uint8_t mat2[SIZE][SIZE], uint8_t result[SIZE][SIZE]) {
    for (uint8_t i = 0; i < SIZE; i++) {
        for (uint8_t j = 0; j < SIZE; j++) {
            result[i][j] = 0;
            for (uint8_t k = 0; k < SIZE; k++) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}

void EKF_Init(LKF *LKF)
{
	LKF->CovHea=0;
	LKF->CovPx=0;
	LKF->CovPy=0;
	LKF->CovStee=0;
	LKF->CovVel=0;
	LKF->FriHea=0;
	LKF->FriPx=0;
	LKF->FriPy=0;
	LKF->FriStee=0;
	LKF->FriVel=0;
	LKF->NexHea=0;
	LKF->NexPy=0;
	LKF->NexPx=0;
	LKF->NexStee=0;
	LKF->NexVel=0;
	memset(LKF->Prediction_CovarianceNex,0,sizeof(LKF->Prediction_CovarianceNex));
	memset(LKF->Prediction_CovarianceFri,0,sizeof(LKF->Prediction_CovarianceFri));
}

void GPS_Init(GPS *GPS){
	GPS->GPSCovariance[0][0]=0; //input fromsensor
	GPS->GPSCovariance[1][1]=0; //input fromsensor

	GPS->GPSPosition[0][0]=0; //input fromsensor
	GPS->GPSPosition[1][1]=0;//input fromsensor

	GPS->GPS_Model[0][0]=1;
	GPS->GPS_Model[1][1]=1;

	GPS->GPS_Model[0][1]=0;
	GPS->GPS_Model[0][2]=0;
	GPS->GPS_Model[0][3]=0;
	GPS->GPS_Model[0][4]=0;
	GPS->GPS_Model[1][0]=0;
	GPS->GPS_Model[1][2]=0;
	GPS->GPS_Model[1][3]=0;
	GPS->GPS_Model[1][4]=0;

}
void EKF_PredictionStep(LKF *LKF, Angle *Angle, Input *Input){
// Prediction State
	 Angle->AngleBeta= atan((LENGTH_REAR*tan(LKF->FriStee))/LENGTH_CAR);
	 LKF->NexPx= LKF->FriPx + (LKF->FriVel * cos(Angle->AngleBeta+LKF->FriHea))*Input->Time;
	 LKF->NexPy= LKF->FriPy + (LKF->FriVel * sin(Angle->AngleBeta +LKF->FriHea))*Input->Time;
	 LKF->NexVel= LKF->FriVel + Input->Acceleration *Input->Time;
	 LKF->NexHea= LKF->FriHea + Input->Hea*Input->Time;
	 LKF->NexStee = LKF->FriStee + Input->Stee*Input->Time;
// Prediction Covariance
	 uint8_t Mul_Result[5][5];
	 uint8_t Trans_Result[5][5];
	 uint8_t Prediction_CovarianceNex[5][5];
	 memset(Trans_Result,0,sizeof(Trans_Result));
	 memset(Prediction_CovarianceNex,0,sizeof(Prediction_CovarianceNex));
	 uint8_t Jacobian[5][5]={{1,0,0,0,0},
			 	 	 	 	 {0,1,0,0,0},
							 {Input->Time*cos(Angle->AngleBeta+LKF->FriHea),Input->Time * sin(Angle->AngleBeta+LKF->FriHea),1,0,0},
							 {-(Input->Time)*LKF->FriVel*sin(Angle->AngleBeta+LKF->FriHea),Input->Time * sin(Angle->AngleBeta+LKF->FriHea)*LKF->FriVel,0,1,0},
							 {0,0,0,0,1}
	 	 	 	 	 	 	 	 	 	 };
	 LKF->Prediction_CovarianceFri[0][0]=LKF->CovPx;   //find covariance
	 LKF->Prediction_CovarianceFri[1][1]=LKF->CovPy;	//find covariance
	 LKF->Prediction_CovarianceFri[2][2]=LKF->CovVel;	//find covariance
	 LKF->Prediction_CovarianceFri[3][3]=LKF->CovHea;	//find covariance
	 LKF->Prediction_CovarianceFri[4][4]=LKF->CovStee;	//find covariance

	 matrixMultiplication(Jacobian,LKF->Prediction_CovarianceFri, Mul_Result);
	 transposeMatrix(Jacobian, Trans_Result);
	 matrixMultiplication(Mul_Result,Trans_Result,LKF->Prediction_CovarianceNex); // Covaricaace result
}
void EFK_GPSHandleMeasurement(GPS *GPS, LKF *LKF ){
	float MeasurementFri[2][2];
	float Inovation[2][2];
	float GPSMeasurement[2][2];
	float InovationCov[5][5];
	float GPSModle[2][2];
	float PredictionNex[2][2];
	float GPSCov[2][2];
	float KalmanGian[2][2];
	float Identity[2][2];
//identity
	 Identity[0][0]=1;
	 Identity[1][1]=1;
//GPS covariance
	GPSCov[0][0]=GPS->GPSCovariance[0][0];
	GPSCov[1][1]=GPS->GPSCovariance[1][1];
//GPS measurement
	GPSMeasurement[0][0]=GPS->GPSPosition[0][0];
	GPSMeasurement[1][1]=GPS->GPSPosition[1][1];
//GPS Modle
	GPSModle[0][0]=GPS->GPS_Model[0][0];
	GPSModle[1][1]=GPS->GPS_Model[1][1];

	MeasurementFri[0][0]= LKF->FriPx;
	MeasurementFri[1][1]= LKF->FriPy;

	PredictionNex[0][0]=LKF->Prediction_CovarianceNex[0][0];
	PredictionNex[1][1]=LKF->Prediction_CovarianceNex[1][1];
	//inovation
	Inovation[0][0]= GPSMeasurement[0][0] - MeasurementFri[0][0];
	Inovation[1][1]= GPSMeasurement[1][1] - MeasurementFri[1][1];
	//inovation covariacne
	InovationCov[0][0]= GPSModle[0][0] * PredictionNex[0][0];
	InovationCov[1][1]= GPSModle[1][1] * PredictionNex[1][1];
	InovationCov[0][0]=  InovationCov[0][0] +GPSCov[0][0];
	InovationCov[1][1]=  InovationCov[1][1] + GPSCov[1][1];
	//Kalman Gain
	KalmanGian[0][0]=  PredictionNex[0][0] *  GPSModle[0][0] * InovationCov[0][0];
	KalmanGian[1][1]=  PredictionNex[1][1] *  GPSModle[1][1] * InovationCov[1][1];
	//update
	LKF->FriPx = LKF->NexPx + (KalmanGian[0][0] * Inovation[0][0]);
	LKF->FriPy = LKF->NexPy + (KalmanGian[1][1] * Inovation[1][1]);
	//update covariance
	LKF->Prediction_CovarianceFri[0][0]=(Identity[0][0]-(KalmanGian[0][0] * GPSModle[0][0]))-PredictionNex[0][0];
	LKF->Prediction_CovarianceFri[1][1]=(Identity[1][1]-(KalmanGian[1][1] * GPSModle[1][1]))-PredictionNex[0][0];
}

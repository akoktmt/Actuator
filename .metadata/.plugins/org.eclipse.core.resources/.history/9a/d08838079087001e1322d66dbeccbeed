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
void transposeMatrix(float mat[SIZE][SIZE], float result[SIZE][SIZE]) {
    for (uint8_t i = 0; i < SIZE; i++) {
        for (uint8_t j = 0; j < SIZE; j++) {
            result[j][i] = mat[i][j];
        }
    }
}

void matrixMultiplication(float mat1[SIZE][SIZE], float mat2[SIZE][SIZE], float result[SIZE][SIZE]) {
    for (uint8_t i = 0; i < SIZE; i++) {
        for (uint8_t j = 0; j < SIZE; j++) {
            result[i][j] = 0;
            for (uint8_t k = 0; k < SIZE; k++) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}

void EKF_Init(LKF *LKF,Input *Input)
{
//First Step
	LKF->FriPx= 106.802343;  //input
	LKF->FriPy=10.869826; //input
	LKF->FriVel=5; //input
	LKF->FriHea=-PI/6; //input
	LKF->FriStee=PI/6; //input
//Next Step
	LKF->NexPx=0;
	LKF->NexPy=0;
	LKF->NexVel=0;
	LKF->NexHea=0;
	LKF->NexStee=0;
//Covariance
	LKF->CovPx=0; //input
	LKF->CovPy=0; //input
	LKF->CovVel=0; //input
	LKF->CovHea=0; //input
	LKF->CovStee=0; //input
//FirstIput
	Input->Acceleration=0.2; //input
	Input->Time=0.1; //input
//Covariance
	memset(LKF->Prediction_CovarianceNex,0,sizeof(LKF->Prediction_CovarianceNex));
	memset(LKF->Prediction_CovarianceFri,0,sizeof(LKF->Prediction_CovarianceFri));

	LKF->Prediction_CovarianceFri[0][0]= LKF->CovPx;
	LKF->Prediction_CovarianceFri[0][0]= LKF->CovPy;
	LKF->Prediction_CovarianceFri[0][0]= LKF->CovVel;
	LKF->Prediction_CovarianceFri[0][0]= LKF->CovHea;
	LKF->Prediction_CovarianceFri[0][0]= LKF->CovStee;
}

void GPS_Init(GPS *GPS){

	memset(GPS->GPSCovariance,0,sizeof(GPS->GPSCovariance));
	memset(GPS->GPSGetPosition,0,sizeof(GPS->GPSGetPosition));
	memset(GPS->GPS_Model,0,sizeof(GPS->GPS_Model));

	GPS->GPSCovariance[0][0]=0; //input fromsensor
	GPS->GPSCovariance[1][1]=0; //input fromsensor

	GPS->GPSGetPosition[0][0]=106.802343; //input fromsensor

	GPS->GPSGetPosition[1][1]=10.869826;//input fromsensor

	GPS->GPS_Model[0][0]=1;
	GPS->GPS_Model[1][1]=1;

}
void EKF_PredictionStep(LKF *LKF, Angle *Angle, Input *Input){
// Prediction State
	 Angle->AngleBeta= (atan(LENGTH_REAR*tan(LKF->FriStee))/LENGTH_CAR);
	 LKF->NexPx= LKF->FriPx + (LKF->FriVel*0.1 * cos(Angle->AngleBeta+LKF->FriHea))*Input->Time*0.0001;
	 LKF->NexPy= LKF->FriPy + (LKF->FriVel*0.1 * sin(Angle->AngleBeta +LKF->FriHea))*Input->Time*0.0001;
	 LKF->NexVel= LKF->FriVel + Input->Acceleration *Input->Time*0.1;
	 LKF->NexHea= LKF->FriHea + ((LKF->FriVel*tan(LKF->FriStee)*cos(Angle->AngleBeta))/LENGTH_CAR)*Input->Time*0.1;
	 LKF->NexStee = LKF->FriStee + Input->Stee*Input->Time*0.1;
// Prediction Covariance
	 float Mul_Result[5][5];
	 float Trans_Result[5][5];
	 float Jacobian[5][5];

	 memset(Trans_Result,0,sizeof(Trans_Result));
	 memset(Jacobian,0,sizeof(Jacobian));

	 Jacobian[0][0]=1;
	 Jacobian[0][1]=0;
	 Jacobian[0][2]=Input->Time*cos(LKF->FriHea+atan((LENGTH_REAR*tan(LKF->FriStee))/LENGTH_CAR));
	 Jacobian[0][3]=-(LKF->FriVel *LENGTH_CAR * Input->Time *sin(LKF->FriHea+atan(LENGTH_REAR*tan(LKF->FriStee)/LENGTH_CAR))*1/cos(LKF->FriStee)*cos(LKF->FriStee))/LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(LKF->FriStee)*tan(LKF->FriStee);
	 Jacobian[0][4]=-LKF->FriVel*sin(LKF->FriHea+atan(LENGTH_REAR*tan(LKF->FriStee)/LENGTH_CAR));

	 Jacobian[1][0]=0;
	 Jacobian[1][1]=1;
	 Jacobian[1][2]=Input->Time*sin(LKF->FriHea+atan((LENGTH_REAR*tan(LKF->FriStee))/LENGTH_CAR));
	 Jacobian[1][3]=(LKF->FriVel *LENGTH_CAR * Input->Time *cos(LKF->FriHea+atan(LENGTH_REAR*tan(LKF->FriStee)/LENGTH_CAR))*1/cos(LKF->FriStee)*cos(LKF->FriStee))/LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(LKF->FriStee)*tan(LKF->FriStee);
	 Jacobian[1][4]=LKF->FriVel*sin(LKF->FriHea+atan(LENGTH_REAR*tan(LKF->FriStee)/LENGTH_CAR));

	 Jacobian[2][0]=0;
	 Jacobian[2][1]=0;
	 Jacobian[2][2]=1;
	 Jacobian[2][3]=0;
	 Jacobian[2][4]=0;

	 Jacobian[3][0]=0;
	 Jacobian[3][1]=0;
	 Jacobian[3][2]=(Input->Time*tan(LKF->FriStee)*abs(LKF->FriStee))/(LKF->FriStee*sqrt(LKF->FriStee*LKF->FriStee+LENGTH_REAR*LENGTH_REAR*tan(LKF->FriStee)*tan(LKF->FriStee)));
	 Jacobian[3][3]=(LKF->FriVel*LENGTH_CAR*LENGTH_CAR*LENGTH_CAR*Input->Time*(1/cos(LKF->FriStee)*1/cos(LKF->FriStee)))/(pow(LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(LKF->FriStee)*tan(LKF->FriStee),1.5)*abs(LKF->FriStee));
	 Jacobian[3][4]=1;

	 Jacobian[4][0]=0;
	 Jacobian[4][1]=0;
	 Jacobian[4][2]=0;
	 Jacobian[4][3]=1;
	 Jacobian[4][4]=0;

	 matrixMultiplication(Jacobian,LKF->Prediction_CovarianceFri, Mul_Result);
	 transposeMatrix(Jacobian, Trans_Result);
	 matrixMultiplication(Mul_Result,Trans_Result,LKF->Prediction_CovarianceNex); // Covaricaace result
}
void EFK_GPSHandleMeasurement(GPS *GPS, LKF *LKF ){

	float MeasurementFri[2][2];

	float Inovation[2][2];
	float InovationCov[5][5];

	float GPSMeasurement[2][2];
	float GPSModel[2][5];
	float GPSModelT[2][5];
	float GPSCov[2][2];

	float PredictionNex[2][2];
	float KalmanGian[2][2];
	float Identity[2][2];
//identity
	 Identity[0][0]=1;
	 Identity[1][1]=1;
//GPS covariance
	GPSCov[0][0]=GPS->GPSCovariance[0][0];
	GPSCov[1][1]=GPS->GPSCovariance[1][1];
//GPS measurement
	GPSMeasurement[0][0]=GPS->GPSGetPosition[0][0];
	GPSMeasurement[1][1]=GPS->GPSGetPosition[1][1];
//GPS Modle

	memset(GPSModel,0,sizeof(GPSModel));
	GPSModel[0][0]=GPS->GPS_Model[0][0];
	GPSModel[1][1]=GPS->GPS_Model[1][1];
	transposeMatrix(GPSModel,GPSModelT);

	//First Measurement
	MeasurementFri[0][0]= LKF->FriPx;
	MeasurementFri[1][1]= LKF->FriPy;
	//prediction
	PredictionNex[0][0]=LKF->Prediction_CovarianceNex[0][0];
	PredictionNex[1][1]=LKF->Prediction_CovarianceNex[1][1];
	//inovation
	Inovation[0][0]= GPSMeasurement[0][0] - MeasurementFri[0][0];
	Inovation[1][1]= GPSMeasurement[1][1] - MeasurementFri[1][1];
	//inovation covariacne
	InovationCov[0][0]= GPSModel[0][0] * PredictionNex[0][0]*GPSModelT[0][0];
	InovationCov[1][1]= GPSModel[1][1] * PredictionNex[1][1]*GPSModelT[1][1];
	InovationCov[0][0]=  InovationCov[0][0] +GPSCov[0][0];
	InovationCov[1][1]=  InovationCov[1][1] + GPSCov[1][1];
	//Kalman Gain
	KalmanGian[0][0]=  PredictionNex[0][0] *  GPSModel[0][0] * InovationCov[0][0];
	KalmanGian[1][1]=  PredictionNex[1][1] *  GPSModel[1][1] * InovationCov[1][1];
	//update
	LKF->FriPx = LKF->NexPx + (KalmanGian[0][0] * Inovation[0][0]);
	LKF->FriPy = LKF->NexPy + (KalmanGian[1][1] * Inovation[1][1]);
	//update covariance
	LKF->Prediction_CovarianceFri[0][0]=(Identity[0][0]-(KalmanGian[0][0] * GPSModel[0][0]))*PredictionNex[0][0];
	LKF->Prediction_CovarianceFri[1][1]=(Identity[1][1]-(KalmanGian[1][1] * GPSModel[1][1]))*PredictionNex[1][1];
}


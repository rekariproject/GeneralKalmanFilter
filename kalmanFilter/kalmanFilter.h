//
// Created by felix on 02.05.22.
//

#ifndef KALMANFILTER_KALMANFILTER_H
#define KALMANFILTER_KALMANFILTER_H


#include "Matrix.h"

class kalmanFilter {
public:
    explicit kalmanFilter(int dimension);
    void initial();
    void update();

   //Matrix<float> *StateTransitionMatrix=new Matrix<float>(2,2);
   Matrix<float, 6, 6> F_StateTransitionMatrix;
   Matrix<float, 6, 6> P_EstimateUncertaintyMatrix_currentState;
    Matrix<float, 6, 6> P_EstimateUncertaintyMatrix_nextState;
    Matrix<float, 6, 6> Q_ProcessNoiseMatrix;
    Matrix<float, 2, 6> H_observationMatrix;
    Matrix<float, 2, 1> z_measurement;
    Matrix<float, 6, 2> K_KalmanGain;

    //Matrix<float> *ControlMatrix;
   //Matrix<float> *ObservationMatrix;
   //Matrix<float> *MeasurementCovarianceMatrix;
   //Matrix<float> *currentStateMatrix;

   float KalmanGain;

private:
};


#endif //KALMANFILTER_KALMANFILTER_H

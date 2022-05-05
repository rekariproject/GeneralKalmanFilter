//
// Created by felix on 02.05.22.
//

#ifndef KALMANFILTER_KALMANFILTER_H
#define KALMANFILTER_KALMANFILTER_H


#include "Matrix.h"
#include "DMatrix.h"

class kalmanFilter {
public:
    explicit kalmanFilter(int dimension);
    void initial();
    void update(DMatrix<float> measurement);

    DMatrix<float> F_StateTransitionMatrix = DMatrix<float>(6, 6);
    DMatrix<float> P_EstimateUncertaintyMatrix[2] = {DMatrix<float>(6, 6), DMatrix<float>(6, 6)};
    DMatrix<float> Q_ProcessNoiseMatrix = DMatrix<float>(6, 6);
    DMatrix<float> K_KalmanGainMatrix = DMatrix<float>(6, 2);
    DMatrix<float> H_observationMatrix = DMatrix<float>(2, 6);
    DMatrix<float> R_MeasurementUncertainty = DMatrix<float>(2, 2);
    DMatrix<float> x0_CurrentState = DMatrix<float>(6, 1);
    DMatrix<float> x1_PredictedState = DMatrix<float>(6, 1);
    DMatrix<float> z_Measurement = DMatrix<float>(2, 1);

private:
};


#endif //KALMANFILTER_KALMANFILTER_H

//
// Created by felix on 02.05.22.
//

#include <valarray>
#include "kalmanFilter.h"

kalmanFilter::kalmanFilter(int dimension) {

}

void kalmanFilter::initial() {
    float *deltaT = new float (1);

    F_StateTransitionMatrix.set<6,6>(new float[6][6]{1, *deltaT, 0.5f * (*deltaT) * (*deltaT), 0, 0, 0,
                                                     0, 1, *deltaT, 0, 0, 0,
                                                     0, 0, 1, 0, 0, 0,
                                                     0, 0, 0, 1, *deltaT,0.5f*(*deltaT)*(*deltaT),
                                                     0, 0, 0, 0, 1, *deltaT,
                                                     0, 0, 0, 0, 0, 1});

    P_EstimateUncertaintyMatrix[0].set<6,6>(new float[6][6]{500, 0, 0, 0, 0, 0,
                                                                      0, 500, 0, 0, 0, 0,
                                                                      0, 0, 500, 0, 0, 0,
                                                                      0, 0, 0, 500, 0, 0,
                                                                      0, 0, 0, 0, 500, 0,
                                                                      0, 0, 0, 0, 0, 500});

    Q_ProcessNoiseMatrix.set<6,6>(new float[6][6]{(float)pow((double)*deltaT, 4.0) / 4, (float)pow((double)*deltaT, 3.0) / 2, (float)pow((double)*deltaT, 2.0) / 2, 0, 0, 0,
                                                (float)pow((double)*deltaT,3.0)/2,*deltaT**deltaT, *deltaT, 0, 0, 0,
                                                (float)pow((double)*deltaT,2.0)/2, *deltaT, 1, 0, 0, 0,
                                                  0, 0, 0,(float)pow((double)*deltaT,4.0)/4,(float)pow((double)*deltaT,3.0)/2,(float)pow((double)*deltaT,2.0)/2,
                                                  0, 0, 0,(float)pow((double)*deltaT,3.0)/2,*deltaT**deltaT, *deltaT,
                                                  0, 0, 0,(float)pow((double)*deltaT,2.0)/2, *deltaT, 1});

    H_observationMatrix.set<2,6>(new float[2][6]{1, 0, 0, 0, 0, 0,
                                                 0, 0, 0, 1, 0, 0,});

    R_MeasurementUncertainty.set<2,2>(new float[2][2]{9, 0,
                                                      0, 9});

    x0_CurrentState.set<6,1>(new float[6][1]{0,
                                             0,
                                             0,
                                             0,
                                             0,
                                             0,});

    z_Measurement.set<2,1>(new float[2][1]{0,
                                           0,});


    //Covariance Extrapolation
    P_EstimateUncertaintyMatrix[1]= F_StateTransitionMatrix * P_EstimateUncertaintyMatrix[0] * F_StateTransitionMatrix.transpose() + Q_ProcessNoiseMatrix * (0.2 * 0.2);
    P_EstimateUncertaintyMatrix[1].printMatrix();
}

void kalmanFilter::update(DMatrix<float> measurement) {



    //Kalman Gain
    K_KalmanGainMatrix = (P_EstimateUncertaintyMatrix[1] * H_observationMatrix.transpose()) * (H_observationMatrix * P_EstimateUncertaintyMatrix[1] * H_observationMatrix.transpose() + R_MeasurementUncertainty).inverse(2);
    K_KalmanGainMatrix.printMatrix();

    //Measure
    z_Measurement=measurement;

    //Estimate Current State
    x0_CurrentState= x0_CurrentState + K_KalmanGainMatrix * (z_Measurement - H_observationMatrix * x0_CurrentState);
    x0_CurrentState.printMatrix();

    //Estimate Uncertainty
    P_EstimateUncertaintyMatrix[0]= (identity<float>(6) - K_KalmanGainMatrix * H_observationMatrix) * P_EstimateUncertaintyMatrix[1] * (identity<float>(6) - K_KalmanGainMatrix * H_observationMatrix).transpose() + (K_KalmanGainMatrix * R_MeasurementUncertainty * K_KalmanGainMatrix.transpose());
    P_EstimateUncertaintyMatrix[0].printMatrix();

    //Predict Next State
    x1_PredictedState= F_StateTransitionMatrix * x0_CurrentState;
    x1_PredictedState.printMatrix();

    //Covariance Extrapolation
    P_EstimateUncertaintyMatrix[1]= F_StateTransitionMatrix * P_EstimateUncertaintyMatrix[0] * F_StateTransitionMatrix.transpose() + Q_ProcessNoiseMatrix * (0.2 * 0.2);
    P_EstimateUncertaintyMatrix[1].printMatrix();

}

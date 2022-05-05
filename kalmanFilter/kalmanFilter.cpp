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

    P_EstimateUncertaintyMatrix_currentState.set<6,6>(new float[6][6]{500, 0, 0, 0, 0, 0,
                                                                      0, 500, 0, 0, 0, 0,
                                                                      0, 0, 500, 0, 0, 0,
                                                                      0, 0, 0, 500, 0, 0,
                                                                      0, 0, 0, 0, 500, 0,
                                                                      0, 0, 0, 0, 0, 500});

    ProcessNoiseMatrix.set<6,6>(new float[6][6]{(float)pow((double)*deltaT,4.0)/4,(float)pow((double)*deltaT,3.0)/2,(float)pow((double)*deltaT,2.0)/2,0,0,0,
                                                (float)pow((double)*deltaT,3.0)/2,*deltaT**deltaT,*deltaT,0,0,0,
                                                (float)pow((double)*deltaT,2.0)/2,*deltaT,1,0,0,0,
                                                0,0,0,(float)pow((double)*deltaT,4.0)/4,(float)pow((double)*deltaT,3.0)/2,(float)pow((double)*deltaT,2.0)/2,
                                                0,0,0,(float)pow((double)*deltaT,3.0)/2,*deltaT**deltaT,*deltaT,
                                                0,0,0,(float)pow((double)*deltaT,2.0)/2,*deltaT,1});

    observationMatrix.set<2,6>(new float[2][6]{1,0,0,0,0,0,
                                                0,0,0,1,0,0,});

    MeasurementUncertainty.set<2,2>(new float[2][2]{9,0,
                                                    0,9});

    CurrentState.set<6,1>(new float[6][1]{0,
                                          0,
                                          0,
                                          0,
                                          0,
                                          0,});

    Measurement.set<2,1>(new float[2][1]{0,
                                         0,});


    //Covariance Extrapolation
    P_EstimateUncertaintyMatrix_nextState= F_StateTransitionMatrix * P_EstimateUncertaintyMatrix_currentState * F_StateTransitionMatrix.transpose() + ProcessNoiseMatrix * (0.2 * 0.2);
    P_EstimateUncertaintyMatrix_nextState.printMatrix();
}

void kalmanFilter::update(DMatrix<float> measurement) {



    //Kalman Gain
    KalmanGainMatrix = (P_EstimateUncertaintyMatrix_nextState * observationMatrix.transpose()) * (observationMatrix * P_EstimateUncertaintyMatrix_nextState * observationMatrix.transpose() + MeasurementUncertainty).inverse(2);
    KalmanGainMatrix.printMatrix();

    //Measure
    Measurement=measurement;

    //Estimate Current State
    CurrentState=CurrentState+KalmanGainMatrix*(Measurement-observationMatrix*CurrentState);
    CurrentState.printMatrix();

    //Estimate Uncertainty
    P_EstimateUncertaintyMatrix_currentState= (identity<float>(6) - KalmanGainMatrix * observationMatrix) * P_EstimateUncertaintyMatrix_nextState * (identity<float>(6) - KalmanGainMatrix * observationMatrix).transpose() + (KalmanGainMatrix * MeasurementUncertainty * KalmanGainMatrix.transpose());
    P_EstimateUncertaintyMatrix_currentState.printMatrix();

    //Predict Next State
    PredictedState= F_StateTransitionMatrix * CurrentState;
    PredictedState.printMatrix();

    //Covariance Extrapolation
    P_EstimateUncertaintyMatrix_nextState= F_StateTransitionMatrix * P_EstimateUncertaintyMatrix_currentState * F_StateTransitionMatrix.transpose() + ProcessNoiseMatrix * (0.2 * 0.2);
    P_EstimateUncertaintyMatrix_nextState.printMatrix();

}

//
// Created by felix on 02.05.22.
//

#include <valarray>
#include "kalmanFilter.h"

kalmanFilter::kalmanFilter(int dimension) {

}

void kalmanFilter::initial() {
    float *deltaT = new float (1);

    StateTransitionMatrix.set<6,6>(new float[6][6]{1,*deltaT,0.5f*(*deltaT)*(*deltaT),0,0,0,
                                                   0,1,*deltaT,0,0,0,
                                                   0,0,1,0,0,0,
                                                   0,0,0,1,*deltaT,0.5f*(*deltaT)*(*deltaT),
                                                   0,0,0,0,1,*deltaT,
                                                   0,0,0,0,0,1});

    EstimateUncertaintyMatrix_currentState.set<6,6>(new float[6][6]{500, 0, 0, 0, 0, 0,
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
    EstimateUncertaintyMatrix_nextState=StateTransitionMatrix * EstimateUncertaintyMatrix_currentState * StateTransitionMatrix.transpose() + ProcessNoiseMatrix*(0.2*0.2);
    EstimateUncertaintyMatrix_nextState.printMatrix();
}

void kalmanFilter::update() {



    //Kalman Gain
    KalmanGainMatrix = (EstimateUncertaintyMatrix_nextState*observationMatrix.transpose()) * (observationMatrix*EstimateUncertaintyMatrix_nextState*observationMatrix.transpose()+MeasurementUncertainty).inverse(2);
    KalmanGainMatrix.printMatrix();

    //Measure
    Measurement.matrix[0][0] = -393.66;
    Measurement.matrix[1][0] = 300.4;

    //Estimate Current State
    CurrentState=CurrentState+KalmanGainMatrix*(Measurement-observationMatrix*CurrentState);
    CurrentState.printMatrix();

    //Estimate Uncertainty
    EstimateUncertaintyMatrix_currentState=(identity<float>(6)-KalmanGainMatrix*observationMatrix)*EstimateUncertaintyMatrix_nextState*(identity<float>(6)-KalmanGainMatrix*observationMatrix).transpose()+(KalmanGainMatrix*MeasurementUncertainty*KalmanGainMatrix.transpose());
    EstimateUncertaintyMatrix_currentState.printMatrix();

    //Predict Next State
    PredictedState=StateTransitionMatrix*CurrentState;
    PredictedState.printMatrix();

    //Covariance Extrapolation
    EstimateUncertaintyMatrix_nextState=StateTransitionMatrix * EstimateUncertaintyMatrix_currentState * StateTransitionMatrix.transpose() + ProcessNoiseMatrix*(0.2*0.2);
    EstimateUncertaintyMatrix_nextState.printMatrix();

}

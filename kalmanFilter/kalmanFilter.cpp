//
// Created by felix on 02.05.22.
//

#include <valarray>
#include "kalmanFilter.h"

kalmanFilter::kalmanFilter(int dimension) {

}

void kalmanFilter::initial() {
    float *deltaT = new float (1);
    F_StateTransitionMatrix={1,*deltaT,0.5f*(*deltaT)*(*deltaT),0,0,0,
                           0,1,*deltaT,0,0,0,
                           0,0,1,0,0,0,
                           0,0,0,1,*deltaT,0.5f*(*deltaT)*(*deltaT),
                           0,0,0,0,1,*deltaT,
                           0,0,0,0,0,1};
    P_EstimateUncertaintyMatrix_currentState={500, 0, 0, 0, 0, 0,
                                              0, 500, 0, 0, 0, 0,
                                              0, 0, 500, 0, 0, 0,
                                              0, 0, 0, 500, 0, 0,
                                              0, 0, 0, 0, 500, 0,
                                              0, 0, 0, 0, 0, 500};



    Q_ProcessNoiseMatrix={(float)pow((double)*deltaT,4.0)/4,(float)pow((double)*deltaT,3.0)/2,(float)pow((double)*deltaT,2.0)/2,0,0,0,
                          (float)pow((double)*deltaT,3.0)/2,*deltaT**deltaT,*deltaT,0,0,0,
                          (float)pow((double)*deltaT,2.0)/2,*deltaT,1,0,0,0,
                          0,0,0,(float)pow((double)*deltaT,4.0)/4,(float)pow((double)*deltaT,3.0)/2,(float)pow((double)*deltaT,2.0)/2,
                          0,0,0,(float)pow((double)*deltaT,3.0)/2,*deltaT**deltaT,*deltaT,
                          0,0,0,(float)pow((double)*deltaT,2.0)/2,*deltaT,1};


    H_observationMatrix={1,0,0,0,0,0,
                         0,0,0,1,0,0,};

    F_StateTransitionMatrix.printMatrix();

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
}

void kalmanFilter::update() {

    //Covariance Extrapolation
   // P_EstimateUncertaintyMatrix_nextState=F_StateTransitionMatrix * P_EstimateUncertaintyMatrix_currentState * F_StateTransitionMatrix.transpose() + Q_ProcessNoiseMatrix*(0.2*0.2);
   // P_EstimateUncertaintyMatrix_nextState.printMatrix();

    //Covariance Extrapolation
    EstimateUncertaintyMatrix_nextState=StateTransitionMatrix * EstimateUncertaintyMatrix_currentState * StateTransitionMatrix.transpose() + ProcessNoiseMatrix*(0.2*0.2);
    EstimateUncertaintyMatrix_nextState.printMatrix();

    //Kalman Gain

   // KalmanGainMatrix=(EstimateUncertaintyMatrix_nextState*observationMatrix.transpose());

    KalmanGainMatrix = (EstimateUncertaintyMatrix_nextState*observationMatrix.transpose()) * (observationMatrix*EstimateUncertaintyMatrix_nextState*observationMatrix.transpose()+MeasurementUncertainty).inverse(2);
    KalmanGainMatrix.printMatrix();


   // observationMatrix.transpose().printMatrix();
   // Matrix<float, 6,2> m = P_EstimateUncertaintyMatrix_nextState*H_observationMatrix.transpose();

    //m.printMatrix();



}

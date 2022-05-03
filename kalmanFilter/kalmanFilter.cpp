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
}

void kalmanFilter::update() {

    //Covariance Extrapolation
    P_EstimateUncertaintyMatrix_nextState=F_StateTransitionMatrix * P_EstimateUncertaintyMatrix_currentState * F_StateTransitionMatrix.transpose() + Q_ProcessNoiseMatrix*(0.2*0.2);
    P_EstimateUncertaintyMatrix_nextState.printMatrix();


/*
    Matrix<float, 6,2> m = P_EstimateUncertaintyMatrix_nextState.multiply<6,6,6,2>(H_observationMatrix.transpose());

    m.printMatrix();
*/
    Matrix<float, 6,2> m = P_EstimateUncertaintyMatrix_nextState.multiply<6,6,6,2>(H_observationMatrix.transpose());
    Matrix<float, 6,2> b=H_observationMatrix.multiply<2,6,6,6>(P_EstimateUncertaintyMatrix_nextState.transpose()).multiply<6,6,6,2>(H_observationMatrix.transpose());
    m.printMatrix();

}

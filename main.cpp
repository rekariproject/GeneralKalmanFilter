#include <iostream>

#include "kalmanFilter/kalmanFilter.h"
kalmanFilter kf=kalmanFilter(6);
int main() {
    kf.initial();
    DMatrix<float> Measurement= DMatrix<float>(2,1);
    Measurement.matrix[0][0] = -393.66;
    Measurement.matrix[1][0] = 300.4;
    kf.update(Measurement);

    Measurement.matrix[0][0] = -375.93;
    Measurement.matrix[1][0] = 301.78;
    kf.update(Measurement);
    std::cout << "Hello, World!" << std::endl;

    return 0;
}


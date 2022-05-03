#include <iostream>

#include "kalmanFilter/kalmanFilter.h"
kalmanFilter kf=kalmanFilter(6);
int main() {
    kf.initial();
    kf.update();
    std::cout << "Hello, World!" << std::endl;

    return 0;
}


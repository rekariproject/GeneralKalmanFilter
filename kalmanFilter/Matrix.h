//
// Created by felix on 02.05.22.
//

#ifndef KALMANFILTER_MATRIX_H
#define KALMANFILTER_MATRIX_H

#include <iostream>

template<typename T, int h, int w>
class Matrix {
public:
    T matrix[h][w];

    Matrix<T, h, w> operator*(const Matrix<T, h, w> &other) const {
        Matrix<T, h, w> result;
        for(int row_left=0;row_left<h;row_left++) {
            for(int col_right=0;col_right<w;col_right++) {
                result.matrix[row_left][col_right] = 0;
                for(int col_left=0;col_left<w;col_left++) {
                    result.matrix[row_left][col_right] += matrix[row_left][col_left] * other.matrix[col_left][col_right];
                }
            }
        }
        return result;
    }


    Matrix<T, h, w> operator*(const T &factor) const {

        Matrix<T, h, w> result;
        for (int row = 0; row < h; row++) {
            for (int col = 0; col < w; col++) {
                result.matrix[row][col] = matrix[row][col] * factor;
            }
        }
        return result;
    }


    Matrix<T, h, w> operator+(const Matrix<T, h, w> &other) const {
        Matrix<T, h, w> result;
        for(int row=0;row<h;row++) {
            for(int col=0;col<w;col++) {
                result.matrix[row][col] = matrix[row][col] + other.matrix[row][col];
            }
        }
        return result;
    }

    void printMatrix() {
        printf("Matrix:\n");
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                std::cout << matrix[i][j] << " ";
            }
            std::cout << std::endl;
        }
        printf("\n");

    }

    Matrix<T, w, h> transpose(){
        Matrix<T, w, h> result=Matrix<T, w, h>();
        for(int row=0;row<h;row++) {
            for(int col=0;col<w;col++) {
                result.matrix[col][row] = matrix[row][col];
            }
        }
        return result;
    }

    template<int hl, int wl, int hr, int wr>
    Matrix<T, hr, wr> multiply(Matrix<T, hr, wr> other){
        Matrix<T, hr, wr> result;
        for(int row_left=0;row_left<h;row_left++) {
            for(int col_right=0;col_right<w;col_right++) {
                result.matrix[row_left][col_right] = 0;
                for(int col_left=0;col_left<w;col_left++) {
                    result.matrix[row_left][col_right] += matrix[row_left][col_left] * other.matrix[col_left][col_right];
                }
            }
        }


        return result;
    }



   // void set(const T matrix[6][6]);

private:
};



#endif //KALMANFILTER_MATRIX_H

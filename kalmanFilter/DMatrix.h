//
// Created by felix on 04.05.22.
//

#ifndef KALMANFILTER_DMATRIX_H
#define KALMANFILTER_DMATRIX_H

#include <iostream>
#include "invert.h"

template<typename T>
class DMatrix {
public:
    DMatrix(int rows, int cols){
        this->rows = rows;
        this->cols = cols;
        this->matrix = new T*[rows];
        for(int i = 0; i < rows; i++){
            this->matrix[i] = new T[cols];
        }
    }

    template<int rows, int cols>
    void set(const T m[rows][cols]){
        for(int i = 0; i < rows; i++){
            for(int j = 0; j < cols; j++){
                this->matrix[i][j] = m[i][j];
            }
        }
    }

    DMatrix<T> operator*(const DMatrix<T> &other) const {

        DMatrix<T> result=DMatrix<T>(rows, other.cols);
        for(int row_left=0;row_left<rows;row_left++) {
            for(int col_right=0;col_right<cols;col_right++) {
                result.matrix[row_left][col_right] = 0;
                for(int col_left=0;col_left<cols;col_left++) {
                    result.matrix[row_left][col_right] += matrix[row_left][col_left] * other.matrix[col_left][col_right];
                }
            }
        }
        return result;
    }

    DMatrix<T> operator+(const DMatrix<T> &other) const {
        DMatrix<T> result = DMatrix<T>(this->rows, this->cols);
        if(this->rows == other.rows && this->cols == other.cols){
            result = DMatrix<T>(this->rows, this->cols);
            for(int i = 0; i < this->rows; i++){
                for(int j = 0; j < this->cols; j++){
                    result.matrix[i][j] = this->matrix[i][j] + other.matrix[i][j];
                }
            }
        }
        return result;
    }

    DMatrix<T> operator*(const T &factor) const {

        DMatrix<T> result = DMatrix<T>(this->rows, this->cols);
        for(int i = 0; i < this->rows; i++){
            for(int j = 0; j < this->cols; j++){
                result.matrix[i][j] = this->matrix[i][j] * factor;
            }
        }
        return result;
    }

    void printMatrix(){
        printf("matrix:\n");
        for(int i = 0; i < rows; i++){
            for(int j = 0; j < cols; j++){
                std::cout << matrix[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }

    DMatrix<T> transpose() const {
        DMatrix<T> result=DMatrix<T>(this->cols, this->rows);
        for(int row_left=0;row_left<rows;row_left++) {
            for(int col_right=0;col_right<cols;col_right++) {
                result.matrix[col_right][row_left] = matrix[row_left][col_right];
            }
        }
        return result;
    }




    DMatrix<T> inverse(int order) const {
        DMatrix<T> result=DMatrix<T>(order, order);

        for(int i = 0; i < order; i++){
            for(int j = 0; j < order; j++){
                result.matrix[i][j] = matrix[i][j];
            }
        }

        float determinante=matrix[0][0]*matrix[1][1]-matrix[0][1]*matrix[1][0];
        result=result*(1/determinante);

        return result;
    }

    int rows = 0;
    int cols = 0;

    T** matrix = nullptr;

private:
};
#endif //KALMANFILTER_DMATRIX_H

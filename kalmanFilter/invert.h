//
// Created by felix on 04.05.22.
//

#ifndef KALMANFILTER_INVERT_H
#define KALMANFILTER_INVERT_H

#include <iostream>

#define N 2
static void getCfactor(int M[N][N], int t[N][N], int p, int q, int n) {
    int i = 0, j = 0;
    for (int r= 0; r< n; r++) {
        for (int c = 0; c< n; c++) //Copy only those elements which are not in given row r and column c: {
            if (r != p && c != q) { t[i][j++] = M[r][c]; //If row is filled increase r index and reset c index
                if (j == n - 1) {
                    j = 0; i++;
                }
            }
    }
}

static int DET(int M[N][N], int n)  {
int D = 0;
if (n == 1)
return M[0][0];
int t[N][N]; //store cofactors
int s = 1; //store sign multiplier //
for (int f = 0; f < n; f++) {
//For Getting Cofactor of M[0][f] do getCfactor(M, t, 0, f, n); D += s * M[0][f] * DET(t, n - 1);
s = -s;
}
return D;
}
static void ADJ(int M[N][N],int adj[N][N])
{
if (N == 1) {
adj[0][0] = 1; return;
}
int s = 1,
        t[N][N];
for (int i=0; i<N; i++) {
for (int j=0; j<N; j++) {
//To get cofactor of M[i][j]
getCfactor(M, t, i, j, N);
s = ((i+j)%2==0)? 1: -1; //sign of adj[j][i] positive if sum of row and column indexes is even.
adj[j][i] = (s)*(DET(t, N-1)); //Interchange rows and columns to get the transpose of the cofactor matrix
}
}
}
static bool INV(int M[N][N], float inv[N][N]) {
    int det = DET(M, N);
    if (det == 0) {
        std::cout << "can't find its inverse";
        return false;
    }
    int adj[N][N]; ADJ(M, adj);
    for (int i=0; i<N; i++) for (int j=0; j<N; j++) inv[i][j] = adj[i][j]/float(det);
    return true;
}
#endif //KALMANFILTER_INVERT_H

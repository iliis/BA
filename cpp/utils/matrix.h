#ifndef MATRIX_H_INCLUDED
#define MATRIX_H_INCLUDED

#include <limits>

#include <Eigen/Dense>
#include <fstream>
#include <limits.h>

#define INIT_NAN(matrix, W, H) do { \
    matrix.resize((H),(W)); \
    for(unsigned int r = 0; r < (H); r++) { \
        for(unsigned int c = 0; c < (W); c++) { \
            (matrix)(r,c) = std::numeric_limits<float>::quiet_NaN(); \
        } \
    } \
} while (0)

void memcpyCharToMatrix(Eigen::MatrixXf& mat, const unsigned char* source);
void writeMatrixToFile(const Eigen::MatrixXf& mat, const char* path);
void writeRawDataToFile(const char* data, unsigned int size, const char* path);

#endif /* end of include guard: MATRIX_H_INCLUDED */

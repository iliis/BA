#ifndef MATRIX_H_INCLUDED
#define MATRIX_H_INCLUDED

#include <limits>
#include <cmath>
#include <fstream>

#include <Eigen/Dense>

#define IS_INVALID(x)   (!std::isfinite(x))
#define INVALID()       (std::numeric_limits<float>::quiet_NaN())
//#define IS_INVALID(x)   ((x) < 0)
//#define INVALID()       (-1)

#define INIT_INVALID(matrix, W, H) do { \
    matrix.resize((H),(W)); \
    matrix.fill(INVALID()); \
} while(0)

float maxNoInvalid(const Eigen::MatrixXf& mat);
float minNoInvalid(const Eigen::MatrixXf& mat);

void memcpyCharToMatrix(Eigen::MatrixXf& mat, const unsigned char* source);
void writeMatrixToFile(const Eigen::MatrixXf& mat, const char* path);
void writeRawDataToFile(const char* data, unsigned int size, const char* path);

#endif /* end of include guard: MATRIX_H_INCLUDED */

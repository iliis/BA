/*
 * matrix.cpp
 *
 *  Created on: Jun 18, 2015
 *      Author: samuel
 */

#include "matrix.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
float maxNoNaN(const Eigen::MatrixXf& mat)
{
    float far = -std::numeric_limits<float>::infinity();

    for (int r = 0; r < mat.rows(); r++) {
        for (int c = 0; c < mat.cols(); c++) {
            if (far < mat(r,c))
                far = mat(r,c);
        }
    }

    return far;
}
///////////////////////////////////////////////////////////////////////////////
float minNoNaN(const Eigen::MatrixXf& mat)
{
    float near =  std::numeric_limits<float>::infinity();

    for (int r = 0; r < mat.rows(); r++) {
        for (int c = 0; c < mat.cols(); c++) {
            if (near > mat(r,c))
                near = mat(r,c);
        }
    }

    return near;
}
///////////////////////////////////////////////////////////////////////////////
void memcpyCharToMatrix(MatrixXf& mat, const unsigned char* source)
{

    for (int r = 0; r < mat.rows(); r++) {
        for (int c = 0; c < mat.cols(); c++) {
            const unsigned char v = *source++;
            if (v > 0 && v < 240)
                mat(r,c) = (float) v;
            else
                mat(r,c) = std::numeric_limits<float>::quiet_NaN();
        }
    }
}
///////////////////////////////////////////////////////////////////////////////
void writeMatrixToFile(const MatrixXf& mat, const char* path)
{
    ofstream outfile(path);

    for (int r = 0; r < mat.rows(); r++) {
        for (int c = 0; c < mat.cols(); c++) {

            outfile << mat(r,c);

            if (c < mat.cols()-1)
                outfile << ", ";
        }
        outfile << endl;
    }

    outfile.close();
}
///////////////////////////////////////////////////////////////////////////////
void writeRawDataToFile(const char* data, unsigned int size, const char* path)
{
    ofstream outfile(path);

    while (size-- > 0) {
        //outfile << (unsigned int) (*data++);
        outfile << (float) (*data++);

        if (size > 1)
            outfile << ", ";
    }

    outfile << endl;

    outfile.close();
}
///////////////////////////////////////////////////////////////////////////////

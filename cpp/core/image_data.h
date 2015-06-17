#ifndef IMAGE_DATA_H_INCLUDED
#define IMAGE_DATA_H_INCLUDED

#include <cmath>
#include <string>
#include <iostream>
#include <limits>
#include <sensor_msgs/Image.h>
#include <Eigen/Dense>


/*!
 * This class represents a basic image, with some additional helper functions
 *
 */
class ImageData
{
public:

    // allocates memory, but doesn't initialize it!
    // write to this->data and call updateImageFromMatrix()
    void create(const unsigned int W, const unsigned int H);

    // load data
    void loadFromMatrix(const Eigen::MatrixXf& source_data);

    // scale values, so that 0 -> near, 1 -> far
    void normalizeTo(float near, float far);

    inline unsigned int getWidth()  const { return data.cols(); }
    inline unsigned int getHeight() const { return data.rows(); }
    inline Eigen::Vector2f getSize()   const { return Eigen::Vector2f(getWidth(), getHeight()); }

    inline const Eigen::MatrixXf& getData() const { return data; }
    inline       Eigen::MatrixXf& getData()       { return data; }
    inline float  getValue(Eigen::Vector2i pos) const { return data(pos.y(), pos.x()); }
    inline float& getValue(Eigen::Vector2i pos)       { return data(pos.y(), pos.x()); }
    inline float& operator()(unsigned int x, unsigned int y) { return data(y,x); }

    float sampleValue(Eigen::Vector2f pos) const;
    Eigen::Matrix<float, 1, 2> sampleDiff(Eigen::Vector2f pos) const;
    Eigen::Matrix<float, 1, 2> getDiff(Eigen::Vector2i pos) const;

    // halve image size
    void downsample2();



    // authorative data
    Eigen::MatrixXf data;
};


#endif /* end of include guard: IMAGE_DATA_H_INCLUDED */

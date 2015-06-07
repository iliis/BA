#ifndef TRANSFORMATION_H_INCLUDED
#define TRANSFORMATION_H_INCLUDED

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "../utils/csv.h"

class Transformation
{
public:
    Transformation();
    Transformation(float x, float y, float z, float alpha, float beta, float gamma);

    friend std::ostream& operator <<(std::ostream &output, const Transformation &q);

    static std::vector<Transformation> loadFromCSV(const std::string& filename);

    float x, y, z;
    float alpha, beta, gamma;

    Eigen::Matrix3f getRotationMatrix() const;
    inline Eigen::Vector3f getTranslation() const { return Eigen::Vector3f(x,y,z); };

    Eigen::Vector3f operator()(const Eigen::Vector3f& vect) const;
};

#endif /* end of include guard: TRANSFORMATION_H_INCLUDED */

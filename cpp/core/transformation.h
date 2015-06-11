#ifndef TRANSFORMATION_H_INCLUDED
#define TRANSFORMATION_H_INCLUDED

#include <iostream>
#include <iomanip>
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

    friend std::ostream& operator <<(std::ostream &output, const Transformation &T);
           std::ostream& printCSV   (std::ostream &output);

    static std::vector<Transformation> loadFromCSV(const std::string& filename);

    //float x, y, z;
    //float alpha, beta, gamma; = pitch, yaw, roll
    Eigen::Matrix<float, 6, 1> value;


    inline float x()     const { return value(0); }
    inline float y()     const { return value(1); }
    inline float z()     const { return value(2); }
    inline float alpha() const { return value(3); } // pitch
    inline float beta()  const { return value(4); } // yaw
    inline float gamma() const { return value(5); } // roll

    inline const Eigen::Matrix3f& getRotationMatrix() const { return R; }
    inline Eigen::Vector3f getTranslation() const { return Eigen::Vector3f(x(),y(),z()); };

    Eigen::Matrix<float, 3, 6> getJacobian(const Eigen::Vector3f& point) const;

    Eigen::Vector3f operator()(const Eigen::Vector3f& vect) const;

    Transformation operator+(const Transformation& other) const;

    void updateRotationMatrix();

private:
    Eigen::Matrix3f R;
};

#endif /* end of include guard: TRANSFORMATION_H_INCLUDED */

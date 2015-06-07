#ifndef TRANSFORMATION_H_INCLUDED
#define TRANSFORMATION_H_INCLUDED

#include <iostream>
#include <vector>
#include <string>

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
};

#endif /* end of include guard: TRANSFORMATION_H_INCLUDED */

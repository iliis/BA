#ifndef EIGEN_SFML_CONVERSIONS_CPP_INCLUDED
#define EIGEN_SFML_CONVERSIONS_CPP_INCLUDED

#include "eigen_sfml_conversions.h"

///////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf* image_to_matrix(const sf::Image& image)
{
    Eigen::MatrixXf * matrix = new Eigen::MatrixXf(image.getSize().y, image.getSize().x);

    for (size_t y = 0; y < image.getSize().y; y++) {
        for (size_t x = 0; x < image.getSize().x; x++) {
            // convert image to grayscale
            const sf::Color& c = image.getPixel(x,y);
            (*matrix)(y,x) = (c.r / 255.0f + c.g / 255.0f + c.b / 255.0f) / 3;
        }
    }

    return matrix;
}
///////////////////////////////////////////////////////////////////////////////
sf::Image*       matrix_to_image(const Eigen::MatrixXf& matrix)
{
    sf::Image* image = new sf::Image();
    image->create(matrix.cols(), matrix.rows());

    for (int y = 0; y < matrix.rows(); y++) {
        for (int x = 0; x < matrix.cols(); x++) {
            float v = matrix(y,x);
            image->setPixel(x, y, sf::Color(v*255, v*255, v*255));
        }
    }

    return image;
}
///////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: EIGEN_SFML_CONVERSIONS_CPP_INCLUDED */

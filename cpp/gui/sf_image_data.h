#ifndef SF_IMAGE_DATA_H_INCLUDED
#define SF_IMAGE_DATA_H_INCLUDED

#include <math.h>
#include <SFML/Graphics.hpp>
#include <sensor_msgs/Image.h>

#include "colormap.h"
#include "../core/image_data.h"


// helper tools to convert between Eigen Matrices and SFML Images

void loadImageDataFromImage(ImageData& dest, const sf::Image& source_data);
void loadImageDataFromFile (ImageData& dest, const std::string source_file);

void loadImageDataFromROSgrayscale(ImageData& dest, const sensor_msgs::Image& source_data);
void loadImageDataFromROSdepthmap (ImageData& dest, const sensor_msgs::Image& source_data);

void image_to_matrix(const sf::Image& source, Eigen::MatrixXf& dest);
void matrix_to_image(const Eigen::MatrixXf& source, sf::Image& dest, const Colormap::Colormap& colormap = Colormap::Colormap()); // like imagesc()

// similar to imagesc()
void drawImageAt (sf::RenderTarget& target, const sf::Image& img,       const sf::Vector2f& pos, const std::string& label = "", const sf::Font* font = NULL, const float& scale = 1);
void drawMatrixAt(sf::RenderTarget& target, const Eigen::MatrixXf& mat, const sf::Vector2f& pos, const std::string& label = "", const sf::Font* font = NULL, const Colormap::Colormap& colormap = Colormap::Colormap(), const float& scale = 1);

#endif /* end of include guard: SF_IMAGE_DATA_H_INCLUDED */

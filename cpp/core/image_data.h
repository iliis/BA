#ifndef IMAGE_DATA_H_INCLUDED
#define IMAGE_DATA_H_INCLUDED

#include <cmath>
#include <string>
#include <iostream>
#include <SFML/Graphics.hpp>
#include <Eigen/Dense>

#include "../utils/colormap.h"

/*!
 * This class represents a basic image, with some additional helper functions
 * to make it usable both with Eigen and with SFML.
 *
 * It keeps it's data in an Eigen::MatrixXf and has a copy in a grayscale sf::Image.
 *
 */
class ImageData : public sf::Drawable
{
public:

    ImageData() : width(0), height(0) {};

    // allocates memory, but doesn't initialize it!
    // write to this->data and call updateImageFromMatrix()
    void create(const unsigned int W, const unsigned int H);

    // load data
    void loadFromMatrix(const Eigen::MatrixXf& source_data, const Colormap::Colormap& colormap = Colormap::Colormap());
    void loadFromImage(const sf::Image& source_data);
    void loadFromFile(const std::string source_file);

    // float sample(const PointCamera& point);

    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;
    void drawAt(sf::RenderTarget& target, const sf::Vector2f pos, const float scale = 1, sf::RenderStates states = sf::RenderStates()) const;

    static void image_to_matrix(const sf::Image& source, Eigen::MatrixXf& dest);
    static void matrix_to_image(const Eigen::MatrixXf& source, sf::Image& dest, const Colormap::Colormap& colormap = Colormap::Colormap()); // like imagesc()

    void updateMatrixFromImage();
    void updateImageFromMatrix(const Colormap::Colormap& colormap = Colormap::Colormap());

    // scale values, so that 0 -> near, 1 -> far
    void normalizeTo(float near, float far);

    inline unsigned int getWidth()  const { return width; }
    inline unsigned int getHeight() const { return height; }
    inline sf::Vector2f getSize()   const { return sf::Vector2f(width, height); }

    inline const Eigen::MatrixXf& getData() const { return data; }
    inline       Eigen::MatrixXf& getData()       { return data; }
    inline float  getValue(Eigen::Vector2i pos) const { return data(pos.y(), pos.x()); }
    inline float& getValue(Eigen::Vector2i pos)       { return data(pos.y(), pos.x()); }
    inline float& operator()(unsigned int x, unsigned int y) { return data(y,x); }

    float sampleValue(Eigen::Vector2f pos) const;
    Eigen::Matrix<float, 1, 2> sampleDiff(Eigen::Vector2f pos) const;


    // halve image size
    void downsample2(const Colormap::Colormap& colormap = Colormap::Colormap());

private:
    unsigned int width, height;

    // authorative data
    Eigen::MatrixXf data;

    // for fast drawing, keep a copy as SFML::Image (and another copy as texture in the GPU's memory)
    sf::Image   sf_image; // grayscale
    sf::Texture sf_texture;

};

void drawImageAt(const sf::Image& img, const sf::Vector2f& pos, sf::RenderTarget& target, sf::RenderStates states = sf::RenderStates());

#endif /* end of include guard: IMAGE_DATA_H_INCLUDED */

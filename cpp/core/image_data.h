#ifndef IMAGE_DATA_H_INCLUDED
#define IMAGE_DATA_H_INCLUDED

#include <cmath>
#include <string>
#include <iostream>
#include <SFML/Graphics.hpp>
#include <Eigen/Dense>

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

    // load data
    void loadFromMatrix(const Eigen::MatrixXf& source_data);
    void loadFromImage(const sf::Image& source_data);
    void loadFromFile(const std::string source_file);

    // float sample(const PointCamera& point);

    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;
    void draw_at(sf::RenderTarget& target, const sf::Vector2f pos, sf::RenderStates states = sf::RenderStates()) const;

    static void image_to_matrix(const sf::Image& source, Eigen::MatrixXf& dest);
    static void matrix_to_image(const Eigen::MatrixXf& source, sf::Image& dest);

    void updateMatrixFromImage();
    void updateImageFromMatrix();

    inline unsigned int getWidth()  const { return width; }
    inline unsigned int getHeight() const { return height; }
    inline sf::Vector2f getSize()   const { return sf::Vector2f(width, height); }

    inline const Eigen::MatrixXf& getData() const { return data; }
    inline float getValue(Eigen::Vector2i pos) const { return data(pos.y(), pos.x()); }
    float sampleValue(Eigen::Vector2f pos) const;

private:
    unsigned int width, height;

    // authorative data
    Eigen::MatrixXf data;

    // for fast drawing, keep a copy as SFML::Image (and another copy as texture in the GPU's memory)
    sf::Image   sf_image; // grayscale
    sf::Texture sf_texture;

};

#endif /* end of include guard: IMAGE_DATA_H_INCLUDED */

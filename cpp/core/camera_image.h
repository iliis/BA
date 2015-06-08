#ifndef SCENE_IMAGE_H_INCLUDED
#define SCENE_IMAGE_H_INCLUDED

#include <vector>
#include <SFML/Graphics.hpp>
#include <Eigen/Dense>

#include "image_data.h"
#include "camera_intrinsics.h"

struct Pixel
{
    Eigen::Vector2f pos;
    float intensity;
    float depth;
};

struct WorldPoint
{
    Pixel pixel;
    Eigen::Vector3f pos;
};

/*!
 * an image with its depth map
 * i.e. a single datapoint from the stereo camera
 */
class CameraImage : public sf::Drawable
{
public:

    void loadFromSceneDirectory(const std::string& scene_path, const unsigned int index, const CameraIntrinsics& intrinsics);
    void loadFromMatrices(const Eigen::MatrixXf& mat_intensities, const Eigen::MatrixXf& mat_depths);

    inline int getWidth()  const { return intensities.getWidth(); }
    inline int getHeight() const { return intensities.getHeight(); }

    Pixel getPixel   (Eigen::Vector2i pos) const; // directly read out pixel from integer coordinates
    float samplePixel(Eigen::Vector2f pos) const; // linearly interpolate between four pixels from float coordinates

    bool isValidPixel(Eigen::Vector2f pos) const;


    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;

private:
    ImageData intensities, depths;
};

#endif /* end of include guard: SCENE_IMAGE_H_INCLUDED */

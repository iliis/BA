#ifndef SCENE_IMAGE_H_INCLUDED
#define SCENE_IMAGE_H_INCLUDED

#include <vector>
#include <Eigen/Dense>

#include "image_data.h"

///////////////////////////////////////////////////////////////////////////////

struct Pixel
{
    Pixel(float x = 0, float y = 0, float depth = 0, float intensity = 0)
        : pos(x,y), intensity(intensity), depth(depth) {}

    Eigen::Vector2f pos;
    float intensity;
    float depth;
};

///////////////////////////////////////////////////////////////////////////////

struct WorldPoint
{
    WorldPoint(float x = 0, float y = 0, float z = 0, Pixel p = Pixel())
        : pos(x,y,z), pixel(p) {}

    Eigen::Vector3f pos;
    Pixel pixel;
};

///////////////////////////////////////////////////////////////////////////////

/*!
 * an image with its depth map
 * i.e. a single datapoint from the stereo camera
 *
 * TODO: rename to CameraFrame ?
 */
struct CameraImage
{
    void loadFromMatrices(const Eigen::MatrixXf& mat_intensities, const Eigen::MatrixXf& mat_depths);

    inline int getWidth()  const { return intensities.getWidth(); }
    inline int getHeight() const { return intensities.getHeight(); }

    Pixel getPixel   (Eigen::Vector2i pos) const; // directly read out pixel from integer coordinates
    float samplePixel(Eigen::Vector2f pos) const; // linearly interpolate between four pixels from float coordinates

    bool isValidPixel(Eigen::Vector2f pos) const;

    inline const ImageData& getIntensityData() const { return intensities; }
    inline const ImageData& getDepthData()     const { return depths; }

    void downsample2() { intensities.downsample2(); depths.downsample2(); }

    ImageData intensities, depths;
};

///////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: SCENE_IMAGE_H_INCLUDED */

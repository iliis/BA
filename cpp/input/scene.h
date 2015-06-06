#ifndef SCENE_H_INCLUDED
#define SCENE_H_INCLUDED

#include <vector>
#include <string>
#include <sstream>
#include <SFML/Graphics.hpp>
#include <Eigen/Dense>

#include "../core/image_data.h"

// TODO: how to organize memory? Where to keep objects, where to use pointers? Smart pointers?

class Scene;

///////////////////////////////////////////////////////////////////////////////
// TODO: move this into separate file or as a subclass into Scene

// an image with its depth map
class SceneImage : public sf::Drawable
{
public:

    void loadFromSceneDirectory(const std::string& scene_path, const unsigned int index);
    void loadFromMatrices(const Eigen::MatrixXf& mat_intensities, const Eigen::MatrixXf& mat_depths);

    inline int getWidth()  const { return intensities.getWidth(); }
    inline int getHeight() const { return intensities.getHeight(); }

    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;

private:
    ImageData intensities, depths;
};

///////////////////////////////////////////////////////////////////////////////
// TODO: move this as a subclass into Scene?

// a single step in a longer trajectory
struct SceneStep
{
    SceneImage* frame_first;
    SceneImage* frame_second;

    // for convenience
    unsigned int index;
    Scene* scene;
};

///////////////////////////////////////////////////////////////////////////////
// TODO: implement this class

class Scene
{
public:
    inline unsigned int getFrameCount() { return frames.size(); }
    inline unsigned int getStepCount()  { return getFrameCount() - 1; }

    SceneStep getStep(unsigned int index);

private:
    std::vector<SceneImage> frames;
};

///////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: SCENE_H_INCLUDED */

#ifndef SCENE_H_INCLUDED
#define SCENE_H_INCLUDED

#include <vector>
#include <string>
#include <sstream>
#include <Eigen/Dense>

#include "camera_intrinsics.h"
#include "image_data.h"
#include "camera_step.h"
#include "transformation.h"

///////////////////////////////////////////////////////////////////////////////

/*!
 * Keeps the data for a longer trajectory, containig an arbitrary amount of frames (list of CameraImages)
 */

class Scene
{
public:
    void loadFromSceneDirectory(const std::string& scene_path);

    inline unsigned int getFrameCount() const { return frames.size(); }
    inline unsigned int getStepCount()  const { return getFrameCount() - 1; }

          CameraStep   getStep(unsigned int index);
    const CameraImage& getFrame(unsigned int index) const;
    inline const CameraIntrinsics& getIntrinsics() const { return intrinsics; }

private:
    std::string source_path; // path to the scene directory
    std::vector<CameraImage> frames;
    std::vector<Transformation> ground_truth;
    CameraIntrinsics intrinsics;
};

///////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: SCENE_H_INCLUDED */

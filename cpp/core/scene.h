#ifndef SCENE_H_INCLUDED
#define SCENE_H_INCLUDED

#include <vector>
#include <string>
#include <sstream>
#include <boost/foreach.hpp>
#include <Eigen/Dense>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "../utils/progress.h"
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
    void loadFromBagFile(const std::string& bag_path);

    inline unsigned int getFrameCount() const { return frames.size(); }
    inline unsigned int getStepCount()  const { return getFrameCount() - 1; }

          CameraStep   getStep(unsigned int index) const;
          CameraStep   getStep(unsigned int indexA, unsigned int indexB) const;
    const CameraImage& getFrame(unsigned int index) const;
    inline const CameraIntrinsics& getIntrinsics() const { return intrinsics; }

    inline std::string getSourceDirectory() const { return source_path; }

private:
    std::string source_path; // path to the scene directory (no trailing slash)
    std::vector<CameraImage> frames;
    std::vector<Transformation> ground_truth;
    CameraIntrinsics intrinsics;
};

///////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: SCENE_H_INCLUDED */

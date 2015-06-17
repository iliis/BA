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

#include "sf_image_data.h"
#include "../utils/progress.h"
#include "../core/warp.h"
#include "../core/camera_intrinsics.h"
#include "../core/image_data.h"
#include "../core/camera_step.h"
#include "../core/transformation.h"
#include "../core/minimization.h"


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

void loadCameraImageFromSceneDirectory(CameraImage& img, const std::string& scene_path, const unsigned int index, const CameraIntrinsics& intrinsics);

std::vector<Transformation> findTrajectory(const Scene& scene, const Warp::Parameters& params);
std::vector<Transformation> findTrajectoryFromRosbag(const std::string& rosbag_path, const Warp::Parameters& params);

///////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: SCENE_H_INCLUDED */

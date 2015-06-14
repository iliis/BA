#include "scene.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
void Scene::loadFromSceneDirectory(const std::string& scene_path)
{
    assert(frames.empty());
    assert(ground_truth.empty());
    assert(scene_path[scene_path.length()-1] != '/'); // path should not end with slash

    this->intrinsics.loadFromCSV(scene_path + "/camera_intrinsics.csv");
    this->ground_truth = Transformation::loadFromCSV(scene_path + "/camera_trajectory_relative.csv");

    // use ground truth trajectory to determine how many frames this scene has
    this->frames.resize(ground_truth.size());
    for (unsigned int i = 0; i < ground_truth.size(); i++) {
        this->frames[i].loadFromSceneDirectory(scene_path, i, intrinsics);
    }

    this->source_path = scene_path;

    cout << "loaded scene with " << this->getFrameCount() << " frames." << endl;
}
///////////////////////////////////////////////////////////////////////////////
void Scene::loadFromBagFile(const std::string& bag_path)
{
    assert(frames.empty());
    assert(ground_truth.empty());

    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    cout << "bag is " << bag.getSize() << " bytes (?) big." << endl;

    rosbag::View view_intensity(bag, rosbag::TopicQuery("/stereo_dense_reconstruction/image_fused"));
    rosbag::View view_depth    (bag, rosbag::TopicQuery("/stereo_dense_reconstruction/disparity"));

    rosbag::View::const_iterator it_intensity = view_intensity.begin();
    rosbag::View::const_iterator it_depth     = view_depth    .begin();

    //const unsigned int N = view_intensity.size();
    const unsigned int N = 40; // just load the beginning of the recording

    this->frames.resize(N);
    this->ground_truth.resize(N);

    unsigned int i = 0;
    while (it_intensity++ != view_intensity.end() && it_depth++ != view_depth.end()) {


        sensor_msgs::Image::Ptr          p_intensities = it_intensity->instantiate<sensor_msgs::Image>();
        stereo_msgs::DisparityImage::Ptr p_depths      = it_depth    ->instantiate<stereo_msgs::DisparityImage>();

        if (!p_intensities || !p_depths) {
            cerr << "failed to load data from bag file in frame " << i << endl;
            return;
        }

        ImageData intensities, depths;
        intensities.loadFromROSgrayscale(*p_intensities);
        depths     .loadFromROSdepthmap (p_depths->image);

        frames[i].loadFromMatrices(intensities.getData(), depths.getData());

        // TODO: load ground truth from bag
        ground_truth[i] = Transformation(0,0,0,0,0,0);

        // TODO: depths values are actually disparity values!
        this->intrinsics = CameraIntrinsics(p_intensities->width, p_intensities->height, 1.0f / 480); //(p_depths->f * p_depths->T));

        printfProgress(i, 0, N);

        i++;

        // just load the beginning of the recording
        if (i >= N)
            break;
    }

    cout << "loaded scene with " << this->getFrameCount() << " frames from ROS bag." << endl;
}
///////////////////////////////////////////////////////////////////////////////
CameraStep Scene::getStep(unsigned int index) const
{
    assert(index < this->getStepCount());

    return CameraStep(frames[index], frames[index+1], ground_truth[index+1], this, index);
}
///////////////////////////////////////////////////////////////////////////////
const CameraImage& Scene::getFrame(unsigned int index) const
{
    assert(index < this->getFrameCount());

    return frames[index];
}
///////////////////////////////////////////////////////////////////////////////

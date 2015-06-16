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


#if 0
    ofstream outfile("./ground_truth_trajectory.csv");

    outfile << "x, y, z, X, Y, Z, W" << endl;

    rosbag::View view(bag, rosbag::TopicQuery("/aslam/aslam_odometry"));

    BOOST_FOREACH(const rosbag::MessageInstance m, view) {
        // TODO: display ground truth data

        nav_msgs::Odometry::Ptr p_odometry = m.instantiate<nav_msgs::Odometry>();

        if (p_odometry) {
            outfile << p_odometry->pose.pose.position.x << ", ";
            outfile << p_odometry->pose.pose.position.y << ", ";
            outfile << p_odometry->pose.pose.position.z << ", ";
            outfile << p_odometry->pose.pose.orientation.x << ", ";
            outfile << p_odometry->pose.pose.orientation.y << ", ";
            outfile << p_odometry->pose.pose.orientation.z << ", ";
            outfile << p_odometry->pose.pose.orientation.w << endl;
        } else {
            cerr << "couldn't get pose" << endl;
        }
    }

    outfile.close();

    cout << "wrote ground truth trajectory to CSV" << endl;
#endif



    rosbag::View view_intensity(bag, rosbag::TopicQuery("/stereo_dense_reconstruction/image_fused"));
    rosbag::View view_depth    (bag, rosbag::TopicQuery("/stereo_dense_reconstruction/disparity"));

    rosbag::View::const_iterator it_intensity = view_intensity.begin();
    rosbag::View::const_iterator it_depth     = view_depth    .begin();

    //const unsigned int N = view_intensity.size();
    int START = 250;
    const unsigned int N = 100; // don't load the whole scene (would need a bit too much RAM)

    this->frames.resize(N);
    this->ground_truth.resize(N);

    unsigned int i = 0;
    while (it_intensity++ != view_intensity.end() && it_depth++ != view_depth.end()) {

        // skip the first few frames
        if (START-- > 0)
            continue;


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

        // real sensor produces disparity data, not directly depth values
        this->intrinsics = CameraIntrinsics(Eigen::Vector2f(p_intensities->width, p_intensities->height), Eigen::Vector2f(370.105, 226.664), 471.7, 0.110174);

#if 1
        frames[i].downsample2();
        intrinsics.downsample2();
#endif

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
    return this->getStep(index, index+1);
}
///////////////////////////////////////////////////////////////////////////////
CameraStep Scene::getStep(unsigned int indexA, unsigned int indexB) const
{
    assert(indexA < this->getStepCount());
    assert(indexB < this->getStepCount());

    // TODO: calculate correct ground truth...

    return CameraStep(frames[indexA], frames[indexB], ground_truth[indexB], this, indexA);
}
///////////////////////////////////////////////////////////////////////////////
const CameraImage& Scene::getFrame(unsigned int index) const
{
    assert(index < this->getFrameCount());

    return frames[index];
}
///////////////////////////////////////////////////////////////////////////////

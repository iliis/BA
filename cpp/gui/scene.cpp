#include "scene.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
CameraIntrinsics visensor_intrinsics = CameraIntrinsics(
        /* sensor size     */ Eigen::Vector2f(752, 480),
        /* principal point */ Eigen::Vector2f(370.105, 226.664),
        /* focal length    */ 471.7,
        /* stereo baseline */ 0.110174);
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
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
        loadCameraImageFromSceneDirectory(frames[i], scene_path, i, intrinsics);
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
    this->intrinsics = visensor_intrinsics;

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
        loadImageDataFromROSgrayscale(intensities, *p_intensities);
        loadImageDataFromROSdepthmap (depths,      p_depths->image);

        frames[i].loadFromMatrices(intensities.getData(), depths.getData());

        // TODO: load ground truth from bag
        ground_truth[i] = Transformation(0,0,0,0,0,0);

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
void Scene::addFrames(const std::vector<CameraImage>& fs)
{
    this->frames.insert(frames.end(), fs.begin(), fs.end());

    for (unsigned int i = 0; i < fs.size(); i++)
        ground_truth.push_back(Transformation(0,0,0,0,0,0));
}
///////////////////////////////////////////////////////////////////////////////
CameraStep Scene::getStep(unsigned int index) const
{
    assert(index < this->getStepCount());
    return this->getStep(index, index+1);
}
///////////////////////////////////////////////////////////////////////////////
CameraStep Scene::getStep(unsigned int indexA, unsigned int indexB) const
{
    assert(indexA < this->getFrameCount());
    assert(indexB < this->getFrameCount());

    // TODO: calculate correct ground truth...

    return CameraStep(frames[indexA], frames[indexB], intrinsics, ground_truth[indexB], indexA, indexB);
}
///////////////////////////////////////////////////////////////////////////////
const CameraImage& Scene::getFrame(unsigned int index) const
{
    assert(index < this->getFrameCount());

    return frames[index];
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void loadCameraImageFromSceneDirectory(CameraImage& img, const std::string& scene_path, const unsigned int index, const CameraIntrinsics& intrinsics)
{
    //std::string index_string = "000001";
    ostringstream index_string;
    index_string.width(4);
    index_string.fill('0');
    index_string << (index + 1); // filenames start with 1

    loadImageDataFromFile(img.intensities, scene_path + "/color" + index_string.str() + ".png");
    loadImageDataFromFile(img.depths     , scene_path + "/depth" + index_string.str() + ".png");

    img.depths.normalizeTo(intrinsics.getNearClipping(), intrinsics.getFarClipping());
}
///////////////////////////////////////////////////////////////////////////////
std::vector<Transformation> findTrajectory(const Scene& scene, const Warp::Parameters& params)
{
    std::vector<Transformation> traj(scene.getStepCount());

    for (size_t i = 0; i < scene.getStepCount(); i++) {
        traj[i] = findTransformationWithPyramid(scene.getStep(i), params);
    }

    return traj;
}
///////////////////////////////////////////////////////////////////////////////
std::vector<Transformation> findTrajectoryFromRosbag(const string& rosbag_path, const Warp::Parameters& params)
{
    std::vector<Transformation> traj;


    rosbag::Bag bag;
    bag.open(rosbag_path, rosbag::bagmode::Read);


    rosbag::View view_intensity(bag, rosbag::TopicQuery("/stereo_dense_reconstruction/image_fused"));
    rosbag::View view_depth    (bag, rosbag::TopicQuery("/stereo_dense_reconstruction/disparity"));

    if (view_intensity.size() == 0) {
        cerr << "ERROR: rosbag file '" << rosbag_path << "' doesn't contain /stereo_dense_reconstruction/image_fused." << endl;
        return traj;
    }

    rosbag::View::const_iterator it_intensity = view_intensity.begin();
    rosbag::View::const_iterator it_depth     = view_depth    .begin();

    CameraImage prev_frame, current_frame;


    unsigned int N = view_intensity.size();

    unsigned int i = 0;

    while (it_intensity != view_intensity.end() && it_depth != view_depth.end()) {

        ImageData intensities, depths;

        sensor_msgs::Image::Ptr          p_intensities = it_intensity->instantiate<sensor_msgs::Image>();
        stereo_msgs::DisparityImage::Ptr p_depths      = it_depth    ->instantiate<stereo_msgs::DisparityImage>();

        if (!p_intensities || !p_depths) {
            cerr << "failed to load data from bag file in frame " << i << endl;
            return traj;
        }

        loadImageDataFromROSgrayscale(intensities, *p_intensities);
        loadImageDataFromROSdepthmap (depths     , p_depths->image);


        prev_frame = current_frame;
        current_frame.loadFromMatrices(intensities.getData(), depths.getData());

#if 0
        frames[i].downsample2();
        intrinsics.downsample2();
#endif

        if (i > 0) {

            CameraStep step(prev_frame, current_frame, visensor_intrinsics, Transformation(0,0,0,0,0,0), i-1, i);

            traj.push_back(findTransformationWithPyramid(step, params));
        }

        printfProgress(i, 0, N);

        i++;

        if (i >= N)
            break;

        it_intensity++;
        it_depth++;
    }


    return traj;
}
///////////////////////////////////////////////////////////////////////////////
std::vector<Transformation> findTrajectoryFromRosbagRaw(const string& rosbag_path, const Warp::Parameters& params)
{
    std::vector<Transformation> traj;


    rosbag::Bag bag;
    bag.open(rosbag_path, rosbag::bagmode::Read);


    rosbag::View view_intensity(bag, rosbag::TopicQuery("/cam0/image_raw"));
    rosbag::View view_depth    (bag, rosbag::TopicQuery("/dense/image_raw"));

    if (view_intensity.size() == 0) {
        cerr << "ERROR: rosbag file '" << rosbag_path << "' doesn't contain /cam0/image_raw." << endl;
        return traj;
    }

    if (view_intensity.size() != view_depth.size()) {
        cerr << "WARNING: number of frames doesn't match:" << endl;
        cerr << "got " << view_intensity.size() << " image frames and " << view_depth.size() << " depth frames." << endl;
    }

    rosbag::View::const_iterator it_intensity = view_intensity.begin();
    rosbag::View::const_iterator it_depth     = view_depth    .begin();

    CameraImage prev_frame, current_frame;


    unsigned int N = view_depth.size();

    unsigned int i = 0;

    while (it_intensity != view_intensity.end() && it_depth != view_depth.end()) {

        ImageData intensities, depths;

        sensor_msgs::Image::Ptr p_intensities = it_intensity->instantiate<sensor_msgs::Image>();
        sensor_msgs::Image::Ptr p_depths      = it_depth    ->instantiate<sensor_msgs::Image>();

        // use intensity frame just before depth image
        while (it_intensity->getTime() < it_depth->getTime()) {

            //cout << "looking for newer image: cam0:  " << it_intensity->getTime() << endl;

            p_intensities = it_intensity->instantiate<sensor_msgs::Image>();

            it_intensity++;

            if (it_intensity == view_intensity.end()) {
                cout << "found no more camera images :(" << endl;
                return traj;
            }
        }

        if (!p_intensities || !p_depths) {
            cerr << "failed to load data from bag file in frame " << i << endl;
            return traj;
        }

        loadImageDataFromROSraw(intensities, *p_intensities);
        loadImageDataFromROSraw(depths,      *p_depths);

        prev_frame = current_frame;
        current_frame.loadFromMatrices(intensities.getData(), depths.getData());

#if 0
        frames[i].downsample2();
        intrinsics.downsample2();
#endif

        if (i > 0) {

            CameraStep step(prev_frame, current_frame, visensor_intrinsics, Transformation(0,0,0,0,0,0), i-1, i);

            traj.push_back(findTransformationWithPyramid(step, params));
        }

        printfProgress(i, 0, N);

        i++;

        if (i >= N)
            break;

        it_depth++;
    }


    return traj;
}
///////////////////////////////////////////////////////////////////////////////

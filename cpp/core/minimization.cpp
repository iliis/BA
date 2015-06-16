#include "minimization.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
// TODO: clean this up, too lazy to implement this cleanly :P
int iteration_count = 0;

//#define PRINT_DEBUG_MESSAGES

///////////////////////////////////////////////////////////////////////////////
float IterGaussNewton(const CameraStep& step, Transformation& T, const Warp::Parameters& params)
{
    ++iteration_count;

    VectorXf errors;
    Matrix<float, Dynamic, 6> J;


#ifdef PRINT_DEBUG_MESSAGES
    float total_error = Warp::calcError(step, T, errors, J, params);
    cout << "[GN] " << T << " --> err: " << total_error;
#else
    Warp::calcError(step, T, errors, J, params);
#endif

    // step = - (J'*W*J) \ J' * W * err';
    VectorXf weights = (*params.weight_function)(errors);

    Matrix<float, 6, 1> delta = - (J.transpose() * weights.asDiagonal() * J).fullPivLu().solve(
            J.transpose() * weights.asDiagonal() * errors );

#ifdef PRINT_DEBUG_MESSAGES
    cout << "  delta: " << delta.transpose() << "  norm: " << delta.norm() << endl;
#endif

    Transformation T_new;
    T_new.value = T.value + delta;
    T_new.updateRotationMatrix();

    T = T_new;

    return delta.norm();
}
///////////////////////////////////////////////////////////////////////////////
float IterGradientDescent(const CameraStep& step, Transformation& T, const Matrix<float,6,1>& stepSize, const Warp::Parameters& params)
{
    ++iteration_count;

    VectorXf errors;
    Matrix<float, Dynamic, 6> J;

#ifdef PRINT_DEBUG_MESSAGES
    float total_error = Warp::calcError(step, T, errors, J, params);
    cout << "[GD] " << T << " --> err: " << total_error;
#else
    Warp::calcError(step, T, errors, J, params);
#endif

    // step = - step_size .* (J' * (w.^2 .* err)');
    VectorXf weights = (*params.weight_function)(errors);

    // there is probably a better way than using two diagonal matrices ;) (asArray() or something)
    Matrix<float, 6, 1> delta = -stepSize.array() * (J.transpose() * weights.asDiagonal() * weights.asDiagonal() * errors).array();

#ifdef PRINT_DEBUG_MESSAGES
    cout << "  delta: " << delta.transpose() << endl;
#endif

    Transformation T_new;
    T_new.value = T.value + delta;
    T_new.updateRotationMatrix();

    T = T_new;

    return delta.norm();
}
///////////////////////////////////////////////////////////////////////////////
Transformation findTransformation(const CameraStep& step, const Warp::Parameters& params)
{
    Transformation T = params.T_init;

    float prev_delta = 0;
    float delta = 0;

    unsigned int iterations = 0;

    while (iterations++ < params.max_iterations) {
        prev_delta = delta;
        delta = IterGaussNewton(step, T, params);

        if (delta < 0.0001) // found good enough solution
            break;

        if (abs(prev_delta - delta) < 0.0001) // convergence rate is decreasing
            break;

        //cout << abs(prev_delta - delta) << " ";
    }

#ifdef PRINT_DEBUG_MESSAGES
    cout << " >>>>> found solution: " << T << endl;
#endif

    return T;
}
///////////////////////////////////////////////////////////////////////////////
Transformation findTransformationWithPyramid(const CameraStep& step, const Warp::Parameters& params)
{
    // downsample images
    CameraStep s = step;
    std::vector<CameraStep> pyramid;
    pyramid.push_back(s);

    sf::Clock clock;
    clock.restart();

    for (unsigned int i = 1; i < params.pyramid_levels; i++) {
        s.downsampleBy(1);
        pyramid.push_back(s);
    }

    iteration_count = 0;

    // actually process them
    Warp::Parameters p = params;
    for (std::vector<CameraStep>::const_reverse_iterator it = pyramid.rbegin(); it != pyramid.rend(); ++it) {
        p.T_init = findTransformation(*it, p);
    }

    sf::Time t = clock.getElapsedTime();
    cout << " >>>>> found solution in " << iteration_count << " iterations and " << ((double) t.asMicroseconds())/1000 << "ms ( " << 1000*1000/((double)t.asMicroseconds()) << " FPS)" << endl;
    cout << " >>>>> this is " << t.asMicroseconds() / ((double) iteration_count * 1000) << "ms per Iteration (on average, with " << params.pyramid_levels << " pyramid levels)" << endl;


    return p.T_init;
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

    rosbag::View::const_iterator it_intensity = view_intensity.begin();
    rosbag::View::const_iterator it_depth     = view_depth    .begin();

    CameraIntrinsics intrinsics = CameraIntrinsics(
            /* sensor size     */ Eigen::Vector2f(752, 480),
            /* principal point */ Eigen::Vector2f(370.105, 226.664),
            /* focal length    */ 471.7,
            /* stereo baseline */ 0.110174);

    CameraImage prev_frame, current_frame;


    unsigned int N = view_intensity.size()-10;

    unsigned int i = 0;
    while (it_intensity++ != view_intensity.end() && it_depth++ != view_depth.end()) {


        sensor_msgs::Image::Ptr          p_intensities = it_intensity->instantiate<sensor_msgs::Image>();
        stereo_msgs::DisparityImage::Ptr p_depths      = it_depth    ->instantiate<stereo_msgs::DisparityImage>();

        if (!p_intensities || !p_depths) {
            cerr << "failed to load data from bag file in frame " << i << endl;
            return traj;
        }

        ImageData intensities, depths;
        intensities.loadFromROSgrayscale(*p_intensities);
        depths     .loadFromROSdepthmap (p_depths->image);

        prev_frame = current_frame;
        current_frame.loadFromMatrices(intensities.getData(), depths.getData());

#if 0
        frames[i].downsample2();
        intrinsics.downsample2();
#endif

        if (i > 0) {

            CameraStep step(prev_frame, current_frame, Transformation(0,0,0,0,0,0), intrinsics);

            traj.push_back(findTransformationWithPyramid(step, params));
        }

        printfProgress(i, 0, N);

        i++;

        if (i >= N)
            break;
    }


    return traj;
}
///////////////////////////////////////////////////////////////////////////////

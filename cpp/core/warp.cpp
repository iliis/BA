#include "warp.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
std::string Warp::Parameters::toString()
{
    return "Error weighting function: " + weight_function->toString()
       + "\npyramid levels: " + boost::lexical_cast<string>(pyramid_levels)
       + "\ngradient norm threshold: " + boost::lexical_cast<string>(gradient_norm_threshold)
       + "\nmeasure gradient on image-to-be-warped: " + boost::lexical_cast<string>(this->filter_on_unwarped_gradient);
       //+ "\nmax. iterations: " + boost::lexical_cast<string>(max_iterations);
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
WorldPoint Warp::projectInv(const Pixel& pixel, const CameraIntrinsics& intrinsics)
{
    WorldPoint p;
    p.pixel = pixel;

    // project into 3D space
    if (intrinsics.isDisparityCamera()) {
        // quick hack to use disparity images
        Vector2f pxy = (pixel.pos - intrinsics.getPrincipalPoint()) * intrinsics.getBaseline() / pixel.depth;
        p.pos(0) = pxy(0);
        p.pos(1) = pxy(1);
        p.pos(2) = intrinsics.getBaseline() * intrinsics.getFocalLength() / pixel.depth;
    } else {
        Vector2f pxy = (pixel.pos - intrinsics.getPrincipalPoint()) * pixel.depth / intrinsics.getFocalLength();
        p.pos(0) = pxy(0);
        p.pos(1) = pxy(1);
        p.pos(2) = pixel.depth;
    }

    return p;
}
///////////////////////////////////////////////////////////////////////////////
Pixel Warp::project(const WorldPoint& point, const CameraIntrinsics& intrinsics)
{
    Pixel p = point.pixel;

    p.pos(0) = point.pos.x();
    p.pos(1) = point.pos.y();
    p.pos = p.pos / point.pos.z() * intrinsics.getFocalLength() + intrinsics.getPrincipalPoint();

    return p;
}
///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 2, 3> Warp::projectJacobian(const WorldPoint& point, const CameraIntrinsics& intrinsics)
{
    float x = point.pos.x();
    float y = point.pos.y();
    float z = point.pos.z();
    float F = intrinsics.getFocalLength();

    Eigen::Matrix<float, 2, 3> J;
    J << F/z,   0, -(F*x)/(z*z),
         0,   F/z, -(F*y)/(z*z);

    return J;
}
///////////////////////////////////////////////////////////////////////////////
WorldPoint Warp::transform(const WorldPoint& point, const Transformation& transformation)
{
    WorldPoint p;
    p.pixel = point.pixel;
    p.pos   = transformation(point.pos);
    return p;
}
///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 6> Warp::transformJacobian(const WorldPoint& point, const Transformation& transformation)
{
    return transformation.getJacobian(point.pos);
}
///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 1, 2> Warp::sampleJacobian(const Pixel& pixel, const CameraImage& image)
{
    return image.getIntensityData().sampleDiff(pixel.pos);
}
///////////////////////////////////////////////////////////////////////////////
float Warp::calcError(const CameraStep& step, const Transformation& T, Eigen::VectorXf& error_out, Eigen::Matrix<float, Eigen::Dynamic, 6>& J_out, const Parameters& params, WarpDebugData* debug_out)
{
    const unsigned int W = step.intrinsics.getCameraWidth();
    const unsigned int H = step.intrinsics.getCameraHeight();

    //cout << "warping with " << T << endl;

    float total_error = 0;


    if (debug_out) {
        INIT_NAN(debug_out->J_norm, W, H);
        INIT_NAN(debug_out->selection_heuristic, W, H);
        INIT_NAN(debug_out->warped_image, W, H);
        INIT_NAN(debug_out->errors_in_current, W, H);
        INIT_NAN(debug_out->weighted_errors, W, H);
    }

    //Matrix3f R = T.getRotationMatrix();
    //Vector3f M = T.getTranslation();

    // amount of successfully warped pixels
    unsigned int pixel_count = 0;

    error_out.resize(W*H);
    J_out.resize(W*H, 6);

    for (unsigned int y = 0; y < H; ++y) {
        for (unsigned int x = 0; x < W; ++x) {


            Pixel pixel_current  = step.frame_second.getPixel(Vector2i(x,y));

            // skip pixels without depth or intensity value
            if (isnan(pixel_current.depth) || isnan(pixel_current.intensity))
                continue;

            float pixel_current_gradient_norm = step.frame_second.getIntensityData().getDiff(Vector2i(x,y)).norm();

            // skip pixels with weak gradient (approximate keyframe gradient with gradient from current frame)
            if (params.filter_on_unwarped_gradient) {
                if (pixel_current_gradient_norm < params.gradient_norm_threshold)
                    continue;
            }

            WorldPoint point = projectInv(pixel_current, step.intrinsics);

            WorldPoint point_transformed = transform(point, T); // 6.2ms per point, 3.7ms with cached rotation matrix
            //point.pos = R * point.pos + M; // 3.5ms per point

            Pixel pixel_in_keyframe = project(point_transformed, step.intrinsics);

            // skip pixels that project outside the keyframe
            if (!step.frame_first.isValidPixel(pixel_in_keyframe.pos))
                continue;

            Matrix<float, 1, 2> J_I = Warp::sampleJacobian(pixel_in_keyframe, step.frame_first);

            // skip pixels that don't provide a good gradient
            if (!params.filter_on_unwarped_gradient) {
                if (J_I.norm() < params.gradient_norm_threshold)
                    continue;
            }

            float intensity_keyframe = step.frame_first.samplePixel(pixel_in_keyframe.pos);

            // skip pixels that map onto invalid color
            if (isnan(intensity_keyframe) || isnan(J_I(0)) || isnan(J_I(1)))
            	continue;

            float error = (intensity_keyframe - pixel_current.intensity);

            assert(!isnan(error));

            // calculate Jacobian
            Matrix<float, 3, 6> J_T = Warp::transformJacobian(point, T);
            Matrix<float, 2, 3> J_P = Warp::projectJacobian(point_transformed, step.intrinsics);
            J_out.row(pixel_count) = J_I * J_P * J_T;

            // store in output values
            error_out(pixel_count) = error;


            total_error += error * error;


            if (debug_out) {

                debug_out->errors_in_current(y,x) = abs(error);

                Vector2i p(pixel_in_keyframe.pos.x()+0.5,pixel_in_keyframe.pos.y()+0.5);
                debug_out->warped_image(p.y(), p.x()) = pixel_current.intensity;

                debug_out->J_norm(y,x) = J_out.row(pixel_count).norm();

                if (params.filter_on_unwarped_gradient) {
                    debug_out->selection_heuristic(y,x) = step.frame_second.getIntensityData().getDiff(Vector2i(x,y)).norm();
                } else {
                    debug_out->selection_heuristic(y,x) = J_I.norm();
                }

                debug_out->weighted_errors(y,x) = (*params.weight_function)(error) * error;
            }

            ++pixel_count;
        }
    }

    // catch the border case where pixels are warped so that *none* actually falls on to the other image
    if (pixel_count == 0) {
        error_out.resize(1);
        J_out.resize(1, Eigen::NoChange);

        error_out.setZero();
        J_out.setZero();
    } else {
        error_out.conservativeResize(pixel_count);
        J_out.conservativeResize(pixel_count, Eigen::NoChange);
    }

    //cout << "total error: " << total_error << "  =  " << sqrt(total_error) << endl;

    return sqrt(total_error);
}
///////////////////////////////////////////////////////////////////////////////
void Warp::renderErrorSurface(MatrixXf& target_out, Matrix<float,Dynamic,6>& gradients_out, const CameraStep& step, const Transformation& Tcenter, const PlotRange& range1, const PlotRange& range2, const Warp::Parameters& params)
{
    assert(range1.dim <  6);
    assert(range2.dim <  6);
    assert(range1.steps > 0);
    assert(range2.steps > 0);

    target_out.resize(range1.steps, range2.steps);
    gradients_out.resize(range1.steps * range2.steps, 6);

    for (unsigned int y = 0; y < range2.steps; ++y) {
        for (unsigned int x = 0; x < range1.steps; ++x) {

            Transformation T = Tcenter;

            float xv = ((float) x / (range1.steps-1));
            float yv = ((float) y / (range2.steps-1));
            T.value(range1.dim) = Tcenter.value(range1.dim) + xv * (range1.to-range1.from) + range1.from;
            T.value(range2.dim) = Tcenter.value(range2.dim) + yv * (range2.to-range2.from) + range2.from;
            T.updateRotationMatrix();

            Matrix<float, Eigen::Dynamic, 6> J;
            VectorXf errs;

            float error = calcError(step, T, errs, J, params);

            //Matrix<float, 1, 6> gradient = J.transpose() * errs;
            gradients_out.row(y*range1.steps +x) = J.transpose() * errs;

            //for (int i = 0; i < 6; i++)
                //gradients_out(y * range1.steps + x, i) = gradient(i);

            target_out(y, x) = error;

            //cout << x << " " << y << "  --> " << xv << " " << yv << endl;
            //cout << T << "  -->  " << error << endl;
        }

        printfProgress(y,0,range2.steps-1);
    }
}
///////////////////////////////////////////////////////////////////////////////
void Warp::PlotRange::readFromStdin()
{
    do {
        cout << "dimension [0-5]? ";
        cin >> this->dim;
    } while (this->dim > 5);

    cout << "from? ";
    cin >> this->from;

    do {
        cout << "to (>from)? ";
        cin >> this->to;
    } while (to <= from);

    do {
        cout << "steps? ";
        cin >> this->steps;
    } while (steps < 1);
}
///////////////////////////////////////////////////////////////////////////////

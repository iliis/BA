#include "warp.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
std::string Warp::Parameters::toString()
{
    return "Error weighting function: " + weight_function->toString()
       + "\npyramid levels: " + boost::lexical_cast<string>(pyramid_levels)
       + "\ngradient norm threshold: " + boost::lexical_cast<string>(gradient_norm_threshold);
       //+ "\nmax. iterations: " + boost::lexical_cast<string>(max_iterations);
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
WorldPoint Warp::projectInv(const Pixel& pixel, const CameraIntrinsics& intrinsics)
{
    WorldPoint p;
    p.pixel = pixel;

#if 0
    // project into 3D space
    Vector2f pxy = (pixel.pos - intrinsics.getPrincipalPoint()) * pixel.depth / intrinsics.getFocalLength();
    p.pos(0) = pxy(0);
    p.pos(1) = pxy(1);
    p.pos(2) = pixel.depth;
#else
    // TODO: implement this the correct way :P
    // quick hack to use disparity images
    Vector2f pxy = (pixel.pos - intrinsics.getPrincipalPoint()) * 0.011 / pixel.depth;
    p.pos(0) = pxy(0);
    p.pos(1) = pxy(1);
    p.pos(2) = 0.011 * intrinsics.getFocalLength() / pixel.depth;
#endif

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
float Warp::calcError(const CameraStep& step, const Transformation& T, Eigen::VectorXf& error_out, Eigen::Matrix<float, Eigen::Dynamic, 6>& J_out, const Parameters& params, sf::RenderTarget* plotTarget, sf::Font* font)
{
    const unsigned int W = step.intrinsics.getCameraWidth();
    const unsigned int H = step.intrinsics.getCameraHeight();

    //cout << "warping with " << T << endl;

    float total_error = 0;

    sf::Image img_c, img_k;
    Eigen::MatrixXf img_errs_weighted, img_J_norm, img_selection_heuristic;
    if (plotTarget) {
        img_c.create(W,H, sf::Color(0,0,255));
        img_k.create(W,H, sf::Color(0,0,255));
        img_J_norm.resize(H,W);
        img_J_norm.setZero();
        img_selection_heuristic.resize(H,W);
        img_selection_heuristic.setZero();

        img_errs_weighted.resize(H,W);
        img_errs_weighted.setZero();
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

            // skip pixels without depth value
            if (isnan(pixel_current.depth))
                continue;

            WorldPoint point = projectInv(pixel_current, step.intrinsics);

            WorldPoint point_transformed = transform(point, T); // 6.2ms per point, 3.7ms with cached rotation matrix
            //point.pos = R * point.pos + M; // 3.5ms per point

            Pixel pixel_in_keyframe = project(point_transformed, step.intrinsics);

            // skip pixels that project outside the keyframe
            if (!step.frame_first.isValidPixel(pixel_in_keyframe.pos))
                continue;

            Matrix<float, 1, 2> J_I = Warp::sampleJacobian(pixel_in_keyframe, step.frame_first);

            // skip pixels that don't provide a good gradient
            if (J_I.norm() < params.gradient_norm_threshold)
                continue;

            float intensity_keyframe = step.frame_first.samplePixel(pixel_in_keyframe.pos);

            float error = (intensity_keyframe - pixel_current.intensity);

            // calculate Jacobian
            Matrix<float, 3, 6> J_T = Warp::transformJacobian(point, T);
            Matrix<float, 2, 3> J_P = Warp::projectJacobian(point_transformed, step.intrinsics);
            J_out.row(pixel_count) = J_I * J_P * J_T;

            // store in output values
            error_out(pixel_count) = error;


            total_error += error * error;


            if (plotTarget) {
                //error = error * error;
                error = abs(error);
                //img_c.setPixel(x,y, sf::Color(error*255, (1-error)*255, 0));
                Colormap::Jet m;
                img_c.setPixel(x,y, m(error, 0, 1));
                //img_k.setPixel(pixel_in_keyframe.pos.x(),pixel_in_keyframe.pos.y(), sf::Color(error*255, (1-error)*255, 0));
                img_k.setPixel(pixel_in_keyframe.pos.x()+0.5,pixel_in_keyframe.pos.y()+0.5, sf::Color(255*pixel_current.intensity, 255*pixel_current.intensity, 255*pixel_current.intensity));

                img_J_norm(y,x) = J_out.row(pixel_count).norm();

                img_selection_heuristic(y,x) = J_I.norm();

                img_errs_weighted(y,x) = (*params.weight_function)(error) * error;
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


    if (plotTarget && font) {
        drawImageAt(img_c, sf::Vector2f(0,0),   *plotTarget, "errors in current", font);
        drawImageAt(img_k, sf::Vector2f(0,H+2), *plotTarget, "warped current frame", font);
        step.frame_second.getIntensityData().drawAt( *plotTarget, sf::Vector2f(W+2,0));
        step.frame_first .getIntensityData().drawAt( *plotTarget, sf::Vector2f(W+2,H+2));
        //step.frame_second.getDepthData().drawAt( *plotTarget, sf::Vector2f(0,0));
        //step.frame_first .getDepthData().drawAt( *plotTarget, sf::Vector2f(0,H+2));


        drawMatrixAt(img_errs_weighted,       sf::Vector2f(2*W+4, 0), *plotTarget, Colormap::Jet(), "weighted errors", font);
        drawMatrixAt(img_J_norm,              sf::Vector2f(2*W+4, H+2),   *plotTarget, Colormap::Jet(), "norm(J) [max: " + boost::lexical_cast<string>(img_J_norm.maxCoeff()) + "]", font);
        drawMatrixAt(img_selection_heuristic, sf::Vector2f(2*W+4, 2*H+4), *plotTarget, Colormap::Jet(), "image gradient [max: " + boost::lexical_cast<string>(img_selection_heuristic.maxCoeff()) + "]", font);


        sf::Text t;
        t.setFont(*font);
        t.setCharacterSize(12);
        t.setString("current frame"); t.setPosition(W+4,H-t.getCharacterSize()-2);     plotTarget->draw(t);
        t.setString("keyframe");      t.setPosition(W+4,H-t.getCharacterSize()-2+H+2); plotTarget->draw(t);

        string s = "total error: " + boost::lexical_cast<string>(sqrt(total_error))
                //+ "\nabs min: " + boost::lexical_cast<string>(error_out.array().abs().minCoeff())
                + "\nabs max: " + boost::lexical_cast<string>(error_out.array().abs().maxCoeff());
        t.setString(s); t.setPosition(2,H+H+4); plotTarget->draw(t);
        t.setString("step "+boost::lexical_cast<string>(step.index));      t.setPosition(W+4,H+H+4); plotTarget->draw(t);

        ostringstream str_T_current; str_T_current << T;
        t.setString("current: "+str_T_current.str());      t.setPosition(2,H+H+8+2*t.getCharacterSize()); plotTarget->draw(t);
        ostringstream str_T_gt; str_T_gt << step.ground_truth;
        t.setString("truth:   "+str_T_gt.str());      t.setPosition(2,H+H+8+3*t.getCharacterSize()); plotTarget->draw(t);
        string r = boost::lexical_cast<string>((T.value-step.ground_truth.value).norm());
        t.setString("T error: "+r);      t.setPosition(2,H+H+10+4*t.getCharacterSize()); plotTarget->draw(t);
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

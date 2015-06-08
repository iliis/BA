#include "warp.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
WorldPoint Warp::projectInv(const Pixel& pixel, const CameraIntrinsics& intrinsics)
{
    WorldPoint p;
    p.pixel = pixel;

    // project into 3D space
    Vector2f pxy = (pixel.pos - intrinsics.getPrincipalPoint()) * pixel.depth / intrinsics.getFocalLength();
    p.pos(0) = pxy(0);
    p.pos(1) = pxy(1);
    p.pos(2) = pixel.depth;

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
WorldPoint Warp::transform(const WorldPoint& point, const Transformation& transformation)
{
    WorldPoint p;
    p.pixel = point.pixel;
    p.pos   = transformation(point.pos);
    return p;
}
///////////////////////////////////////////////////////////////////////////////
float Warp::calcError(const CameraStep& step, const Transformation& T)
{
    const unsigned int W = step.scene->getIntrinsics().getCameraWidth();
    const unsigned int H = step.scene->getIntrinsics().getCameraHeight();

    //cout << "warping with " << T << endl;

    float total_error = 0;

    /*
    sf::Image img_c, img_k;
    img_c.create(W,H, sf::Color(0,0,255));
    img_k.create(W,H, sf::Color(0,0,255));
    */

    //Matrix3f R = T.getRotationMatrix();
    //Vector3f M = T.getTranslation();

    for (unsigned int y = 0; y < H; ++y) {
        for (unsigned int x = 0; x < W; ++x) {
            Pixel pixel_current  = step.frame_second.getPixel(Vector2i(x,y));

            WorldPoint point = projectInv(pixel_current, step.scene->getIntrinsics());

            point = transform(point, T); // 6.2ms per point, 3.7ms with cached rotation matrix
            //point.pos = R * point.pos + M; // 3.5ms per point

            Pixel pixel_in_keyframe = project(point, step.scene->getIntrinsics());

            // skip pixels that project outside the keyframe
            if (!step.frame_first.isValidPixel(pixel_in_keyframe.pos))
                continue;

            float intensity_keyframe = step.frame_first.samplePixel(pixel_in_keyframe.pos);

            float error = (intensity_keyframe - pixel_current.intensity);
            error = error * error; // squared differences

            total_error += error;

            //img_c.setPixel(x,y, sf::Color(error*255, (1-error)*255, 0));
            //img_k.setPixel(pixel_in_keyframe.pos.x(),pixel_in_keyframe.pos.y(), sf::Color(error*255, (1-error)*255, 0));
        }
    }

    //drawImageAt(img_c, sf::Vector2f(0,0), target);
    //drawImageAt(img_k, sf::Vector2f(0,H+2), target);

    //cout << "total error: " << total_error << "  =  " << sqrt(total_error) << endl;

    return sqrt(total_error);
}
///////////////////////////////////////////////////////////////////////////////
void Warp::renderErrorSurface(ImageData& target, const CameraStep& step, const Transformation& Tcenter, const PlotRange& range1, const PlotRange& range2)
{
    assert(range1.dim >= 0); assert(range1.dim <  6);
    assert(range2.dim >= 0); assert(range2.dim <  6);
    assert(range1.steps > 0);
    assert(range2.steps > 0);

    target.create(range1.steps, range2.steps);

    for (unsigned int y = 0; y < range2.steps; ++y) {
        for (unsigned int x = 0; x < range1.steps; ++x) {

            Transformation T = Tcenter;

            float xv = ((float) x / (range1.steps-1));
            float yv = ((float) y / (range2.steps-1));
            T.value(range1.dim) = Tcenter.value(range1.dim) + xv * (range1.to-range1.from) + range1.from;
            T.value(range2.dim) = Tcenter.value(range2.dim) + yv * (range2.to-range2.from) + range2.from;
            T.updateRotationMatrix();

            float error = calcError(step, T);

            target(x, y) = error;

            //cout << x << " " << y << "  --> " << xv << " " << yv << endl;
            //cout << T << "  -->  " << error << endl;
        }
    }

    target.updateImageFromMatrix();
}
///////////////////////////////////////////////////////////////////////////////

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
void Warp::drawError(sf::RenderTarget& target, const CameraStep& step, const Transformation& T)
{
    const unsigned int W = step.scene->getIntrinsics().getCameraWidth();
    const unsigned int H = step.scene->getIntrinsics().getCameraHeight();

    //cout << "warping with " << T << endl;

    float total_error = 0;

    for (unsigned int y = 0; y < H; ++y) {
        for (unsigned int x = 0; x < W; ++x) {
            Pixel pixel_current  = step.frame_second.getPixel(Vector2i(x,y));

            // TODO: use CameraIntrinsics here!!! (and do this upon loading the image)
            pixel_current.depth = pixel_current.depth * (100-0.1) + 0.1;

            WorldPoint point = projectInv(pixel_current, step.scene->getIntrinsics());

            point = transform(point, T);

            Pixel pixel_in_keyframe = project(point, step.scene->getIntrinsics());

            // skip pixels that project outside the keyframe
            if (!step.frame_first.isValidPixel(pixel_in_keyframe.pos))
                continue;

            float intensity_keyframe = step.frame_first.samplePixel(pixel_in_keyframe.pos);

            float error = (intensity_keyframe - pixel_current.intensity);
            error = error * error; // squared differences

            total_error += error;

            sf::RectangleShape r;
            r.setSize(sf::Vector2f(1,1));
            r.setPosition(x, y);
            r.setFillColor(sf::Color(error*255, error*255, error*255));
            //r.setFillColor(sf::Color(255,0,0));
            target.draw(r);

            r.setPosition(pixel_in_keyframe.pos.x(), pixel_in_keyframe.pos.y() + H);
            target.draw(r);

            //cout << x << " " << y << "  error: " << error << "  warped to: " << pixel_in_keyframe.pos << endl;
        }
    }

    cout << "total error: " << total_error << "  =  " << sqrt(total_error) << endl;
}
///////////////////////////////////////////////////////////////////////////////

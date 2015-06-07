#include "camera_image.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
void CameraImage::loadFromSceneDirectory(const std::string& scene_path, const unsigned int index)
{
    //std::string index_string = "000001";
    ostringstream index_string;
    index_string.width(4);
    index_string.fill('0');
    index_string << (index + 1); // filenames start with 1

    intensities.loadFromFile(scene_path + "/color" + index_string.str() + ".png");
    depths     .loadFromFile(scene_path + "/depth" + index_string.str() + ".png");
}
///////////////////////////////////////////////////////////////////////////////
void CameraImage::loadFromMatrices(const Eigen::MatrixXf& mat_intensities, const Eigen::MatrixXf& mat_depths)
{
    intensities.loadFromMatrix(mat_intensities);
    depths     .loadFromMatrix(mat_depths);
}
///////////////////////////////////////////////////////////////////////////////
Pixel CameraImage::getPixel(Eigen::Vector2i pos) const
{
    assert(pos.x() >= 0);
    assert(pos.y() >= 0);
    assert(pos.x() < this->getWidth());
    assert(pos.y() < this->getHeight());

    Pixel p;
    p.pos(0) = pos.x();
    p.pos(1) = pos.y();
    p.intensity = intensities.getValue(pos);
    p.depth     = depths.getValue(pos);

    return p;
}
///////////////////////////////////////////////////////////////////////////////
float CameraImage::samplePixel(Eigen::Vector2f pos) const
{
    assert(isValidPixel(pos));
    return intensities.sampleValue(pos);
}
///////////////////////////////////////////////////////////////////////////////
bool CameraImage::isValidPixel(Eigen::Vector2f pos) const
{
    // center of top-left pixel is at [0,0]
    // center of bottom-right pixel is at [W-1, H-1]
    // therefore, both these coordinates are valid
    return pos.x() >= 0
        && pos.x() <= getWidth() - 1
        && pos.y() >= 0
        && pos.y() <= getHeight() - 1;
}
///////////////////////////////////////////////////////////////////////////////
void CameraImage::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    intensities.draw_at(target, sf::Vector2f(0,0), states);
    depths     .draw_at(target, sf::Vector2f(0,intensities.getHeight()), states);
}
///////////////////////////////////////////////////////////////////////////////

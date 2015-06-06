#include "scene.h"

///////////////////////////////////////////////////////////////////////////////
void SceneImage::loadFromSceneDirectory(const std::string& scene_path, const unsigned int index)
{
    //std::string index_string = "000001";
    std::ostringstream index_string;
    index_string.width(4);
    index_string.fill('0');
    index_string << index;

    intensities.loadFromFile(scene_path + "/color" + index_string.str() + ".png");
    depths     .loadFromFile(scene_path + "/depth" + index_string.str() + ".png");
}
///////////////////////////////////////////////////////////////////////////////
void SceneImage::loadFromMatrices(const Eigen::MatrixXf& mat_intensities, const Eigen::MatrixXf& mat_depths)
{
    intensities.loadFromMatrix(mat_intensities);
    depths     .loadFromMatrix(mat_depths);
}
///////////////////////////////////////////////////////////////////////////////
void SceneImage::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    intensities.draw_at(target, sf::Vector2f(0,0), states);
    depths     .draw_at(target, sf::Vector2f(0,intensities.getHeight()), states);
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SceneStep Scene::getStep(unsigned int index)
{
    assert(index < this->getStepCount());

    SceneStep step;
    step.scene = this;
    step.index = index;

    step.frame_first  = &frames[index];
    step.frame_second = &frames[index+1];

    return step;
}
///////////////////////////////////////////////////////////////////////////////

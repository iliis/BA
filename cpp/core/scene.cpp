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
CameraStep Scene::getStep(unsigned int index) const
{
    assert(index < this->getStepCount());

    return CameraStep(frames[index], frames[index+1], ground_truth[index+1], index, this);
}
///////////////////////////////////////////////////////////////////////////////
const CameraImage& Scene::getFrame(unsigned int index) const
{
    assert(index < this->getFrameCount());

    return frames[index];
}
///////////////////////////////////////////////////////////////////////////////

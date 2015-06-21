#ifndef STATE_TRACKER_H_INCLUDED
#define STATE_TRACKER_H_INCLUDED

#include <list>
#include <boost/foreach.hpp>
#include "../core/transformation.h"

// Eigen needs one of those: :(
#include <GL/glew.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/glext.h>
#include <unsupported/Eigen/OpenGLSupport>

#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>

class CameraState : public sf::Drawable
{
public:
    CameraState();

    void apply(const Transformation& T);

    sf::RectangleShape shape;

    float scale;

    void reset();

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;

    Eigen::Vector3f position;
    Eigen::Matrix3f orientation;

    std::list<Eigen::Vector3f> trajectory;
};

#endif /* end of include guard: STATE_TRACKER_H_INCLUDED */

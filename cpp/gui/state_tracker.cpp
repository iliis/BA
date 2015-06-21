#include "state_tracker.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
CameraState::CameraState()
  : scale(1000),
    position(0,0,0),
    orientation(Eigen::Matrix3f::Identity())
{
    shape.setSize(sf::Vector2f(400,100));
    shape.setFillColor(sf::Color(0,0,0));
    shape.setOutlineColor(sf::Color(100,100,100));
    shape.setOutlineThickness(1);

    trajectory.push_back(position);
}
///////////////////////////////////////////////////////////////////////////////
void CameraState::apply(const Transformation& T)
{
    position = position + T.value.head<3>();
    trajectory.push_back(position);

    orientation = T.getRotationMatrix() * orientation;
}
///////////////////////////////////////////////////////////////////////////////
void CameraState::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    target.draw(shape, states);

    target.pushGLStates();

    glEnable(GL_LINE_SMOOTH);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0f, target.getSize().x, target.getSize().y, 0.0f, -1000.0f, 1000.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(shape.getPosition().x + shape.getSize().x/4, shape.getPosition().y + shape.getSize().y/2, 0.0f);
    glScalef(scale,scale,scale);

    // draw previous positions relative to current
    ///////////////////////////////////////////////////////////////////////////

    glBegin(GL_LINE_STRIP);
        glColor3f(1,0.5,0);
        BOOST_FOREACH(const Eigen::Vector3f& v, trajectory) {
            glVertex(v - position);
        }
    glEnd();

    // draw camera
    ///////////////////////////////////////////////////////////////////////////

    // calculate model matrix from rotation matrix
    glLoadIdentity();

    // move to center of left subwindow
    glTranslatef(shape.getPosition().x + shape.getSize().x/4, shape.getPosition().y + shape.getSize().y/2, 0.0f);

    Matrix4f m = Matrix4f::Identity();
    m.block<3,3>(0,0) = orientation;
    glMultMatrix(m);


    glScalef(40,40,40);
    //glScalef(10,10,10);

    drawCamera();


    // draw absolute trajectory from top view
    ///////////////////////////////////////////////////////////////////////////

    glLoadIdentity();

    // move to center of right subwindow
    glTranslatef(shape.getPosition().x + shape.getSize().x/4*3, shape.getPosition().y + shape.getSize().y/2, 0.0f);

    // view everything from top
    glRotatef(90,1,0,0);

    glBegin(GL_LINE_STRIP);
        glColor3f(1,0.5,0);
        BOOST_FOREACH(const Eigen::Vector3f& v, trajectory) {
            glVertex(v*scale);
        }
    glEnd();


    glTranslate(position*scale);
    glMultMatrix(m);
    glScalef(10,10,10);

    drawCamera();


    target.popGLStates();
}
///////////////////////////////////////////////////////////////////////////////
void CameraState::drawCamera() const
{
    float w = 2, h = 1.5, d = 2; // camera size
    Vector3f c(0,0,0), r1(-w/2,-h/2,d), r2(w/2,-h/2,d), r3(w/2,h/2,d), r4(-w/2,h/2,d);
    Vector3f t1(r1*0.7+r2*0.3), t2(r1*0.3+r2*0.7), t3(r1/2+r2/2+Vector3f(0,-0.3,0));

    // camera 'sensor'
    glBegin(GL_LINE_LOOP);
        glColor3f(1,1,1);
        glLineWidth(2);
        glVertex(r1);
        glVertex(r2);
        glVertex(r3);
        glVertex(r4);
    glEnd();

    // camera up arrow
    glBegin(GL_TRIANGLES);
        glVertex(t1);
        glVertex(t2);
        glVertex(t3);
    glEnd();

    // lines from center to 'sensor'
    glBegin(GL_LINES);
        glColor3f(0.5,0.5,0.5);
        glLineWidth(1);
        glVertex(c); glVertex(r1);
        glVertex(c); glVertex(r2);
        glVertex(c); glVertex(r3);
        glVertex(c); glVertex(r4);
    glEnd();
}
///////////////////////////////////////////////////////////////////////////////
void CameraState::reset()
{
    position = Vector3f(0,0,0);
    orientation = Matrix3f::Identity();

    trajectory.clear();
    trajectory.push_back(position);
}
///////////////////////////////////////////////////////////////////////////////

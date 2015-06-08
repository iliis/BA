#include <stdlib.h>
#include <iostream>
#include <SFML/Graphics.hpp>

#include "core/scene.h"
#include "core/image_data.h"
#include "core/transformation.h"
#include "core/camera_intrinsics.h"
#include "core/warp.h"

using namespace std;
using namespace Eigen;


inline sf::Vector2f toSF(Eigen::Vector2f v)
{
    return sf::Vector2f(v.x(), v.y());
}

inline Eigen::Vector2f toEigen(sf::Vector2f v)
{
    return Eigen::Vector2f(v.x, v.y);
}

void drawArrow(sf::RenderTarget& target, float x, float y, float vect_x, float vect_y)
{
    sf::Vector2f pos(x,y);
    sf::Vector2f vect(vect_x, vect_y);
    Eigen::Vector2f v = toEigen(vect).normalized() * 4;

    sf::Vertex line[] = {
        // arrow base
        sf::Vertex(pos),
        sf::Vertex(pos + vect),
        // arrow head
        sf::Vertex(pos + vect + toSF(Eigen::Rotation2Df( M_PI*0.85) * v)),
        sf::Vertex(pos + vect),
        sf::Vertex(pos + vect + toSF(Eigen::Rotation2Df(-M_PI*0.85) * v)),
        sf::Vertex(pos + vect),
    };

    for(unsigned int i = 0; i < sizeof(line)/sizeof(sf::Vertex); i++)
        line[i].color = sf::Color(0,0,255);

    target.draw(line, sizeof(line)/sizeof(sf::Vertex), sf::Lines);
}

int main()
{
    sf::RenderWindow window(sf::VideoMode(1280, 768), "SFML test");

    Scene testscene;
    testscene.loadFromSceneDirectory("../matlab/input/trajectory1");

    CameraStep teststep = testscene.getStep(24);

    VectorXf errors;
    Matrix<float, Dynamic, 6> jacobian;

    cout << teststep.ground_truth << endl;
    cout << "total error: " << Warp::calcError(teststep, teststep.ground_truth, errors, jacobian) << endl;


    Warp::PlotRange range1(0, -1, 1,40);
    Warp::PlotRange range2(1, -1, 1,40);
    //Warp::PlotRange range2(4,-.1,.1,40); // beta = yaw

    Eigen::MatrixXf errorsurface(range1.steps, range2.steps);
    Eigen::Matrix<float, Dynamic, 6> errorgradients(range1.steps*range2.steps, 6);

    cout << "rendering error surface ..." << endl;

    sf::Clock clock;
    clock.restart();
    Warp::renderErrorSurface(errorsurface, errorgradients, teststep, teststep.ground_truth, range1, range2);
    sf::Int32 ms = clock.getElapsedTime().asMilliseconds();

    cout << "rendered surface in " << ms << "ms (" << ((float) ms) / (errorsurface.rows() * errorsurface.cols()) << "ms per point)" << endl;

    ImageData errorplot;
    errorplot.loadFromMatrix(errorsurface, Colormap::Hot());


    Eigen::VectorXf error_tmp;
    Eigen::Matrix<float, Eigen::Dynamic, 6> J_tmp;

    unsigned int i = 0;
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
        //window.draw(errorplot);
        errorplot.drawAt(window, sf::Vector2f(0,0), 16);

#if 0
        // plot gradients (they are clearly wrong at the moment!)
        for (unsigned int y = 0; y < errorplot.getHeight(); ++y) {
            for (unsigned int x = 0; x < errorplot.getWidth(); ++x) {
                unsigned int idx = y*errorplot.getWidth()+x;
                float factor = 0.1;
                drawArrow(window, x*16+8, y*16+8, errorgradients(idx, range1.dim)*factor, errorgradients(idx, range2.dim)*factor);
            }
        }
#endif

        CameraStep step = testscene.getStep(i);
        Warp::calcError(step, step.ground_truth, error_tmp, J_tmp, &window);

        i++;
        if (i >= testscene.getStepCount())
            i = 0;

        window.display();

        sf::sleep(sf::milliseconds(100));
    }

    return 0;
}

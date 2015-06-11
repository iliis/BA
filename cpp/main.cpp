#include <stdlib.h>
#include <iostream>
#include <SFML/Graphics.hpp>

#include "core/scene.h"
#include "core/image_data.h"
#include "core/transformation.h"
#include "core/camera_intrinsics.h"
#include "core/warp.h"
#include "core/minimization.h"

using namespace std;
using namespace Eigen;

sf::Font font;
//sf::RenderWindow window(sf::VideoMode(1280, 768), "SFML test");
sf::RenderWindow window(sf::VideoMode(256*2+2, 128*2+20), "SFML test");

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
///////////////////////////////////////////////////////////////////////////////
void test_warping(const Scene& scene)
{
    Eigen::VectorXf error_tmp;
    Eigen::Matrix<float, Eigen::Dynamic, 6> J_tmp;

    int i = 0;
    bool paused = false;
    bool show_keyframe_over_errors = false;
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::KeyPressed) {
                switch (event.key.code) {
                    // pause/play all frames
                    case sf::Keyboard::Space:
                        paused = !paused;
                        break;

                    // manually step trough frames
                    case sf::Keyboard::Right:
                        i++;
                        break;

                    case sf::Keyboard::Left:
                        i--;
                        break;

                    // overlay warped current image with keyframe
                    case sf::Keyboard::K:
                        show_keyframe_over_errors = !show_keyframe_over_errors;
                        break;

                    default:
                        break;
                }
            }
        }

        window.clear();

        if (!paused)
            i++;

        if (i < 0)
            i = scene.getStepCount()-1;

        if (i >= (int) scene.getStepCount())
            i = 0;

        CameraStep step = scene.getStep(i);
        Warp::calcError(step, step.ground_truth, error_tmp, J_tmp, &window, &font);

        if (show_keyframe_over_errors)
            step.frame_first.getIntensityData().drawAt(window, sf::Vector2f(0,step.frame_first.getHeight()+2));

        window.display();

        sf::sleep(sf::milliseconds(100));
    }
}
///////////////////////////////////////////////////////////////////////////////
void draw_error_surface(const CameraStep& step)
{
    Warp::PlotRange range1(0, -1,1, 3);
    Warp::PlotRange range2(1, -1,1, 3);
    //Warp::PlotRange range2(4,-.5,.5,40); // beta = yaw





    Eigen::MatrixXf errorsurface(range1.steps, range2.steps);
    Eigen::Matrix<float, Dynamic, 6> errorgradients(range1.steps*range2.steps, 6);

    cout << "rendering error surface ..." << endl;

    sf::Clock clock;
    clock.restart();
    Warp::renderErrorSurface(errorsurface, errorgradients, step, step.ground_truth, range1, range2);
    sf::Int32 ms = clock.getElapsedTime().asMilliseconds();

    cout << "rendered surface in " << ms << "ms (" << ((float) ms) / (errorsurface.rows() * errorsurface.cols()) << "ms per point)" << endl;

    ImageData errorplot;
    errorplot.loadFromMatrix(errorsurface, Colormap::Hot());
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
        errorplot.drawAt(window, sf::Vector2f(0,0), 16);

        for (unsigned int y = 0; y < errorplot.getHeight(); ++y) {
            for (unsigned int x = 0; x < errorplot.getWidth(); ++x) {
                unsigned int idx = y*errorplot.getWidth()+x;
                float factor = -0.004; // point *down*
                drawArrow(window, x*16+8, y*16+8, errorgradients(idx, range1.dim)*factor, errorgradients(idx, range2.dim)*factor);
            }
        }


        window.display();

        sf::sleep(sf::milliseconds(100));
    }
}
///////////////////////////////////////////////////////////////////////////////
void run_minimization(const CameraStep& step, const Transformation& T_init)
{
    Transformation T = T_init;

    cout << step.ground_truth << " <<-- ground truth (The Solution)" << endl;

    Eigen::Matrix<float,6,1> step_size;
    step_size << 1, 1, 1, 0.05, 0.05, 0.05;
    step_size /= 30000;

    Eigen::VectorXf error_tmp;
    Eigen::Matrix<float, Eigen::Dynamic, 6> J_tmp;

    int iter = 0;
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Space)
                    return;
            }
        }

        window.clear();

#if 1
        IterGaussNewton(step, T);
#else
        IterGradientDescent(step, T, step_size);
#endif
        iter++;

        Warp::calcError(step, T, error_tmp, J_tmp, &window, &font);

        window.display();

#if 0
        // record animation (save all frames to disk)
        sf::Image i = window.capture();

        ostringstream index_string;
        index_string.width(6);
        index_string.fill('0');
        index_string << iter;
        i.saveToFile("../recordings/rec_" + index_string.str() + ".png");
#endif

        //sf::sleep(sf::seconds(0.01));
    }

}
///////////////////////////////////////////////////////////////////////////////
int main()
{
    font.loadFromFile("resources/fonts/default.otf");

    Scene testscene;
    //testscene.loadFromSceneDirectory("../matlab/input/test_wide");
    //testscene.loadFromSceneDirectory("../matlab/input/trajectory1");
    //testscene.loadFromSceneDirectory("../matlab/input/testscene1");
    testscene.loadFromSceneDirectory("../matlab/input/courtyard/lux");
    //testscene.loadFromSceneDirectory("../matlab/input/courtyard");

    //test_warping(testscene);
    //draw_error_surface(testscene.getStep(0));

#if 1
    CameraStep teststep = testscene.getStep(5);
    run_minimization(teststep, Transformation(0.1,-0.1,0,0.1,0.1,0));
#else

    unsigned int i = 0;
    while (window.isOpen()) {
        CameraStep teststep = testscene.getStep(i);

        run_minimization(teststep, Transformation(0,0,0,0,0,0));
        //run_minimization(teststep, teststep.ground_truth + Transformation(0.4,0,0.2,0.05,0,0));
        //run_minimization(teststep, teststep.ground_truth + Transformation(0.4,0,0.2,0.02,-0.02,0));

        i++;
        if (i >= testscene.getStepCount())
            i = 0;
    }
#endif

    return 0;
}
///////////////////////////////////////////////////////////////////////////////

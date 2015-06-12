#include <stdlib.h>
#include <iostream>
#include <fstream>
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
//sf::RenderWindow window(sf::VideoMode(1280, 768), "dense odometry");
//sf::RenderWindow window(sf::VideoMode(256*4+2, 128*5), "dense odometry");
sf::RenderWindow window(sf::VideoMode(256*2+2, 128*3), "dense odometry");

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
void write_trajectory(const Scene& scene)
{
    std::vector<Transformation> traj = findTrajectory(scene);

    string path = scene.getSourceDirectory() + "/measured_trajectory.csv";
    ofstream outfile(path.c_str());

    outfile << "x, y, z, alpha, beta, gamma" << endl;

    for(size_t i = 0; i < traj.size(); i++) {
        traj[i].printCSV(outfile) << endl;
    }

    outfile.close();
}
///////////////////////////////////////////////////////////////////////////////
bool min_paused = true; // start in paused state, global to keep value :P
void run_minimization(const Scene& scene)
{
    unsigned int index = 0;
    CameraStep step = scene.getStep(index);

//    while (window.isOpen()) {

 //       index++;
  //      if (index >= testscene.getStepCount())
   //         index = 0;

    Transformation T(0,0,0,0,0,0);

    Eigen::Matrix<float,6,1> step_size;
    step_size << 1, 1, 1, 0.05, 0.05, 0.05;
    step_size /= 30000;

    Eigen::VectorXf error_tmp;
    Eigen::Matrix<float, Eigen::Dynamic, 6> J_tmp;

    int iter = 0;
    bool show_keyframe = false;
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::KeyPressed) {

                switch (event.key.code) {
                    // go to prev frame
                    case sf::Keyboard::B:
                        if (index == 0)
                            index = scene.getStepCount();
                        index--;
                        step = scene.getStep(index);
                        T = Transformation(0,0,0,0,0,0);
                        break;

                    // go to next frame
                    case sf::Keyboard::N:
                        index++;
                        if (index > scene.getStepCount())
                            index = 0;
                        step = scene.getStep(index);
                        T = Transformation(0,0,0,0,0,0);
                        break;

                    // distrub
                    case sf::Keyboard::D:
                        T.value = T.value.array() + T.value.Random().array() * Transformation(0.2,0.2,0.2,0.002,0.002,0.002).value.array();
                        T.updateRotationMatrix();
                        break;

                    // reset
                    case sf::Keyboard::R:
                        T = Transformation(0,0,0,0,0,0);
                        break;

                    // set to ground truth
                    case sf::Keyboard::E:
                        T = step.ground_truth;
                        break;

                    // [un]pause minimization
                    case sf::Keyboard::Space:
                    case sf::Keyboard::P:
                        min_paused = !min_paused;
                        break;

                    // overlay keyframe over warped current frame
                    case sf::Keyboard::K:
                        show_keyframe = !show_keyframe;
                        break;

                    case sf::Keyboard::M:
                        {
                            int opt = 0;
                            cout << "choose an option:" << endl;
                            cout << "1: show T (radians)" << endl;
                            cout << "2: show T (degrees)" << endl;
                            cout << "3: downsample images by 2x" << endl;
                            cout << "4: enter new T (degrees)" << endl;
                            cout << "5: disturb T (degrees)" << endl;
                            cout << "0: exit" << endl;
                            cin >> opt;

                            switch (opt) {
                                case 0:
                                    break;

                                case 1:
                                    T.printCSV(cout) << endl;
                                    break;

                                case 2:
                                    cout << T << endl;
                                    break;

                                case 3:
                                    step.downsampleBy(1);
                                    cout << "OK" << endl;
                                    break;

                                case 4:
                                    cout << "x [m]: "; cin >> T.value(0);
                                    cout << "y [m]: "; cin >> T.value(1);
                                    cout << "z [m]: "; cin >> T.value(2);
                                    cout << "alpha / pitch [degrees]: "; cin >> T.value(3);
                                    cout << "beta / yaw    [degrees]: "; cin >> T.value(4);
                                    cout << "gamma / roll  [degrees]: "; cin >> T.value(5);
                                    // convert to radians
                                    T.value(3) = deg2rad(T.value(3));
                                    T.value(4) = deg2rad(T.value(4));
                                    T.value(5) = deg2rad(T.value(5));
                                    T.updateRotationMatrix();
                                    cout << "new T: " << T << endl;
                                    break;

                                case 5:
                                    {
                                        Transformation tmp;
                                        cout << "x [m]: "; cin >> tmp.value(0);
                                        cout << "y [m]: "; cin >> tmp.value(1);
                                        cout << "z [m]: "; cin >> tmp.value(2);
                                        cout << "alpha / pitch [degrees]: "; cin >> tmp.value(3);
                                        cout << "beta / yaw    [degrees]: "; cin >> tmp.value(4);
                                        cout << "gamma / roll  [degrees]: "; cin >> tmp.value(5);
                                        // convert to radians
                                        tmp.value(3) = deg2rad(tmp.value(3));
                                        tmp.value(4) = deg2rad(tmp.value(4));
                                        tmp.value(5) = deg2rad(tmp.value(5));
                                        tmp.updateRotationMatrix();
                                        T = T + tmp;
                                        T.updateRotationMatrix();
                                        cout << "delta: " << tmp << endl;
                                        cout << "new T: " << T << endl;
                                    }
                                    break;

                                default:
                                    cout << "unknown command" << endl;
                                    break;
                            }
                        }
                        break;



                    default:
                        break;
                }
            }
        }

        window.clear();

        if (!min_paused) {
#if 1
            IterGaussNewton(step, T);
#else
            IterGradientDescent(step, T, step_size);
#endif
            iter++;
        }

        Warp::calcError(step, T, error_tmp, J_tmp, &window, &font);

        if (show_keyframe)
            step.frame_first.getIntensityData().drawAt(window, sf::Vector2f(0,step.frame_second.getHeight()+2));

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
    //testscene.loadFromSceneDirectory("../matlab/input/courtyard/lux");
    testscene.loadFromSceneDirectory("../matlab/input/courtyard/normal"); // step 22 is nice!

    // choose one of these functions:
    //test_warping(testscene);
    //draw_error_surface(testscene.getStep(0));
    //write_trajectory(testscene);
    run_minimization(testscene);


    return EXIT_SUCCESS;
}
///////////////////////////////////////////////////////////////////////////////

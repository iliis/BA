#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <SFML/Graphics.hpp>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "core/scene.h"
#include "core/image_data.h"
#include "core/transformation.h"
#include "core/camera_intrinsics.h"
#include "core/warp.h"
#include "core/minimization.h"

using namespace std;
using namespace Eigen;

sf::Font font;
sf::RenderWindow window(sf::VideoMode(1280, 768), "dense odometry");
//sf::RenderWindow window(sf::VideoMode(4*752+6, 3*480), "dense odometry");
//sf::RenderWindow window(sf::VideoMode(256*4+2, 128*5), "dense odometry");
//sf::RenderWindow window(sf::VideoMode(256*3+4, 128*4+4), "dense odometry");

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
void draw_error_surface(const CameraStep& step, const Warp::PlotRange& range1, const Warp::PlotRange& range2, const Warp::Parameters params)
{
    Eigen::MatrixXf errorsurface(range1.steps, range2.steps);
    Eigen::Matrix<float, Dynamic, 6> errorgradients(range1.steps*range2.steps, 6);

    cout << "rendering error surface ..." << endl;

    sf::Clock clock;
    clock.restart();
    Warp::renderErrorSurface(errorsurface, errorgradients, step, step.ground_truth, range1, range2, params);
    sf::Int32 ms = clock.getElapsedTime().asMilliseconds();

    cout << "rendered surface in " << ms << "ms (" << ((float) ms) / (errorsurface.rows() * errorsurface.cols()) << "ms per point)" << endl;

    window.setVisible(true);


    float TILE_SIZE = window.getSize().y / range2.steps;

    bool run = true;
    ImageData errorplot;
    errorplot.loadFromMatrix(errorsurface, Colormap::Hot());
    while (window.isOpen() && run)
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::KeyPressed && (event.key.code == sf::Keyboard::Escape || event.key.code == sf::Keyboard::Q))
                run = false;
        }

        window.clear();
        errorplot.drawAt(window, sf::Vector2f(0,0), TILE_SIZE);

        for (unsigned int y = 0; y < errorplot.getHeight(); ++y) {
            for (unsigned int x = 0; x < errorplot.getWidth(); ++x) {
                unsigned int idx = y*errorplot.getWidth()+x;
                float factor = -0.004; // point *down*
                drawArrow(window, x*TILE_SIZE+TILE_SIZE/2, y*TILE_SIZE+TILE_SIZE/2, errorgradients(idx, range1.dim)*factor, errorgradients(idx, range2.dim)*factor);
            }
        }


        window.display();

        sf::sleep(sf::milliseconds(100));
    }
    cout << "closing error surface plot" << endl;
}
///////////////////////////////////////////////////////////////////////////////
void write_trajectory(const Scene& scene, const Warp::Parameters& params, string output_dir = "")
{
    std::vector<Transformation> traj = findTrajectory(scene, params);

    if (output_dir.empty()) {
        output_dir = scene.getSourceDirectory();
    }

    string path = output_dir + "/measured_trajectory.csv";
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
    const sf::Vector2u window_size = window.getSize();

    unsigned int index = 0;
    CameraStep step = scene.getStep(index);

    Transformation T(0,0,0,0,0,0);

    Eigen::VectorXf error_tmp;
    Eigen::Matrix<float, Eigen::Dynamic, 6> J_tmp;

    Warp::Parameters params(new ErrorWeightNone());
    params.pyramid_levels = 3;
    params.max_iterations = 1000;
    params.T_init = Transformation(0,0,0,0,0,0);
    params.gradient_norm_threshold = 0.1;

    cout << "starting main loop" << endl;

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
                    // exit
                    case sf::Keyboard::Escape:
                        exit(0);
                        return;

                    // go to prev frame
                    case sf::Keyboard::B:
                        if (index == 0)
                            index = scene.getStepCount();
                        index--;
                        step = scene.getStep(index);
                        T = Transformation(0,0,0,0,0,0);
                        break;

                    // go to next (prev) frame
                    case sf::Keyboard::N:
                        if (event.key.shift) {
                            if (index == 0)
                                index = scene.getStepCount();
                            index--;
                        } else {
                            index++;
                            if (index >= scene.getStepCount())
                                index = 0;
                        }
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

                    // half image size
                    case sf::Keyboard::S:
                        step.downsampleBy(1);
                        break;

                    // reset image to original size
                    case sf::Keyboard::A:
                        step = scene.getStep(index);
                        break;

                    // run minimization algorithm at full speed (without visualization and starting at T = [0 0 0 0 0 0])
                    case sf::Keyboard::F5:
                        T = findTransformationWithPyramid(step, params);
                        break;

                    // move virtual camera / modify T
                    case sf::Keyboard::Numpad4:
                        if (event.key.control) {
                            T.value(4) -= deg2rad(1);
                            T.updateRotationMatrix();
                        } else {
                            T.value(0) -= 0.1;
                        }
                        break;

                    case sf::Keyboard::Numpad6:
                        if (event.key.control) {
                            T.value(4) += deg2rad(1);
                            T.updateRotationMatrix();
                        } else {
                            T.value(0) += 0.1;
                        }
                        break;

                    case sf::Keyboard::Numpad8:
                        if (event.key.control) {
                            T.value(3) += deg2rad(1);
                            T.updateRotationMatrix();
                        } else {
                            T.value(1) -= 0.1;
                        }
                        break;

                    case sf::Keyboard::Numpad5:
                        if (event.key.control) {
                            T.value(3) -= deg2rad(1);
                            T.updateRotationMatrix();
                        } else {
                            T.value(1) += 0.1;
                        }
                        break;

                    case sf::Keyboard::Numpad7:
                        if (event.key.control) {
                            T.value(5) -= deg2rad(1);
                            T.updateRotationMatrix();
                        } else {
                            T.value(2) -= 0.1;
                        }
                        break;

                    case sf::Keyboard::Numpad1:
                        T.value(2) += 0.1;
                        break;

                    case sf::Keyboard::Numpad9:
                        if (event.key.control) {
                            T.value(5) += deg2rad(1);
                            T.updateRotationMatrix();
                        }
                        break;


                    // some error weighting function presets
                    case sf::Keyboard::Num0:
                        params.setWeightFunction(new ErrorWeightNone());
                        break;

                    case sf::Keyboard::Num1: params.setWeightFunction(new ErrorWeightHuber(0.0001)); break;
                    case sf::Keyboard::Num2: params.setWeightFunction(new ErrorWeightHuber(0.001)); break;
                    case sf::Keyboard::Num3: params.setWeightFunction(new ErrorWeightHuber(0.01)); break;
                    case sf::Keyboard::Num4: params.setWeightFunction(new ErrorWeightHuber(0.1)); break;
                    case sf::Keyboard::Num5: params.setWeightFunction(new ErrorWeightHuber(1)); break;
                    case sf::Keyboard::Num6: params.setWeightFunction(new ErrorWeightHuber(10)); break;

                    case sf::Keyboard::M:
                        window.setVisible(false);
                        {
                            int opt = 0;
                            cout << "choose an option:" << endl;
                            cout << "[  1 ]: show T (radians)" << endl;
                            cout << "[  2 ]: show T (degrees)" << endl;
                            cout << "[  3 ]: downsample images by 2x" << endl;
                            cout << "[  4 ]: enter new T (degrees)" << endl;
                            cout << "[  5 ]: disturb T (degrees)" << endl;
                            cout << "[  6 ]: reload step" << endl;
                            cout << "[  7 ]: choose step number" << endl;
                            cout << "[  8 ]: load step from two frames" << endl;
                            cout << "[  9 ]: choose weight function" << endl;
                            cout << "[ 10 ]: choose number of pyramid levels" << endl;
                            cout << "[ 11 ]: set gradient norm threshold" << endl;
                            cout << "[ 12 ]: render error surface" << endl;
                            cout << "[  0 ]: exit" << endl;
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

                                case 6:
                                    step = scene.getStep(index);
                                    cout << "OK" << endl;
                                    break;

                                case 7:
                                    do {
                                        cout << "enter scene number [0-" << (scene.getStepCount()-1) << "]: ";
                                        cin >> index;
                                    } while(index >= scene.getStepCount());
                                    step = scene.getStep(index);
                                    break;

                                case 8:
                                    {
                                        unsigned int indexA, indexB;
                                        do {
                                            cout << "enter frame number [0-" << (scene.getFrameCount()-1) << "] for first frame: ";
                                            cin >> indexA;
                                        } while(indexA >= scene.getFrameCount());

                                        do {
                                            cout << "enter frame number [0-" << (scene.getFrameCount()-1) << "] for second frame: ";
                                            cin >> indexB;
                                        } while(indexB >= scene.getFrameCount());

                                        step = scene.getStep(indexA, indexB);
                                    }
                                    break;

                                case 9:
                                    do {
                                        cout << "choose:" << endl;
                                        cout << "1: None" << endl;
                                        cout << "2: Huber" << endl;
                                        cout << "0: cancel" << endl;
                                        cin >> opt;
                                    } while (opt < 0 || opt > 2);

                                    switch (opt) {
                                        case 1:
                                            params.setWeightFunction(new ErrorWeightNone());
                                            break;

                                        case 2:
                                            {
                                                float d = 0;
                                                cout << "delta? ";
                                                cin >> d;
                                                params.setWeightFunction(new ErrorWeightHuber(d));
                                            }
                                            break;

                                        default:
                                            break;
                                    }
                                    break;

                                case 10:
                                    do {
                                        cout << "number of levels [>=1] (current: " << params.pyramid_levels << "): ";
                                        cin >> params.pyramid_levels;
                                    } while (params.pyramid_levels < 1);
                                    break;

                                case 11:
                                    cout << "new gradient norm threshold [0-1] (current: " << params.gradient_norm_threshold << "): ";
                                    cin >> params.gradient_norm_threshold;
                                    break;

                                case 12:
                                    {
                                        bool around_T = false;
                                        Warp::PlotRange range1(0,0,1,1), range2(1,0,1,1);
                                        cout << "around current Transformation? [0/1] ";
                                        cin >> around_T;
                                        cout << around_T;
                                        cout << " >>> range 1:" << endl;
                                        range1.readFromStdin();
                                        cout << " >>> range 2:" << endl;
                                        range2.readFromStdin();
                                        if (around_T) {
                                            range1.from += T.value(range1.dim); range1.to += T.value(range1.dim);
                                            range2.from += T.value(range2.dim); range2.to += T.value(range2.dim);
                                        }
                                        draw_error_surface(step, range1, range2, params);
                                    }
                                    break;

                                default:
                                    cout << "unknown command" << endl;
                                    break;
                            }
                        }
                        window.setVisible(true);
                        break;



                    default:
                        break;
                }
            }
        }

        window.clear();

        if (!min_paused) {
            bool finished;
#if 1
            finished = IterGaussNewton(step, T, params) < 0.0001;
#else
            finished = IterGradientDescent(step, T, step_size, *weight_function) < 0.0001;
#endif

// auto upscale / pause when converged
            if (finished) {
#if 0
                if (step.scale > 0) {
                    // upscale image again
                    unsigned s = step.scale-1;
                    step = scene.getStep(index);
                    step.downsampleBy(s);
                } else {
                    min_paused = true;
                }
#endif
            }
        }

        Warp::calcError(step, T, error_tmp, J_tmp, params, &window, &font);

        if (show_keyframe)
            step.frame_first.getIntensityData().drawAt(window, sf::Vector2f(0,step.frame_second.getHeight()+2));


        sf::Text t;
        t.setFont(font);
        t.setCharacterSize(12);
        t.setString(params.toString()); t.setPosition(2,window_size.y-3*14); window.draw(t);

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

    delete params.weight_function;
}
///////////////////////////////////////////////////////////////////////////////
void run_on_real_data()
{
    rosbag::Bag bag;
    bag.open("/home/samuel/data/2015-06-11-16-30-01.bag", rosbag::bagmode::Read);

    cout << "bag is " << bag.getSize() << " bytes (?) big." << endl;

    vector<string> topics;
    topics.push_back("/stereo_dense_reconstruction/image_fused");
    topics.push_back("/stereo_dense_reconstruction/disparity");

    //rosbag::View view(bag, rosbag::TopicQuery("/stereo_dense_reconstruction/image_fused"));
    //rosbag::View view(bag, rosbag::TopicQuery("/stereo_dense_reconstruction/disparity"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    cout << "it has " << view.size() << " views." << endl;


    BOOST_FOREACH(rosbag::MessageInstance const m, view) {

        //cout << "time: " << m.getTime() << endl;
        //cout << "topic: " << m.getTopic() << endl;
        //cout << "data type: " << m.getDataType() << endl;
        //cout << "caller ID: " << m.getCallerId() << endl;
        //cout << "connection header: " << m.getConnectionHeader() << endl;
        //cout << " //////////////////////////////////////////////// " << endl;
        //cout << "message def: " << m.getMessageDefinition() << endl;

        window.clear();

        stereo_msgs::DisparityImage::Ptr image = m.instantiate<stereo_msgs::DisparityImage>();

        if (image) {
            //cout << "OK" << endl;
            //cout << image->image.width << " x " << image->image.height << endl;
            //cout << "encoding: " << image->image.encoding << endl;

            ImageData i;
            //i.loadFromROSgrayscale(image->image);
            i.loadFromROSdepthmap(image->image, Colormap::Jet());
            window.draw(i);
        }

        sensor_msgs::Image::Ptr intensities = m.instantiate<sensor_msgs::Image>();
        if (intensities) {
            ImageData i;
            i.loadFromROSgrayscale(*intensities);
            window.draw(i);
        }

        window.display();

        //getchar();

    }

    bool run = false;
    while (window.isOpen() && run)
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::KeyPressed && (event.key.code == sf::Keyboard::Escape || event.key.code == sf::Keyboard::Q))
                run = false;
        }

        window.clear();
        window.display();
    }

    bag.close();
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
    //testscene.loadFromSceneDirectory("../matlab/input/courtyard/normal"); // step 22 is nice!
    testscene.loadFromBagFile("/home/samuel/data/2015-06-11-16-30-01.bag");

    cout << testscene.getIntrinsics() << endl;

    Warp::Parameters params(new ErrorWeightNone());
    params.pyramid_levels = 3;
    params.max_iterations = 100;
    params.T_init = Transformation(0,0,0,0,0,0);
    params.gradient_norm_threshold = 0.1;

    //write_trajectory(testscene, params, ".");

    run_minimization(testscene);

    //run_on_real_data();


    return EXIT_SUCCESS;
}
///////////////////////////////////////////////////////////////////////////////

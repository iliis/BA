#include "minimization_gui.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
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
void draw_error_surface(sf::RenderWindow& window, const CameraStep& step, const Warp::PlotRange& range1, const Warp::PlotRange& range2, const Warp::Parameters params)
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
    errorplot.loadFromMatrix(errorsurface);

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

        drawMatrixAt(window, errorplot.data, sf::Vector2f(0,0), "", NULL, Colormap::Hot(), TILE_SIZE);

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
void plot_warp_debug_data(sf::RenderWindow& window, sf::Font& font, const Warp::WarpDebugData& data, const CameraStep& step, const bool show_keyframe, const float view_scale)
{
    const float scale = pow(2, step.scale) * view_scale;

    const unsigned int W = step.intrinsics.getCameraWidth() * scale;
    const unsigned int H = step.intrinsics.getCameraHeight() * scale;

    drawMatrixAt(window, data.errors_in_current, sf::Vector2f(0,0),   "errors in current", &font, Colormap::Jet(), scale);

    if (show_keyframe)
        drawMatrixAt(window, step.frame_first.getIntensityData().data, sf::Vector2f(0,H+2), "keyframe", &font, Colormap::Colormap(), scale);
    else
        drawMatrixAt(window, data.warped_image     , sf::Vector2f(0,H+2), "warped current frame", &font, Colormap::Colormap(), scale);

    drawMatrixAt(window, step.frame_second.getIntensityData().data, sf::Vector2f(W+2,0), "current frame [frame " + boost::lexical_cast<string>(step.index_second) + "]", &font, Colormap::Colormap(), scale);
    drawMatrixAt(window, step.frame_first .getIntensityData().data, sf::Vector2f(W+2,H+2), "keyframe [frame " + boost::lexical_cast<string>(step.index_first) + "]", &font, Colormap::Colormap(), scale);
    //step.frame_second.getDepthData().drawAt( *plotTarget, sf::Vector2f(0,0));
    //step.frame_first .getDepthData().drawAt( *plotTarget, sf::Vector2f(0,H+2));


    drawMatrixAt(window, data.weighted_errors,     sf::Vector2f(2*W+4, 0),     "weighted errors", &font, Colormap::Jet(), scale);
    drawMatrixAt(window, data.J_norm,              sf::Vector2f(2*W+4, H+2),   "norm(J) [max: " + boost::lexical_cast<string>(data.J_norm.maxCoeff()) + "]", &font, Colormap::Jet(), scale);
    drawMatrixAt(window, data.selection_heuristic, sf::Vector2f(2*W+4, 2*H+4), "image gradient [max: " + boost::lexical_cast<string>(data.selection_heuristic.maxCoeff()) + "]", &font, Colormap::Jet(), scale);
}
///////////////////////////////////////////////////////////////////////////////
bool min_paused = true; // start in paused state, global to keep value :P
void run_minimization(sf::RenderWindow& window, sf::Font& font, const Scene& scene, Warp::Parameters params)
{
    const sf::Vector2u window_size = window.getSize();

    Warp::WarpDebugData warpdebugdata;

    unsigned int index = 0;
    CameraStep step = scene.getStep(index);

    Transformation T = params.T_init;

    Eigen::VectorXf error_tmp;
    Eigen::Matrix<float, Eigen::Dynamic, 6> J_tmp;

    float view_scale = 1;

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

                    case sf::Keyboard::Add:
                        view_scale *= 2;
                        break;

                    case sf::Keyboard::Subtract:
                        view_scale /= 2;
                        break;

                    case sf::Keyboard::Multiply:
                        view_scale = 1;
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
                            cout << "[ 13 ]: toggle pixel gradient filter (before or after warping)" << endl;
                            cout << "[  0 ]: exit" << endl;
                            cin.clear(); cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
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
                                        cout << "min level [>=0]   (current: " << params.min_pyramid_levels << "): ";
                                        cin >> params.min_pyramid_levels;
                                        cout << "max level [>=min] (current: " << params.max_pyramid_levels << "): ";
                                        cin >> params.max_pyramid_levels;
                                    } while (params.min_pyramid_levels <= params.max_pyramid_levels);
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
                                        draw_error_surface(window, step, range1, range2, params);
                                    }
                                    break;

                                case 13:
                                    params.filter_on_unwarped_gradient = ! params.filter_on_unwarped_gradient;
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

        float total_error = Warp::calcError(step, T, error_tmp, J_tmp, params, &warpdebugdata);

        plot_warp_debug_data(window, font, warpdebugdata, step, show_keyframe, view_scale);

        sf::Text t;
        t.setFont(font);
        t.setCharacterSize(12);

        ostringstream s;
        s << "total error: " << sqrt(total_error) << endl;
        s << "abs max: " << (error_tmp.array().abs().maxCoeff()) << endl;
        s << "current: " << T << endl;
        s << "truth:   " << step.ground_truth << endl;
        s << "T error: " << (T.value-step.ground_truth.value).norm() << endl;
        s << " --------- " << endl;
        s << params.toString();
        t.setString(s.str()); t.setPosition(2,window_size.y-t.getLocalBounds().height-4); window.draw(t);

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

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
std::string float_to_string(const float& f)
{
    ostringstream s;
    s.width(4);
    s << f;
    return s.str();
}
///////////////////////////////////////////////////////////////////////////////
void render_error_surface(
        sf::RenderTarget& target,
        const Eigen::MatrixXf& errorsurface,
        const Eigen::Matrix<float, Eigen::Dynamic, 6>& errorgradients,
        const Warp::PlotRange& range1,
        const Warp::PlotRange& range2,
        const bool show_gradient,
        sf::Font& font,
        const Colormap::Colormap& colormap)
{
    std::vector<std::string> dimension_labels;
    dimension_labels.push_back("X");
    dimension_labels.push_back("Y");
    dimension_labels.push_back("Z");
    dimension_labels.push_back("alpha / pitch");
    dimension_labels.push_back("beta / yaw");
    dimension_labels.push_back("gamma / roll");

    sf::Image img;
    matrix_to_image(errorsurface, img, colormap);

    float border_x = 150;
    float border_y = 140;

    float scale_x = (target.getSize().x - 2*border_x) / range1.steps;
    float scale_y = (target.getSize().y - 2*border_y) / range2.steps;

    drawImageAt(target, img, sf::Vector2f(border_x,border_y), "", NULL, sf::Vector2f(scale_x, scale_y));

    if (show_gradient) {
        for (unsigned int y = 0; y < errorsurface.rows(); ++y) {
            for (unsigned int x = 0; x < errorsurface.cols(); ++x) {
                unsigned int idx = y*errorsurface.cols()+x;
                float factor = -0.004; // point *down*
                drawArrow(target,
                        x*scale_x + scale_x/2 + border_x,
                        y*scale_y + scale_y/2 + border_y,
                        errorgradients(idx, range1.dim)*factor,
                        errorgradients(idx, range2.dim)*factor);
            }
        }
    }


    sf::Color decoration_color(0,0,0);

    // draw border around surface
    sf::RectangleShape frame;
    frame.setFillColor(sf::Color::Transparent);
    frame.setOutlineColor(decoration_color);
    frame.setOutlineThickness(1);
    frame.setSize(sf::Vector2f(target.getSize().x-2*border_x+2, target.getSize().y-2*border_y+2));
    frame.setPosition(border_x-1, border_y-1);
    target.draw(frame);

    // draw axis ticks
    sf::RectangleShape r;
    r.setFillColor(decoration_color);
    r.setOutlineColor(sf::Color::Transparent);
    r.setOutlineThickness(0);

    sf::Text t;
    t.setColor(decoration_color);
    t.setFont(font);
    t.setCharacterSize(20);

    // vertical ticks
    r.setSize(sf::Vector2f(10,1));
    unsigned int stepsize_y = (range2.steps+1)/6;
    if (stepsize_y < 1)
        stepsize_y = 1;

    for (unsigned int i = 0; i < range2.steps; i+=stepsize_y) {

        float y = border_y-1+i*scale_y+scale_y/2;

        r.setPosition(border_x-1-10, y);
        target.draw(r);

        t.setString(float_to_string( i*(range2.to-range2.from)/(range2.steps-1) + range2.from ));
        t.setPosition(border_x-1-10-3-t.getLocalBounds().width, y-t.getLocalBounds().height/2-2);

        target.draw(t);
    }

    // horizontal ticks
    r.setSize(sf::Vector2f(1,10));
    float y = border_y - 1 + frame.getSize().y;
    unsigned int stepsize_x = (range1.steps+1)/6;
    if (stepsize_x < 1)
        stepsize_x = 1;
    for (unsigned int i = 0; i < range1.steps; i+=stepsize_x) {

        float x = border_x-1+i*scale_x+scale_x/2;

        r.setPosition(x, y);
        target.draw(r);

        t.setString(float_to_string( i*(range1.to-range1.from)/(range1.steps-1) + range1.from ));
        t.setPosition(x-1-t.getLocalBounds().width/2, y+10+2);

        target.draw(t);
    }

    // X label
    t.setCharacterSize(t.getCharacterSize()*1.5);
    t.setString(dimension_labels[range1.dim]);
    t.setPosition(border_x-1+frame.getSize().x/2-t.getLocalBounds().width/2, y+10+t.getLocalBounds().height*2);
    target.draw(t);

    // Y label
    t.setString(dimension_labels[range2.dim]);
    t.setRotation(-90);
    t.setPosition(border_x-1-10-t.getLocalBounds().height*4, border_y-1+frame.getSize().y/2 + t.getLocalBounds().width/2);
    target.draw(t);
}
///////////////////////////////////////////////////////////////////////////////
void draw_error_surface(sf::RenderWindow& window, sf::Font& font, const CameraStep& step, const Warp::PlotRange& range1, const Warp::PlotRange& range2, const Warp::Parameters params)
{
    Eigen::MatrixXf errorsurface(range1.steps, range2.steps);
    Eigen::Matrix<float, Dynamic, 6> errorgradients(range1.steps*range2.steps, 6);

    cout << "rendering error surface ..." << endl;

    sf::Clock clock;
    clock.restart();
    Warp::renderErrorSurface(errorsurface, errorgradients, step, step.ground_truth, range1, range2, params);
    sf::Int32 ms = clock.getElapsedTime().asMilliseconds();

    cout << "rendered surface in " << ms << "ms (" << ((float) ms) / (errorsurface.rows() * errorsurface.cols()) << "ms per point)" << endl;

    save_matrix_to_image(errorsurface, "error_surface_raw.png", Colormap::Hot());

    sf::RenderTexture surfaceplot;
    surfaceplot.create(1300,1300);
    surfaceplot.clear(sf::Color(255,255,255,0));
    render_error_surface(surfaceplot, errorsurface, errorgradients, range1, range2, false, font, Colormap::Hot());
    surfaceplot.getTexture().copyToImage().saveToFile("error_surface.png");
    // TODO: store to disk

    window.setVisible(true);

    bool run = true;

    bool show_gradient = true;

    while (window.isOpen() && run)
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::KeyPressed) {
                switch(event.key.code) {
                    case sf::Keyboard::Escape:
                    case sf::Keyboard::Q:
                        run = false;
                        break;

                    case sf::Keyboard::G:
                        show_gradient = !show_gradient;
                        break;

                    default:
                        break;
                }
            }
        }

        window.clear(sf::Color(255,255,255));

        render_error_surface(window, errorsurface, errorgradients, range1, range2, show_gradient, font, Colormap::Hot());

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
    bool record_frames = false; int iterations = 0;
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
                    case sf::Keyboard::T:
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

                    // save all images
                    case sf::Keyboard::F1:
                        {
                            save_matrix_to_image(warpdebugdata.warped_image, "warped_image.png", Colormap::Colormap());
                            save_matrix_to_image(warpdebugdata.errors_in_current, "errors_current.png", Colormap::Jet());
                            save_matrix_to_image(warpdebugdata.J_norm, "J_norm.png", Colormap::Jet());
                        }
                        break;

                    case sf::Keyboard::F2:
                        record_frames = !record_frames;
                        if (record_frames)
                            iterations = 0; // reset iteration counter
                        break;

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
                            readInput(opt);

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
                                    cout << "x [m]: "; readInput(T.value(0));
                                    cout << "y [m]: "; readInput(T.value(1));
                                    cout << "z [m]: "; readInput(T.value(2));
                                    cout << "alpha / pitch [degrees]: "; readInput(T.value(3));
                                    cout << "beta / yaw    [degrees]: "; readInput(T.value(4));
                                    cout << "gamma / roll  [degrees]: "; readInput(T.value(5));
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
                                        cout << "x [m]: "; readInput(tmp.value(0));
                                        cout << "y [m]: "; readInput(tmp.value(1));
                                        cout << "z [m]: "; readInput(tmp.value(2));
                                        cout << "alpha / pitch [degrees]: "; readInput(tmp.value(3));
                                        cout << "beta / yaw    [degrees]: "; readInput(tmp.value(4));
                                        cout << "gamma / roll  [degrees]: "; readInput(tmp.value(5));
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
                                        readInput(index);
                                    } while(index >= scene.getStepCount());
                                    step = scene.getStep(index);
                                    break;

                                case 8:
                                    {
                                        unsigned int indexA, indexB;
                                        do {
                                            cout << "enter frame number [0-" << (scene.getFrameCount()-1) << "] for first frame: ";
                                            readInput(indexA);
                                        } while(indexA >= scene.getFrameCount());

                                        do {
                                            cout << "enter frame number [0-" << (scene.getFrameCount()-1) << "] for second frame: ";
                                            readInput(indexB);
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
                                        readInput(opt);
                                    } while (opt < 0 || opt > 2);

                                    switch (opt) {
                                        case 1:
                                            params.setWeightFunction(new ErrorWeightNone());
                                            break;

                                        case 2:
                                            {
                                                float d = 0;
                                                cout << "delta? ";
                                                readInput(d);
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
                                        readInput(params.min_pyramid_levels);
                                        cout << "max level [>=min] (current: " << params.max_pyramid_levels << "): ";
                                        readInput(params.max_pyramid_levels);
                                    } while (params.min_pyramid_levels <= params.max_pyramid_levels);
                                    break;

                                case 11:
                                    cout << "new gradient norm threshold [0-1] (current: " << params.gradient_norm_threshold << "): ";
                                    readInput(params.gradient_norm_threshold);
                                    break;

                                case 12:
                                    {
                                        bool around_T = false;
                                        Warp::PlotRange range1(0,0,1,1), range2(1,0,1,1);
                                        cout << "around current Transformation? [0/1] ";
                                        readInput(around_T);
                                        cout << around_T;
                                        cout << " >>> range 1:" << endl;
                                        range1.readFromStdin();
                                        cout << " >>> range 2:" << endl;
                                        range2.readFromStdin();
                                        if (around_T) {
                                            range1.from += T.value(range1.dim); range1.to += T.value(range1.dim);
                                            range2.from += T.value(range2.dim); range2.to += T.value(range2.dim);
                                        }
                                        draw_error_surface(window, font, step, range1, range2, params);
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

        if (record_frames)
            window.clear(sf::Color(155,0,0));
        else
            window.clear();

        if (!min_paused) {
            bool finished;
            ++iterations;
#if 1
            finished = IterGaussNewton(step, T, params) < 0.0001;
#else
            finished = IterGradientDescent(step, T, step_size, *weight_function) < 0.0001;
#endif

// auto upscale / pause when converged
            if (finished) {
                record_frames = false;
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

        if (record_frames) {
            ostringstream index_string;
            index_string.width(4);
            index_string.fill('0');
            index_string << iterations;
            const std::string num = index_string.str();
            save_matrix_to_image(warpdebugdata.warped_image, "recording/" + num + "_warped_image.png", Colormap::Colormap());
            save_matrix_to_image(warpdebugdata.errors_in_current, "recording/" + num + "_errors_current.png", Colormap::Jet());
            save_matrix_to_image(warpdebugdata.J_norm, "recording/" + num + "_J_norm.png", Colormap::Jet());
        }

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

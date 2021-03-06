#include "live.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
ImageData cam0, cam1, depth;
CameraIntrinsics current_intrinsics = CameraIntrinsics(
        /* sensor size     */ Eigen::Vector2f(752, 480),
        /* principal point */ Eigen::Vector2f(370.105, 226.664),
        /* focal length    */ 471.7,
        /* stereo baseline */ 0.110174);
Telemetry telemetry;
CameraState camera_state;

sf::Clock fps_clock;
sf::Time time_per_step;

// average over multiple frames
sf::Time last_step_times_sum;
std::queue<sf::Time> last_step_times;
///////////////////////////////////////////////////////////////////////////////
void calibration_callback(const visensor_msgs::visensor_calibration::ConstPtr& m)
{
    current_intrinsics = CameraIntrinsics(
        /* sensor size     */ Eigen::Vector2f(m->image_width, m->image_height),
        /* principal point */ Eigen::Vector2f(m->principal_point[0], m->principal_point[1]),
        /* focal length    */ m->focal_length[0],
        /* stereo baseline */ 0.110174);
}
///////////////////////////////////////////////////////////////////////////////
void telemetry_callback(const sensor_msgs::Image::ConstPtr& m)
{
    memcpy((void*) &telemetry, (const void*) m->data.data(), sizeof(Telemetry));

    if (!telemetry.checkOK()) {
        cerr << "WARNING: telemetry format is incorrect. Check that all structs have the same size on 64bit and 32bit!" << endl;

        cout << "params: " << sizeof(Warp::Parameters) << endl;
        cout << "telemetry: " << sizeof(Telemetry) << endl;

        cout << std::hex << telemetry.magic_constant1 << endl;
        cout << std::hex << telemetry.magic_constant2 << endl;
        cout << std::dec;
    }

    telemetry.transformation.updateRotationMatrix();
    camera_state.apply(telemetry.transformation);


    time_per_step = fps_clock.getElapsedTime();

    last_step_times.push(time_per_step);
    last_step_times_sum += time_per_step;
    if (last_step_times.size() > 30) {
        last_step_times_sum -= last_step_times.front();
        last_step_times.pop();
    }

    fps_clock.restart();
}
///////////////////////////////////////////////////////////////////////////////
void image0_callback(const sensor_msgs::Image::ConstPtr& m)
{
    loadImageDataFromROSraw(cam0, *m);
    // scale correctly
    cam0.data /= 255.0f;
}
///////////////////////////////////////////////////////////////////////////////
void image1_callback(const sensor_msgs::Image::ConstPtr& m)
{
    loadImageDataFromROSraw(cam1, *m);
    // scale correctly
    cam1.data /= 255.0f;
}
///////////////////////////////////////////////////////////////////////////////
void depth_callback(const sensor_msgs::Image::ConstPtr& m)
{
    loadImageDataFromROSraw(depth, *m);
    // scale correctly
    depth.data /= 6.0f;
}
///////////////////////////////////////////////////////////////////////////////
void set_coarse_shutter_width(int w)
{
    string cmd("rosrun dynamic_reconfigure dynparam set /visensor_node \"{ \
        'cam_coarse_shutter_width': " + boost::lexical_cast<string>(w) + " }\"");
    system(cmd.c_str());
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void show_live_data(sf::RenderWindow& window, sf::Font& font, int argc, char* argv[])
{
    // remember 'original' window size to keep stuff where it belongs even when window gets resized
    const sf::Vector2u window_size = window.getSize();

    ros::init(argc, argv, "listener");

    ros::NodeHandle node;

    ros::Subscriber subI = node.subscribe("/cam0/calibration", 3, calibration_callback);
    ros::Subscriber subT = node.subscribe("/cam2/image_raw", 3, telemetry_callback);
    ros::Subscriber sub0 = node.subscribe("/cam0/image_raw", 3, image0_callback);
    ros::Subscriber sub1 = node.subscribe("/cam1/image_raw", 3, image1_callback);
    ros::Subscriber subD = node.subscribe("/dense/image_raw", 3, depth_callback);

    // appareantly, there is no C++ API for this: https://github.com/ros/dynamic_reconfigure/issues/4
    system("rosrun dynamic_reconfigure dynparam set /visensor_node \"{ \
        'individual_cam_config': 0, \
        'cam_agc_enable': 0, \
        'cam_aec_enable': 0,  \
        'cam_coarse_shutter_width': 300, \
        'penalty_1': 20, \
        'penalty_2': 200, \
        'threshold': 100, \
        'lr_check': 2 \
    }\"");

    int shutter = 300;

    // TODO: read this from ROS messages

    std::vector<CameraImage> recorded_frames;

    int traj_files_count = 0;

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

                    // reset scaling of window
                    case sf::Keyboard::F12:
                        {
                            window.resetGLStates();
                            sf::View v = window.getView();
                            v.reset(sf::FloatRect(0, 0, window.getSize().x, window.getSize().y));
                            window.setView(v);
                        }
                        break;

                    // open camera shutter (for darker conditions)
                    case sf::Keyboard::PageUp:
                        shutter *= 1.5;
                        set_coarse_shutter_width(shutter);
                        break;

                    // close camera shutter (if the scene gets too bright)
                    case sf::Keyboard::PageDown:
                        shutter /= 1.5; if (shutter < 1) {shutter = 1;}
                        set_coarse_shutter_width(shutter);
                        break;

                    case sf::Keyboard::Add:
                        camera_state.scale *= 1.5;
                        break;

                    case sf::Keyboard::Subtract:
                        camera_state.scale /= 1.5;
                        break;

                    case sf::Keyboard::S:
                        window.setVisible(false);
                        cin.clear(); cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                        cout << "set shutter width (current: " << shutter << "): ";
                        cin >> shutter;
                        set_coarse_shutter_width(shutter);
                        window.setVisible(true);
                        break;

                    // reset visualization of camera state
                    case sf::Keyboard::R:
                        camera_state.reset();
                        break;

                    // record a frame
                    case sf::Keyboard::Space:
                        {
                            CameraImage frame;
                            recorded_frames.push_back(frame);
                            recorded_frames.back().loadFromMatrices(cam0.data, depth.data);
                        }
                        break;

                    // save current trajectory to disk
                    case sf::Keyboard::F1:
                        camera_state.saveToDisk("traj_" + boost::lexical_cast<string>(traj_files_count) + ".csv");
                        ++traj_files_count;
                        break;

                    // run normal minimization gui
                    case sf::Keyboard::Return:
                        {
                            sub0.shutdown();
                            sub1.shutdown();
                            subD.shutdown();

                            Warp::Parameters params(new ErrorWeightNone());
                            params.min_pyramid_levels = 1;
                            params.max_pyramid_levels = 4;
                            params.max_iterations = 100;
                            params.T_init = Transformation(0,0,0,0,0,0);
                            params.gradient_norm_threshold = 0.01;
                            params.cutout_left   = 20;
                            params.cutout_right  = 20;
                            params.cutout_top    = 50;
                            params.cutout_bottom = 20;

                            Scene scene;
                            scene.addFrames(recorded_frames);
                            scene.setIntrinsics(current_intrinsics);
                            run_minimization(window, font, scene, params);
                        }
                        break;

                    default:
                        break;
                }
            }
        }

        window.clear(sf::Color(20,20,20));
        //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        drawMatrixAt(window, cam0.data,  sf::Vector2f(0,0),   "camera 0",  &font, Colormap::Colormap(), 0.5);
        drawMatrixAt(window, cam1.data,  sf::Vector2f(cam0.getWidth()/2,0), "camera 1",  &font, Colormap::Colormap(), 0.5);
        drawMatrixAt(window, depth.data, sf::Vector2f(cam0.getWidth(),0), "disparity", &font, Colormap::Jet(), 0.5);


        // calculate average fps
        float average_time_per_step = last_step_times_sum.asMilliseconds();
        if (last_step_times.size() > 0)
            average_time_per_step /= last_step_times.size();
        if (average_time_per_step == 0)
            average_time_per_step = -1;

        sf::Text t;
        t.setFont(font);
        t.setCharacterSize(12);

        ostringstream s;
        s << telemetry.transformation << endl;
        s << "time per step: " << time_per_step.asMilliseconds() << "ms = " << 1000.0f/time_per_step.asMilliseconds() << " FPS" << endl;
        s << "average time per step: " << average_time_per_step << "ms = " << 1000/average_time_per_step << " FPS" << endl;
        s << "current position: " << camera_state.getPosition().transpose() << endl;
        s << camera_state.getOrientationMatrix() << endl;
        s << " ------------- " << endl;
        s << "number of recorded frames: " << recorded_frames.size() << endl;
        s << "cam_coarse_shutter_width: " << shutter;
        t.setString(s.str()); t.setPosition(2,window_size.y-t.getLocalBounds().height-4); window.draw(t);


        if (depth.getWidth() > 0) {
            float disparity = depth.getValue(depth.getSize().cast<int>()/2);
            float depth = current_intrinsics.getBaseline() * current_intrinsics.getFocalLength() / disparity;

            ostringstream s2;
            s2 << "disparity: " << boost::lexical_cast<string>(disparity) << endl;
            s2 << "depth:     " << boost::lexical_cast<string>(depth);
            t.setString(s2.str()); t.setPosition(cam0.getWidth()+2,cam0.getHeight()/2); window.draw(t);

            // draw crosshair
            sf::Vector2f center(cam0.getWidth()*1.25, cam0.getHeight()/4.0f);
            sf::RectangleShape r1(sf::Vector2f(2,50)), r2(sf::Vector2f(50,2));
            r1.setPosition(center-sf::Vector2f(0,25)), r2.setPosition(center-sf::Vector2f(25,0));
            r1.setFillColor(sf::Color::White); r2.setFillColor(sf::Color::White);
            window.draw(r1); window.draw(r2);
        }

        camera_state.shape.setPosition(sf::Vector2f(0,cam0.getHeight()/2));
        camera_state.shape.setSize(sf::Vector2f(cam0.getWidth(), cam0.getHeight()/2));
        window.draw(camera_state);



        window.display();

        ros::spinOnce();
    }
}
///////////////////////////////////////////////////////////////////////////////

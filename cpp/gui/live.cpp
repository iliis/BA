#include "live.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
ImageData cam0, cam1, depth;
///////////////////////////////////////////////////////////////////////////////
void image0_callback(const sensor_msgs::Image::ConstPtr& m)
{
    loadImageDataFromROSraw(cam0, *m);
}
///////////////////////////////////////////////////////////////////////////////
void image1_callback(const sensor_msgs::Image::ConstPtr& m)
{
    loadImageDataFromROSraw(cam1, *m);
}
///////////////////////////////////////////////////////////////////////////////
void depth_callback(const sensor_msgs::Image::ConstPtr& m)
{
    loadImageDataFromROSraw(depth, *m);
    depth.data *= 255;
}
///////////////////////////////////////////////////////////////////////////////
void set_coarse_shutter_width(int w)
{
    string cmd("rosrun dynamic_reconfigure dynparam set /visensor_node \"{ \
        'cam_coarse_shutter_width': " + boost::lexical_cast<string>(w) + " }\"");
    system(cmd.c_str());
}
///////////////////////////////////////////////////////////////////////////////
void show_live_data(sf::RenderWindow& window, sf::Font& font, int argc, char* argv[])
{
    // remember 'original' window size to keep stuff where it belongs even when window gets resized
    const sf::Vector2u window_size = window.getSize();

    ros::init(argc, argv, "listener");

    ros::NodeHandle node;

    ros::Subscriber sub0 = node.subscribe("/cam0/image_raw", 3, image0_callback);
    ros::Subscriber sub1 = node.subscribe("/cam1/image_raw", 3, image1_callback);
    ros::Subscriber subD = node.subscribe("/dense/image_raw", 3, depth_callback);

    // appareantly, there is no C++ API for this: https://github.com/ros/dynamic_reconfigure/issues/4
    system("rosrun dynamic_reconfigure dynparam set /visensor_node \"{ \
        'individual_cam_config': 0, \
        'cam_agc_enable': 0, \
        'cam_aec_enable': 0,  \
        'cam_coarse_shutter_width': 150 \
    }\"");

    int shutter = 150;

    std::vector<CameraImage> recorded_frames;

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

                    // open camera shutter (for darker conditions)
                    case sf::Keyboard::Add:
                        shutter += 10;
                        set_coarse_shutter_width(shutter);
                        break;

                    // close camera shutter (if the scene gets too bright)
                    case sf::Keyboard::Subtract:
                        shutter -= 10; if (shutter < 0) {shutter = 0;}
                        set_coarse_shutter_width(shutter);
                        break;

                    // record a frame
                    case sf::Keyboard::Space:
                        {
                            CameraImage frame;
                            recorded_frames.push_back(frame);
                            recorded_frames.back().loadFromMatrices(cam0.data, depth.data);
                        }
                        break;

                    // run normal minimization gui
                    case sf::Keyboard::Return:
                        {
                            sub0.shutdown();
                            sub1.shutdown();
                            subD.shutdown();

                            Warp::Parameters params(new ErrorWeightNone());
                            params.pyramid_levels = 4;
                            params.max_iterations = 100;
                            params.T_init = Transformation(0,0,0,0,0,0);
                            params.gradient_norm_threshold = 0.01;

                            // TODO: read this from ROS messages
                            CameraIntrinsics visensor_intrinsics = CameraIntrinsics(
                                    /* sensor size     */ Eigen::Vector2f(752, 480),
                                    /* principal point */ Eigen::Vector2f(370.105, 226.664),
                                    /* focal length    */ 471.7,
                                    /* stereo baseline */ 0.110174);

                            Scene scene;
                            scene.addFrames(recorded_frames);
                            scene.setIntrinsics(visensor_intrinsics);
                            run_minimization(window, font, scene, params);
                        }
                        break;

                    default:
                        break;
                }
            }
        }

        window.clear(sf::Color(20,20,20));

        drawMatrixAt(window, cam0.data,  sf::Vector2f(0,0),   "camera 0",  &font, Colormap::Colormap(), 0.5);
        drawMatrixAt(window, cam1.data,  sf::Vector2f(cam0.getWidth()/2,0), "camera 1",  &font, Colormap::Colormap(), 0.5);
        drawMatrixAt(window, depth.data, sf::Vector2f(cam0.getWidth(),0), "disparity", &font, Colormap::Jet(), 0.5);


        sf::Text t;
        t.setFont(font);
        t.setCharacterSize(12);

        ostringstream s;
        s << "number of recorded frames: " << recorded_frames.size() << endl;
        s << "cam_coarse_shutter_width: " << shutter;
        t.setString(s.str()); t.setPosition(2,window_size.y-t.getLocalBounds().height-4); window.draw(t);

        window.display();

        ros::spinOnce();
    }
}
///////////////////////////////////////////////////////////////////////////////

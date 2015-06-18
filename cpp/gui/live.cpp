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
}
///////////////////////////////////////////////////////////////////////////////
void show_live_data(sf::RenderWindow& window, sf::Font& font, int argc, char* argv[])
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle node;

    ros::Subscriber sub0 = node.subscribe("/cam0/image_raw", 3, image0_callback);
    ros::Subscriber sub1 = node.subscribe("/cam1/image_raw", 3, image1_callback);
    ros::Subscriber subD = node.subscribe("/dense/image_raw", 3, depth_callback);

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

                    default:
                        break;
                }
            }
        }

        window.clear(sf::Color(20,20,20));

        drawMatrixAt(window, cam0.data,  sf::Vector2f(0,0),   "camera 0",  &font, Colormap::Colormap(), 0.5);
        drawMatrixAt(window, cam1.data,  sf::Vector2f(cam0.getWidth()/2,0), "camera 1",  &font, Colormap::Colormap(), 0.5);
        drawMatrixAt(window, depth.data, sf::Vector2f(cam0.getWidth(),0), "disparity", &font, Colormap::Jet(), 0.5);

        window.display();

        ros::spinOnce();
    }
}
///////////////////////////////////////////////////////////////////////////////

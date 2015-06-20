#ifndef LIVE_H_INCLUDED
#define LIVE_H_INCLUDED

#include <stdlib.h>
#include <iostream>
#include <SFML/Graphics.hpp>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <visensor_msgs/visensor_calibration.h>
#include <ros/ros.h>

#include "scene.h"
#include "sf_image_data.h"
#include "../core/camera_image.h"
#include "minimization_gui.h"

void show_live_data(sf::RenderWindow& window, sf::Font& font, int argc, char* argv[]);

#endif /* end of include guard: LIVE_H_INCLUDED */

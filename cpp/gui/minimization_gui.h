#ifndef MINIMIZATION_GUI_H_INCLUDED
#define MINIMIZATION_GUI_H_INCLUDED

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <SFML/Graphics.hpp>

#include "scene.h"
#include "sf_image_data.h"
#include "../utils/system.h"

///////////////////////////////////////////////////////////////////////////////

inline sf::Vector2f toSF(Eigen::Vector2f v)
{
    return sf::Vector2f(v.x(), v.y());
}

inline Eigen::Vector2f toEigen(sf::Vector2f v)
{
    return Eigen::Vector2f(v.x, v.y);
}

///////////////////////////////////////////////////////////////////////////////

void drawArrow(sf::RenderTarget& target, float x, float y, float vect_x, float vect_y);

void draw_error_surface(sf::RenderWindow& window, sf::Font& font, const CameraStep& step, const Warp::PlotRange& range1, const Warp::PlotRange& range2, const Warp::Parameters params);

void plot_warp_debug_data(sf::RenderWindow& window, sf::Font& font, const Warp::WarpDebugData& data, const CameraStep& step, const bool show_keyframe, const float view_scale = 1);

void run_minimization(sf::RenderWindow& window, sf::Font& font, const Scene& scene, Warp::Parameters params);

#endif /* end of include guard: MINIMIZATION_GUI_H_INCLUDED */

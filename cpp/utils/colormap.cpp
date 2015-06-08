#include "colormap.h"

#include <iostream>
using namespace std;

namespace Colormap {
///////////////////////////////////////////////////////////////////////////////
float Colormap::calcValue(const float& value, const float& min, const float& max) const
{
    assert(value >= min);
    assert(value <= max);

    return (value - min) / (max - min);
}
///////////////////////////////////////////////////////////////////////////////
sf::Color Colormap::operator()(const float& value, const float& min, const float& max) const
{
    float v = this->calcValue(value, min, max);
    return sf::Color(255*v, 255*v, 255*v);
}
///////////////////////////////////////////////////////////////////////////////
float Logarithmic::calcValue(const float& value, const float& min, const float& max) const
{
    return log2( Colormap::calcValue(value, min, max) );
}
///////////////////////////////////////////////////////////////////////////////
sf::Color RedToGreen::operator()(const float& value, const float& min, const float& max) const
{
    float v = this->calcValue(value, min, max);
    return sf::Color(255 * (1-v), 255 * v, 0);
}
///////////////////////////////////////////////////////////////////////////////
sf::Color Autumn::operator()(const float& value, const float& min, const float& max) const
{
    float v = this->calcValue(value, min, max);
    return sf::Color(255, 255 * v, 0);
}
///////////////////////////////////////////////////////////////////////////////
sf::Color Hot::operator()(const float& value, const float& min, const float& max) const
{
    float v = this->calcValue(value, min, max);

    if (v < 1/3.0f) {
        return sf::Color(255*v*3, 0, 0);
    } else if (v < 2/3.0f) {
        return sf::Color(255, 255*(v*3-1), 0);
    } else {
        return sf::Color(255, 255, 255 * (v*3-2));
    }
}
///////////////////////////////////////////////////////////////////////////////
sf::Color Jet::operator()(const float& value, const float& min, const float& max) const
{
    float v = this->calcValue(value, min, max);

    // black
    // blue
    // bluegreen (cyan)
    // green
    // greenred (yellow)
    // red

         if (v < 1/5.0f) { return sf::Color(0,0,255*v*3); }
    else if (v < 2/5.0f) { return sf::Color(0,255*(v*5-1), 255); }
    else if (v < 3/5.0f) { return sf::Color(0,255, 255*(1-(v*5-2))); }
    else if (v < 4/5.0f) { return sf::Color(255*(v*5-3), 255, 0); }
    else                 { return sf::Color(255, 255*(1-(v*5-4)), 0); }
}
///////////////////////////////////////////////////////////////////////////////
} // namespace Colormap

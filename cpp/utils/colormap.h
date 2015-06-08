#ifndef COLORMAP_H_INCLUDED
#define COLORMAP_H_INCLUDED

#include <math.h>
#include <SFML/Graphics.hpp>
#include <Eigen/Dense> // for assert()

namespace Colormap {

///////////////////////////////////////////////////////////////////////////////
class Colormap
{
public:
    virtual sf::Color operator()(const float& value, const float& min, const float& max) const;
    virtual float     calcValue (const float& value, const float& min, const float& max) const;
};
///////////////////////////////////////////////////////////////////////////////
class Logarithmic : public Colormap
{
public:
    virtual sf::Color operator()(const float& value, const float& min, const float& max) const;
    virtual float     calcValue (const float& value, const float& min, const float& max) const;
};
///////////////////////////////////////////////////////////////////////////////
class RedToGreen : public Colormap
{
public:
    virtual sf::Color operator()(const float& value, const float& min, const float& max) const;
};
///////////////////////////////////////////////////////////////////////////////
class Autumn : public Colormap
{
public:
    virtual sf::Color operator()(const float& value, const float& min, const float& max) const;
};
///////////////////////////////////////////////////////////////////////////////
class Hot : public Colormap
{
public:
    virtual sf::Color operator()(const float& value, const float& min, const float& max) const;
};
///////////////////////////////////////////////////////////////////////////////
class Jet : public Colormap
{
public:
    virtual sf::Color operator()(const float& value, const float& min, const float& max) const;
};
///////////////////////////////////////////////////////////////////////////////

} // namespace Colormap

#endif /* end of include guard: COLORMAP_H_INCLUDED */

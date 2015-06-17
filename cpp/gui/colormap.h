#ifndef COLORMAP_H_INCLUDED
#define COLORMAP_H_INCLUDED

#include <math.h>
#include <limits.h>
#include <SFML/Graphics.hpp>
#include <Eigen/Dense> // for assert()

namespace Colormap {

///////////////////////////////////////////////////////////////////////////////
class Colormap
{
public:
    // call operator() to get a scaled color
    virtual sf::Color operator()(const float& value, const float& min, const float& max) const;

protected:
    virtual float     calcValue (const float& value, const float& min, const float& max) const;

    virtual sf::Color getColor(const float& value) const;
    virtual sf::Color invalidValue() const { return sf::Color(255,0,255); } // for NaN, infinity, etc.

    inline bool isValid(const float& value) const { return std::isfinite(value); }
};
///////////////////////////////////////////////////////////////////////////////
class Logarithmic : public Colormap
{
protected:
    virtual float calcValue(const float& value, const float& min, const float& max) const;
};
///////////////////////////////////////////////////////////////////////////////
class RedToGreen : public Colormap
{
protected:
    virtual sf::Color getColor(const float& value) const;
};
///////////////////////////////////////////////////////////////////////////////
class Autumn : public Colormap
{
protected:
    virtual sf::Color getColor(const float& value) const;
};
///////////////////////////////////////////////////////////////////////////////
class Hot : public Colormap
{
protected:
    virtual sf::Color getColor(const float& value) const;
};
///////////////////////////////////////////////////////////////////////////////
class Jet : public Colormap
{
protected:
    virtual sf::Color getColor(const float& value) const;
    virtual sf::Color invalidValue() const { return sf::Color(20,20,20); } // for NaN, infinity, etc.
};
///////////////////////////////////////////////////////////////////////////////

} // namespace Colormap

#endif /* end of include guard: COLORMAP_H_INCLUDED */

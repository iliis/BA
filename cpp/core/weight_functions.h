#ifndef WEIGHT_FUNCTIONS_H_INCLUDED
#define WEIGHT_FUNCTIONS_H_INCLUDED

#include <Eigen/Dense>
#include <string.h>
#include <boost/lexical_cast.hpp>

///////////////////////////////////////////////////////////////////////////////
class ErrorWeightFunction
{
public:
    virtual ~ErrorWeightFunction() {};

    virtual float   operator()(float) const { return 1; }
    Eigen::VectorXf operator()(const Eigen::VectorXf& x) const;
    void apply(Eigen::VectorXf& x);

    virtual std::string toString() const { return "None"; }
};
///////////////////////////////////////////////////////////////////////////////
// just an explicit 'none' ;)
class ErrorWeightNone : public ErrorWeightFunction
{ };
///////////////////////////////////////////////////////////////////////////////
class ErrorWeightHuber : public ErrorWeightFunction
{
public:
    ErrorWeightHuber(float delta) : delta(delta) {};

    virtual float operator()(float x) const;

    virtual std::string toString() const { return "Huber(" + boost::lexical_cast<std::string>(delta) + ")"; }

    float delta;
};
///////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: WEIGHT_FUNCTIONS_H_INCLUDED */

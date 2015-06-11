#ifndef WEIGHT_FUNCTIONS_H_INCLUDED
#define WEIGHT_FUNCTIONS_H_INCLUDED

#include <Eigen/Dense>

///////////////////////////////////////////////////////////////////////////////
class ErrorWeightFunction
{
public:
    virtual float operator()(float x) const = 0;

    Eigen::VectorXf operator()(const Eigen::VectorXf& x) const;
    void apply(Eigen::VectorXf& x);
};
///////////////////////////////////////////////////////////////////////////////
class ErrorWeightNone : public ErrorWeightFunction
{
public:
    virtual float operator()(float x) const { return x; }
};
///////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: WEIGHT_FUNCTIONS_H_INCLUDED */

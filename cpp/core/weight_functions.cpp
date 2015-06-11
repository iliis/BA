#include "weight_functions.h"

///////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf ErrorWeightFunction::operator()(const Eigen::VectorXf& x) const
{
    Eigen::VectorXf out(x.size());

    for (unsigned int i = 0; i < x.size(); i++)
        out(i) = (*this)(x(i));

    return out;
}
///////////////////////////////////////////////////////////////////////////////
void ErrorWeightFunction::apply(Eigen::VectorXf& x)
{
    for (unsigned int i = 0; i < x.size(); i++)
        x(i) = (*this)(x(i));
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifndef CSV_H_INCLUDED
#define CSV_H_INCLUDED

#include <string>
#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

///////////////////////////////////////////////////////////////////////////////
inline std::string trim(std::string s)
{
    boost::algorithm::trim(s);
    return s;
}
///////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: CSV_H_INCLUDED */

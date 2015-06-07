#include "transformation.h"

using namespace std;
using namespace boost;

///////////////////////////////////////////////////////////////////////////////
Transformation::Transformation()
  : x(0), y(0), z(0),
    alpha(0), beta(0), gamma(0)
{
}
///////////////////////////////////////////////////////////////////////////////
Transformation::Transformation(float x, float y, float z, float alpha, float beta, float gamma)
  : x(x), y(y), z(z),
    alpha(alpha), beta(beta), gamma(gamma)
{
}
///////////////////////////////////////////////////////////////////////////////
std::ostream& operator <<(std::ostream &output, const Transformation &T)
{
    output << "T[ " << T.x << " " << T.y << " " << T.z << " / ";
    output << T.alpha << " " << T.beta << " " << T.gamma << " ]";
    return output;
}
///////////////////////////////////////////////////////////////////////////////
std::vector<Transformation> Transformation::loadFromCSV(const std::string& filename)
{
    std::vector<Transformation> transformations;

    // read file
    ///////////////////////////////////

    ifstream inputfile(filename.c_str(), ifstream::in);

    if (!inputfile.is_open()) {
        // TODO: throw error 404
        cerr << "could not open CSV file '" << filename << "'" << endl;
        return transformations;
    }


    // parse data
    ///////////////////////////////////
    string line;

    // skip firs line (header)
    getline(inputfile, line);

    while (getline(inputfile, line)) {
        assert(!line.empty());

        tokenizer< escaped_list_separator<char> > tok(line);

        tokenizer<escaped_list_separator<char> >::iterator it = tok.begin();

        Transformation T;
        T.x     = lexical_cast<float>(trim(*it++));
        T.y     = lexical_cast<float>(trim(*it++));
        T.z     = lexical_cast<float>(trim(*it++));
        T.alpha = lexical_cast<float>(trim(*it++));
        T.beta  = lexical_cast<float>(trim(*it++));
        T.gamma = lexical_cast<float>(trim(*it++));

        transformations.push_back(T);
    }

    inputfile.close();

    return transformations;
}
///////////////////////////////////////////////////////////////////////////////

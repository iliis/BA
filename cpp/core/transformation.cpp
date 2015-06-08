#include "transformation.h"

using namespace std;
using namespace boost;

///////////////////////////////////////////////////////////////////////////////
Transformation::Transformation()
{
    this->value << 0,0,0,0,0,0;
}
///////////////////////////////////////////////////////////////////////////////
Transformation::Transformation(float x, float y, float z, float alpha, float beta, float gamma)
{
    this->value << x, y, z, alpha, beta, gamma;
    updateRotationMatrix();
}
///////////////////////////////////////////////////////////////////////////////
std::ostream& operator <<(std::ostream &output, const Transformation &T)
{
    output << "T[ " << T.x() << " " << T.y() << " " << T.z() << " / ";
    output << T.alpha() << " " << T.beta() << " " << T.gamma() << " ]";
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

        T.value(0) = lexical_cast<float>(trim(*it++));
        T.value(1) = lexical_cast<float>(trim(*it++));
        T.value(2) = lexical_cast<float>(trim(*it++));
        T.value(3) = lexical_cast<float>(trim(*it++));
        T.value(4) = lexical_cast<float>(trim(*it++));
        T.value(5) = lexical_cast<float>(trim(*it++));
        T.updateRotationMatrix();

        transformations.push_back(T);
    }

    inputfile.close();

    return transformations;
}
///////////////////////////////////////////////////////////////////////////////
void Transformation::updateRotationMatrix()
{
    R = Eigen::AngleAxisf(this->alpha(), Eigen::Vector3f::UnitX())
      * Eigen::AngleAxisf(this->beta(),  Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(this->gamma(), Eigen::Vector3f::UnitZ());
}
///////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f Transformation::operator()(const Eigen::Vector3f& vect) const
{
    return this->getRotationMatrix() * vect + this->getTranslation();
}
///////////////////////////////////////////////////////////////////////////////

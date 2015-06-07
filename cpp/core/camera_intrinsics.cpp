#include "camera_intrinsics.h"

/*
 * example CSV file:
 *
 * focal length,focal length mm,image width,image height,sensor width mm,sensor height mm,clip start,clip end,color depth,depth depth
 * 280.0,35.0,256.0,128.0,32.0,18.0,0.10000000149011612,100.0,255,255
 *
 */

using namespace std;
using namespace boost;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void CameraIntrinsics::loadFromCSV(const std::string& filename)
{
    // read file
    ///////////////////////////////////

    ifstream inputfile(filename.c_str(), ifstream::in);

    if (!inputfile.is_open()) {
        // TODO: throw error 404
        cerr << "could not open CSV file '" << filename << "'" << endl;
        return;
    }

    // get second line with data
    string line;
    getline(inputfile, line);
    getline(inputfile, line);
    inputfile.close();

    assert(!line.empty());

    // parse data
    ///////////////////////////////////

    tokenizer< escaped_list_separator<char> > tok(line);

    tokenizer<escaped_list_separator<char> >::iterator it = tok.begin();

    this->focal_length  = lexical_cast<float>(trim(*it++));
    ++it; // skip focal length in mm
    this->camera_width  = lexical_cast<float>(trim(*it++)); // actually an uint, but Matlab/Blender writes this as float
    this->camera_height = lexical_cast<float>(trim(*it++));
    ++it; // skip sensor height
    ++it; // skip sensor width
    this->near_clipping = lexical_cast<float>(trim(*it++));
    this->far_clipping  = lexical_cast<float>(trim(*it++));
    // skip rest of csv

    // calculate other parameters
    ///////////////////////////////////

    // [0, 0]     is at the center of the first pixel at the top left
    // [W-1, H-1] is at the center of the last pixel at the bottom right
    // i.e. integer coordinates directly corespond to pixel values
    this->principal_point_x = this->camera_width  / 2 - 0.5;
    this->principal_point_y = this->camera_height / 2 - 0.5;

    // print loaded values for easier debugging
    ///////////////////////////////////

    cout << "loaded camera intrinsics:" << endl;
    cout << "focal_length = " << this->focal_length << endl;
    cout << "width = " << this->camera_width << "  height = " << this->camera_height << endl;
    cout << "principal point = " << this->principal_point_x << " " << this->principal_point_y << endl;
    cout << "clipping = " << this->near_clipping << " to " << this->far_clipping << endl;
}
///////////////////////////////////////////////////////////////////////////////

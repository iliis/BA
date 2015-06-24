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
CameraIntrinsics::CameraIntrinsics()
  : camera_width(0), camera_height(0),
    principal_point_x(0), principal_point_y(0),
    focal_length(0), baseline(0), is_disparity(false),
    near_clipping(0), far_clipping(0)
{
}
///////////////////////////////////////////////////////////////////////////////
CameraIntrinsics::CameraIntrinsics(unsigned int W, unsigned int H, float focal, float near, float far)
  : camera_width(W), camera_height(H),
    focal_length(focal), baseline(0), is_disparity(false),
    near_clipping(near), far_clipping(far)
{
    this->updatePrincipalPoint();
}
///////////////////////////////////////////////////////////////////////////////
CameraIntrinsics::CameraIntrinsics(const Eigen::Vector2f& camera_size, const Eigen::Vector2f& principal_point, float focal_length, float baseline)
  : camera_width(camera_size.x()), camera_height(camera_size.y()),
    principal_point_x(principal_point.x()), principal_point_y(principal_point.y()),
    focal_length(focal_length), baseline(baseline), is_disparity(true),
    near_clipping(0), far_clipping(0)
{
}
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

    updatePrincipalPoint();
}
///////////////////////////////////////////////////////////////////////////////
void CameraIntrinsics::updatePrincipalPoint()
{
    // [0, 0]     is at the center of the first pixel at the top left
    // [W-1, H-1] is at the center of the last pixel at the bottom right
    // i.e. integer coordinates directly corespond to pixel values
    this->principal_point_x = this->camera_width  / 2.0f - 0.5;
    this->principal_point_y = this->camera_height / 2.0f- 0.5;
}
///////////////////////////////////////////////////////////////////////////////
void CameraIntrinsics::downsample2()
{
    this->camera_width  /= 2;
    this->camera_height /= 2;
    this->focal_length  /= 2;
    this->updatePrincipalPoint();
}
///////////////////////////////////////////////////////////////////////////////
std::ostream& operator <<(std::ostream &output, const CameraIntrinsics &intrinsics)
{
    cout << "focal length = " << intrinsics.focal_length << endl;
    cout << "width = " << intrinsics.camera_width << "  height = " << intrinsics.camera_height << endl;
    cout << "principal point = " << intrinsics.principal_point_x << " " << intrinsics.principal_point_y << endl;
    cout << "clipping = " << intrinsics.near_clipping << " to " << intrinsics.far_clipping << endl;

    return output;
}
///////////////////////////////////////////////////////////////////////////////

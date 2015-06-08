#ifndef CAMERA_INTRINSICS_H_INCLUDED
#define CAMERA_INTRINSICS_H_INCLUDED

#include <Eigen/Dense>

#include "../utils/csv.h"

class CameraIntrinsics
{
public:
    void loadFromCSV(const std::string& filename);

    inline unsigned int getCameraWidth()  const { return camera_width; }
    inline unsigned int getCameraHeight() const { return camera_height; }
    inline Eigen::Vector2f getCameraSize() const { return Eigen::Vector2f(camera_width, camera_height); }
    inline float getPrincipalPointX()     const { return principal_point_x; }
    inline float getPrincipalPointY()     const { return principal_point_y; }
    inline Eigen::Vector2f getPrincipalPoint() const { return Eigen::Vector2f(principal_point_x, principal_point_y); }
    inline float getFocalLength()         const { return focal_length; }
    inline float getNearClipping()        const { return near_clipping; }
    inline float getFarClipping()         const { return far_clipping; }

    friend std::ostream& operator <<(std::ostream &output, const CameraIntrinsics &intrinsics);

private:
    // TODO: use Eigen::Vector2f to store these
    unsigned int camera_width, camera_height;   // size of image [pixels]
    float principal_point_x, principal_point_y; // center of image [pixels]
    float focal_length;                         // [1/pixels or meter/pixels]

    // optional parameters
    float near_clipping, far_clipping; // for mapping raw depth values to 'real' values (meters)
};

#endif /* end of include guard: CAMERA_INTRINSICS_H_INCLUDED */

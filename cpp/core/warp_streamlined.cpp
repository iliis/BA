#include "warp_streamlined.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
float WarpStreamlined::calcError(
        const Eigen::MatrixXf& keyframe_intensities,
        const Eigen::MatrixXf& current_intensities,
        const Eigen::MatrixXf& current_depths,
        const Eigen::Matrix<float, 6, 1>& T,
        const CameraIntrinsics& intrinsics,
        Eigen::VectorXf& error_out,
        Eigen::Matrix<float, Eigen::Dynamic, 6>& J_out,
        const float step_scale,
        const Warp::Parameters& params)
{
	GlobalTiming::warping.Start();

    const unsigned int W = intrinsics.getCameraWidth();
    const unsigned int H = intrinsics.getCameraHeight();


    if (IS_INVALID(T(0))
     || IS_INVALID(T(1))
     || IS_INVALID(T(2))
     || IS_INVALID(T(3))
     || IS_INVALID(T(4))
     || IS_INVALID(T(5))) {
    	cerr << "ERROR: invalid transformation passed to calcError: " << T.transpose() << endl;
    	return -1;
    }

    GlobalTiming::warping_rotation.Start();

    const float cA = cos(T(3)), sA = sin(T(3));
    const float cB = cos(T(4)), sB = sin(T(4));
    const float cC = cos(T(5)), sC = sin(T(5));

    ///Matrix3f R = T.getRotationMatrix();
    Matrix3f Rx; Rx <<
        1, 0, 0,
        0, cA, -sA,
        0, sA,  cA;

    Matrix3f Ry; Ry <<
        cB, 0, sB,
        0, 1, 0,
        -sB, 0, cB;

    Matrix3f Rz; Rz <<
        cC, -sC, 0,
        sC,  cC, 0,
        0, 0, 1;

    const Matrix3f R = Rx * Ry * Rz;

    GlobalTiming::warping_rotation.Stop();


    if (params.filter_on_unwarped_gradient) {
        cerr << "ERROR: filtering on unwarped gradient is not implemented in streamlined warping." << endl;
    }

    if (!intrinsics.isDisparityCamera()) {
        cerr << "ERROR: only disparity depth data is handled by streamlined warping code." << endl;
    }

    // amount of successfully warped pixels
    unsigned int pixel_count = 0;

    //float total_error = 0;

    GlobalTiming::warping_jacobian_init.Start();

    // TODO: this might be costly!
    error_out.resize(W*H);
    J_out.resize(W*H, 6);

    GlobalTiming::warping_jacobian_init.Stop();

    const float s = pow(2,step_scale);

    const unsigned int X_START = params.cutout_left / s;
    const unsigned int X_END   = W - (params.cutout_right / s);

    const unsigned int Y_START = params.cutout_top / s;
    const unsigned int Y_END   = H - (params.cutout_bottom / s);

    for (unsigned int y = Y_START; y < Y_END; ++y) {
        for (unsigned int x = X_START; x < X_END; ++x) {

            const float current_intensity = current_intensities(y,x);

            if (IS_INVALID(current_intensity))
                continue;

            const float current_depth = current_depths(y,x);

            if (IS_INVALID(current_depth) || current_depth <= 0 )
                continue;

            // warp point
            ///////////////////////////////////////////////////////////////////

            GlobalTiming::warping_warping.Start();

            const Vector3f world_point(
                    (x - intrinsics.getPrincipalPointX()) * intrinsics.getBaseline() / current_depth,
                    (y - intrinsics.getPrincipalPointY()) * intrinsics.getBaseline() / current_depth,
                    intrinsics.getBaseline() * intrinsics.getFocalLength() / current_depth
                );


            const Vector3f world_point_transformed = R * world_point + T.block<3,1>(0,0);

            // point in keyframe image
            const float pkf_x = world_point_transformed.x() / world_point_transformed.z() * intrinsics.getFocalLength() + intrinsics.getPrincipalPointX();
            const float pkf_y = world_point_transformed.y() / world_point_transformed.z() * intrinsics.getFocalLength() + intrinsics.getPrincipalPointY();


            const float pkf_x_floor = floor(pkf_x);
            const float pkf_y_floor = floor(pkf_y);

            GlobalTiming::warping_warping.Stop();

            // skip pixels that project outside the keyframe
            if (pkf_x < 0 || pkf_y < 0 || pkf_x_floor > W-2 || pkf_y_floor > H-2)
                continue;


            // sample image and image differential
            ///////////////////////////////////////////////////////////////////

            GlobalTiming::warping_sampling.Start();

            // remember: data[row, col] = data[y, x]!
            const float v1 = keyframe_intensities(pkf_y_floor,   pkf_x_floor);
            const float v2 = keyframe_intensities(pkf_y_floor,   pkf_x_floor+1);
            const float v3 = keyframe_intensities(pkf_y_floor+1, pkf_x_floor);
            const float v4 = keyframe_intensities(pkf_y_floor+1, pkf_x_floor+1);

            // skip pixels which have invalid gradients
            if (IS_INVALID(v1) || IS_INVALID(v2) || IS_INVALID(v3) || IS_INVALID(v4)) {
            	GlobalTiming::warping_sampling.Stop();
                continue;
            }

            // weight (how close to img[ceil(x), ceil(y)] are we?
            const float sx = pkf_x - pkf_x_floor;
            const float sy = pkf_y - pkf_y_floor;

            const float dx1 = v2 - v1,  dy1 = v3 - v1;
            const float dx2 = v4 - v3,  dy2 = v4 - v2;

            // weighted average over pixel intensities
			const float keyframe_intensity =
				   v1 * ( (1-sx) * (1-sy) )
				 + v2 * (    sx  * (1-sy) )
				 + v3 * ( (1-sx) *    sy  )
				 + v4 * (    sx  *    sy  );

			GlobalTiming::warping_sampling.Stop();

            // calculate Jacobians
            ///////////////////////////////////////////////////////////////////

            GlobalTiming::warping_jacobians.Start();

            const Eigen::Matrix<float, 1, 2> J_I(
                    dx1 * (1-sy)  +  dx2 * sy,
                    dy1 * (1-sx)  +  dy2 * sx
                );

            // skip pixels that don't provide a good gradient
            if (!params.filter_on_unwarped_gradient) {
                const float J_norm = abs(J_I(0)) + abs(J_I(1));
                if (J_norm < params.gradient_norm_threshold) {
                	GlobalTiming::warping_jacobians.Stop();
                    continue;
                }
            }

            const float& w_x = world_point.x();
            const float& w_y = world_point.y();
            const float& w_z = world_point.z();

            Eigen::Matrix<float, 3, 6> J_T; J_T <<
                1, 0, 0, 0, w_z*cB - w_x*cC*sB + w_y*sB*sC, - w_y*cB*cC - w_x*cB*sC,
                0, 1, 0, - w_x*(sA*sC - cA*cC*sB) - w_y*(cC*sA + cA*sB*sC) - w_z*cA*cB, w_z*sA*sB + w_x*cB*cC*sA - w_y*cB*sA*sC, w_x*(cA*cC - sA*sB*sC) - w_y*(cA*sC + cC*sA*sB),
                0, 0, 1,   w_x*(cA*sC + cC*sA*sB) + w_y*(cA*cC - sA*sB*sC) - w_z*cB*sA, w_y*cA*cB*sC - w_z*cA*sB - w_x*cA*cB*cC, w_x*(cC*sA + cA*sB*sC) - w_y*(sA*sC - cA*cC*sB);

            const float& F = intrinsics.getFocalLength();

            Eigen::Matrix<float, 2, 3> J_P; J_P <<
                F/w_z,   0, -(F*w_x)/(w_z*w_z),
                0,   F/w_z, -(F*w_y)/(w_z*w_z);


            J_out.row(pixel_count) = J_I * J_P * J_T;

            GlobalTiming::warping_jacobians.Stop();

            ///////////////////////////////////////////////////////////////////

            float error = keyframe_intensity - current_intensity;

            // store in output values
            error_out(pixel_count) = error;

            //total_error += error * error;

            ++pixel_count;
        }
    }

    // catch the border case where pixels are warped so that *none* actually falls on to the other image
    if (pixel_count == 0) {
        error_out.resize(1);
        J_out.resize(1, Eigen::NoChange);

        error_out.setZero();
        J_out.setZero();
    } else {
        error_out.conservativeResize(pixel_count);
        J_out.conservativeResize(pixel_count, Eigen::NoChange);
    }

    //cout << "total error: " << total_error << "  =  " << sqrt(total_error) << endl;

    GlobalTiming::warping.Stop();

    //return INVALID(); //sqrt(total_error);
    return pixel_count / ((float) (X_END-X_START)*(Y_END-Y_START));

}
///////////////////////////////////////////////////////////////////////////////

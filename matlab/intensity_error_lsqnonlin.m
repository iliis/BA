function [err, J] = intensity_error_lsqnonlin( D1, I1, I2, T_translation, T_rotation )
% input:  2 intensitiy and 1 depth images, T transformation that maps I2/D2 to I1/D1
% output: linear error vector (not summed and not squared)
%         Jacobian of error term for T_translation

global_parameters

H = size(D1, 1);
W = size(D1, 2);

Ta = T_rotation(1);
Tb = T_rotation(2);
Tc = T_rotation(3);

warped = warp_image(D1, I1, T_translation, T_rotation);

errs = (I2-warped);
%errs(isnan(warped)) = 0; % ignore empty pixels
errs(isnan(warped)) = NONMATCHED_PIXEL_PENALTY; % not yet tested, penalty for unmatched pixels

err = image_to_list(errs)';

%disp(['intensity error:  --> ', num2str(sum(err.^2))]);
%disp(['translation: ' num2str(T_translation)]);
%disp(['rotation:    ' num2str(T_rotation)]);

if nargout > 1
    
    %disp('ERROR: Jacobian not implemented!');
    
    % calculate derivatives at current 'position' (T)
    
    diff_I1x = diff(I1,1,1);
    diff_I1y = diff(I1,1,2);
    
    J = zeros(numel(err), 6);
    
    for v = 1:H
        for u = 1:W
            
            % index into J (resulting Jacobian)
            J_idx = (u-1)*H + v;
            
            % get points on camera sensor from pixel coordinates
            pu = (u - W/2) * CAMERA_WIDTH / W;
            pv = (v - H/2) * CAMERA_WIDTH / W;
            
            % project points on camera into 3D space
            x = D1(v,u) * pu / CAMERA_FOCAL;
            y = D1(v,u) * pv / CAMERA_FOCAL;
            z = D1(v,u);
            
            % innermost jacobian for transformation operator
            J_T = ...
            [ 1, 0, 0,   y*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) + z*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)), z*cos(Ta)*cos(Tb)*cos(Tc) - x*cos(Tc)*sin(Tb) + y*cos(Tb)*cos(Tc)*sin(Ta), z*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) - y*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)) - x*cos(Tb)*sin(Tc); ...
              0, 1, 0, - y*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) - z*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)), z*cos(Ta)*cos(Tb)*sin(Tc) - x*sin(Tb)*sin(Tc) + y*cos(Tb)*sin(Ta)*sin(Tc), z*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - y*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)) + x*cos(Tb)*cos(Tc); ...
              0, 0, 1,                                                           y*cos(Ta)*cos(Tb) - z*cos(Tb)*sin(Ta),                       - x*cos(Tb) - z*cos(Ta)*sin(Tb) - y*sin(Ta)*sin(Tb),                                                                                                                 0];

                   
            % apply transformation to 3D points
            Tpoint = angle2dcm(T_rotation) * [x y z]' + T_translation';
            x = Tpoint(1); y = Tpoint(2); z = Tpoint(3);
            
            % jacobian for transformation back onto camera sensor
            J_pi = ...
                [ CAMERA_FOCAL/z,       0, -(CAMERA_FOCAL*x)/z^2; ...
                        0, CAMERA_FOCAL/z, -(CAMERA_FOCAL*y)/z^2];
                    
            % and back onto pixel coordinates
            J_pi = J_pi * W / CAMERA_WIDTH;
            
            pu = int32(x * CAMERA_FOCAL/z * W/CAMERA_WIDTH + W/2);
            pv = int32(y * CAMERA_FOCAL/z * W/CAMERA_WIDTH + H/2);
            
            if (pu < 1 || pv < 1 || pu > W-1 || pv > H-1)
                % pixel is out of range of our image (or the differential of it)
                
                J(J_idx, :) = [0 0 0 0 0 0];
                
                continue;
            end
            
            % approximate Jacobian for intensity image
            J_I = [ diff_I1x(pv, pu) diff_I1y(pv, pu) ];
            
            
            % final Jacobian
            J(J_idx, :) = J_I * J_pi * J_T;
            
            
            
            %iu = (focal*(Tx + (u*D1(u, v))/focal))/(Tz + D1(u, v));
            %iv = (focal*(Ty + (v*D1(u, v))/focal))/(Tz + D1(u, v));
            
            % TODO: calculate actual differentials here
    
            % dx = -(focal*D([1], I1)((focal*(Tx + (u*D1(u, v))/focal))/(Tz + D1(u, v)), (focal*(Ty + (v*D1(u, v))/focal))/(Tz + D1(u, v))))/(Tz + D1(u, v))
            % dy = -(focal*D([2], I1) (px, py)  ) /(Tz + D1(u, v))
            % dz = (focal*(Tx + (u*D1(u, v))/focal)*D([1], I1)((focal*(Tx + (u*D1(u, v))/focal))/(Tz + D1(u, v)), (focal*(Ty + (v*D1(u, v))/focal))/(Tz + D1(u, v))))/(Tz + D1(u, v))^2 + (focal*(Ty + (v*D1(u, v))/focal)*D([2], I1)((focal*(Tx + (u*D1(u, v))/focal))/(Tz + D1(u, v)), (focal*(Ty + (v*D1(u, v))/focal))/(Tz + D1(u, v))))/(Tz + D1(u, v))^2
        end
    end
    
end

end


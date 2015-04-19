function J_T = jacobi_transformation( u, v, depth_uv, T )
% jacobian for transformation operator

global_parameters;

H = size(D1, 1);
W = size(D1, 2);

% get points on camera sensor from pixel coordinates
pu = (u - W/2) * CAMERA_WIDTH / W;
pv = (v - H/2) * CAMERA_WIDTH / W;

% project points on camera into 3D space
x = depth_uv * pu / CAMERA_FOCAL;
y = depth_uv * pv / CAMERA_FOCAL;
z = depth_uv;

Ta = T(4); Tb = T(5); Tc = T(6);

% innermost jacobian for transformation operator
J_T = ...
            [ 1, 0, 0,   y*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) + z*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)), z*cos(Ta)*cos(Tb)*cos(Tc) - x*cos(Tc)*sin(Tb) + y*cos(Tb)*cos(Tc)*sin(Ta), z*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) - y*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)) - x*cos(Tb)*sin(Tc); ...
              0, 1, 0, - y*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) - z*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)), z*cos(Ta)*cos(Tb)*sin(Tc) - x*sin(Tb)*sin(Tc) + y*cos(Tb)*sin(Ta)*sin(Tc), z*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - y*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)) + x*cos(Tb)*cos(Tc); ...
              0, 0, 1,                                                           y*cos(Ta)*cos(Tb) - z*cos(Tb)*sin(Ta),                       - x*cos(Tb) - z*cos(Ta)*sin(Tb) - y*sin(Ta)*sin(Tb),                                                                                                                 0];
end


function J_T = jacobi_transformation( x, depth_uv, T )
% jacobian for transformation operator T*pi^-1(x, D(x))
% output: J_T will be 3x6xN

global_parameters;

u = x(:, 1);
v = x(:, 2);

% get points on camera sensor from pixel coordinates
pu = (u - W/2) * CAMERA_WIDTH / W;
pv = (v - H/2) * CAMERA_WIDTH / W;

% project points on camera into 3D space
x = depth_uv .* pu / CAMERA_FOCAL;
y = depth_uv .* pv / CAMERA_FOCAL;
z = depth_uv;

Ta = T(4); Tb = T(5); Tc = T(6);

% innermost jacobian for transformation operator
% J_T = ...
% [ 1, 0, 0,   y*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) + z*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)), z*cos(Ta)*cos(Tb)*cos(Tc) - x*cos(Tc)*sin(Tb) + y*cos(Tb)*cos(Tc)*sin(Ta), z*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) - y*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)) - x*cos(Tb)*sin(Tc); ...
%   0, 1, 0, - y*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) - z*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)), z*cos(Ta)*cos(Tb)*sin(Tc) - x*sin(Tb)*sin(Tc) + y*cos(Tb)*sin(Ta)*sin(Tc), z*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - y*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)) + x*cos(Tb)*cos(Tc); ...
%   0, 0, 1,                                                           y*cos(Ta)*cos(Tb) - z*cos(Tb)*sin(Ta),                       - x*cos(Tb) - z*cos(Ta)*sin(Tb) - y*sin(Ta)*sin(Tb),                                                                                                                 0];

% innermost jacobian for transformation operator
J_T = ...
cat(3, ...
[   y*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) + z*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)), z*cos(Ta)*cos(Tb)*cos(Tc) - x*cos(Tc)*sin(Tb) + y*cos(Tb)*cos(Tc)*sin(Ta), z*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) - y*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)) - x*cos(Tb)*sin(Tc)], ...
[ - y*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) - z*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)), z*cos(Ta)*cos(Tb)*sin(Tc) - x*sin(Tb)*sin(Tc) + y*cos(Tb)*sin(Ta)*sin(Tc), z*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - y*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)) + x*cos(Tb)*cos(Tc)], ...
[                                                           y*cos(Ta)*cos(Tb) - z*cos(Tb)*sin(Ta),                       - x*cos(Tb) - z*cos(Ta)*sin(Tb) - y*sin(Ta)*sin(Tb),                                                                                                 zeros(numel(x),1)]);

% we wan't a 3x6xN matrix instead of Nx6x3
J_T = permute(J_T, [3 2 1]);

% the differential is constant for the movement terms
J_T = [repmat(eye(3),1,1,size(x,1)) J_T];


end
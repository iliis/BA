function J_T = jacobi_transformation_from_points( points, T )
% calculates transformation jacobi from list of N 3D points (Nx3)
% output: J_T will be 3x6xN

x = points(:,1);
y = points(:,2);
z = points(:,3);

Ta = T(4);
Tb = T(5);
Tc = T(6);

% innermost jacobian for transformation operator
J_T = ...
cat(3, ...
[   y*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) + z*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)), z*cos(Ta)*cos(Tb)*cos(Tc) - x*cos(Tc)*sin(Tb) + y*cos(Tb)*cos(Tc)*sin(Ta), z*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) - y*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)) - x*cos(Tb)*sin(Tc)], ...
[ - y*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) - z*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)), z*cos(Ta)*cos(Tb)*sin(Tc) - x*sin(Tb)*sin(Tc) + y*cos(Tb)*sin(Ta)*sin(Tc), z*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - y*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)) + x*cos(Tb)*cos(Tc)], ...
[                                                           y*cos(Ta)*cos(Tb) - z*cos(Tb)*sin(Ta),                       - x*cos(Tb) - z*cos(Ta)*sin(Tb) - y*sin(Ta)*sin(Tb),                                                                                                 zeros(numel(x),1)]);

% we wan't a 3x6xN matrix instead of Nx6x3
J_T = permute(J_T, [3 2 1]);

% the differential is constant for the movement terms
J_T = [repmat(eye(3),1,1,size(points,1)) J_T];

assert(size(J_T,1) == 3);
assert(size(J_T,2) == 6);
assert(size(J_T,3) == size(points,1));

end


function [DCM] = angle2dcm(angles)
% angles = [yaw pitch roll] = [gamma beta alpha]

% this uses 'ZYX' angles

assert(isequal(numel(angles), 3));

Rx = [ ...
    1 0 0; ...
    0 cos(angles(1)) -sin(angles(1)); ...
    0 sin(angles(1))  cos(angles(1))];

Ry = [ ...
    cos(angles(2)) 0 sin(angles(2)); ...
    0 1 0; ...
    -sin(angles(2)) 0 cos(angles(2))];

Rz = [ ...
    cos(angles(3)) -sin(angles(3)) 0; ...
    sin(angles(3))  cos(angles(3)) 0; ...
    0 0 1];

% from Wikipedia:
%DCM = Rz * Ry * Rx;

% TODO: why does this give correct results?
% Thins that will affect the rotation matrix and which are not all well defined:
% - order of angles (a, b, c vs. c, b, a)
% - order of matrixes (Rx*Ry*Rz vs. Rz*Ry*Rx)
% - intrinsic vs. extrinsic rotation (DCM vs. DCM')
% - forward or backward transformation (k->c vs. c->k)
% At least this is compatible with my hand calculations and Blender's output
DCM = Rx * Ry * Rz;

end

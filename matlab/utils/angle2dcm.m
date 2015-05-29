function [DCM] = angle2dcm(angles)
% angles = [yaw pitch roll]

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

DCM = Rz * Ry * Rx;

end
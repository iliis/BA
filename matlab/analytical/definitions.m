% camera parameters
%focal = sym(0.035);
focal = sym('FOCAL');

% pixel coordinates
% center of image = origin, width of image = 'width' (0.032m)
syms u v I1(u, v) I2(u, v) D1(u, v) real

% parameters for camera transformation
% [x y z] = translation
% [a b c] = rotation (Euler angles)
syms Tx Ty Tz Ta Tb Tc real
T = [Tx Ty Tz Ta Tb Tc];

% inverse projection operator (image to 3D space)
proj_inv = D1(u, v) * [ u / focal; v / focal; 1 ];

% camera transformation
T_translation = [ Tx ; Ty ; Tz ];

Rx = [ ...
    1 0 0; ...
    0 cos(Ta) -sin(Ta); ...
    0 sin(Ta)  cos(Ta)];

Ry = [ ...
    cos(Tb) 0 sin(Tb); ...
    0 1 0; ...
    -sin(Tb) 0 cos(Tb)];

Rz = [ ...
    cos(Tc) -sin(Tc) 0; ...
    sin(Tc)  cos(Tc) 0; ...
    0 0 1];

% TODO: check order (shouldn't matter, as long as we keep it consistent)
T_rotation = Rz * Ry * Rx;

% apply transformation
transformed = T_rotation * proj_inv + T_translation;

% project back onto image
proj = [ transformed(1) * focal ; transformed(2) * focal ] / transformed(3);

% error term
residual = I2(u, v) - I1(proj(1), proj(2));
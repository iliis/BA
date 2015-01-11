%% calculate jacobi matrix
clear all;

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

% get final Jacobi matrix for error
J = simplify(jacobian(residual, [Tx Ty Tz Ta Tb Tc]))';

% result:
% [ -(focal*D([1], I1)((focal*(Tx + D1(u, v)*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - (v*D1(u, v)*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)))/focal + (u*cos(Tb)*cos(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal), (focal*(Ty - D1(u, v)*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) + (v*D1(u, v)*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)))/focal + (u*cos(Tb)*sin(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal)))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal), -(focal*D([2], I1)((focal*(Tx + D1(u, v)*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - (v*D1(u, v)*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)))/focal + (u*cos(Tb)*cos(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal), (focal*(Ty - D1(u, v)*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) + (v*D1(u, v)*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)))/focal + (u*cos(Tb)*sin(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal)))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal), (focal*D([1], I1)((focal*(Tx + D1(u, v)*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - (v*D1(u, v)*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)))/focal + (u*cos(Tb)*cos(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal), (focal*(Ty - D1(u, v)*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) + (v*D1(u, v)*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)))/focal + (u*cos(Tb)*sin(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal))*(Tx + D1(u, v)*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - (v*D1(u, v)*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)))/focal + (u*cos(Tb)*cos(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal)^2 + (focal*D([2], I1)((focal*(Tx + D1(u, v)*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - (v*D1(u, v)*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)))/focal + (u*cos(Tb)*cos(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal), (focal*(Ty - D1(u, v)*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) + (v*D1(u, v)*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)))/focal + (u*cos(Tb)*sin(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal))*(Ty - D1(u, v)*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) + (v*D1(u, v)*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)))/focal + (u*cos(Tb)*sin(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal)^2, ((focal*(D1(u, v)*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)) + (v*D1(u, v)*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal) - (focal*(cos(Tb)*sin(Ta)*D1(u, v) - (v*cos(Ta)*cos(Tb)*D1(u, v))/focal)*(Ty - D1(u, v)*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) + (v*D1(u, v)*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)))/focal + (u*cos(Tb)*sin(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal)^2)*D([2], I1)((focal*(Tx + D1(u, v)*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - (v*D1(u, v)*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)))/focal + (u*cos(Tb)*cos(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal), (focal*(Ty - D1(u, v)*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) + (v*D1(u, v)*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)))/focal + (u*cos(Tb)*sin(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal)) - D([1], I1)((focal*(Tx + D1(u, v)*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - (v*D1(u, v)*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)))/focal + (u*cos(Tb)*cos(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal), (focal*(Ty - D1(u, v)*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) + (v*D1(u, v)*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)))/focal + (u*cos(Tb)*sin(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal))*((focal*(D1(u, v)*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)) + (v*D1(u, v)*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal) + (focal*(cos(Tb)*sin(Ta)*D1(u, v) - (v*cos(Ta)*cos(Tb)*D1(u, v))/focal)*(Tx + D1(u, v)*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - (v*D1(u, v)*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)))/focal + (u*cos(Tb)*cos(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal)^2), - D([2], I1)((focal*(Tx + D1(u, v)*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - (v*D1(u, v)*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)))/focal + (u*cos(Tb)*cos(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal), (focal*(Ty - D1(u, v)*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) + (v*D1(u, v)*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)))/focal + (u*cos(Tb)*sin(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal))*((focal*(cos(Ta)*cos(Tb)*sin(Tc)*D1(u, v) - (u*sin(Tb)*sin(Tc)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*sin(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal) + (focal*(cos(Ta)*sin(Tb)*D1(u, v) + (u*cos(Tb)*D1(u, v))/focal + (v*sin(Ta)*sin(Tb)*D1(u, v))/focal)*(Ty - D1(u, v)*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) + (v*D1(u, v)*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)))/focal + (u*cos(Tb)*sin(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal)^2) - D([1], I1)((focal*(Tx + D1(u, v)*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - (v*D1(u, v)*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)))/focal + (u*cos(Tb)*cos(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal), (focal*(Ty - D1(u, v)*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) + (v*D1(u, v)*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)))/focal + (u*cos(Tb)*sin(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal))*((focal*(cos(Ta)*cos(Tb)*cos(Tc)*D1(u, v) - (u*cos(Tc)*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*cos(Tc)*sin(Ta)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal) + (focal*(cos(Ta)*sin(Tb)*D1(u, v) + (u*cos(Tb)*D1(u, v))/focal + (v*sin(Ta)*sin(Tb)*D1(u, v))/focal)*(Tx + D1(u, v)*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - (v*D1(u, v)*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)))/focal + (u*cos(Tb)*cos(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal)^2), (focal*D([1], I1)((focal*(Tx + D1(u, v)*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - (v*D1(u, v)*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)))/focal + (u*cos(Tb)*cos(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal), (focal*(Ty - D1(u, v)*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) + (v*D1(u, v)*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)))/focal + (u*cos(Tb)*sin(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal))*((v*D1(u, v)*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)))/focal - D1(u, v)*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) + (u*cos(Tb)*sin(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal) - (focal*D([2], I1)((focal*(Tx + D1(u, v)*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - (v*D1(u, v)*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)))/focal + (u*cos(Tb)*cos(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal), (focal*(Ty - D1(u, v)*(cos(Tc)*sin(Ta) - cos(Ta)*sin(Tb)*sin(Tc)) + (v*D1(u, v)*(cos(Ta)*cos(Tc) + sin(Ta)*sin(Tb)*sin(Tc)))/focal + (u*cos(Tb)*sin(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal))*(D1(u, v)*(sin(Ta)*sin(Tc) + cos(Ta)*cos(Tc)*sin(Tb)) - (v*D1(u, v)*(cos(Ta)*sin(Tc) - cos(Tc)*sin(Ta)*sin(Tb)))/focal + (u*cos(Tb)*cos(Tc)*D1(u, v))/focal))/(Tz + cos(Ta)*cos(Tb)*D1(u, v) - (u*sin(Tb)*D1(u, v))/focal + (v*cos(Tb)*sin(Ta)*D1(u, v))/focal)]


%% Jacobians of individual functions

syms x y z real

J_T = jacobian(T_rotation * [x y z]' + T_translation, [Tx Ty Tz Ta Tb Tc]);

J_pi  = jacobian([x * focal / z, y * focal / z], [x y z]);

syms W H width real
J_pi2 = jacobian([x * focal / z, y * focal / z]*W/width+0.5*[W H], [x y z]);


%% derive Jacobian for translation only
transf_transl   = proj_inv + T_translation;
proj_transl     = [ transf_transl(1) * focal ; transf_transl(2) * focal ] / transf_transl(3);
residual_transl = I2(u, v) - I1(proj_transl(1), proj_transl(2));

J_translation = jacobian(residual_transl, [Tx Ty Tz]);

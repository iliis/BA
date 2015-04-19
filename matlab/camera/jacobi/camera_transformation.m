function point = camera_transformation( x, depth_uv, T )
% p = [x,y,z] = T*pi(x = [u,v])

global_parameters;

u = x(1);
v = x(2);

% get points on camera sensor from pixel coordinates
pu = (u - W/2) * CAMERA_WIDTH / W;
pv = (v - H/2) * CAMERA_WIDTH / W;

% project points on camera into 3D space
x = depth_uv * pu / CAMERA_FOCAL;
y = depth_uv * pv / CAMERA_FOCAL;
z = depth_uv;

T_translation = T(1:3);
T_rotation    = T(4:6);

point = (angle2dcm(T_rotation) * [x y z]')' + T_translation;

end


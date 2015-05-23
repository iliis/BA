function T_inv = reverse_transformation( T )
% reverses camera transformation T

assert(numel(T) == 6);

rotation    = angle2dcm(T(4:6));
translation = T(1:3);

translation_new = -rotation' * translation;

assert(false, 'TODO: implement this when Euler angles are properly defined. And write tests.');

end
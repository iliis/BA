function [translation_rev, rotation_rev] = reverse_transformation(translation, rotation)

rotation_rev = -rotation;
translation_rev = - translation * angle2dcm(rotation_rev);

end
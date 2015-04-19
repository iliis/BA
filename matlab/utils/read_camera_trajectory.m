function trajectory_data = read_camera_trajectory( input_path )
%READ_CAMERA_TRAJECTORY loads 'camera_trajectory.csv' and returns it as matrix

% TODO: there's something wrong with blender->matlab conversion
% maybe it's just the normalize_trajectory() function, but test_image_warp
% seems a bit off. probably something with the order of the Euler angles.
trajectory_data = csvread(fullfile(input_path, 'camera_trajectory.csv'), 1, 0);

end


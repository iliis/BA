function [ normalized_trajectory ] = normalize_trajectory( trajectory )
% modifies trajectory so that initial position is at 0,0,0 with orientation 0,0,0

init_pos = trajectory(1,:);
R = angle2dcm(init_pos(4:6));

normalized_trajectory = zeros(size(trajectory));
for i = 2:size(normalized_trajectory,1)
    
    pos = trajectory(i,1:3) - init_pos(1:3);
    rot = angle2dcm(trajectory(i,4:6));
    
    normalized_trajectory(i,1:3) = R*pos';
    
     % TODO: this is brute force code and NOT based on any coherent angle representation
    %[r1, r2, r3] = dcm2angle(R*rot', 'XYZ');
    %normalized_trajectory(i,4:6) = [r1 r2 r3];
    
    % instead, just ignore initial rotation :P
    normalized_trajectory(i,4:6) = trajectory(i,4:6);
end

end


classdef SceneStep
    % a pair of frames, a single movement step
    
    properties
        % keyframe and current images
        I1, I2 % intensities
        D1, D2 % depth values
        
        intrinsics
        ground_truth
    end
    
    methods
    end
    
end


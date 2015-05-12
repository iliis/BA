classdef CameraIntrinsics
    %CAMERA_INTRINSICS basic parameters of camera
    
    properties
        camera_width    % width of image  [pixels]
        camera_height   % height of image [pixels]
        principal_point % [u,v] center of image [pixels]
        focal_length    % [TODO:units?]
    end
    
    methods
        function obj = CameraIntrinsics(width, height, focal)
            assert(width > 0);
            assert(height > 0);
            assert(focal > 0);
            
            obj.camera_width    = width;
            obj.camera_height   = height;
            obj.focal_length    = focal;
            obj.principal_point = [width/2 height/2];
        end
    end
    
end


classdef CameraIntrinsics
    %CAMERA_INTRINSICS basic parameters of camera
    
    properties
        camera_width    % width of image  [pixels]
        camera_height   % height of image [pixels]
        principal_point % [u,v] center of image [pixels]
        focal_length    % [1/pixels or meter/pixels]
    end
    
    methods
        function obj = CameraIntrinsics(width, height, focal)
            assert(width > 0);
            assert(height > 0);
            assert(focal > 0);
            
            obj.camera_width    = width;
            obj.camera_height   = height;
            obj.focal_length    = focal;
            
            % [1,1] is at center of first pixel at the top left
            % [W,H] is at center of last pixel at the bottom right
            % i.e. integer coordinates directly corespond to pixel values
            %      (this is the same grid as interp2() uses)
            obj.principal_point = [width/2+0.5 height/2+0.5];
        end
    end
    
end


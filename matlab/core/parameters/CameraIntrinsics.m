classdef CameraIntrinsics
    %CAMERA_INTRINSICS basic parameters of camera
    
    properties
        camera_width    % width of image  [pixels]
        camera_height   % height of image [pixels]
        principal_point % [u,v]' center of image [pixels]
        focal_length    % [1/pixels or meter/pixels]
        baseline % NOT REALLY USED

        % optional properties
        % TODO: use these values in load_image()
        intensity_img_depth, depth_img_depth % for reading raw images and converting them to doubles in range [0, 1]
        near_clipping, far_clipping % for mapping raw depth values to 'real' values
    end
    
    methods
        function obj = CameraIntrinsics(width, height, focal)
            assert(width > 0);
            assert(height > 0);
            assert(focal > 0);
            
            assert(all(size(width)  == [1,1]));
            assert(all(size(height) == [1,1]));
            assert(all(size(focal)  == [1,1]));
            
            obj.camera_width    = width;
            obj.camera_height   = height;
            obj.focal_length    = focal;
            
            % [1,1] is at center of first pixel at the top left
            % [W,H] is at center of last pixel at the bottom right
            % i.e. integer coordinates directly corespond to pixel values
            %      (this is the same grid as interp2() uses)
            obj.principal_point = [width/2+0.5 height/2+0.5]';

            % default values for Blender
            obj.intensity_img_depth = 255;
            obj.depth_img_depth     = 255;
            obj.near_clipping       = 0.1;
            obj.far_clipping        = 100;
        end
    end

    methods(Static)
        function obj = loadFromCSV(input_path)
            parameters = csvread(fullfile(input_path, 'camera_intrinsics.csv'), 1, 0);

            % focal length, focal length mm, image width, image height, sensor width mm, sensor height mm, clip_start, clip_end

            obj = CameraIntrinsics(parameters(3), parameters(4), parameters(1));
            obj.near_clipping = parameters(7);
            obj.far_clipping  = parameters(8);
            obj.intensity_img_depth = parameters(9);
            obj.depth_img_depth     = parameters(10);
        end
    end
    
end


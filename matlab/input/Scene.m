classdef Scene
    % represents a step between two frames
    % TODO: generalize for longer trajectories
    
    properties
        I1, I2 % intensity images
        D1, D2 % depth images
        intrinsics % camera parameters
        source_path % path to original images
        ground_truth % true transformation between frames
    end
    
    methods
        function obj = Scene( scene_path, scale )
            % loads a pair of images, including intrinsics and ground truth

            obj.source_path  = scene_path;
            obj.ground_truth = [-2 0 -4 0 0 0]'; % TODO: read this from camera_trajectory.csv
            obj.intrinsics = CameraIntrinsics.loadFromCSV(scene_path);
            
            obj.D1 = read_depth_image(scene_path, 1, obj.intrinsics);
            obj.I1 = read_intensity_image(scene_path, 1, obj.intrinsics);

            obj.D2 = read_depth_image(scene_path, 2, obj.intrinsics);
            obj.I2 = read_intensity_image(scene_path, 2, obj.intrinsics);
            
            [H, W] = size(obj.D1);
            assert(W == obj.intrinsics.camera_width,  'image must have same width as specified in camera intrinsics');
            assert(H == obj.intrinsics.camera_height, 'image must have same height as specified in camera intrinsics');
            assert(all(size(obj.I1) == [H,W]), 'images must all have the same size');
            assert(all(size(obj.I2) == [H,W]), 'images must all have the same size');
            assert(all(size(obj.D2) == [H,W]), 'images must all have the same size');


            if nargin > 1
                obj = scale_scene(obj, scale);
            end
        end
        
        function obj = scale_down( obj, scale_factor )
            % downscales images by factor of 1/2^scale

            assert(scale_factor >= 1, 'scale factor must be larger than 1, otherwise no scaling will occur');

            scale_factor = 1/2^(scale_factor-1);

            obj.D1 = imresize(obj.D1, scale_factor);
            obj.I1 = imresize(obj.I1, scale_factor);
            %C1 = imresize(C1, 1/2^(scale-1));

            obj.D2 = imresize(obj.D2, scale_factor);
            obj.I2 = imresize(obj.I2, scale_factor);
            %C2 = imresize(C2, 1/2^(scale-1));

            % don't scale focal length (TODO: is this correct?)
            obj.intrinsics = CameraIntrinsics(size(obj.I1,2), size(obj.I1,1), obj.intrinsics.focal_length);

        end
    end
    
end


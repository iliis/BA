classdef Scene
    % represents a trajectory, i.e. a full data set
    
    properties
        frame_count  % number of frames N
        step_count   % number of steps = N-1
        intensities  % N intensity images
        depths       % N depth images
        intrinsics   % camera parameters
        source_path  % path to original images
        ground_truth % true transformation between frames (first column gives absolute position and can be ignored)
    end
    
    methods
        function obj = Scene( scene_path, scale )
            % loads a pair of images, including intrinsics and ground truth
            
            obj.source_path  = scene_path;
            obj.ground_truth = csvread(fullfile(scene_path, 'camera_trajectory_relative.csv'), 1, 0)'; % we want column vectors!
            obj.intrinsics = CameraIntrinsics.loadFromCSV(scene_path);
            
            obj.frame_count = size(obj.ground_truth,2);
            obj.step_count  = obj.frame_count - 1;
            
            assert(size(obj.ground_truth,1) == 6, 'ground truth csv must have correct format (6 columns, 1 row per frame)');
            assert(size(obj.ground_truth,2) >  1, 'ground truth must containt at least two frames');
            
            obj.intensities = cell(obj.frame_count, 1);
            obj.depths      = cell(obj.frame_count, 1);
            
            for k = 1:obj.frame_count
                D = read_depth_image    (scene_path, k, obj.intrinsics);
                I = read_intensity_image(scene_path, k, obj.intrinsics);
            
                [H, W] = size(D);
                assert(W == obj.intrinsics.camera_width,  'image must have same width as specified in camera intrinsics');
                assert(H == obj.intrinsics.camera_height, 'image must have same height as specified in camera intrinsics');
                assert(all(size(I) == [H,W]), 'images must all have the same size');
                
                obj.intensities{k} = I;
                obj.depths{k}      = D;
            end

            if nargin > 1
                obj = scale_scene(obj, scale);
            end
            
            disp(['loaded scene with ' num2str(obj.frame_count) ' frames']);
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
        
        function step = getStep(obj, step_idx)
            % returns a SceneStep, representing a single movement step
            % first step is between first and second frame
            
            assert(step_idx >= 1, 'step index begins with one');
            assert(step_idx < obj.frame_count, 'max step index is frame_count-1 (fence post error)');
            
            step = SceneStep;
            step.I1 = obj.intensities{step_idx};
            step.D1 = obj.depths{step_idx};
            step.I2 = obj.intensities{step_idx+1};
            step.D2 = obj.depths{step_idx+1};
            step.intrinsics = obj.intrinsics;
            step.ground_truth = obj.ground_truth(:, step_idx+1);
        end
    end
    
end


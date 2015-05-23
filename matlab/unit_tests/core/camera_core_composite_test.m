classdef camera_core_composite_test < matlab.unittest.TestCase
% test core functions together
    
    properties
        intrinsics = CameraIntrinsics(7, 5, 1);
    end
    
    methods (Test)
        function test_no_transformation(tc)
            % generate some random pixels and depth values
            N = 10000;
            points = rand(2,N) * 100 - 50;
            depths = rand(1,N) * 100 - 50;
            
            % project into world
            points_world = camera_project_inverse(points, depths, tc.intrinsics);
            
            % transform using null transformation
            points_world_new = camera_transform(points_world, [0 0 0 0 0 0]');
            
            % and back onto image
            points_camera = camera_project(points_world_new, tc.intrinsics);
            
            % we should get the original points back (minus some numeric error)
            tc.assertEqual(points, points_camera, 'AbsTol', 1e-13);
        end
        
        function test_translation_only(tc)
            % TODO
            % settle direction of transformation first!
        end
        
        function test_rotation_only(tc)
            % TODO
            % settle direction of transformation first!
        end
        
        function test_full_transformation(tc)
            % TODO
            % settle direction of transformation first!
        end
    end
    
end


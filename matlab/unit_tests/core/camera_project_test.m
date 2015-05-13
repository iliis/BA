classdef camera_project_test < matlab.unittest.TestCase
    
    properties
        % these values shouldn't affect the test at all
        % (TODO: actually verify this!)
        intrinsics = CameraIntrinsics(100, 50, 1.5);
    end
    
    methods (Test)
        function test_proj_and_inv(tc)
            % generate some random pixels and depth values
            N = 10000;
            points = rand(N,2) * 100 - 50;
            depths = rand(N,1) * 100;
            
            % project into world
            points_world = camera_project_inverse(points, depths, tc.intrinsics);
            
            % and back onto image
            points_camera = camera_project(points_world, tc.intrinsics);
            
            % we should get the original points back (minus some numeric error)
            tc.assertEqual(points, points_camera, 'AbsTol', 1e-13);
        end
        
        function manual_tests(tc)
            ci = CameraIntrinsics(7, 5, 1);
            
            % make sure the principal point is where we expect him
            tc.assertEqual(ci.principal_point, [4 3]);
            
            % center pixel (should be equal to principal point)
            point_world = camera_project_inverse([4 3], 10, ci);
            tc.assertEqual(point_world, [0 0 -10]);
            
            % top left pixel
            point_world = camera_project_inverse([1 1], 10, ci);
            tc.assertEqual(point_world, [-30 -20 -10]);
            
            % top right pixel
            point_world = camera_project_inverse([7 1], 2, ci);
            tc.assertEqual(point_world, [6 -4 -2]);
        end
    end
    
end


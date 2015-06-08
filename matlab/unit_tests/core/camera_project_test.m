classdef camera_project_test < matlab.unittest.TestCase
% unit tests for core::camera_project() and core::camera_project_inverse()

    properties
        % these values shouldn't affect the test at all
        % (TODO: actually verify this!)
        intrinsics = CameraIntrinsics(100, 50, 1.5);
    end
    
    methods (Test)
        function test_proj_and_inv(tc)
            % generate some random pixels and depth values
            N = 10000;
            points = rand(2,N) * 100 - 50;
            depths = rand(1,N) * 100 - 50;
            
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
            tc.assertEqual(ci.principal_point, [4 3]');
            
            % center pixel (should be equal to principal point)
            point_world = camera_project_inverse([4 3]', 10, ci);
            tc.assertEqual(point_world, [0 0 10]');
            tc.assertEqual(camera_project(point_world, ci), [4 3]');
            
            % top left pixel
            point_world = camera_project_inverse([1 1]', 10, ci);
            tc.assertEqual(point_world, [-30 -20 10]');
            tc.assertEqual(camera_project(point_world, ci), [1 1]');
            
            % top right pixel
            point_world = camera_project_inverse([7 1]', 2, ci);
            tc.assertEqual(point_world, [6 -4 2]');
            tc.assertEqual(camera_project(point_world, ci), [7 1]');
            
            % bottom left pixel
            point_world = camera_project_inverse([1 5]', 10, ci);
            tc.assertEqual(point_world, [-30 20 10]');
            tc.assertEqual(camera_project(point_world, ci), [1 5]');
            
            % bottom right pixel
            point_world = camera_project_inverse([7 5]', 10, ci);
            tc.assertEqual(point_world, [30 20 10]');
            tc.assertEqual(camera_project(point_world, ci), [7 5]');
            
            % a bit right of center
            point_world = camera_project_inverse([5 3]', 10, ci);
            tc.assertEqual(point_world, [10 0 10]');
            tc.assertEqual(camera_project(point_world, ci), [5 3]');
            
            % a bit above center
            point_world = camera_project_inverse([4 2]', 10, ci);
            tc.assertEqual(point_world, [0 -10 10]');
            tc.assertEqual(camera_project(point_world, ci), [4 2]');
        end
    end
    
end


classdef camera_transformation_tests < matlab.unittest.TestCase
% test core::camera_transformation()
    
    properties
        intrinsics = CameraIntrinsics(7, 5, 1);
    end
    
    methods (Test)
        function test_no_transformation(tc)
            % generate some random points
            N = 10000;
            points = rand(3,N) * 100 - 50;
            
            % transform using null transformation
            points_new = camera_transform(points, [0 0 0 0 0 0]');
            
            % they should remain untouched
            tc.assertEqual(points, points_new);
        end
        
        function test_translation_only(tc)
            points_new = [ ...
                0 0 0; ...
                1 0 0; ...
                0 1 0; ...
                0 0 1; ...
                1 1 1; ...
                -1 0 1]';
            
            points = camera_transform(points_new, [1 2 3 0 0 0]);
            
            tc.assertEqual(points(:,1), [1 2 3]');
            tc.assertEqual(points(:,2), [2 2 3]');
            tc.assertEqual(points(:,3), [1 3 3]');
            tc.assertEqual(points(:,4), [1 2 4]');
            tc.assertEqual(points(:,5), [2 3 4]');
            tc.assertEqual(points(:,6), [0 2 4]');
        end
        
        function test_rotation_X90(tc)
            points_new = [ ...
                0 0 0; ...
                1 0 0; ...
                0 0 -1; ...
                0 1 0; ...
                1 1 -1; ...
                -1 1 0]';
            
            points = camera_transform(points_new, [0 0 0 deg2rad(90) 0 0]);
            
            tc.assertEqual(points(:,1), [0 0 0]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,2), [1 0 0]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,3), [0 1 0]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,4), [0 0 1]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,5), [1 1 1]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,6), [-1 0 1]', 'AbsTol', 1e-15);
        end
        
        function test_rotation_Y90(tc)
            points_new = [ ...
                0 0 0; ...
                0 0 1; ...
                0 1 0; ...
                -1 0 0; ...
                -1 1 1; ...
                -1 0 -1]';
            
            points = camera_transform(points_new, [0 0 0 0 deg2rad(90) 0]);
            
            tc.assertEqual(points(:,1), [0 0 0]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,2), [1 0 0]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,3), [0 1 0]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,4), [0 0 1]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,5), [1 1 1]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,6), [-1 0 1]', 'AbsTol', 1e-15);
        end
        
        function test_rotation_Z90(tc)
            points_new = [ ...
                0 0 0; ...
                0 -1 0; ...
                1 0 0; ...
                0 0 1; ...
                1 -1 1; ...
                0 1 1]';
            
            points = camera_transform(points_new, [0 0 0 0 0 deg2rad(90)]);
            
            tc.assertEqual(points(:,1), [0 0 0]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,2), [1 0 0]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,3), [0 1 0]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,4), [0 0 1]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,5), [1 1 1]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,6), [-1 0 1]', 'AbsTol', 1e-15);
        end
        
        function test_rotation_X90Y90(tc)
            points_new = [ ...
                0 0 0; ...
                0 0 1; ...
                1 0 0; ...
                0 1 0; ...
                1 1 1; ...
                0 1 -1]';
            
            points = camera_transform(points_new, [0 0 0 deg2rad(90) deg2rad(90) 0]);
            
            tc.assertEqual(points(:,1), [0 0 0]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,2), [1 0 0]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,3), [0 1 0]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,4), [0 0 1]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,5), [1 1 1]', 'AbsTol', 1e-15);
            tc.assertEqual(points(:,6), [-1 0 1]', 'AbsTol', 1e-15);
        end
        
        function test_full_transformation(tc)
            % TODO
        end
    end
    
end
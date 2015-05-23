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
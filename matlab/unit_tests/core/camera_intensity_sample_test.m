classdef camera_intensity_sample_test < matlab.unittest.TestCase
% unit tests for core::camera_intensity_sample()
    
    methods (Test)
        function test_intensity_sample(tc)
            img = [  1,  2,  3,  4,  5,  6; ...
                    10, 20, 30, 40, 50, 60; ...
                    20, 40, 60, 80,100, 10; ...
                     8,  1,  1,  1,  1,  9];
            
            % sample the four corners
            % [1,1]' is at bottom left
            samples = [ ...
                1, 4; ... % top left
                6, 4; ... % top right
                1, 1; ... % bottom left
                6, 1]';   % bottom right
      
            [intensities, gradients] = camera_intensity_sample(samples, img);
            
            tc.assertEqual(intensities, [1,6,8,9]);
            
            % remember:
            % gradients are zero outside the image range
            % image coordinates are vertically flipped (1,1 ist at bottom left)
            tc.assertEqual(gradients(:,:,1), [ 0.5   -4.5]);
            tc.assertEqual(gradients(:,:,2), [ 0.5  -27  ]);
            tc.assertEqual(gradients(:,:,3), [-3.5    6  ]);
            tc.assertEqual(gradients(:,:,4), [ 4      0.5]);
            
            % sample inside image
            samples = [ ...
                1.5, 3.5; ... % mean([1,2,10,20])   =  8.25
                2.5, 2.5; ... % mean([20,30,40,60]) = 37.5
                4.8,   4; ... % 4.8
                4.8,   3; ... % 48
                2,   2.2]';   % 36
            
            [intensities, gradients] = camera_intensity_sample(samples, img);
            tc.assertEqual(intensities, [8.25, 37.5, 4.8, 48, 36]);
            
            tc.assertEqual(gradients(:,:,1), [ 5.5  -13.5]);
            tc.assertEqual(gradients(:,:,2), [15    -25]);
            tc.assertEqual(gradients(:,:,3), [ 1    -21.6]);
            tc.assertEqual(gradients(:,:,4), [10    -45.6]);
            tc.assertEqual(gradients(:,:,5), [18     -2.3], 'AbsTol', 1e-13); % strangely, this value is quite inaccurate...
        end
    end
    
end


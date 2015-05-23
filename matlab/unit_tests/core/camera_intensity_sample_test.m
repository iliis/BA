classdef camera_intensity_sample_test < matlab.unittest.TestCase
% unit tests for core::camera_intensity_sample()
    
    methods (Test)
        function test_intensity_sample(tc)
            img = [  1,  2,  3,  4,  5,  6; ...
                    10, 20, 30, 40, 50, 60; ...
                    20, 40, 60, 80,100,120; ...
                     8,  1,  1,  1,  1,  9];
            
            % sample the four corners
            % [1,1]' is at bottom left
            samples = [ ...
                1, 4; ... % top left
                6, 4; ... % top right
                1, 1; ... % bottom left
                6, 1]';   % bottom right
      
            tc.assertEqual(camera_intensity_sample(samples, img), [1,6,8,9]);
            
            % sample inside image
            samples = [ ...
                1.5, 3.5; ... % mean([1,2,10,20])   =  8.25
                2.5, 2.5; ... % mean([20,30,40,60]) = 37.5
                4.8,   4; ... % 4.8
                4.8,   3; ... % 48
                2,   2.2]';    % 36
            
            tc.assertEqual(camera_intensity_sample(samples, img), [8.25, 37.5, 4.8, 48, 36]);
        end
    end
    
end


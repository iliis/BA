classdef utils_tests < matlab.unittest.TestCase
%UTILS_TESTS tests various utily functions in utils/
   
    methods (Test)
        function test_image_to_list(tc)
            img = cat(3, [1,2;3,4], [10,20;30,40]);
            list = image_to_list(img);
            
            tc.assertEqual(list, [1 3 2 4; 10 30 20 40]);
            
            A = [1,2,3,4;5,6,7,8];
            list = image_to_list(cat(3, A, A*10, A*100));
            
            tc.assertEqual(size(list), [3,numel(A)]);
            tc.assertEqual(list(1,:), reshape(A,1,[]));
            tc.assertEqual(list(2,:), reshape(A*10,1,[]));
            tc.assertEqual(list(3,:), reshape(A*100,1,[]));
        end
        
        function test_list_to_image(tc)
             list = [1 3 2 4; 10 30 20 40];
             img = list_to_image(list, [2,2]);
             
             tc.assertEqual(img, cat(3, [1,2;3,4], [10,20;30,40]));
        end
        
        function test_list_image_combined(tc)
            % generate randomly sized images with random data
            img = rand(randi(100,1,3));
            
            list = image_to_list(img);
            
            tc.assertEqual(img, list_to_image(list, size(img)));
        end
    end
    
end


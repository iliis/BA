classdef util_reverse_transformation_tests < matlab.unittest.TestCase
    methods (Test)
        
        function test_reverse_translation(tc)
            for i=1:100
                translation = rand(1, 3)*100-50;
                [rt, rr] = reverse_transformation(translation, [0 0 0]);
                tc.assertEqual(-translation, rt);
                tc.assertEqual(rr, [0 0 0]);
            end
        end
        
        function test_reverse_rotation(tc)
            for i=1:100
                rotation = rand(1, 3)*10*pi-5*pi;
                [rt, rr] = reverse_transformation([0 0 0], rotation);
                tc.assertEqual(rt, [0 0 0]);
                tc.assertEqual(-rotation, rr);
            end
        end
        
        function test_reverse_rotation_and_translation(tc)
            for i=1:100
                translation = rand(1, 3)*100-50;
                rotation    = rand(1, 3)*10*pi-5*pi;
                [rt, rr] = reverse_transformation(translation, rotation);
                
                R = angle2dcm(rotation);
                Rrev = angle2dcm(rr);
                
                for j = 1:100
                    point = rand(1, 3)*100-50;
                    tc.assertEqual(point, (point*R+translation)*Rrev+rt);
                    pt = point*R+translation;
                    tc.assertEqual(pt, (pt*Rrev+rt)*R+translation);
                end
            end
        end
        
    end
end
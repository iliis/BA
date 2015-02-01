classdef jacobi_tests < matlab.unittest.TestCase
    % test on a simple linear transformation [-1 2 0]
    
    properties
        image_scale = 1;
        image_path  = 'unit_tests/rotation_and_translation';
        show_plots  = true;
        
        perturbation_range = -0.1:0.001:0.1;
        
        I1, I2, D1, D2;
    end
    
    methods (TestClassSetup)        
        function load_images(tc)
            [I1, D1] = read_image(tc.image_path, 1, tc.image_scale);
            [I2, D2] = read_image(tc.image_path, 2, tc.image_scale);
            
            tc.I1 = I1;
            tc.I2 = I2;
            tc.D1 = D1;
            tc.D2 = D2;
        end
    end
    
    methods (Test)
        
        function test_numerical_jacobi(tc)
            
            correct_translation = [1.6237 0 -1.5374];
            correct_rotation    = [0 degtorad(-20) 0];
            
            trans = correct_translation;
            rot   = correct_rotation;
            
            I1_w = warp_image(tc.D1, tc.I1,  trans, rot);
            errs = image_to_list((tc.I2 - I1_w).^2);
            
            [unused, J] = intensity_error_lsqnonlin(tc.D1, tc.I1, tc.I2, trans, rot);
            
            diff = zeros(size(J,1), numel(tc.perturbation_range));
            for i = 1:numel(tc.perturbation_range)
                I1_w = warp_image(tc.D1, tc.I1,  trans + [tc.perturbation_range(i) 0 0], rot);
                diff(:,i) = (image_to_list((tc.I2 - I1_w).^2) - errs) / tc.perturbation_range(i);
            end
            
            pixel_idx = 100;
            for i=1:size(J,1)
                if abs(J(i,1)) > 1 && numel(diff(i, diff(i,:) > 0)) > 50
                    pixel_idx = i;
                    break;
                end
            end
            
            plot(tc.perturbation_range, diff(pixel_idx,:), '-'); %, tc.perturbation_range, repmat(J(pixel_idx,1),1,numel(tc.perturbation_range)), ':');
            title(['perturbed along X axis. J(' num2str(pixel_idx) ', 1) = ' num2str(J(pixel_idx,1))]);
            xlabel('perturbation');
            ylabel('change in error');
            diff(pixel_idx,:)
            
            %figure;
            %bar(J(:,1));
            %title('Jacobi(:,1)');
            
        end
        
    end
end
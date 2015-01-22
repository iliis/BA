classdef linear_translation_tests < matlab.unittest.TestCase
    % test on a simple linear transformation [-1 2 0]
    
    properties
        image_scale = 1;
        image_path  = 'unit_tests/linear_translation';
        show_plots  = false;
        
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
        
        function test_image_warping_translation(tc)
            persistent fig;
            
            correct_translation = [1 0 0];
            
            I1_w = warp_image(tc.D1, tc.I1,  correct_translation, [0 0 0]);
            I2_w = warp_image(tc.D2, tc.I2, -correct_translation, [0 0 0]);
            
            e1 = difference_error(tc.I1, I2_w);
            e2 = difference_error(tc.I2, I1_w);
            
            %tc.assertLessThan(
            
            % plot
            if tc.show_plots
                
                if isempty(fig) fig = figure; else figure(fig); end
                
                subplot(2, 3, 1);
                imagesc(tc.I1);
                title('Image 1 (original)');
                                
                subplot(2, 3, 2);
                imagesc(I2_w);
                title('Image 2 (warped to image 1)');
                
                subplot(2, 3, 3);
                diff = abs(I2_w - tc.I1);
                diff(isnan(I2_w)) = 0;
                imagesc(diff);
                title(['diff (err = ', num2str(e1), ')']);
                
                subplot(2, 3, 4);
                imagesc(tc.I2);
                title('Image 2 (original)');
                
                subplot(2, 3, 5);
                imagesc(I1_w);
                
                title('Image 1 (warped to image 2)');
                
                subplot(2, 3, 6);
                diff = abs(I1_w - tc.I2);
                diff(isnan(I2_w)) = 0;
                imagesc(diff);
                title(['diff (err = ', num2str(e2), ')']);
            end
            
        end
        
    end
end


function err = difference_error(orig, warped)
errs = (orig - warped).^2;
errs(isnan(warped)) = [];
err = sum(sum(errs));
end
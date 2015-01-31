classdef diagonal_translation_tests < matlab.unittest.TestCase
    % test on a simple linear transformation [-1 2 0]
    
    properties
        image_scale = 1;
        image_path  = 'unit_tests/diagonal_translation';
        show_plots  = true;
        
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
            
            % TODO: this test fails, perturbation by 0.1 0.1 0.1 gives
            % better error?!
            correct_translation = [1 0.4 -2];
            
            I1_w = warp_image(tc.D1, tc.I1,  correct_translation, [0 0 0]);
            I2_w = warp_image(tc.D2, tc.I2, -correct_translation, [0 0 0]);
            
            e1 = difference_error(tc.I1, I2_w);
            e2 = difference_error(tc.I2, I1_w);
                        
            % plot
            if tc.show_plots
                
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
            
            % ASSERTIONS
            % (after plotting, so plots are shown even when assertions fail)
            % //////////////////////////////////////////////////////////////
            
            tc.assertLessThan(e1, 150);
            tc.assertLessThan(e2, 150);
                        
            % ensure this is really the local minimum by perturbing a bit
            perturbations = [-0.1, -0.01, 0.01, 0.1];
            for px = perturbations
                for py = perturbations
                    for pz = perturbations
                        I1_w = warp_image(tc.D1, tc.I1,  correct_translation + [px py pz], [0 0 0]);
                        I2_w = warp_image(tc.D2, tc.I2, -correct_translation - [px py pz], [0 0 0]);

                        e1_perturbed = difference_error(tc.I1, I2_w);
                        e2_perturbed = difference_error(tc.I2, I1_w);

                        tc.assertLessThan(e1, e1_perturbed, ['I2 warped to I1: translation by ' num2str(correct_translation) ' + ' num2str([px py pz]) ' results in error of ' num2str(e1_perturbed) ' instead of ' num2str(e1)]);
                        tc.assertLessThan(e2, e2_perturbed, ['I1 warped to I2: translation by ' num2str(correct_translation) ' + ' num2str([px py pz]) ' results in error of ' num2str(e2_perturbed) ' instead of ' num2str(e2)]);
                    end
                end
            end
            
            % also do random perturbations
            for i = 1:20
                perturbation = rand(1,3)-0.5; % +/-0.5
                I1_w = warp_image(tc.D1, tc.I1,  correct_translation + perturbation, [0 0 0]);
                I2_w = warp_image(tc.D2, tc.I2, -correct_translation - perturbation, [0 0 0]);

                e1_perturbed = difference_error(tc.I1, I2_w);
                e2_perturbed = difference_error(tc.I2, I1_w);

                tc.assertLessThan(e1, e1_perturbed, ['translation by ' num2str(correct_translation) ' + ' num2str(perturbation) ' results in error of ' num2str(e1_perturbed) ' instead of ' num2str(e1)]);
                tc.assertLessThan(e2, e2_perturbed);
            end
            
        end
        
    end
end


function err = difference_error(orig, warped)
errs = (orig - warped).^2;
errs(isnan(warped)) = [];
err = sum(sum(errs));
end
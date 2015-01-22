classdef basic_projection_tests < matlab.unittest.TestCase
    %BASIC_PROJECTION_TESTS tests very trivial things, like keeping colors constant across projecttions
    
    properties
        image_scale = 1;
        image_path  = 'unit_tests/linear_translation';
        show_plots  = false;
        
        I, D;
    end
    
    methods (TestClassSetup)        
        function load_images(tc)
            [tc.I, tc.D] = read_image(tc.image_path, 1, tc.image_scale);
        end
    end
    
    methods (Test)
        function test_proj_to_space(tc)
            persistent fig;
            
            Imin = min(min(tc.I));
            Imax = max(max(tc.I));
            
            % project into space and back onto camera -> should result in
            % identical image
            XYZ = project_to_space(tc.D);
            warped = project_to_camera(XYZ, tc.I);
            
            if tc.show_plots
                if isempty(fig) fig = figure; else figure(fig); end
                
                subplot(1, 2, 1);
                imagesc(tc.I);
                title('original');
                                
                subplot(1, 2, 2);
                imagesc(warped);
                title('projected to space and back');
                
            end
            
            tc.assertEqual(min(min(warped)), Imin);
            tc.assertEqual(max(max(warped)), Imax);
            
            tc.assertEqual(tc.I, warped);
        end
        
        
        function test_proj_to_space_nulltransformation(tc)
            persistent fig;
            
            Imin = min(min(tc.I));
            Imax = max(max(tc.I));
            
            % project into space and back onto camera
            % but with null-transformation in between
            %-> should result in identical image
            XYZ = project_to_space(tc.D);
            XYZ = apply_camera_transformation(XYZ, [0 0 0], [0 0 0]);
            warped = project_to_camera(XYZ, tc.I);
            
            if tc.show_plots
                if isempty(fig) fig = figure; else figure(fig); end
                
                subplot(1, 2, 1);
                imagesc(tc.I);
                title('original');
                                
                subplot(1, 2, 2);
                imagesc(warped);
                title('projected to space and back (null transformed)');
                
            end
            
            tc.assertEqual(min(min(warped)), Imin);
            tc.assertEqual(max(max(warped)), Imax);
            
            tc.assertEqual(tc.I, warped);
        end
        
        % same as above, but with warp_image() instead of doing it manually
        function test_proj_to_space_warp_null(tc)
            persistent fig;
            
            Imin = min(min(tc.I));
            Imax = max(max(tc.I));
            
            % project into space and back onto camera
            % but with null-transformation in between
            %-> should result in identical image
            warped = warp_image(tc.D, tc.I, [0 0 0], [0 0 0]);
            
            if tc.show_plots
                if isempty(fig) fig = figure; else figure(fig); end
                
                subplot(1, 2, 1);
                imagesc(tc.I);
                title('original');
                                
                subplot(1, 2, 2);
                imagesc(warped);
                title('projected to space and back (null transformed)');
                
            end
            
            tc.assertEqual(min(min(warped)), Imin);
            tc.assertEqual(max(max(warped)), Imax);
            
            tc.assertEqual(tc.I, warped);
        end
    end
    
end


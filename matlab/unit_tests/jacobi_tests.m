classdef jacobi_tests < matlab.unittest.TestCase
    % test on a simple linear transformation [-1 2 0]
    
    properties
        image_scale = 1;
        %image_path  = 'unit_tests/rotation_and_translation';
        image_path  = 'unit_tests/rot_trans_verysmall';
        show_plots  = true;
        
        perturbation_range = -0.1:0.01:0.1;
        diff_max_error = 0.1;
        
        I1, I2, D1, D2;
    end
    
    methods (TestClassSetup)
        function load_images(tc)
            [tc.I1, tc.D1] = read_image(tc.image_path, 1, tc.image_scale);
            [tc.I2, tc.D2] = read_image(tc.image_path, 2, tc.image_scale);
        end
    end
    
    methods
        function assertEqualTolerance(tc, assertable, actual, tolerance)
            tc.assertLessThan(   assertable, actual + tolerance*ones(size(actual)));
            tc.assertGreaterThan(assertable, actual - tolerance*ones(size(actual)));
        end
        
        function numeric_diff(tc, correct_solution, test_function)
            % input:
            % correct_solution: float, value of d/dx of test_function
            % test_function:    f(x), numeric derivation at x=0+delta
            %
            % output_dim:       size of y = f(x)
            % (with numel(x) == size(x,2) == size(correct_solution,2)
            %   and numel(y) == size(y,1) == size(correct_solution,1))
            %
            % perturbation_range: set of delta's to test
            
            
            
            [output_dim, input_dim] = size(correct_solution);
            
            % calculate f(x)
            I = eye(input_dim);
            yval = zeros(input_dim, output_dim, numel(tc.perturbation_range));
            for idim = 1:input_dim
                for i = 1:numel(tc.perturbation_range)
                    yval(idim,:,i) = test_function(tc.perturbation_range(i)*I(idim, :))';
                end
            end
            
            % calculate df/dx (by fitting line trough previously calculated points (yval)
            diff = zeros(input_dim, output_dim);
            error = 0;
            for odim = 1:output_dim
                for idim = 1:input_dim
                    p = polyfit(tc.perturbation_range, permute(yval(idim,odim,:), [2 3 1]), 1);
                    diff(idim, odim) = p(1);
                    
                    error = error + (p(1) - correct_solution(odim,idim))^2;
                end
            end
            
            if (error >= tc.diff_max_error)
                % plot all slopes
                figure('Name', ['Comparison numerical to analytical Jacobi (total error = ' num2str(error) ')'], 'NumberTitle', 'Off');
                for odim = 1:output_dim
                    for idim = 1:input_dim
                        subplot(output_dim,input_dim,idim + (odim-1)*input_dim);
                        f0 = test_function(zeros(1,input_dim))';
                        % plot 'correct' slope
                        plot(minmax(tc.perturbation_range), minmax(tc.perturbation_range)*correct_solution(odim, idim)+f0(odim), 'g-');
                        hold on;
                        % plot numerical slope
                        plot(minmax(tc.perturbation_range), minmax(tc.perturbation_range)*diff(idim, odim)+f0(odim), 'b:');
                        % plot function values
                        plot(tc.perturbation_range, reshape(yval(idim, odim, :), [1 numel(tc.perturbation_range)]), 'r.');
                        title({['analytical = ' num2str(correct_solution(odim,idim))], ['numerical = ' num2str(diff(idim,odim))]});
                        hold off;
                    end
                end
            end
            
            tc.assertLessThan(error, tc.diff_max_error);
        end
    end
    
    methods (Test)
        
        function test_numerical_jacobi_transformation(tc)
            
            % load global parameters
            global_parameters
            
            focal = sym('FOCAL');
            syms u v Img1(u, v) Img2(u, v) Depth real;
            
            % parameters for camera transformation
            % [x y z] = translation
            % [a b c] = rotation (Euler angles)
            syms Tx Ty Tz Ta Tb Tc real;
            T = [Tx Ty Tz Ta Tb Tc];

            % inverse projection operator (image to 3D space)
            proj_inv = Depth * [ u / focal; v / focal; 1 ];

            % camera transformation
            T_translation = [ Tx ; Ty ; Tz ];
            
            % TODO: use jacobian from intensity_error_lsqnonlin here
            % i.e. refactor all the things!

            Rx = [ ...
                1 0 0; ...
                0 cos(Ta) -sin(Ta); ...
                0 sin(Ta)  cos(Ta)];

            Ry = [ ...
                cos(Tb) 0 sin(Tb); ...
                0 1 0; ...
                -sin(Tb) 0 cos(Tb)];

            Rz = [ ...
                cos(Tc) -sin(Tc) 0; ...
                sin(Tc)  cos(Tc) 0; ...
                0 0 1];
            
            T_rotation = Rz * Ry * Rx;
            transformed = T_rotation * proj_inv + T_translation;
            
            J_T = simplify(jacobian(transformed, T));
            
            J_T_analytical = @(t, pu_real, pv_real, depht_real) eval(subs(J_T, [T u v Depth focal], [t pu_real pv_real depht_real CAMERA_FOCAL]));
            
            
                
            H = size(tc.D1, 1);
            W = size(tc.D1, 2);
            
            for i = 1:100
                
                %pu = 100; pv = 20; % just a random pixel
                pu = randi(H); pv = randi(W);

                if (tc.D1(pu,pv) >= 100-eps)
                    disp('WARNING: chosen pixel is probably invalid (outside far clipping plane)!');
                end

                % calculate pixel positions on camera sensor
                % this is a huge mess! pu = y and pv = x!
                px_real = (pv - W/2) * (CAMERA_WIDTH / W);
                py_real = (pu - H/2) * (CAMERA_WIDTH / W);

                %disp(['depth = ' num2str(tc.D1(pu,pv))]);
                %disp(J_T_analytical([0 0 0 0 0 0], px_real, py_real, tc.D1(pu, pv)));
                %disp(['point in 3D space analytical: ' num2str(eval(subs(proj_inv, [T u v Depth focal], [0 0 0 0 0 0 px_real py_real tc.D1(pu, pv) CAMERA_FOCAL]))')]);

                XYZ = project_to_space(tc.D1);
                
                %disp(['point in 3D space from func:  ' num2str(XYZ(pu,pv,:))]);
                %disp(proj_inv);

                tc.assertEqualTolerance(reshape(XYZ(pu,pv,:), [3 1]), eval(subs(proj_inv, [T u v Depth focal], [0 0 0 0 0 0 px_real py_real tc.D1(pu, pv) CAMERA_FOCAL])), 1e-6);

                T_numerical = @(t) reshape(apply_camera_transformation(cat(3,XYZ(pu,pv,1),XYZ(pu,pv,2),XYZ(pu,pv,3)), t(1:3), t(4:6)), [3, 1]);

                tc.numeric_diff(J_T_analytical([0 0 0 0 0 0], px_real, py_real, tc.D1(pu, pv)), T_numerical);
            end
        end
        
        function test_numerical_jacobi(tc)
            
            return;
            
            % correct motion for 'rotation_and_translation'
            %correct_translation = [1.6237 0 -1.5374];
            %correct_rotation    = [0 degtorad(-20) 0];
            
            % correct motion for 'verysmall' set
            correct_translation = [0.04 0 -0.03];
            correct_rotation    = [0 degtorad(-5) 0];
            
            trans = correct_translation;
            rot   = correct_rotation;
            
            I1_w = warp_image(tc.D1, tc.I1,  trans, rot);
            errs = image_to_list(tc.I2 - I1_w);
            
            [unused, J] = intensity_error_lsqnonlin(tc.D1, tc.I1, tc.I2, trans, rot);
            
            diff = zeros(size(J,1), numel(tc.perturbation_range));
            for i = 1:numel(tc.perturbation_range)
                I1_w = warp_image(tc.D1, tc.I1,  trans + [tc.perturbation_range(i) 0 0], rot);
                diff(:,i) = (image_to_list(tc.I2 - I1_w) - errs) / tc.perturbation_range(i);
            end
            
            % possible problems / solutions:
            % - Jacobi auseinandernehmen, einzelne Teile testen
            % - maybe perturbation too big (or too small)
            % - stupid bugs
            
            % find some 'interesting' pixel to look at (one that actually has a
            % defined differential and isn't projected into the void)
            
            % TODO: grade pixel by how good they match the analytical
            % Jacobian and determine if this tells us something about which
            % pixels should be used for matching and which can safely be
            % optimized away (of course, this requires correct Jacobian)
            pixel_idx = 100;
            for i=1:size(J,1)
                % change the min. Jacobian value here to get another pixel ;)
                if abs(J(i,1)) > 1 && numel(diff(i, diff(i,:) > 0)) > 50
                    pixel_idx = i;
                    break;
                end
            end
            
            plot(tc.perturbation_range, diff(pixel_idx,:), '-'); %, tc.perturbation_range, repmat(J(pixel_idx,1),1,numel(tc.perturbation_range)), ':');
            title(['perturbed along X axis. J(' num2str(pixel_idx) ', 1) = ' num2str(J(pixel_idx,1))]);
            xlabel('perturbation');
            ylabel('change in error');
            %diff(pixel_idx,:)
            
            %figure;
            %bar(J(:,1));
            %title('Jacobi(:,1)');
            
        end
        
    end
end

function [err, J, invalid_terms] = intensity_error( D1, I1, I2, T, plot)
% input:  2 intensitiy and 1 depth images, T transformation that maps I2/D2 to I1/D1
% output: linear error vector (not summed and not squared)
%         Jacobian of error term for T_translation

global_parameters

warped = warp_image(D1, I1, T);

errs = (warped-I2);
errs(isnan(warped)) = 0; % ignore empty pixels
%errs(isnan(warped)) = NONMATCHED_PIXEL_PENALTY; % not yet tested, penalty for unmatched pixels

err = image_to_list(errs)';

% keep a copy for plotting later on
errs_plot = errs.^2;
errs_plot(isnan(warped)) = 0;
err_total = sum(err.^2);

invalid_terms = false(size(err));
invalid_terms(isnan(warped)) = true;

%disp(['intensity error:  --> ', num2str(sum(err.^2))]);
%disp(['translation: ' num2str(T_translation)]);
%disp(['rotation:    ' num2str(T_rotation)]);

if nargout > 1
    
    % calculate derivatives at current 'position' (T)
    
    J = zeros(numel(err), 6);
    
    for v = 1:H
        for u = 1:W
            
            % TODO: rename variables to standard names (cu/cv instead of pu/pv)
            % TODO: use standard math (i.e. focal length in pixel units etc.)
            
            % apply transformation to 3D points
            Tpoint = angle2dcm(T_rotation) * [x y z]' + T_translation';
            x = Tpoint(1); y = Tpoint(2); z = Tpoint(3);
            
            % index into J (resulting Jacobian)
            J_idx = (u-1)*H + v;
            
            pu = int32(x * CAMERA_FOCAL/z * W/CAMERA_WIDTH + W/2);
            pv = int32(y * CAMERA_FOCAL/z * W/CAMERA_WIDTH + H/2);
            
            if (pu < 1 || pv < 1 || pu > W-1 || pv > H-1 || invalid_terms(J_idx) == true)
                % pixel is out of range of our image (or the differential of it)
                
                J(J_idx, :) = [0 0 0 0 0 0];
                invalid_terms(J_idx) = true;
                
                continue;
            end
            
            J_T  = jacobi_transformation(u,v,D1(v,u),T);
            J_pi = jacobi_projection(Tpoint);
            J_I  = jacobi_image(I1,pu,pv);
            
            % final Jacobian
            J(J_idx, :) = J_I * J_pi * J_T;
            
        end
    end
    
    %J(invalid_terms, :) = [0 0 0 0 0 0];
    
end % nargout > 1

if nargin >= 5
    if numel(plot) ~= 3
        plot = [subplot(1,3,1) subplot(1,3,2) subplot(1,3,3)];
    end
    
    axes(plot(1));
    imagesc(I2);
    title('I2');

    axes(plot(2));
    imagesc(warped);
    title('I1 warped');

    axes(plot(3));
    imagesc(abs(errs_plot));
    title(err_total);
end

end


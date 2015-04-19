function [T_translation, T_rotation] = gauss_newton(D1,I1,I2, T_translation, T_rotation)

x = [T_translation, T_rotation]';

for i = 1:10
    % TODO: remove invalid_terms!
    [err, J] = intensity_error_lsqnonlin(D1,I1,I2, x(1:3)', x(4:6)');
    
    %step = inv(J'*J) * J' * err';
    step = (J'*J) \ J' * err';
    
    x = x - step;
    
    disp(['step ' num2str(i) ': error = ' num2str(sum(err.^2)) '  T = ' num2str(x')]);
end

T_translation = x(1:3)';
T_rotation = x(4:6)';

end
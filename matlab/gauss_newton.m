function [T_translation, T_rotation] = gauss_newton(D1,I1,I2, T_translation, T_rotation)

x = [T_translation, T_rotation]';

for i = 1:10
    [err, J] = intensity_error_lsqnonlin(D1,I1,I2, x(1:3)', x(4:6)');
    
    x = x - inv(J'*J) * J' * err';
    
    disp(['step ' num2str(i) ': error = ' num2str(sum(err.^2)) '  T = ' num2str([T_translation T_rotation])]);
end

T_translation = x(1:3)';
T_rotation = x(4:6)';

end
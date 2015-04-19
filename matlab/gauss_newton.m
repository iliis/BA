function T = gauss_newton(D1,I1,I2, T)

for i = 1:10
    % TODO: remove invalid_terms!
    [err, J] = intensity_error_lsqnonlin(D1,I1,I2,T);
    
    %step = inv(J'*J) * J' * err';
    step = (J'*J) \ J' * err';
    
    T = T - step';
    
    disp(['step ' num2str(i) ': error = ' num2str(sum(err.^2)) '  T = ' num2str(T)]);
end

end
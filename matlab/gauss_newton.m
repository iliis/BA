function T = gauss_newton(D1,I1,I2, T)

for i = 1:1000
    % invalid terms are zero in Jacobi
    [err, J] = intensity_error(D1,I1,I2,T);
    
    disp(['step ' num2str(i) ': error = ' num2str(sum(err.^2)) '  T = [ ' num2str(T) ' ]']);
    
    %step = inv(J'*J) * J' * err';
    step = (J'*J) \ J' * err';
    
    T = T - step';
end

disp(['final step : error = ' num2str(sum(err.^2)) '  T = [ ' num2str(T) ' ]']);

end
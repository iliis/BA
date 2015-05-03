function [T, T_path] = gradient_descent(D1,I1,I2, T, step_size)

if nargout > 1
    T_path = [];
end

for i = 1:1000
    % invalid terms are zero in Jacobi
    [err, J] = intensity_error(D1,I1,I2,T);
    
    disp(['[GD] step ' num2str(i) ': error = ' num2str(sum(err.^2)) '  T = [ ' num2str(T) ' ]']);
    
    if nargout > 1
        T_path = [T_path; T];
    end
    
    %J(:,3:end) = 0;
    
    step = step_size * J' * err;
    
    T = T - step';
    
    hold on;
    plot(T(1), T(2), '.g');
    drawnow;
    hold off;
end

disp(['[GD] final step : error = ' num2str(sum(err.^2)) '  T = [ ' num2str(T) ' ]']);

if nargout > 1
    T_path = [T_path; T];
end

end


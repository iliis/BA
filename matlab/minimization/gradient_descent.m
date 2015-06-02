function [T, T_path] = gradient_descent(D1,I1,I2,T,intrinsics,restrict,step_size)

global minimization_running;

if nargout > 1
    T_path = [];
end

for i = 1:1000
    % invalid terms don't exist in Jacobi
    [err, J] = camera_warp(I1,D1,I2,T,intrinsics);
    
    disp(['[GD] step ' num2str(i) ': error = ' num2str(norm(err)) '  T = [ ' num2str(T') ' ]']);
    
    if nargout > 1
        T_path = [T_path; T];
    end
    
    if restrict
        J(:,4:end) = 0;
    end
    
    step = - step_size * J' * err';
    
    % TODO: shouldn't this be -step?
    T = T + step;
    
    hold on;
    plot(T(1), T(2), '.g');
    drawnow;
    hold off;
    
    if (~minimization_running)
        break;
    end
end

disp(['[GD] final step : error = ' num2str(norm(err)) '  T = [ ' num2str(T') ' ]']);

if nargout > 1
    T_path = [T_path; T];
end

end


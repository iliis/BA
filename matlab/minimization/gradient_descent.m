function [T, T_path] = gradient_descent(scene, T_init, rel_tol, weight_function, restrict, step_size)

global minimization_running;

if nargout > 1
    T_path = [];
end

if nargin < 4
    weight_function = @uniform_weights;
end

T = T_init;

for i = 1:1000
    % invalid terms don't exist in Jacobi
    [err, J] = camera_warp(scene, T);
    
    disp(['[GD] step ' num2str(i) ': error = ' num2str(norm(err)) '  T = [ ' num2str(T') ' ]']);
    
    if nargout > 1
        T_path = [T_path; T];
    end
    
    if restrict
        J(:,4:end) = 0;
    end
    
    w = weight_function(err);
    
    % TODO: stimmt das eigentlich?
    %step = - step_size * J' * err';
    step = - step_size .* (J' * (w.^2 .* err)');
    
    T = T + step;
    
    if (norm(step) < rel_tol)
        disp('[GD] found minimum!');
        break;
    end
    
    %hold on;
    %plot(T(1), T(2), '.g');
    %drawnow;
    %hold off;
    
    if (~minimization_running)
        break;
    end
end

disp(['[GD] final step : error = ' num2str(norm(err)) '  T = [ ' num2str(T') ' ]']);

if nargout > 1
    T_path = [T_path; T];
end

end


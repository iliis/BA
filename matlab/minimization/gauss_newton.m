function T = gauss_newton(scene, T_init, rel_tol, weight_function, restrict)

global minimization_running;

if nargin == 5
    restrict = false;
end

if nargin < 4
    weight_function = @uniform_weights;
end

T = T_init;

hold on;
for i = 1:1000
    % invalid terms don't exist in Jacobi
    [err, J] = camera_warp(scene, T);
    
    disp(['[GN] step ' num2str(i) ': error = ' num2str(norm(err)) '  T = [ ' num2str(T') ' ]']);
    
    
    W = diag(weight_function(err));
    
    
    if restrict
        J(:,4:end) = [];
    end
    
    %step = inv(J'*J) * J' * err';
    step = - (J'*W*J) \ J' * W * err';
    
    if restrict
        step = [step'  0 0 0]';
    end
    
    plot(T(1), T(2), '.g');
    drawnow;
    
    T = T + step;
    
    
    if (norm(step) < rel_tol)
        disp('[GD] found minimum!');
        break;
    end
    
    if (~minimization_running)
        break;
    end
end
hold off;

disp(['[GN] final step : error = ' num2str(norm(err)) '  T = [ ' num2str(T') ' ]']);

end
function T = levenberg_marquardt(scene, T_init, rel_tol, weight_function, restrict)

global minimization_running;

if nargin == 5
    restrict = false;
end

if nargin < 4
    weight_function = @uniform_weights;
end

lambda = 0.01;
lambda_inc_factor = 2;

T = T_init;

[err, J] = camera_warp(scene, T);

hold on;
for i = 1:1000
    % invalid terms are zero in Jacobi
    
    
    disp(['[LM]      step ' num2str(i) ': error = ' num2str(norm(err)) '  T = [ ' num2str(T') ' ]']);
    
    %step = inv(J'*J) * J' * err';
    
    % TODO: add Abbruchkriterium, otherwise this loop might never exit
    while true
        if (~minimization_running)
            break;
        end
        
        if restrict
            J(:,4:end) = 0;
        end

        W = diag(weight_function(err));
        
        JTJ = J'*W*J;
        
        %step = -(JTJ + lambda * diag(diag(JTJ))) \ J' * err;
        step = -(JTJ + lambda * eye(6)) \ J' * W * err';

        % try out step
        T_tmp = T + step;
        [err_tmp, J_tmp] = camera_warp(scene, T_tmp);
        
        disp(['[LM] temp step ' num2str(i) ': error = ' num2str(norm(err_tmp)) ' lambda = ' num2str(lambda) ' T = [ ' num2str(T_tmp') ' ]']);

        plot(T_tmp(1), T_tmp(2), '.r');
        drawnow;

        if (norm(err_tmp) > norm(err))
            % new value is worse, increase dampening factor
            lambda = lambda * lambda_inc_factor;
        else
            % found better value, keep it
            
            % decrease lambda according to how much getter we got
            % TODO: still increase lambda a bit if this was only a tiny improvement
            lambda = lambda / lambda_inc_factor;
            
            break;
        end
    end
    T = T_tmp;
    J = J_tmp;
    err = err_tmp;
    
    plot(T(1), T(2), 'xg');
    drawnow;
    
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
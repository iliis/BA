function T = levenberg_marquardt(D1,I1,I2,T,intrinsics,restrict)

global minimization_running;

if nargin == 5
    restrict = false;
end

lambda = 0.01;
lambda_inc_factor = 2;

[err, J] = camera_warp(I1,D1,I2,T,intrinsics);

hold on;
for i = 1:1000
    % invalid terms are zero in Jacobi
    
    
    disp(['[LM] step ' num2str(i) ': error = ' num2str(sum(err.^2)) '  T = [ ' num2str(T') ' ]']);
    
    %step = inv(J'*J) * J' * err';
    
    while true
        
        if restrict
            J(:,3:end) = 0;
        end

        JTJ = J'*J;
        %step = -(JTJ + lambda * diag(diag(JTJ))) \ J' * err;
        step = -(JTJ + lambda * eye(6)) \ J' * err';

        % try out step
        T_tmp = T + step;
        [err_tmp, J_tmp] = camera_warp(I1,D1,I2,T,intrinsics);
        
        disp(['[LM] temp step ' num2str(i) ': error = ' num2str(sum(err_tmp.^2)) ' lambda = ' num2str(lambda) ' T = [ ' num2str(T_tmp') ' ]']);

        plot(T_tmp(1), T_tmp(2), '.r');
        drawnow;

        if (sum(err_tmp.^2) > sum(err.^2))
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
    
    if (~minimization_running)
        break;
    end
end
hold off;
disp(['[GN] final step : error = ' num2str(sum(err.^2)) '  T = [ ' num2str(T') ' ]']);

end
function T = gauss_newton(D1,I1,I2,T,intrinsics,restrict)

global minimization_running;

if nargin == 5
    restrict = false;
end

hold on;
for i = 1:1000
    % invalid terms don't exist in Jacobi
    [err, J] = camera_warp(I1,D1,I2,T,intrinsics);
    
    disp(['[GN] step ' num2str(i) ': error = ' num2str(norm(err)) '  T = [ ' num2str(T') ' ]']);
    
    if restrict
        J(:,4:end) = [];
    end
    
    %step = inv(J'*J) * J' * err';
    step = - (J'*J) \ J' * err';
    
    if restrict
        step = [step'  0 0 0]';
    end
    
    plot(T(1), T(2), '.g');
    drawnow;
    
    T = T + step;
    
    if (~minimization_running)
        break;
    end
end
hold off;

disp(['[GN] final step : error = ' num2str(norm(err)) '  T = [ ' num2str(T') ' ]']);

end
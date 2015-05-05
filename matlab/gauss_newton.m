function T = gauss_newton(D1,I1,I2,T,restrict)

global minimization_running;

if nargin == 4
    restrict = false;
end

hold on;
for i = 1:1000
    % invalid terms are zero in Jacobi
    [err, J] = intensity_error(D1,I1,I2,T);
    
    disp(['[GN] step ' num2str(i) ': error = ' num2str(sum(err.^2)) '  T = [ ' num2str(T) ' ]']);
    
    if restrict
        J(:,3:end) = [];
    end
    
    %step = inv(J'*J) * J' * err';
    step = (J'*J) \ J' * err;
    
    if restrict
        step = [step'  0 0 0 0]';
    end
    
    plot(T(1), T(2), '.g');
    drawnow;
    
    T = T - step';
    
    if (~minimization_running)
        break;
    end
end
hold off;

disp(['[GN] final step : error = ' num2str(sum(err.^2)) '  T = [ ' num2str(T) ' ]']);

end
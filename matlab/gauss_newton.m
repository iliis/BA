function T = gauss_newton(D1,I1,I2,T,restrict)

global minimization_running;

if nargin == 4
    restrict = false;
end

for i = 1:1000
    % invalid terms are zero in Jacobi
    [err, J] = intensity_error(D1,I1,I2,T);
    
    disp(['[GN] step ' num2str(i) ': error = ' num2str(sum(err.^2)) '  T = [ ' num2str(T) ' ]']);
    
    if restrict
        J(:,3:end) = 0;
    end
    
    %step = inv(J'*J) * J' * err';
    step = (J'*J) \ J' * err;
    
    %step(3:end) = 0;
    %step = [step'  0 0 0 0]; 
    
    hold on;
    plot(T(1), T(2), '.g');
    drawnow;
    hold off;
    
    T = T - step';
    
    if (~minimization_running)
        break;
    end
end

disp(['[GN] final step : error = ' num2str(sum(err.^2)) '  T = [ ' num2str(T) ' ]']);

end
function theta = normalize_euler_angles( theta )
% makes sure theta = [alpha beta gamma] is in range [2pi, pi, 2pi]
% see e.g. http://www.easyspin.org/documentation/eulerangles.html

assert(false, 'TODO: update this to reflect proper Euler angle definition. And write tests.');

theta = mod(theta, 2*pi);

if theta(2) > pi
    
    theta(1) = -theta(1);
    theta(2) =  theta(2) - pi;
    theta(3) = -theta(3);
    
    theta = mod(theta, 2*pi);
end

end


%testscene = Scene('input/testscene2_rotonly');
%min_init = [-1.8 0 -3.9 0 0 0]';

testscene = Scene('input/trajectory1');
%min_init = [1.8 0.1 4 0 0 0]';
min_init = testscene.getStep(1).ground_truth;
disp(['solution: ' num2str(min_init')]);
min_init = min_init + [1 -0.5 0 0 0 0]';

%testscene.scale_down(2);

step_size = [ ...
    1 1 1 ...
    1/20 1/20 1/20]' / 1000;

step_size = ones(6,1);
%gradient_descent(testscene.getStep(1), min_init, 0.0001, @(x) huber_loss(x,0.1), false, step_size);
%gradient_descent(testscene, min_init, @uniform_weights, false, 0.00001);
gauss_newton(testscene.getStep(1), min_init, 0.00001, @uniform_weights, false);

% trajectory1, step 1
%    -0.0372
%     0.0125
%     0.1714
%          0
%    -0.0000
%          0
%
% max. gradients:
%   500.5088
%   419.4853
%   105.3250
%    4.7989e+03
%    1.0865e+04
%    1.0208e+03
% i.e. angles have a x2 - x100 stronger gradient

% mean gradients:
%   145.2457
%   188.0514
%    44.9033
%    1.7122e+03
%    3.1954e+03
%   288.5875
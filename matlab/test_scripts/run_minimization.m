%testscene = Scene('input/testscene2_rotonly');
%min_init = [-1.8 0 -3.9 0 0 0]';

testscene = Scene('input/trajectory1');
%min_init = [1.8 0.1 4 0 0 0]';
min_init = testscene.getStep(1).ground_truth
min_init = min_init + [0.1 0 -0.1 0 0 0]';

step_size = [ ...
    0.01 0.01 0.01 ...
    0.00001 0.00001 0.00001]';
gradient_descent(testscene.getStep(1), min_init, 0.0001, @(x) huber_loss(x,0.1), false, step_size);
%gradient_descent(testscene, min_init, @uniform_weights, false, 0.00001);

% trajectory1, step 1
%    -0.0372
%     0.0125
%     0.1714
%          0
%    -0.0000
%          0
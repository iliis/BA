%testscene = Scene('input/testscene2_rotonly');
%min_init = [-1.8 0 -3.9 0 0 0]';

testscene = Scene('input/testscene1');
min_init = [1.8 0.1 4 0 0 0]';


step_size = [ ...
    0.001 0.001 0.001 ...
    0.00001 0.00001 0.00001]';
gradient_descent(testscene, min_init, 0.001, @(x) huber_loss(x,0.1), false, step_size);
%gradient_descent(testscene, min_init, @uniform_weights, false, 0.00001);
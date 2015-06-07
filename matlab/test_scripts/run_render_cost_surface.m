
testscene = Scene('input/trajectory1');

%testscene = testscene.scale_down(2);

step = testscene.getStep(1);

dim1 = 1;
dim2 = 5;

%T = [2 0 4 0 0 0]';
T = step.ground_truth;

range1 = T(dim1) + linspace(  -1,   1, 20);
range2 = T(dim2) + linspace(-0.1, 0.1, 20);

[errs, gradients] = render_cost_surface(step, [dim1 dim2], {range1, range2}, T, @uniform_weights, true);
%[errs, gradients] = render_cost_surface(step, [dim1 dim2], {range1, range2}, T, @(x) huber_loss(x,0.1), true);
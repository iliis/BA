
testscene = Scene('input/testscene1');

%testscene = testscene.scale_down(2);

dim1 = 5;
dim2 = 6;

T = [2 0 4 0 0 0]';

range1 = linspace(-0.5, 0.5, 30);
range2 = linspace(-0.5, 0.5, 30);

[errs, gradients] = render_cost_surface(testscene.getStep(1), [dim1 dim2], {range1, range2}, T, @uniform_weights, true);
%[errs, gradients] = render_cost_surface(testscene.getStep(1), [dim1 dim2], {range1, range2}, T, @(x) huber_loss(x,0.1), true);
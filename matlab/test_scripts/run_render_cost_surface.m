
testscene = Scene('input/testscene1');

%testscene = testscene.scale_down(2);

dim1 = 1;
dim2 = 3;

T = [-2 0 -4 0 0 0]';

range1 = linspace(-2.2,-1.8,30);
range2 = linspace(-4.2,-3.8,30);

[errs, gradients] = render_cost_surface(testscene, [dim1 dim2], {range1, range2}, T, true);
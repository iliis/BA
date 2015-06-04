
testscene = Scene('input/testscene1');

%testscene = testscene.scale_down(2);

dim1 = 1;
dim2 = 3;

T = [2 0 4 0 0 0]';

range1 = linspace(1, 3, 80);
range2 = linspace(3, 5, 80);

[errs, gradients] = render_cost_surface(testscene, [dim1 dim2], {range1, range2}, T, true);
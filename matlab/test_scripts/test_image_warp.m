% simply warp image according to ground truth
clear all;
test_init;

disp(ground_truth);
tic;
%intensity_error(D1,I1,I2, ground_truth, true); % plot result
[err, J] = intensity_error(D1,I1,I2, ground_truth, true); % plot result
toc;


% after refactoring, before any optimizations
% 8.284776 seconds for warping only
% 21.587971 with Jacobi

% after optimizing camera_transformation
% 4.304441
% 17.066084

% camera_projection
% 2.226166
% 14.891676

% 2.406980
% 25.045037
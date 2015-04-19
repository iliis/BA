% simply warp image according to ground truth
clear all;
test_init;

disp(ground_truth);
intensity_error(D1,I1,I2, ground_truth, true); % plot result
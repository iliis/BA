measured    = csvread('../cpp/measured_trajectory.csv', 1, 0)';

%measured    = csvread('plots/bag_slow/measured_trajectory_p30.csv', 1, 0)';
measured    = csvread('plots/bag_slow/measured_trajectory_p30_new_intrinsics.csv', 1, 0)';



groundtruth = csvread('../cpp/ground_truth_trajectory.csv', 1, 0)';

%delta = [0.15, 0.5];
delta = [0 0];

% convert groundtruth to same format
groundtruth = [-groundtruth(1,:) + delta(1);
               groundtruth(3,:);
               -groundtruth(2,:) + delta(2);];


subplot(2,2,1);
plot(groundtruth(1,:), groundtruth(3,:), '.-');
hold on;
path = plot_relative_trajectory(measured, false, '.-');
hold off;
legend('ASLAM', 'dense odometry');

subplot(2,2,3);
plot3(groundtruth(1,:), groundtruth(3,:), groundtruth(2,:), '.-');
hold on;
plot_relative_trajectory(measured, true, '.-');
hold off;
legend('ASLAM', 'dense odometry');

subplot(2,2,2);
plot_trajectory_error(path([1,3],:), groundtruth([1,3],:));
title('relative error XZ');

subplot(2,2,4);
plot_trajectory_error(path, groundtruth);
title('relative error XYZ');


figure;
plot_trajectory_error(path, groundtruth);
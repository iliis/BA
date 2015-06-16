measured    = csvread('../cpp/measured_trajectory.csv', 1, 0)';
groundtruth = csvread('../cpp/ground_truth_trajectory.csv', 1, 0)';

delta = [0.15, 0.5];
%delta = [0 0];

plot_relative_trajectory(measured(:,1:257), '.-');
hold on;
plot(-groundtruth(1,:)+delta(1), -groundtruth(2,:)+delta(2), '.-');
hold off;
legend('dense odometry', 'ASLAM');
%title('comparison of raw data');
title(['manually shifted by ' num2str(delta)]);

hold on;
plot_relative_trajectory(measured(:,300:end), '.-');
hold off;
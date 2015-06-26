measured    = csvread('../cpp/measured_trajectory.csv', 1, 0)';
groundtruth = csvread('../cpp/ground_truth_trajectory.csv', 1, 0)';

%delta = [0.15, 0.5];
delta = [0 0];

plot(-groundtruth(1,:)+delta(1), -groundtruth(2,:)+delta(2), '.-');
hold on;
plot_relative_trajectory(measured, '.-');
hold off;
legend('ASLAM', 'dense odometry');
%title('comparison of raw data');
%title(['manually shifted by ' num2str(delta)]);
%title(['pyramid 3 - 1']);

%hold on;
%plot_relative_trajectory(measured(:,300:end), '.-');
hold off;
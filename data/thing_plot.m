clc
clear


% Load data from CSV (skip header row)
filename = 'data.csv';
data = dlmread(filename, ',', 1, 0); % Skip 1 header row

% Extract relevant columns
x = data(:, 5);         % X position
y = data(:, 6);         % Y position
theta = data(:, 7);     % Orientation
yaw_bias = data(:, 17);  % Yaw bias (adjust column index if needed)
cov_x = data(:, 15);    % Covariance of x
cov_y = data(:, 16);    % Covariance of y
T = size(data, 1);      % Total number of frames

% Set up figure with custom layout
fig = figure(1);
clf;
set(fig, 'Position', [100 100 1000 700]); % Slightly taller figure

% Create custom grid for subplots
top_height = 0.70;      % 70% for top row (will be split)
bottom_height = 0.25;   % 25% for bottom row
margin = 0.03;          % 3% margin

% Top-left: x-y position (80% of top space)
pos1 = [0.1, bottom_height+margin, 0.7, top_height*0.8];
sp1 = subplot('Position', pos1);
hold on;
title('Robot Position (x-y)');
xlabel('X-axis');
ylabel('Y-axis');
xlim([0 15]);
ylim([0 8]);
grid on;
pos_line = plot(NaN, NaN, 'b-', 'DisplayName', 'Locus');
pos_point = plot(NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Current');
legend('Location', 'best');

% Top-right: yaw bias (20% of top space)
pos2 = [0.82, bottom_height+margin, 0.15, top_height*0.8];
sp2 = subplot('Position', pos2);
hold on;
title('Yaw Bias');
xlabel('Frame');
ylabel('Bias (rad)');
grid on;
yaw_bias_line = plot(NaN, NaN, 'k-', 'LineWidth', 1.5);
ylim([min(yaw_bias)-0.1 max(yaw_bias)+0.1]);

% Bottom subplots: theta | cov_x | cov_y (3 equal panels)
bottom_width = 0.28;
pos3 = [0.08, margin, bottom_width, bottom_height];
sp3 = subplot('Position', pos3);
hold on;
title('Robot Orientation (\theta)');
xlabel('Frame');
ylabel('\theta (rad)');
grid on;
theta_line = plot(NaN, NaN, 'm-', 'LineWidth', 1.5);
ylim([min(theta)-0.5 max(theta)+0.5]);

pos4 = [0.39, margin, bottom_width, bottom_height];
sp4 = subplot('Position', pos4);
hold on;
title('X Covariance');
xlabel('Frame');
ylabel('Cov(x)');
grid on;
cov_x_line = plot(NaN, NaN, 'g-', 'LineWidth', 1.5);
ylim([0 max(cov_x)*1.1]);

pos5 = [0.70, margin, bottom_width, bottom_height];
sp5 = subplot('Position', pos5);
hold on;
title('Y Covariance');
xlabel('Frame');
ylabel('Cov(y)');
grid on;
cov_y_line = plot(NaN, NaN, 'c-', 'LineWidth', 1.5);
ylim([0 max(cov_y)*1.1]);

% Animation loop
for i = 1:T
    % Update position plot
    set(pos_line, 'XData', x(1:i), 'YData', y(1:i));
    set(pos_point, 'XData', x(i), 'YData', y(i));

    % Update yaw bias plot
    set(yaw_bias_line, 'XData', 1:i, 'YData', yaw_bias(1:i));

    % Update theta plot
    set(theta_line, 'XData', 1:i, 'YData', theta(1:i));

    % Update covariance plots
    set(cov_x_line, 'XData', 1:i, 'YData', cov_x(1:i));
    set(cov_y_line, 'XData', 1:i, 'YData', cov_y(1:i));

    % Refresh display
    drawnow;
    pause(0.001);  % Adjust speed as needed
end

% Final message
disp('Animation complete.');

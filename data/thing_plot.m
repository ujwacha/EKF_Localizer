% Load data from CSV (skip header row)
filename = 'data.csv';
data = dlmread(filename, ',', 1, 0); % Skip 1 header row

% Extract relevant columns
x = data(:, 5);         % X position (adjust column index if needed)
y = data(:, 6);         % Y position
cov_x = data(:, 15);    % Covariance of x (adjust if needed)
T = size(data, 1);      % Total number of frames

% Set up figure and subplots
fig = figure(1);
clf;

% Left subplot: x-y position
sp1 = subplot(1, 2, 1);
hold on;
title('Robot Position (x-y)');
xlabel('X-axis');
ylabel('Y-axis');
xlim([0 7.5]);  % Fixed x-range
ylim([0 4]);    % Fixed y-range
grid on;
pos_line = plot(NaN, NaN, 'b-', 'DisplayName', 'Locus');  % Initialize empty line
pos_point = plot(NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Current');  % Current point

% Right subplot: covariance of x
sp2 = subplot(1, 2, 2);
hold on;
title('Covariance of x');
xlabel('Frame');
ylabel('Cov(x)');
grid on;
cov_line = plot(NaN, NaN, 'g-', 'DisplayName', 'Cov x');

% Animation loop
for i = 1:T
    % Update position plot
    set(pos_line, 'XData', x(1:i), 'YData', y(1:i));
    set(pos_point, 'XData', x(i), 'YData', y(i));

    % Update covariance plot
    set(cov_line, 'XData', 1:i, 'YData', cov_x(1:i));

    % Update legend (optional)
    legend('show');

    % Refresh display
    drawnow;
    pause(0.001);  % Adjust speed: 0.05 = medium speed
end

% Final message
disp('Animation complete. Press any key to close...');




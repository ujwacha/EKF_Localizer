x1 = dta(1, :);
y1 = dta(2, :);

x2 = dta(1, 1:6);
y2 = dta(1, 1:6);

dta2 = []



% Fit first dataset (Linear Fit)
p1 = polyfit(x1, y1, 1);
slope1 = p1(1);
y1_fit = polyval(p1, x1);

% Fit second dataset (Quadratic Fit)
p2 = polyfit(x2, y2, 3);
x2_fine = linspace(min(x2), max(x2), 100); % Generate more points for smooth curve
y2_fit = polyval(p2, x2_fine);

% Set figure size to A4 (210x297 mm)
figure;
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', [21 29.7], 'PaperPosition', [0 0 21 29.7]);

% Create first subplot (Linear fit)
subplot(2,1,1);
plot(x1, y1, 'bo', 'MarkerFaceColor', 'b'); hold on;
plot(x1, y1_fit, 'r-', 'LineWidth', 2);
for i = 1:length(x1)
    plot([x1(i), x1(i)], [y1(i), y1_fit(i)], 'k--'); % Dotted lines
end
% Move slope text to the right side
text(100, max(y1) * 0.9, sprintf('Slope: %.2f', slope1), 'FontSize', 12, 'Color', 'red');
xlabel('Armature Voltage (V)');
ylabel('Speed (RPM)');
title('Armature Voltage Vs Speed');
legend('Data points', 'Best-fit line', 'Error lines');
hold off;

% Create second subplot (Quadratic Curve Fit)
subplot(2,1,2);
plot(x2, y2, 'go', 'MarkerFaceColor', 'g'); hold on;
plot(x2_fine, y2_fit, 'm-', 'LineWidth', 2); % Smooth curve
for i = 1:length(x2)
    plot([x2(i), x2(i)], [y2(i), polyval(p2, x2(i))], 'k--'); % Dotted lines
end
xlabel('Field Current (Amp)');
ylabel('Speed (RPM)');
title('Field Current Vs Speed');
legend('Data points', 'Best-fit curve (Cubic)', 'Error lines');
hold off;

% Save figure as PDF in A4 size
print -dpdf '/home/light/plot_A4_generator.pdf';


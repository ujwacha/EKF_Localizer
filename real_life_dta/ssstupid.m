% Experiment 1: Open-Circuit Characteristic
% data2(1,:) = Field Current (A)
% data2(2,:) = Generated Voltage, E_g (V)
x1 = data2(1,:);
y1 = data2(2,:);
p1 = polyfit(x1, y1, 1);   % Linear fit
y1_fit = polyval(p1, x1);

% Experiment 2: Loaded Operation
% data(1,:) = Load Current (A)
% data(2,:) = Terminal Voltage (V)
x2 = data(1,:);
y2 = data(2,:);
p2 = polyfit(x2, y2, 1);   % Linear fit
y2_fit = polyval(p2, x2);

% Create a figure and set it to A4 size (21 x 29.7 cm)
figure;
set(gcf, 'PaperUnits', 'centimeters', ...
         'PaperSize', [21 29.7], ...
         'PaperPosition', [0 0 21 29.7]);

% First subplot: Open-Circuit Characteristic
subplot(2,1,1);
plot(x1, y1, 'bo', 'MarkerFaceColor', 'b'); hold on;
plot(x1, y1_fit, 'r-', 'LineWidth', 2);
xlabel('Field Current (A)');
ylabel('Generated Voltage, E_g (V)');
title('Open-Circuit Characteristic');
legend('Data points', sprintf('Linear Fit (slope = %.2f)', p1(1)));
hold off;

% Second subplot: Loaded Characteristic
subplot(2,1,2);
plot(x2, y2, 'ko', 'MarkerFaceColor', 'k'); hold on;
plot(x2, y2_fit, 'g-', 'LineWidth', 2);
xlabel('Load Current (A)');
ylabel('Terminal Voltage (V)');
title('Loaded Characteristic');
legend('Data points', sprintf('Linear Fit (slope = %.2f)', p2(1)));
hold off;

% Optionally, save the figure as a PDF (A4 size)
print(gcf, '-dpdf', '/home/light/DC_Generator_Characteristics_A4.pdf');


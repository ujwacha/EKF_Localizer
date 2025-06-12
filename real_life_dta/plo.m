clc
clear
clf

data = csvread("latest_sick_data");

% Parameters
Fs = 25;           % Sampling frequency (Hz)
%t = 0:1/Fs:1; % Time vector (1 second)
Fc = 1.5;            % Cutoff frequency (Hz)
Fs2 = Fs/2;



[b, a] = butter(1, Fc/Fs2)

sick_two = data(2:end,1);

sick_filtered = filter(b, a, sick_two);


range_to_check = 690:720;
mean_sick = mean(sick_two(range_to_check))
std_sick = std(sick_two(range_to_check))


filtered_mean_sick = mean(sick_filtered(range_to_check))
filtered_std_sick = std(sick_filtered(range_to_check))




%plot(time, omega_l, "red");
%hold on
plot(sick_two, "green");
hold on
plot(sick_filtered, "red");


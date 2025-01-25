clc
clear
clf


BACK_RADIUS = 0.036;
RIGHT_RADIUS = 0.225;
LEFT_RADIUS =  0.233;
Wheel_Diameter = 0.0574;



data = csvread("10-rot.csv");

% Parameters
Fs = 100;           % Sampling frequency (Hz)
t = 0:1/Fs:1;        % Time vector (1 second)
Fc = 7;             % Cutoff frequency (Hz)


% Design filter
d = designfilt('lowpassiir', ...
    'FilterOrder', 6, ...
    'HalfPowerFrequency', Fc, ...
    'SampleRate', Fs);


vec = data(:,13);

omega_m = filtfilt(d,data(:,7));
omega_l = filtfilt(d,data(:,5));
omega_r = filtfilt(d,data(:,6));
time = data(:,1);





% 
% plot(time, omega_l, "red")
% hold on
% plot(time, omega_r, "blue")
% hold on
% plot(time, omega_m, "yellow");


back_vel = omega_m * Wheel_Diameter / 2.0;
right_vel = omega_r * Wheel_Diameter / 2.0;
left_vel = omega_l * Wheel_Diameter / 2.0;


omega = (right_vel - left_vel) / (RIGHT_RADIUS + LEFT_RADIUS);
vx = (right_vel * LEFT_RADIUS + left_vel * RIGHT_RADIUS) / (RIGHT_RADIUS + LEFT_RADIUS);
vy = back_vel + omega * BACK_RADIUS;

plot(time, vx,"red");
hold on
plot(time, vy, "green");
hold on
plot(time, omega, "yellow");


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
%t = 0:1/Fs:1; % Time vector (1 second)
Fc = 3;            % Cutoff frequency (Hz)
Fs2 = Fs/2;

aFs = 100;
aFc = 3;
aFs2 = aFs/2;
% Design filter
%d = designfilt('lowpassiir', ...
%    'FilterOrder', 1, ...
%    'HalfPowerFrequency', Fc, ..
%    'SampleRate', Fs);


[b, a] = butter(1, Fc/Fs2)

[ab, aa] = butter(2, aFc/aFs2)

ax = data(:,14);
ay = data(:,15);
az = data(:,16);



vec = data(:,13);

omega_m = filter(b, a, data(:,7));
omega_l = filter(b, a, data(:,5));
omega_r = filter(b, a, data(:,6));

fax = filter(ab, aa, ax);
fay = filter(ab, aa, ay);
faz = filter(ab, aa, az);



time = data(:,1);

%plot(time, omega_l, "red");
%hold on
%plot(time, data(:,5), "green");

%plot(time, omega_l, "red")
%hold on
%plot(time, omega_r, "blue")
%hold on
%plot(time, omega_m, "yellow");


back_vel = omega_m * Wheel_Diameter / 2.0;
right_vel = omega_r * Wheel_Diameter / 2.0;
left_vel = omega_l * Wheel_Diameter / 2.0;


omega = (right_vel - left_vel) / (RIGHT_RADIUS + LEFT_RADIUS);
vx = (right_vel * LEFT_RADIUS + left_vel * RIGHT_RADIUS) / (RIGHT_RADIUS + LEFT_RADIUS);
vy = back_vel + omega * BACK_RADIUS;

%plot(time, vx,"red");


plot(time, ax, "green");
hold on
plot(time, fax, "red");
hold on
plot(time, vx);
%hold on
%plot(time, omega, "yellow");


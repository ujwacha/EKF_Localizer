clc
clear
clf

data = csvread("10-rot.csv");

% Parameters
Fs = 100;           % Sampling frequency (Hz)
t = 0:1/Fs:1;        % Time vector (1 second)
Fc = 50;             % Cutoff frequency (Hz)


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


plot(time, omega_l, "red")
hold on
plot(time, omega_r, "blue")
hold on
plot(time, omega_m, "yellow");


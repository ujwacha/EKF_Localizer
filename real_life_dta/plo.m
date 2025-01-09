clc
clear
clf

data = csvread("10-rot.csv");

vec = data(:,13);



%vec = vec .* 3.1415;
%vec = vec ./ 180;

## for i=(1:length(vec))
##  xd = data(1:i,1);
##  yd = data(1:i,5);
##  ad = data(1:i,6);
##  bd = data(1:i,7);
##  clf
##  plot(xd, yd, "red");
##  hold on
##  plot(xd, ad, "blue");
##  plot
##  plot(xd, bd, "purple");
##  pause(0.001);
## end

plot(data(:,1), data(:,14), "red")
hold on
plot(data(:,1), data(:,15), "blue")
#hold on
#plot(data(:,1), data(:,16), "yellow")

% hold on
% plot(data(:,1), data(:,8))

mean(data(:,16))
var(data(:,16))


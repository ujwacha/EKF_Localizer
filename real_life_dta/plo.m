data = csvread("10-rot.csv");

vec = data(:,13);



#vec = vec .* 3.1415;
#vec = vec ./ 180;

#for i=(1:length(vec))
#  xd = data(1:i,5);
#  yd = data(1:i,6);
#  clf
#  plot(xd, yd, "o-");
#  pause(0.001);
#end

plot(data(:,1), data(:,13))

length(data(:,7))



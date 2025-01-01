clc

clear
clf

data = csvread("data.csv");

#for i=(50:length(data))

  time = data(:,1);
  pth = data(:, 10);
  kth = data(:, 7);
  ith = data(:, 14);
  oth = data(:, 13);

  #kx = data(50:i, 5);
  #ky = data(50:i, 6);

  #ox = data(50:i, 11);
  #oy = data(50:i, 12);


  plot(time,kth, "red");
  hold on
  plot(time, oth, "green");
  hold on
  plot(time, ith, "black");
  hold on
  plot(time, pth, "blue");
 # pause(0.0001)
#end


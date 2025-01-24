# # Set terminal type (optional)
# set terminal qt       # Use default window
# # Set labels and grid
# set xlabel "X-axis"
# set ylabel "Y-axis"
# set grid

# # Specify the delimiter as comma
# set datafile separator ","

# # Enable multiplot layout
# # set multiplot layout 2,2 columns

# # First plot: Column 3 (Y) vs. Column 2 (X)
# plot "data.csv" using 8:9 with linespoints title "X-Y Data"

# pause -1 "Press any key to close the window..."

# # Second plot: Column 4 (Angle) vs. Column 1 (Time)
# plot "data.csv" using 1:10 with linespoints title "Angle"



# pause -1 "Press any key to close the window..."
# plot "data.csv" using 6:5 with linespoints title "X-Y Kalman "

# pause -1 "Press any key to close the window..."
# plot "data.csv" using 1:7 with linespoints title "Angle Kalman"

# # End multiplot
# # unset multiplot

# pause -1 "Press any key to close the window..."
# plot "data.csv" using 11:12 with linespoints title "Odom"

# pause -1 "Press any key to close the window..."
# plot "data.csv" using 1:13 with linespoints title "Odom Angle"

# pause -1 "Press any key to close the window..."
# plot "data.csv" using 1:14 with linespoints title "IMU YAW"




# # Pause to keep the window open
# pause -1 "Press any key to close the window..."

# Basic settings
set datafile separator ","
set grid
set xlabel "X-axis"
set ylabel "Y-axis"

# Equal aspect ratio for all plots
set size ratio -1

# Set terminal (optional)
set terminal qt

# First plot: X-Y Data
plot "data.csv" using 8:9 with linespoints title "X-Y Data"
pause -1 "Press any key to continue..."

# Second plot: Angle
plot "data.csv" using 1:10 with linespoints title "Angle"
pause -1 "Press any key to continue..."

# Third plot: Kalman Position
plot "data.csv" using 6:5 with linespoints title "X-Y Kalman"
pause -1 "Press any key to continue..."

# Fourth plot: Kalman Angle
plot "data.csv" using 1:7 with linespoints title "Angle Kalman"
pause -1 "Press any key to continue..."

# Fifth plot: Odometry
plot "data.csv" using 11:12 with linespoints title "Odom"
pause -1 "Press any key to continue..."

# Sixth plot: Odometry Angle
plot "data.csv" using 1:13 with linespoints title "Odom Angle"
pause -1 "Press any key to continue..."

# Seventh plot: IMU YAW
plot "data.csv" using 1:14 with linespoints title "IMU YAW"
pause -1 "Press any key to close..."

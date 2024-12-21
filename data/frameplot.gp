# Set terminal type (optional)
set terminal qt       # Use default window
# Set labels and grid
set xlabel "X-axis"
set ylabel "Y-axis"
set grid

# Specify the delimiter as comma
set datafile separator ","

# Enable multiplot layout
set multiplot layout 1,2 columns

# First plot: Column 3 (Y) vs. Column 2 (X)
plot "data.csv" using 2:3 with linespoints title "X-Y Data"

# Second plot: Column 4 (Angle) vs. Column 1 (Time)
plot "data.csv" using 1:4 with linespoints title "Angle"

# End multiplot
unset multiplot

# Pause to keep the window open
pause -1 "Press any key to close the window..."

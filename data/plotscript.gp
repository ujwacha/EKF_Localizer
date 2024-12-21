# Set the terminal type and output file (optional)
set xlabel "X-axis"
set ylabel "Y-axis"

set grid
# Specify the delimiter as a comma (for CSV files)
set datafile separator ","

# Plot the data
# Assuming the CSV file has no header and X is in column 1 and Y in column 2
plot "data.csv" using 2:3 with linespoints title "X-Y Data"

pause -1 "Press any key to close the window..."


plot "data.csv" using 1:4 with linespoints title "angle"

pause -1 "Press any key to close the window..."

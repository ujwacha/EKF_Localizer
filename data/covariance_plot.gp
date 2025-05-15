# set xlabel "X-axis"
# set ylabel "Y-axis"
# set grid

# # Specify the delimiter as a comma (CSV file)
# set datafile separator ","

# # Animate point
# n = system("wc -l < data.csv")  # Count the number of lines in the CSV file

# do for [i=1:n] {
#     plot "data.csv" using 1:15 every ::1::i with lines title "Cov X" \
# 	 "data.csv" using 1:16 every ::1::i with lines title "Cov Y"

#     pause 0.001  # Adjust delay (in seconds) between frames
    
# }


# pause -1 "Press any key to close the window..."

# Set labels and grid
set xlabel "X-axis"
set ylabel "Y-axis"
set grid

# Specify the delimiter as a comma (CSV file)
set datafile separator ","

# Use stats to count the number of data records
# 'nooutput' prevents stats from printing statistics to the console
stats "data.csv" using 1 nooutput
# The number of records is stored in the variable STATS_records
n = STATS_records

# Animate plotting data point by point
do for [i=1:n] {
    # Plot data from line 1 up to line 'i' (every ::1::i)
    # Plotting with 'lines' connects the points as they are added
    plot "data.csv" using 1:15 every ::1::i with lines title "Cov X"

    # Adjust delay (in seconds) between frames
    # Use 0.001 for a quick animation, increase for slower
    pause 0.001
}

# Pause until a key is pressed after the animation finishes
pause -1 "Press any key to close the window..."

set xlabel "X-axis"
set ylabel "Y-axis"
set grid

# Specify the delimiter as a comma (CSV file)
set datafile separator ","

# Animate point
n = system("wc -l < robot_data_n.csv")  # Count the number of lines in the CSV file

plot "robot_data_n.csv" using 1:15 every ::1::n with lines title "Angle"

    

pause -1 "Press any key to close the window..."

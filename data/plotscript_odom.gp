set xlabel "X-axis"
set ylabel "Y-axis"
set grid

# Specify the delimiter as a comma (CSV file)
set datafile separator ","

# Animate point
n = system("wc -l < data.csv")  # Count the number of lines in the CSV file

do for [i=1:n] {
    plot "data.csv" using 11:12 every ::1::i with lines title "ODOM", \
         "data.csv" using 11:12 every ::i::i with points pointtype 7 pointsize 1 title sprintf("Frame %d", i), \
	 
    # plot "data.csv" using 1:4 every ::1::i with linespoints title "Angle"


    pause 0.001  # Adjust delay (in seconds) between frames
    
}

pause -1 "Press any key to close the window..."
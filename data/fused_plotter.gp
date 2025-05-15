# Set the data file separator to comma
set datafile separator ","

# Count the number of data lines (skip header if needed)
n = system("wc -l < data.csv") - 1  # Adjust if your CSV has a header row

# Set the terminal for visualization
set terminal wxt title "Robot Position and Covariance Animation" enhanced

# Enable multiplot mode (only once, outside the loop)
set multiplot

# Animation loop
do for [i=2:n+1] {  # Start from line 2 if your data has a header
    # Clear the previous frame
    clear
    clear
    clear

    # === LEFT PLOT: XY Position ===
    set origin 0.0, 0.0
    set size 0.49, 1.0
    set xlabel "X-axis"
    set ylabel "Y-axis"
    set xrange [0:7.5]
    set yrange [0:4]
    set grid
    plot "data.csv" using 5:6 every ::2::i with lines title "Locus", \
         '' using 5:6 every ::i::i with points pointtype 7 pointsize 1 title sprintf("Frame %d", i-1)

    # === RIGHT PLOT: Covariance of X ===
    set origin 0.51, 0.0
    set size 0.49, 1.0
    set xlabel "Time Step"
    set ylabel "Covariance X"
    set grid
    set autoscale x
    set autoscale y
    plot "data.csv" using 1:15 every ::2::i with lines title "Cov X"

    # Force gnuplot to refresh the window
    
    # Pause for animation effect
    pause 0.01  # Slower for visibility (adjust as needed)
}

# Exit multiplot mode
unset multiplot

# Wait for user input before closing the window
pause -1 "Press any key to close the window..."
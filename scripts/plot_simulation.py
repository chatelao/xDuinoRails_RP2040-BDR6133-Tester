import pandas as pd
import matplotlib.pyplot as plt
import os

# Define the input and output file paths
CSV_FILE = 'simulation_results.csv'
OUTPUT_FILE = 'simulation_graph.png'

# Check if the simulation results file exists
if not os.path.exists(CSV_FILE):
    print(f"Error: Simulation results file not found at '{CSV_FILE}'")
    # Create a dummy empty file to satisfy the CI artifact upload
    with open(OUTPUT_FILE, 'w') as fp:
        pass
    exit(0)

# Read the simulation data using pandas
try:
    data = pd.read_csv(CSV_FILE)
except pd.errors.EmptyDataError:
    print(f"Warning: '{CSV_FILE}' is empty. No plot will be generated.")
    # Create a dummy empty file
    with open(OUTPUT_FILE, 'w') as fp:
        pass
    exit(0)

# Create a plot
plt.figure(figsize=(12, 8))

# Plot TargetSpeed vs. Timestamp
plt.plot(data['Timestamp'], data['TargetSpeed'], label='Target Speed', linestyle='--', color='gray')

# Plot MeasuredSpeed vs. Timestamp
plt.plot(data['Timestamp'], data['MeasuredSpeed'], label='Measured Speed', color='blue')

# Create a secondary y-axis for the PWM value
plt.twinx()
plt.plot(data['Timestamp'], data['PWM'], label='PWM Duty Cycle', color='red', alpha=0.6, linestyle=':')

# Add labels and title for clarity
plt.title('Motor Control Simulation Results')
plt.xlabel('Timestamp (ms)')

# Set labels for both y-axes
plt.gca().get_yaxis().set_label_coords(1.1, 0.5)
plt.ylabel('PWM Duty Cycle', color='red')
plt.gcf().get_axes()[0].set_ylabel('Speed (pulses/interval)', color='blue')


# Create a unified legend
lines, labels = plt.gcf().get_axes()[0].get_legend_handles_labels()
lines2, labels2 = plt.gcf().get_axes()[1].get_legend_handles_labels()
plt.gcf().get_axes()[0].legend(lines + lines2, labels + labels2, loc='best')


plt.grid(True)

# Save the plot to a file
plt.savefig(OUTPUT_FILE)

print(f"Plot saved to '{OUTPUT_FILE}'")

import pandas as pd
import matplotlib.pyplot as plt

def plot_simulation_results():
    """
    Reads the simulation results from simulation_results.csv and generates a plot.
    """
    df = pd.read_csv("simulation_results.csv")

    plt.figure(figsize=(10, 6))
    plt.plot(df["Timestamp"], df["TargetSpeed"], label="Target Speed")
    plt.plot(df["Timestamp"], df["MeasuredSpeed"], label="Measured Speed")
    plt.xlabel("Timestamp")
    plt.ylabel("Speed")
    plt.title("PI Controller Simulation")
    plt.legend()
    plt.grid(True)
    plt.savefig("simulation_graph.png")
    print("Plot saved to simulation_graph.png")

if __name__ == "__main__":
    plot_simulation_results()

"""Plot logged drone MPC results from a CSV file."""

import argparse
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt


def plot_results(csv_path: str) -> None:
    df = pd.read_csv(csv_path)

    t = df['time']

    plt.figure(figsize=(10, 5))
    plt.plot(t, df['xref'], label='x_ref')
    plt.plot(t, df['x'], label='x')
    plt.plot(t, df['yref'], label='y_ref')
    plt.plot(t, df['y'], label='y')
    plt.plot(t, df['zref'], label='z_ref')
    plt.plot(t, df['z'], label='z')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.title('Drone position tracking')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    plt.figure(figsize=(10, 5))
    plt.plot(t, df['ax_cmd'], label='ax_cmd')
    plt.plot(t, df['ay_cmd'], label='ay_cmd')
    plt.plot(t, df['az_cmd'], label='az_cmd')
    plt.xlabel('Time [s]')
    plt.ylabel('Acceleration command [m/s^2]')
    plt.title('MPC commands')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', type=str, required=True, help='Path to the logged CSV file.')
    args = parser.parse_args()

    if not Path(args.csv).exists():
        raise FileNotFoundError(f'CSV file not found: {args.csv}')

    plot_results(args.csv)

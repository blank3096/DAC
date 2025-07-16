
import pandas as pd
import matplotlib.pyplot as plt
import os

def generate_graphs_from_csv(input_dir):
    """
    Reads all CSV files from a specified directory, generates line graphs
    for numerical sensor data against their respective timestamps,
    and saves them as PNG images.

    Args:
        input_dir (str): The path to the directory containing the CSV files.
    """
    if not os.path.exists(input_dir):
        print(f"Error: Input directory '{input_dir}' not found.")
        return

    # Create a directory to save the generated plots
    plots_output_dir = os.path.join(input_dir, "plots")
    if not os.path.exists(plots_output_dir):
        os.makedirs(plots_output_dir)
        print(f"Created plots output directory: {plots_output_dir}")

    csv_files = [f for f in os.listdir(input_dir) if f.endswith('.csv')]

    if not csv_files:
        print(f"No CSV files found in '{input_dir}'.")
        return

    for csv_file in csv_files:
        file_path = os.path.join(input_dir, csv_file)
        print(f"Processing '{csv_file}'...")

        try:
            # Read the CSV file into a pandas DataFrame
            df = pd.read_csv(file_path)

            # Ensure 'Timestamp' column exists and convert it to datetime objects
            if 'Timestamp' not in df.columns:
                print(f"Warning: 'Timestamp' column not found in '{csv_file}'. Skipping graph generation for this file.")
                continue
            df['Timestamp'] = pd.to_datetime(df['Timestamp'])

            # Identify numerical columns for plotting, excluding timing data
            # We'll plot each sensor data column against the Timestamp.
            columns_to_plot = [
                col for col in df.columns
                if pd.api.types.is_numeric_dtype(df[col]) and
                   col not in ['Start_s', 'End_s', 'Duration_s']
            ]

            if not columns_to_plot:
                print(f"No numerical sensor data columns found in '{csv_file}' to plot. Skipping.")
                continue

            # Generate a plot for each relevant numerical column
            for col in columns_to_plot:
                plt.figure(figsize=(12, 6)) # Set figure size for better readability
                # Plotting sensor data against 'Timestamp'
                plt.plot(df['Timestamp'], df[col], marker='o', linestyle='-', markersize=4)

                plt.title(f'{col.replace("_", " ").title()} Over Time ({os.path.splitext(csv_file)[0].replace("_", " ").title()})')
                plt.xlabel('Timestamp') # Reverted x-axis label
                plt.ylabel(col.replace("_", " ").title())
                plt.grid(True)
                plt.tight_layout() # Adjust layout to prevent labels from overlapping

                # Construct output filename
                # Changed filename to reflect plotting against Timestamp again
                plot_filename = f"{os.path.splitext(csv_file)[0]}_{col}_over_time.png"
                plot_output_path = os.path.join(plots_output_dir, plot_filename)
                plt.savefig(plot_output_path)
                plt.close() # Close the plot to free memory
                print(f"  Generated plot: {plot_output_path}")

        except Exception as e:
            print(f"Error processing '{csv_file}': {e}")

# --- Example Usage ---
if __name__ == '__main__':
    # Assuming your previous script created 'sensor_csv_output'
    # in the same directory as this script.
    output_folder_from_previous_script = "sensor_csv_output"

    # Run the graph generator
    generate_graphs_from_csv(output_folder_from_previous_script)

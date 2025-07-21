import pandas as pd
import matplotlib.pyplot as plt
import os
import logging
import sys

# Configure logging for the graph generator script
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# Console handler
console_handler = logging.StreamHandler(sys.stdout)
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)

# --- Unit Mapping for Y-axis Labels ---
# This dictionary maps common sensor data column names to their units.
# Extend this as needed for other sensor types/columns.
UNIT_MAP = {
    'pressure': 'bar',
    'weight_grams': 'g',
    'flow_rate_lpm': 'LPM',
    'temp_c': '°C',
    'temp_f': '°F',
    'rpm': 'RPM'
}

def get_y_label_with_unit(column_name):
    """
    Generates a human-readable Y-axis label including units.
    """
    title_case_name = column_name.replace("_", " ").title()
    unit = UNIT_MAP.get(column_name.lower()) # Get unit, case-insensitive match
    if unit:
        return f"{title_case_name} ({unit})"
    return title_case_name

def generate_graphs_from_csv(input_dir):
    """
    Reads all CSV files from a specified directory, generates line graphs
    for numerical sensor data against their respective timestamps,
    and saves them as PNG images.

    Args:
        input_dir (str): The path to the directory containing the CSV files.
    """
    if not os.path.exists(input_dir):
        logger.error(f"Error: Input directory '{input_dir}' not found.")
        return

    # Create a directory to save the generated plots
    plots_output_dir = os.path.join(input_dir, "plots")
    if not os.path.exists(plots_output_dir):
        os.makedirs(plots_output_dir)
        logger.info(f"Created plots output directory: {plots_output_dir}")

    csv_files = [f for f in os.listdir(input_dir) if f.endswith('.csv')]

    if not csv_files:
        logger.info(f"No CSV files found in '{input_dir}'.")
        return

    logger.info(f"Found {len(csv_files)} CSV files to process in '{input_dir}'.")

    for csv_file in csv_files:
        file_path = os.path.join(input_dir, csv_file)
        logger.info(f"Processing '{csv_file}'...")

        try:
            # Read the CSV file into a pandas DataFrame
            df = pd.read_csv(file_path)

            # Check if DataFrame is empty
            if df.empty:
                logger.warning(f"CSV file '{csv_file}' is empty. Skipping graph generation.")
                continue

            # Ensure 'timestamp' column exists and convert it to datetime objects
            # The column name is 'timestamp' from log_parser.py
            if 'timestamp' not in df.columns:
                logger.warning(f"Warning: 'timestamp' column not found in '{csv_file}'. Skipping graph generation for this file.")
                continue
            
            # Attempt to convert to datetime. This should now work with the cli.py fix.
            # If it still fails, you might need to specify a format like errors='coerce'
            # to turn unparseable dates into NaT (Not a Time).
            df['timestamp'] = pd.to_datetime(df['timestamp'])

            # Identify numerical columns for plotting, excluding timing data.
            # 'duration_us' is now explicitly excluded.
            columns_to_plot = [
                col for col in df.columns
                if pd.api.types.is_numeric_dtype(df[col]) and
                   col not in ['duration_us'] # Exclude duration from primary plots
            ]

            if not columns_to_plot:
                logger.warning(f"No numerical sensor data columns found in '{csv_file}' to plot. Skipping.")
                continue

            # Generate a plot for each relevant numerical column
            for col in columns_to_plot:
                plt.figure(figsize=(14, 7)) # Adjust figure size for better readability
                
                # Plotting sensor data against 'timestamp'
                plt.plot(df['timestamp'], df[col], marker='o', linestyle='-', markersize=3, label=col.replace("_", " ").title())

                # Dynamically generate title based on file and column
                sensor_name = os.path.splitext(csv_file)[0].replace("_", " ").title()
                plt.title(f'{get_y_label_with_unit(col)} for {sensor_name} Over Time', fontsize=16)
                plt.xlabel('Timestamp', fontsize=12)
                plt.ylabel(get_y_label_with_unit(col), fontsize=12)
                
                plt.grid(True, linestyle='--', alpha=0.7)
                plt.xticks(rotation=45, ha='right') # Rotate x-axis labels for better readability
                plt.legend(loc='best') # Add a legend
                plt.tight_layout() # Adjust layout to prevent labels from overlapping

                # Construct output filename
                plot_filename = f"{os.path.splitext(csv_file)[0]}_{col}_time_series.png"
                plot_output_path = os.path.join(plots_output_dir, plot_filename)
                plt.savefig(plot_output_path)
                plt.close() # Close the plot to free memory
                logger.info(f"  Generated plot: {plot_output_path}")

        except pd.errors.EmptyDataError:
            logger.warning(f"CSV file '{csv_file}' is empty or contains only headers. Skipping.")
        except Exception as e:
            logger.error(f"Error processing '{csv_file}': {e}", exc_info=True) # Log full traceback for debugging

# --- Example Usage ---
if __name__ == '__main__':
    # Assuming your previous script created 'csv_exports'
    # in the same directory as this script.
    output_folder_from_previous_script = "csv_exports"

    # Run the graph generator
    generate_graphs_from_csv(output_folder_from_previous_script)
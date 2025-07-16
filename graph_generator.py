import pandas as pd
import matplotlib.pyplot as plt
import os
import argparse
import re

def generate_graphs(csv_dir='csv_data', output_dir='graphs'):
    """
    Reads CSV files from csv_dir and generates individual and category-wise
    time-series graphs, saving them as PNG files in output_dir.
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created output directory: {output_dir}")

    csv_files = [f for f in os.listdir(csv_dir) if f.endswith('.csv')]
    
    # Dictionary to hold dataframes grouped by sensor category (e.g., 'pressure', 'loadcell')
    category_data = {} 
    
    print(f"Generating graphs from CSVs in: {csv_dir}...")

    for csv_file in csv_files:
        file_path = os.path.join(csv_dir, csv_file)
        try:
            df = pd.read_csv(file_path, index_col='Timestamp', parse_dates=True)
            if df.empty:
                print(f"Skipping empty CSV: {csv_file}")
                continue

            sensor_full_name = os.path.splitext(csv_file)[0].replace('_', ' ').title() # e.g., "Pressure_Id_0_Pressure" -> "Pressure Id 0 Pressure"
            
            # Determine sensor category (e.g., 'pressure', 'loadcell', 'temperature')
            # Extract category from filename (e.g., 'pressure_id_0_pressure.csv' -> 'pressure')
            category_match = re.match(r'([a-z]+)', os.path.splitext(csv_file)[0].lower())
            sensor_category = category_match.group(1) if category_match else "other"
            
            # --- Individual Graph for each sensor ---
            sensor_group_folder = os.path.join(output_dir, sensor_category.capitalize() + '_Individual')
            if not os.path.exists(sensor_group_folder):
                os.makedirs(sensor_group_folder)

            plt.figure(figsize=(12, 6))
            plt.plot(df.index, df['Value'], label=sensor_full_name)
            plt.xlabel('Time')
            plt.ylabel('Value') # Placeholder, can be refined per sensor type if needed
            plt.title(f'{sensor_full_name} Over Time')
            plt.grid(True)
            plt.legend()
            plt.tight_layout()
            
            graph_filename = os.path.join(sensor_group_folder, f'{os.path.splitext(csv_file)[0]}.png')
            plt.savefig(graph_filename)
            plt.close()
            print(f"Generated individual graph: {graph_filename}")

            # --- Prepare for Category Graphs ---
            if sensor_category not in category_data:
                category_data[sensor_category] = {}
            category_data[sensor_category][sensor_full_name] = df # Store df for category plotting

        except Exception as e:
            print(f"Error processing {csv_file}: {e}")

    # --- Category-wise Graphs ---
    for category, sensor_dfs in category_data.items():
        if not sensor_dfs:
            continue # Skip empty categories

        category_folder = os.path.join(output_dir, category.capitalize() + '_Grouped')
        if not os.path.exists(category_folder):
            os.makedirs(category_folder)

        plt.figure(figsize=(14, 8))
        for sensor_full_name, df in sensor_dfs.items():
            plt.plot(df.index, df['Value'], label=sensor_full_name)
        
        plt.xlabel('Time')
        plt.ylabel('Value') # Placeholder, consider refining with actual units
        plt.title(f'All {category.capitalize()} Sensors Over Time')
        plt.grid(True)
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left') # Place legend outside
        plt.tight_layout(rect=[0, 0, 0.85, 1]) # Adjust layout to make space for legend
        
        graph_filename = os.path.join(category_folder, f'{category}_grouped.png')
        plt.savefig(graph_filename)
        plt.close()
        print(f"Generated grouped graph for {category}: {graph_filename}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Generate graphs from sensor CSV data.")
    parser.add_argument('--csv-dir', type=str, default='csv_data',
                        help='Directory containing the CSV files.')
    parser.add_argument('--graph-dir', type=str, default='graphs',
                        help='Directory to save the generated graphs.')
    args = parser.parse_args()

    generate_graphs(args.csv_dir, args.graph_dir)

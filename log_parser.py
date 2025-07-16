import re
import pandas as pd
import os
import subprocess

def parse_log_to_csv(log_file='Work.log', output_dir='csv_data'):
    """
    Parses the Work.log file to extract sensor data and saves it into
    separate CSV files for each unique sensor.
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created output directory: {output_dir}")

    sensor_data = {}
    
    # Regex to capture timestamp and all sensor data lines
    # It tries to capture a common timestamp format and then any line containing sensor data.
    # The key is to grab the full line and then parse it based on known patterns.
    # We'll rely on the specific 'name: value unit' pattern.
    log_pattern = re.compile(r'^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}) - INFO - \[([^\]]+)\]\s+([^:]+):\s+(.*)$')
    
    # Specific patterns for different sensor types
    # Group 1: Sensor Name/ID (e.g., 'Pressure ID 0', 'LoadCell ID 6')
    # Group 2: Measurement name (e.g., 'pressure', 'weight_grams', 'temp_c', 'rpm')
    # Group 3: Value (e.g., '1.23', '100.567', '25.1', '1200.5')
    # Group 4: Unit (e.g., 'bar', 'g', 'C', 'RPM') - Optional for some.
    sensor_value_pattern = re.compile(r'^(.*?)\s*:\s*([-\d\.]+) (\S+)$') # For 'name: value unit'
    sensor_value_no_unit_pattern = re.compile(r'^(.*?)\s*:\s*([-\d\.]+)$') # For 'name: value' (e.g., just 'state' for relays)

    print(f"Parsing log file: {log_file}...")
    with open(log_file, 'r') as f:
        for line in f:
            match = log_pattern.match(line)
            if match:
                timestamp_str, header, measurement_type, value_str = match.groups()
                # Use pd.to_datetime for accurate time parsing
                timestamp = pd.to_datetime(timestamp_str, format='%Y-%m-%d %H:%M:%S,%f')

                # Handle different measurement types (e.g., 'pressure', 'temp_c temp_f')
                # Split value_str into individual measurements if needed (e.g., temp_c and temp_f)
                
                # Check for "No data" or "ERR"
                if "No data" in value_str or "ERR" in value_str:
                    continue # Skip lines with no valid data

                # Handle temperature sensors separately due to multiple values per line
                if "temp_c" in measurement_type and "temp_f" in measurement_type:
                    # Expected format: "temp_c: 25.1 C temp_f: 77.2 F"
                    temp_c_match = re.search(r'temp_c:\s*([-\d\.]+) C', value_str)
                    temp_f_match = re.search(r'temp_f:\s*([-\d\.]+) F', value_str)
                    
                    if temp_c_match:
                        sensor_name_c = f"{header}_temp_c"
                        value_c = float(temp_c_match.group(1))
                        if sensor_name_c not in sensor_data:
                            sensor_data[sensor_name_c] = []
                        sensor_data[sensor_name_c].append({'Timestamp': timestamp, 'Value': value_c})
                    
                    if temp_f_match:
                        sensor_name_f = f"{header}_temp_f"
                        value_f = float(temp_f_match.group(1))
                        if sensor_name_f not in sensor_data:
                            sensor_data[sensor_name_f] = []
                        sensor_data[sensor_name_f].append({'Timestamp': timestamp, 'Value': value_f})

                # Handle other sensor types
                else:
                    value_match = sensor_value_pattern.match(value_str)
                    if value_match:
                        # Extract the actual value and unit
                        # Here, measurement_name would be "pressure", "weight_grams", etc.
                        # We use header for the full sensor ID (e.g., "Pressure ID 0")
                        sensor_name = f"{header}_{measurement_type.replace(':', '').strip()}"
                        value = float(value_match.group(2))
                        # unit = value_match.group(3) # Can store unit if needed later

                        if sensor_name not in sensor_data:
                            sensor_data[sensor_name] = []
                        sensor_data[sensor_name].append({'Timestamp': timestamp, 'Value': value})
                    else:
                        # Fallback for simpler 'name: value' formats without units explicitly parsed by regex
                        value_match_no_unit = sensor_value_no_unit_pattern.match(value_str)
                        if value_match_no_unit:
                            sensor_name = f"{header}_{measurement_type.replace(':', '').strip()}"
                            try:
                                value = float(value_match_no_unit.group(2))
                                if sensor_name not in sensor_data:
                                    sensor_data[sensor_name] = []
                                sensor_data[sensor_name].append({'Timestamp': timestamp, 'Value': value})
                            except ValueError:
                                # Handle cases where value is not a float (e.g., 'OPEN', 'CLOSE')
                                # For this script, we're focusing on numeric sensor values for graphs.
                                pass 
                                # print(f"Skipping non-numeric value for {sensor_name}: {value_match_no_unit.group(2)}")


    # Save data to CSV files
    for sensor_name, data_points in sensor_data.items():
        df = pd.DataFrame(data_points)
        df.set_index('Timestamp', inplace=True)
        # Sanitize sensor_name for filename (remove spaces, brackets, etc.)
        filename = re.sub(r'[\[\]\s:]', '_', sensor_name).strip('_').lower() + '.csv'
        output_path = os.path.join(output_dir, filename)
        df.to_csv(output_path)
        print(f"Saved {sensor_name} data to {output_path}")

    return output_dir

if __name__ == '__main__':
    csv_output_directory = parse_log_to_csv()
    
    # Call the graph generating script after CSVs are created
    print("\nCSV processing complete. Calling graph generation script...")
    # Ensure graph_generator.py is in the same directory or provide its full path
    try:
        subprocess.run(['python', 'graph_generator.py', '--csv-dir', csv_output_directory], check=True)
    except FileNotFoundError:
        print("Error: 'graph_generator.py' not found. Make sure it's in the same directory or its path is correct.")
    except subprocess.CalledProcessError as e:
        print(f"Error running graph_generator.py: {e}")

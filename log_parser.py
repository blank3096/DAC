import re
import pandas as pd
import os
import subprocess
import logging # Import logging for internal messages if needed

# Configure a simple logger for this script's internal messages
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
script_logger = logging.getLogger(__name__)

def parse_log_to_csv(log_file='Work.log', output_dir='csv_data'):
    """
    Parses the Work.log file to extract sensor data and saves it into
    separate CSV files for each unique sensor.
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        script_logger.info(f"Created output directory: {output_dir}")

    sensor_data = {}
    
    # Updated Regex:
    # Group 1: Timestamp (e.g., '2025-07-16 03:39:43')
    # Group 2: Log Level (e.g., 'INFO') - added for robustness, though not strictly used
    # Group 3: Header content (e.g., 'Pressure ID 0', 'Relay ID 0', 'Timing Category Pressure')
    # Group 4: The rest of the line after the header (e.g., 'No data', 'pressure: 1.23 bar', 'state: CLOSE')
    log_line_pattern = re.compile(r'^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}) - (INFO|ERROR) - \[([^\]]+)\]\s*(.*)$')
    
    # Specific patterns for different sensor types within the captured content
    # For 'name: value unit' (e.g., 'pressure: 1.23 bar', 'weight_grams: 100.567 g', 'flow_rate_lpm: 0.750 LPM', 'rpm: 1200.5 RPM')
    value_unit_pattern = re.compile(r'([^:]+):\s*([-\d\.]+) (\S+)$')
    
    # For Temperature sensors with two values (e.g., 'temp_c: 25.1 C temp_f: 77.2 F')
    temp_pattern = re.compile(r'temp_c:\s*([-\d\.]+) C\s*temp_f:\s*([-\d\.]+) F')

    script_logger.info(f"Parsing log file: {log_file}...")
    line_count = 0
    parsed_entries = 0

    with open(log_file, 'r') as f:
        for line in f:
            line_count += 1
            match = log_line_pattern.match(line)
            if match:
                timestamp_str, log_level, header_content, rest_of_line = match.groups()
                timestamp = pd.to_datetime(timestamp_str, format='%Y-%m-%d %H:%M:%S,%f')

                # Skip lines that are not sensor data we want to plot
                if "No data" in rest_of_line or "ERR" in rest_of_line or "RELAY STATES" in header_content or "Relay ID" in header_content or "TIMING DATA" in header_content or "Timing" in header_content:
                    continue
                
                # --- Handle Temperature Sensors (two values) ---
                temp_match = temp_pattern.match(rest_of_line)
                if temp_match:
                    temp_c_value = float(temp_match.group(1))
                    temp_f_value = float(temp_match.group(2))

                    sensor_name_c = f"{header_content}_temp_c"
                    sensor_name_f = f"{header_content}_temp_f"

                    if sensor_name_c not in sensor_data:
                        sensor_data[sensor_name_c] = []
                    sensor_data[sensor_name_c].append({'Timestamp': timestamp, 'Value': temp_c_value})
                    
                    if sensor_name_f not in sensor_data:
                        sensor_data[sensor_name_f] = []
                    sensor_data[sensor_name_f].append({'Timestamp': timestamp, 'Value': temp_f_value})
                    parsed_entries += 2
                    continue # Move to next line after processing temperature

                # --- Handle other single-value sensors ---
                # Split the rest_of_line by spaces to find individual 'field: value unit' pairs
                # This handles cases like "pressure: 1.23 bar" or "rpm: 1200.5 RPM"
                parts = rest_of_line.strip().split(' ')
                current_measurement_name = ""
                current_value_str = ""
                current_unit = ""

                # Iterate through parts to reconstruct field: value unit
                for i, part in enumerate(parts):
                    if ':' in part: # This is likely the start of a new measurement
                        if current_measurement_name: # If we have a pending measurement, process it
                            value_match = value_unit_pattern.match(f"{current_measurement_name}:{current_value_str} {current_unit}".strip())
                            if value_match:
                                try:
                                    value = float(value_match.group(2))
                                    # Use header_content for the sensor ID part, and the measurement name for the specific field
                                    # e.g., "Pressure ID 0_pressure"
                                    sensor_full_id = f"{header_content}_{value_match.group(1).strip()}"
                                    if sensor_full_id not in sensor_data:
                                        sensor_data[sensor_full_id] = []
                                    sensor_data[sensor_full_id].append({'Timestamp': timestamp, 'Value': value})
                                    parsed_entries += 1
                                except ValueError:
                                    script_logger.debug(f"Skipping non-numeric value: {value_match.group(2)} in line {line_count}")
                            current_measurement_name = ""
                            current_value_str = ""
                            current_unit = ""

                        # Start new measurement
                        current_measurement_name = part.split(':')[0].strip()
                        current_value_str = part.split(':')[1].strip() if len(part.split(':')) > 1 else ""
                    else: # This is part of the value or unit for the current measurement
                        if current_measurement_name:
                            if current_value_str and not current_unit: # If value is set, next part is unit
                                current_unit = part.strip()
                            else: # Else, it's part of the value (e.g., if value has spaces, though not expected here)
                                current_value_str += " " + part.strip()
                
                # Process the last measurement after the loop finishes
                if current_measurement_name:
                    value_match = value_unit_pattern.match(f"{current_measurement_name}:{current_value_str} {current_unit}".strip())
                    if value_match:
                        try:
                            value = float(value_match.group(2))
                            sensor_full_id = f"{header_content}_{value_match.group(1).strip()}"
                            if sensor_full_id not in sensor_data:
                                sensor_data[sensor_full_id] = []
                            sensor_data[sensor_full_id].append({'Timestamp': timestamp, 'Value': value})
                            parsed_entries += 1
                        except ValueError:
                            script_logger.debug(f"Skipping non-numeric value: {value_match.group(2)} in line {line_count}")

            else:
                script_logger.debug(f"Skipping non-data line: {line.strip()}")


    # Save data to CSV files
    if not sensor_data:
        script_logger.warning("No sensor data found to save to CSVs.")
        return output_dir

    for sensor_name, data_points in sensor_data.items():
        df = pd.DataFrame(data_points)
        df.set_index('Timestamp', inplace=True)
        # Sanitize sensor_name for filename (remove spaces, brackets, etc.)
        # Example: "Pressure ID 0_pressure" -> "pressure_id_0_pressure.csv"
        filename = re.sub(r'[\[\]\s:]', '_', sensor_name).strip('_').lower() + '.csv'
        output_path = os.path.join(output_dir, filename)
        df.to_csv(output_path)
        script_logger.info(f"Saved {len(data_points)} entries for '{sensor_name}' to {output_path}")

    script_logger.info(f"Successfully parsed {parsed_entries} sensor data entries from {line_count} lines.")
    return output_dir

if __name__ == '__main__':
    csv_output_directory = parse_log_to_csv()
    
    # Call the graph generating script after CSVs are created
    script_logger.info("\nCSV processing complete. Calling graph generation script...")
    # Ensure graph_generator.py is in the same directory or provide its full path
    try:
        # Pass the csv_output_directory to the graph generator
        subprocess.run(['python', 'graph_generator.py', '--csv-dir', csv_output_directory], check=True)
    except FileNotFoundError:
        script_logger.error("Error: 'graph_generator.py' not found. Make sure it's in the same directory or its path is correct.")
    except subprocess.CalledProcessError as e:
        script_logger.error(f"Error running graph_generator.py: {e}")
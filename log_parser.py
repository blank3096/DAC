import re
import csv
import os
from collections import defaultdict

def parse_log_to_csv(log_file_path, output_dir):
    """
    Parses a sensor log file, correlates sensor data with individual timing data,
    and writes the output to separate CSV files for all sensors.

    If specific timing data for a sensor is missing, it defaults to 0.0.

    Args:
        log_file_path (str): The full path to the input log file.
        output_dir (str): The path to the directory where CSV files will be saved.
    """
    # --- Regular Expressions to Parse Log Lines ---
    line_regex = re.compile(r"^(\d{4}-\d{2}-\d{2}\s\d{2}:\d{2}:\d{2})\s-\s(.*)$")
    id_regex = re.compile(r"\[(.*?)\s(?:ID\s)?(\d+)\]")
    data_regex = re.compile(r"(\w+):\s*([-\d\.,]+)")

    # --- Data Storage ---
    final_data = defaultdict(list)
    
    # --- File Processing ---
    try:
        with open(log_file_path, 'r') as f:
            block_timestamp = None
            sensor_readings = {}
            timing_readings = {}

            # Process line by line, grouping data by timestamp
            for line in f:
                line_match = line_regex.match(line)
                if not line_match:
                    continue

                current_timestamp, content = line_match.groups()

                # When timestamp changes, process the completed block of data
                if current_timestamp != block_timestamp and block_timestamp is not None:
                    for sensor_id, sensor_info in sensor_readings.items():
                        # **MODIFIED LOGIC**: Get timing if it exists, otherwise use a default dict.
                        # This ensures no sensor data is ever dropped.
                        timing_info = timing_readings.get(sensor_id, {
                            'Start_s': 0.0, 'End_s': 0.0, 'Duration_s': 0.0
                        })
                        
                        combined_record = {
                            'Timestamp': block_timestamp,
                            **timing_info,
                            **sensor_info['data']
                        }
                        file_key = f"{sensor_info['type']}_{sensor_id}".lower().replace(' ', '_')
                        final_data[file_key].append(combined_record)
                    
                    # Reset for the new block
                    sensor_readings = {}
                    timing_readings = {}

                block_timestamp = current_timestamp
                
                # Parse the content of the current line
                id_match = id_regex.search(content)
                if not id_match:
                    continue

                item_type, item_id = id_match.groups()
                data_payload = content[id_match.end():]
                data_values = {key: float(val.replace(',', '')) for key, val in data_regex.findall(data_payload)}

                if not data_values:
                    continue

                # Categorize and store parsed data
                if item_type == "Timing Individual":
                    timing_readings[item_id] = {
                        'Start_s': data_values.get('Start', 0) / 1_000_000.0,
                        'End_s': data_values.get('End', 0) / 1_000_000.0,
                        'Duration_s': data_values.get('Duration', 0) / 1_000_000.0
                    }
                elif "Timing" not in item_type:
                    sanitized_data = {}
                    for key, value in data_values.items():
                        if 'pressure' in key: new_key = f"{key}_bar"
                        elif 'weight' in key: new_key = f"{key}_g"
                        elif 'flow' in key: new_key = f"{key}_lpm"
                        else: new_key = key
                        sanitized_data[new_key] = value
                    
                    sensor_readings[item_id] = {'type': item_type, 'data': sanitized_data}
            
            # **MODIFICATION**: Process the very last block in the file after the loop finishes
            if block_timestamp is not None:
                for sensor_id, sensor_info in sensor_readings.items():
                    timing_info = timing_readings.get(sensor_id, {
                        'Start_s': 0.0, 'End_s': 0.0, 'Duration_s': 0.0
                    })
                    
                    combined_record = {
                        'Timestamp': block_timestamp,
                        **timing_info,
                        **sensor_info['data']
                    }
                    file_key = f"{sensor_info['type']}_{sensor_id}".lower().replace(' ', '_')
                    final_data[file_key].append(combined_record)

    except FileNotFoundError:
        print(f"Error: The file '{log_file_path}' was not found.")
        return
    except Exception as e:
        print(f"An error occurred: {e}")
        return

    # --- Write Collected Data to CSV Files ---
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created output directory: {output_dir}")

    for file_key, records in final_data.items():
        if not records:
            continue
        
        output_file_path = os.path.join(output_dir, f"{file_key}.csv")
        headers = records[0].keys()

        with open(output_file_path, 'w', newline='') as f_out:
            writer = csv.DictWriter(f_out, fieldnames=headers)
            writer.writeheader()
            writer.writerows(records)
        
        print(f"Successfully created {output_file_path}")

# --- Example Usage ---
if __name__ == '__main__':
    # Dummy log content now includes a sensor (Flow ID 9) WITHOUT matching timing data
    # to demonstrate that the new code correctly processes it.
    log_content = """2025-07-16 22:36:06 - --- SENSOR DATA ---
2025-07-16 22:36:06 - [Pressure ID 0] pressure: 0.16 bar
2025-07-16 22:36:06 - [LoadCell ID 6] weight_grams: -0.880 g
2025-07-16 22:36:06 - [Flow ID 9] flow_rate_lpm: 1.25 LPM
2025-07-16 22:36:06 - --- TIMING DATA (microseconds) ---
2025-07-16 22:36:06 - [Timing Individual 0] Start: 8,034,332 us, End: 8,034,828 us, Duration: 496 us
2025-07-16 22:36:06 - [Timing Individual 6] Start: 7,907,368 us, End: 7,908,004 us, Duration: 636 us
2025-07-16 22:36:07 - --- SENSOR DATA ---
2025-07-16 22:36:07 - [Pressure ID 0] pressure: 0.18 bar
2025-07-16 22:36:07 - [Flow ID 9] flow_rate_lpm: 1.22 LPM
2025-07-16 22:36:07 - --- TIMING DATA (microseconds) ---
2025-07-16 22:36:07 - [Timing Individual 0] Start: 8,134,000 us, End: 8,134,500 us, Duration: 500 us
"""
    dummy_log_file = "sensor_log.txt"
    with open(dummy_log_file, "w") as f:
        f.write(log_content)

    # Define the log file to use and the directory for CSVs
    input_file ='Work.log' 
    output_folder = "sensor_csv_output"

    # Run the parser
    parse_log_to_csv(input_file, output_folder)
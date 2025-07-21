import re
import datetime
import math
import csv
import os
import sys
import logging

# --- Configure Logging for the Parser Script ---
# This logger is separate from the CLI's logger and is just for the parser's operations.
parser_logger = logging.getLogger(__name__)
parser_logger.setLevel(logging.INFO) # Set to INFO for general messages

# Console handler for the parser
parser_console_handler = logging.StreamHandler(sys.stdout)
parser_console_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
parser_console_handler.setFormatter(parser_console_formatter)
parser_logger.addHandler(parser_console_handler)


# --- ANSI escape code regex pattern ---
ANSI_ESCAPE_PATTERN = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')


# --- Constants (matched to Arduino SensorManager.h/cpp and CLI.py) ---
STATUS_OK = 0x00
STATUS_ERROR_INVALID_TARGET_ID      = 0xE1
STATUS_ERROR_INVALID_STATE_VALUE    = 0xE2
STATUS_ERROR_INVALID_PAYLOAD_SIZE   = 0xE3
STATUS_ERROR_INVALID_COMMAND_TYPE   = 0xE4
STATUS_ERROR_HARDWARE_FAILURE       = 0xE5
STATUS_ERROR_UNKNOWN_ISSUE          = 0xE6

SEQUENCE_TYPE_STARTUP = 0x01
SEQUENCE_TYPE_SHUTDOWN = 0x02

SEQ_STARTUP_STEP_OPEN_FUEL_VALVE = 0x01
SEQ_STARTUP_STEP_SET_MOTOR_THROTTLE = 0x02
SEQ_STARTUP_STEP_WAIT_FOR_FLOW = 0x03
SEQ_STARTUP_STEP_OPEN_OXIDIZER_VALVE = 0x04
SEQ_STARTUP_STEP_WAIT_200MS = 0x05
SEQ_STARTUP_STEP_OPEN_IGNITER_VALVE = 0x06
SEQ_STARTUP_COMPLETE = 0xFE
SEQ_STARTUP_FAILED = 0xFF

SEQ_SHUTDOWN_STEP_CLOSE_OXIDIZER_VALVE = 0x01
SEQ_SHUTDOWN_STEP_WAIT_200MS = 0x02
SEQ_SHUTDOWN_STEP_SET_MOTOR_THROTTLE_ZERO = 0x03
SEQ_SHUTDOWN_STEP_WAIT_1000MS = 0x04
SEQ_SHUTDOWN_STEP_CLOSE_FUEL_VALVE = 0x05
SEQ_SHUTDOWN_COMPLETE = 0xFE
SEQ_SHUTDOWN_FAILED = 0xFF

# --- Mappings for human-readable output ---
STATUS_CODE_MAP = {
    STATUS_OK: "OK",
    STATUS_ERROR_INVALID_TARGET_ID: "ERROR: Invalid Target ID",
    STATUS_ERROR_INVALID_STATE_VALUE: "ERROR: Invalid State Value",
    STATUS_ERROR_INVALID_PAYLOAD_SIZE: "ERROR: Invalid Payload Size",
    STATUS_ERROR_INVALID_COMMAND_TYPE: "ERROR: Invalid Command Type",
    STATUS_ERROR_HARDWARE_FAILURE: "ERROR: Hardware Failure",
    STATUS_ERROR_UNKNOWN_ISSUE: "ERROR: Unknown Issue"
}

SEQUENCE_TYPE_MAP = {
    SEQUENCE_TYPE_STARTUP: "Startup",
    SEQUENCE_TYPE_SHUTDOWN: "Shutdown"
}

STARTUP_STEP_MAP = {
    SEQ_STARTUP_STEP_OPEN_FUEL_VALVE: "Step 1/6: Opening fuel valve (R0)",
    SEQ_STARTUP_STEP_SET_MOTOR_THROTTLE: "Step 2/6: Setting motor throttle",
    SEQ_STARTUP_STEP_WAIT_FOR_FLOW: "Step 3/6: Waiting for flow (>= 1.4 LPM)",
    SEQ_STARTUP_STEP_OPEN_OXIDIZER_VALVE: "Step 4/6: Opening oxidizer valve (R1)",
    SEQ_STARTUP_STEP_WAIT_200MS: "Step 5/6: Waiting 200ms",
    SEQ_STARTUP_STEP_OPEN_IGNITER_VALVE: "Step 6/6: Opening igniter valve (R2)",
    SEQ_STARTUP_COMPLETE: "Sequence Complete",
    SEQ_STARTUP_FAILED: "Sequence FAILED"
}

SHUTDOWN_STEP_MAP = {
    SEQ_SHUTDOWN_STEP_CLOSE_OXIDIZER_VALVE: "Step 1/5: Closing oxidizer valve (R1)",
    SEQ_SHUTDOWN_STEP_WAIT_200MS: "Step 2/5: Waiting 200ms",
    SEQ_SHUTDOWN_STEP_SET_MOTOR_THROTTLE_ZERO: "Step 3/5: Setting motor throttle to zero",
    SEQ_SHUTDOWN_STEP_WAIT_1000MS: "Step 4/5: Waiting 1000ms",
    SEQ_SHUTDOWN_STEP_CLOSE_FUEL_VALVE: "Step 5/5: Closing fuel valve (R0)",
    SEQ_SHUTDOWN_COMPLETE: "Sequence Complete",
    SEQ_SHUTDOWN_FAILED: "Sequence FAILED"
}

def strip_ansi_codes(text):
    """Removes ANSI escape codes from a string."""
    return ANSI_ESCAPE_PATTERN.sub('', text)

def parse_log_entry(log_line):
    """
    Parses a single log line from Work.log and extracts relevant data.
    It handles different log message formats (sensor data, timing, responses, sequences).

    Args:
        log_line (str): A single line read from the log file.

    Returns:
        dict: A dictionary containing the parsed data and its type, or None if the line
              does not match any known log format.
    """
    # 1. Strip ANSI escape codes first to ensure clean parsing.
    cleaned_line = strip_ansi_codes(log_line.strip())

    # 2. Regex to match the common log format: TIMESTAMP - LEVEL - MESSAGE
    # The (.*?) for timestamp is crucial. It's a non-greedy match that captures
    # everything up to the first " - ". This handles the literal "%.f" in timestamps.
    match = re.match(r'(.*?) - (\w+) - (.*)', cleaned_line)
    if not match:
        return None

    timestamp_str, level, message = match.groups()
    
    # Store timestamp as a string directly.
    # We cannot parse it into a datetime object if it contains literal '%f'.
    timestamp = timestamp_str 

    # Initialize a dictionary to hold parsed data for this log entry.
    data = {
        'timestamp': timestamp, # Stored as string, e.g., "2025-07-21 17:11:16.%f"
        'level': level,
        'raw_message': message, # The message part after timestamp and level
        'type': 'unknown'       # Default type, updated by specific parsers below
    }

    # --- 3. Specific Parsers for different message types ---

    # --- Sensor Data Parsing (from [RAW SENSOR] lines) ---
    # Example: [RAW SENSOR] LoadCell ID 8: Values=(-0.0,), Timing Duration=416 us
    sensor_match = re.match(r'\[RAW SENSOR\] (\w+) ID (\d+): Values=\(([^)]+)\), Timing Duration=(\d+) us', message)
    if sensor_match:
        sensor_type, sensor_id, values_str, duration_us = sensor_match.groups()
        data['type'] = 'sensor_data'
        data['sensor_type'] = sensor_type
        data['sensor_id'] = int(sensor_id)
        data['duration_us'] = int(duration_us)
        
        # Parse values. The `filter(None, ...)` removes any empty strings
        # that result from trailing commas (e.g., "(-0.0,)" splits to ["-0.0", ""]).
        values = [float(v.strip()) for v in values_str.split(',') if v.strip()]
        
        # Assign values to specific keys based on sensor type.
        if sensor_type == 'Pressure':
            data['pressure'] = values[0] if values else None
            data['fields_for_csv'] = ['pressure'] # Define fields for CSV export
        elif sensor_type == 'LoadCell':
            data['weight_grams'] = values[0] if values else None
            data['fields_for_csv'] = ['weight_grams']
        elif sensor_type == 'Flow':
            data['flow_rate_lpm'] = values[0] if values else None
            data['fields_for_csv'] = ['flow_rate_lpm']
        elif sensor_type == 'Temperature':
            data['temp_c'] = values[0] if len(values) > 0 else math.nan
            data['temp_f'] = values[1] if len(values) > 1 else math.nan
            
            # Create human-readable strings for temperature, handling NaN (Open TC)
            data['temp_c_str'] = "ERR (Open TC)" if math.isnan(data['temp_c']) else f"{data['temp_c']:.1f} C"
            data['temp_f_str'] = "ERR (Open TC)" if math.isnan(data['temp_f']) else f"{data['temp_f']:.1f} F"
            data['fields_for_csv'] = ['temp_c', 'temp_f'] # Use raw values for CSV
        elif sensor_type == 'MotorRPM':
            data['rpm'] = values[0] if values else None
            data['fields_for_csv'] = ['rpm']
        return data

    # --- Timing Data Parsing (from [RAW TIMING] lines for category cycles) ---
    # Example: [RAW TIMING] Type=0x02, CategoryID=6, Duration=4016 us
    timing_match = re.match(r'\[RAW TIMING\] Type=0x([0-9a-fA-F]+), CategoryID=(\d+), Duration=(\d+) us', message)
    if timing_match:
        timing_type_hex, category_id, duration_us = timing_match.groups()
        data['type'] = 'timing_data'
        data['timing_type'] = int(timing_type_hex, 16)
        data['category_id'] = int(category_id)
        data['duration_us'] = int(duration_us)
        
        # Map category ID to a human-readable name.
        if data['category_id'] == 0: data['category_name'] = "Pressure"
        elif data['category_id'] == 6: data['category_name'] = "LoadCell"
        elif data['category_id'] == 9: data['category_name'] = "Flow"
        elif data['category_id'] == 10: data['category_name'] = "Temperature"
        elif data['category_id'] == 14: data['category_name'] = "MotorRPM"
        else: data['category_name'] = "Unknown"
        return data

    # --- Command Response Parsing (from [RAW RESPONSE] lines) ---
    # Example: [RAW RESPONSE] CmdType=0x01, TargetID=0, Status=0x00 (OK)
    response_match = re.match(r'\[RAW RESPONSE\] CmdType=0x([0-9a-fA-F]+), TargetID=(\d+), Status=0x([0-9a-fA-F]+) \((.*)\)', message)
    if response_match:
        cmd_type_hex, target_id, status_code_hex, status_text = response_match.groups()
        data['type'] = 'command_response'
        data['original_cmd_type'] = int(cmd_type_hex, 16)
        data['original_target_id'] = int(target_id)
        data['status_code'] = int(status_code_hex, 16)
        data['status_text'] = status_text # This text is already human-readable from cli.py
        return data

    # --- Sequence Status Parsing (from [RAW SEQUENCE STATUS] lines) ---
    # Example: [RAW SEQUENCE STATUS] Type=0x01, Step=0x01, Status=0x00
    sequence_status_match = re.match(r'\[RAW SEQUENCE STATUS\] Type=0x([0-9a-fA-F]+), Step=0x([0-9a-fA-F]+), Status=0x([0-9a-fA-F]+)', message)
    if sequence_status_match:
        seq_type_hex, step_code_hex, status_code_hex = sequence_status_match.groups()
        data['type'] = 'sequence_status'
        data['sequence_type'] = int(seq_type_hex, 16)
        data['step_code'] = int(step_code_hex, 16)
        data['status_code'] = int(status_code_hex, 16)

        # Map numerical codes to human-readable strings using the defined mappings.
        data['sequence_type_str'] = SEQUENCE_TYPE_MAP.get(data['sequence_type'], f"UNKNOWN_SEQ_TYPE(0x{data['sequence_type']:02X})")
        data['status_code_str'] = STATUS_CODE_MAP.get(data['status_code'], f"UNKNOWN_STATUS(0x{data['status_code']:02X})")
        
        if data['sequence_type'] == SEQUENCE_TYPE_STARTUP:
            data['step_code_str'] = STARTUP_STEP_MAP.get(data['step_code'], f"UNKNOWN_STARTUP_STEP(0x{data['step_code']:02X})")
        elif data['sequence_type'] == SEQUENCE_TYPE_SHUTDOWN:
            data['step_code_str'] = SHUTDOWN_STEP_MAP.get(data['step_code'], f"UNKNOWN_SHUTDOWN_STEP(0x{data['step_code']:02X})")
        else:
            data['step_code_str'] = f"UNKNOWN_STEP(0x{data['step_code']:02X})"
        
        return data

    return data # Return the basic data dictionary if no specific pattern matches


def export_sensor_data_to_csv(sensor_data_list, output_dir="csv_exports"):
    """
    Exports a list of parsed sensor data entries for a single sensor to a CSV file.

    Args:
        sensor_data_list (list): A list of dictionaries, where each dictionary
                                 represents a parsed sensor data entry for one specific sensor.
        output_dir (str): The directory where CSV files will be saved.
    """
    if not sensor_data_list:
        return

    # Get sensor type and ID from the first entry to name the file.
    # Assumes all entries in the list are for the same sensor.
    sensor_type = sensor_data_list[0]['sensor_type'].lower()
    sensor_id = sensor_data_list[0]['sensor_id']
    filename = os.path.join(output_dir, f"{sensor_type}_{sensor_id}.csv")

    # Determine the header dynamically based on the 'fields_for_csv' in the parsed data.
    # Always include 'timestamp' and 'duration_us'.
    header = ['timestamp'] + sensor_data_list[0].get('fields_for_csv', []) + ['duration_us']

    # Ensure the output directory exists.
    os.makedirs(output_dir, exist_ok=True)

    try:
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(header) # Write the header row

            for entry in sensor_data_list:
                row = [entry['timestamp']] # Start with timestamp
                
                # Add sensor-specific data values
                for field in entry.get('fields_for_csv', []):
                    value = entry.get(field)
                    # Handle NaN for temperature sensors specifically for CSV output
                    if field in ['temp_c', 'temp_f'] and math.isnan(value):
                        row.append('') # Write empty string for NaN in CSV
                    else:
                        row.append(value)
                
                row.append(entry['duration_us']) # Add timing duration
                writer.writerow(row)
        parser_logger.info(f"Exported {len(sensor_data_list)} records to '{filename}'")
    except IOError as e:
        parser_logger.error(f"Error writing to CSV file '{filename}': {e}")
    except Exception as e:
        parser_logger.error(f"An unexpected error occurred during CSV export for '{filename}': {e}")


def analyze_log(log_file_path, output_csv_dir="csv_exports"):
    """
    Reads a log file, parses each line, and provides a structured summary
    of sensor data, timing, command responses, and sequence statuses.
    It also exports sensor data to separate CSV files.

    Args:
        log_file_path (str): The path to the log file (e.g., 'Work.log').
        output_csv_dir (str): The directory where generated CSV files will be saved.
    """
    parsed_data = []
    try:
        with open(log_file_path, 'r') as f:
            for line_num, line in enumerate(f, 1):
                parsed_entry = parse_log_entry(line.strip())
                if parsed_entry:
                    parsed_data.append(parsed_entry)
                # else: parser_logger.debug(f"Skipping unparseable line {line_num}: {line.strip()}")
    except FileNotFoundError:
        parser_logger.error(f"Error: Log file not found at '{log_file_path}'")
        return

    print(f"--- Log Analysis for '{log_file_path}' ---")
    print(f"Total lines parsed: {len(parsed_data)}")

    # --- Group Sensor Data for CSV Export and Summary ---
    # Stores all historical data for each sensor.
    all_sensor_data_grouped = {} # Key: (sensor_type, sensor_id), Value: list of parsed_data dicts
    for entry in parsed_data:
        if entry['type'] == 'sensor_data':
            key = (entry['sensor_type'], entry['sensor_id'])
            if key not in all_sensor_data_grouped:
                all_sensor_data_grouped[key] = []
            all_sensor_data_grouped[key].append(entry)
    
    # Export sensor data to CSVs
    if all_sensor_data_grouped:
        parser_logger.info(f"\n--- Exporting Sensor Data to CSVs in '{output_csv_dir}' ---")
        for sensor_key, data_list in sorted(all_sensor_data_grouped.items()):
            export_sensor_data_to_csv(data_list, output_csv_dir)
    else:
        parser_logger.info("\nNo sensor data found to export to CSV.")

    # --- Summarize Sensor Data (latest readings for console output) ---
    print("\n--- Sensor Data Summary (Latest Readings) ---")
    latest_sensor_readings = {}
    for entry in parsed_data:
        if entry['type'] == 'sensor_data':
            key = (entry['sensor_type'], entry['sensor_id'])
            latest_sensor_readings[key] = entry # Overwrites, so it stores the latest

    for (s_type, s_id), data in sorted(latest_sensor_readings.items()):
        output_parts = [f"[{s_type} ID {s_id}] Last Reading at {data['timestamp']} "]
        if s_type == 'Pressure':
            output_parts.append(f"Pressure: {data['pressure']:.2f} bar ")
        elif s_type == 'LoadCell':
            output_parts.append(f"Weight: {data['weight_grams']:.3f} g ")
        elif s_type == 'Flow':
            output_parts.append(f"Flow Rate: {data['flow_rate_lpm']:.3f} LPM ")
        elif s_type == 'Temperature':
            output_parts.append(f"Temp: {data['temp_c_str']} / {data['temp_f_str']} ")
        elif s_type == 'MotorRPM':
            output_parts.append(f"RPM: {data['rpm']:.1f} ")
        
        output_parts.append(f"(Duration: {data['duration_us']:,} us)")
        print("".join(output_parts))

    # --- Summarize Timing Data (Category Cycles) ---
    print("\n--- Category Timing Summary ---")
    latest_category_timings = {}
    for entry in parsed_data:
        if entry['type'] == 'timing_data' and entry['timing_type'] == 0x02:
            key = entry['category_name']
            latest_category_timings[key] = entry
    
    if not latest_category_timings:
        print("No category timing data found.")
    else:
        for category_name, data in sorted(latest_category_timings.items()):
            duration_seconds = data['duration_us'] / 1_000_000.0
            print(f"[{category_name} Cycle] Last Duration: {data['duration_us']:,} us ({duration_seconds:.4f} s) at {data['timestamp']}")

    # --- Summarize Command Responses ---
    print("\n--- Command Response Summary ---")
    command_responses = [e for e in parsed_data if e['type'] == 'command_response']
    if not command_responses:
        print("No command responses found.")
    else:
        for response in command_responses:
            print(f"[{response['timestamp']}] Command Type: 0x{response['original_cmd_type']:02X}, Target ID: {response['original_target_id']}, Status: {response['status_text']}")

    # --- Summarize Sequence Status ---
    print("\n--- Sequence Status Summary ---")
    sequence_status_entries = [e for e in parsed_data if e['type'] == 'sequence_status']
    if not sequence_status_entries:
        print("No sequence status updates found.")
    else:
        startup_history = []
        shutdown_history = []
        for entry in sequence_status_entries:
            if entry['sequence_type'] == SEQUENCE_TYPE_STARTUP:
                startup_history.append(entry)
            elif entry['sequence_type'] == SEQUENCE_TYPE_SHUTDOWN:
                shutdown_history.append(entry)
        
        if startup_history:
            print("\n  ** Startup Sequence History **")
            for entry in startup_history:
                print(f"    [{entry['timestamp']}] {entry['sequence_type_str']}: {entry['step_code_str']} [{entry['status_code_str']}]")
        
        if shutdown_history:
            print("\n  ** Shutdown Sequence History **")
            for entry in shutdown_history:
                print(f"    [{entry['timestamp']}] {entry['sequence_type_str']}: {entry['step_code_str']} [{entry['status_code_str']}]")


# --- Main execution block ---
if __name__ == "__main__":
    # Define the path to your log file.
    log_file = 'Work.log'
    # Define the directory for CSV exports.
    output_csv_directory = 'csv_exports' # This will create a folder named 'csv_exports'

    # Call the analysis function, passing the output directory.
    analyze_log(log_file, output_csv_directory)


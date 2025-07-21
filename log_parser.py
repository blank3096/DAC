import re
import datetime
import math # For math.isnan

# --- Constants (matched to Arduino and CLI) ---
# Status Codes
STATUS_OK = 0x00
STATUS_ERROR_INVALID_TARGET_ID      = 0xE1
STATUS_ERROR_INVALID_STATE_VALUE    = 0xE2
STATUS_ERROR_INVALID_PAYLOAD_SIZE   = 0xE3
STATUS_ERROR_INVALID_COMMAND_TYPE   = 0xE4
STATUS_ERROR_HARDWARE_FAILURE       = 0xE5
STATUS_ERROR_UNKNOWN_ISSUE          = 0xE6

# Sequence Type IDs
SEQUENCE_TYPE_STARTUP = 0x01
SEQUENCE_TYPE_SHUTDOWN = 0x02

# Startup Sequence Step Codes
SEQ_STARTUP_STEP_OPEN_FUEL_VALVE = 0x01
SEQ_STARTUP_STEP_SET_MOTOR_THROTTLE = 0x02
SEQ_STARTUP_STEP_WAIT_FOR_FLOW = 0x03
SEQ_STARTUP_STEP_OPEN_OXIDIZER_VALVE = 0x04
SEQ_STARTUP_STEP_WAIT_200MS = 0x05
SEQ_STARTUP_STEP_OPEN_IGNITER_VALVE = 0x06
SEQ_STARTUP_COMPLETE = 0xFE
SEQ_STARTUP_FAILED = 0xFF

# Shutdown Sequence Step Codes
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


def parse_log_entry(log_line):
    """
    Parses a single log line and extracts relevant data.
    Returns a dictionary with parsed data or None if the line doesn't match a known format.
    """
    match = re.match(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{6}) - (\w+) - (.*)', log_line)
    if not match:
        return None

    timestamp_str, level, message = match.groups()
    timestamp = datetime.datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S.%f')

    data = {
        'timestamp': timestamp,
        'level': level,
        'raw_message': message,
        'type': 'unknown'
    }

    # --- Sensor Data Parsing (from [RAW SENSOR] lines) ---
    sensor_match = re.match(r'\[RAW SENSOR\] (\w+) ID (\d+): Values=\(([^)]+)\), Timing Duration=(\d+) us', message)
    if sensor_match:
        sensor_type, sensor_id, values_str, duration_us = sensor_match.groups()
        data['type'] = 'sensor_data'
        data['sensor_type'] = sensor_type
        data['sensor_id'] = int(sensor_id)
        data['duration_us'] = int(duration_us)
        
        # Parse values based on sensor type
        values = [float(v.strip()) for v in values_str.split(',')]
        if sensor_type == 'Pressure':
            data['pressure'] = values[0]
        elif sensor_type == 'LoadCell':
            data['weight_grams'] = values[0]
        elif sensor_type == 'Flow':
            data['flow_rate_lpm'] = values[0]
        elif sensor_type == 'Temperature':
            data['temp_c'] = values[0]
            data['temp_f'] = values[1]
            if math.isnan(data['temp_c']):
                data['temp_c_str'] = "ERR (Open TC)"
            else:
                data['temp_c_str'] = f"{data['temp_c']:.1f} C"
            if math.isnan(data['temp_f']):
                data['temp_f_str'] = "ERR (Open TC)"
            else:
                data['temp_f_str'] = f"{data['temp_f']:.1f} F"
        elif sensor_type == 'MotorRPM':
            data['rpm'] = values[0]
        return data

    # --- Timing Data Parsing (from [RAW TIMING] lines for category cycles) ---
    timing_match = re.match(r'\[RAW TIMING\] Type=0x([0-9a-fA-F]+), CategoryID=(\d+), Duration=(\d+) us', message)
    if timing_match:
        timing_type_hex, category_id, duration_us = timing_match.groups()
        data['type'] = 'timing_data'
        data['timing_type'] = int(timing_type_hex, 16)
        data['category_id'] = int(category_id)
        data['duration_us'] = int(duration_us)
        # Map category ID to name
        if data['category_id'] == 0: data['category_name'] = "Pressure"
        elif data['category_id'] == 6: data['category_name'] = "LoadCell"
        elif data['category_id'] == 9: data['category_name'] = "Flow"
        elif data['category_id'] == 10: data['category_name'] = "Temperature"
        elif data['category_id'] == 14: data['category_name'] = "MotorRPM"
        else: data['category_name'] = "Unknown"
        return data

    # --- Command Response Parsing (from [RAW RESPONSE] lines) ---
    response_match = re.match(r'\[RAW RESPONSE\] CmdType=0x([0-9a-fA-F]+), TargetID=(\d+), Status=0x([0-9a-fA-F]+) \((.*)\)', message)
    if response_match:
        cmd_type_hex, target_id, status_code_hex, status_text = response_match.groups()
        data['type'] = 'command_response'
        data['original_cmd_type'] = int(cmd_type_hex, 16)
        data['original_target_id'] = int(target_id)
        data['status_code'] = int(status_code_hex, 16)
        data['status_text'] = status_text
        return data

    # --- NEW: Sequence Status Parsing (from [RAW SEQUENCE STATUS] lines) ---
    sequence_status_match = re.match(r'\[RAW SEQUENCE STATUS\] Type=0x([0-9a-fA-F]+), Step=0x([0-9a-fA-F]+), Status=0x([0-9a-fA-F]+)', message)
    if sequence_status_match:
        seq_type_hex, step_code_hex, status_code_hex = sequence_status_match.groups()
        data['type'] = 'sequence_status'
        data['sequence_type'] = int(seq_type_hex, 16)
        data['step_code'] = int(step_code_hex, 16)
        data['status_code'] = int(status_code_hex, 16)

        # Map to human-readable strings
        data['sequence_type_str'] = SEQUENCE_TYPE_MAP.get(data['sequence_type'], f"UNKNOWN_SEQ_TYPE(0x{data['sequence_type']:02X})")
        data['status_code_str'] = STATUS_CODE_MAP.get(data['status_code'], f"UNKNOWN_STATUS(0x{data['status_code']:02X})")
        
        if data['sequence_type'] == SEQUENCE_TYPE_STARTUP:
            data['step_code_str'] = STARTUP_STEP_MAP.get(data['step_code'], f"UNKNOWN_STARTUP_STEP(0x{data['step_code']:02X})")
        elif data['sequence_type'] == SEQUENCE_TYPE_SHUTDOWN:
            data['step_code_str'] = SHUTDOWN_STEP_MAP.get(data['step_code'], f"UNKNOWN_SHUTDOWN_STEP(0x{data['step_code']:02X})")
        else:
            data['step_code_str'] = f"UNKNOWN_STEP(0x{data['step_code']:02X})"
        
        return data

    return data # Return basic info if no specific pattern matches


def analyze_log(log_file_path):
    """
    Reads a log file, parses each line, and provides a summary.
    """
    parsed_data = []
    try:
        with open(log_file_path, 'r') as f:
            for line_num, line in enumerate(f, 1):
                parsed_entry = parse_log_entry(line.strip())
                if parsed_entry:
                    parsed_data.append(parsed_entry)
                # else: print(f"Skipping unparseable line {line_num}: {line.strip()}")
    except FileNotFoundError:
        print(f"Error: Log file not found at '{log_file_path}'")
        return

    print(f"--- Log Analysis for '{log_file_path}' ---")
    print(f"Total lines parsed: {len(parsed_data)}")

    # --- Summarize Sensor Data ---
    print("\n--- Sensor Data Summary ---")
    latest_sensor_readings = {}
    for entry in parsed_data:
        if entry['type'] == 'sensor_data':
            key = (entry['sensor_type'], entry['sensor_id'])
            latest_sensor_readings[key] = entry

    for (s_type, s_id), data in sorted(latest_sensor_readings.items()):
        output_parts = [f"[{s_type} ID {s_id}] Last Reading at {data['timestamp'].strftime('%H:%M:%S.%f')[:-3]} "]
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
        if entry['type'] == 'timing_data' and entry['timing_type'] == 0x02: # 0x02 is TIMING_CATEGORY_CYCLE_ID
            key = entry['category_name']
            latest_category_timings[key] = entry
    
    if not latest_category_timings:
        print("No category timing data found.")
    else:
        for category_name, data in sorted(latest_category_timings.items()):
            duration_seconds = data['duration_us'] / 1_000_000.0
            print(f"[{category_name} Cycle] Last Duration: {data['duration_us']:,} us ({duration_seconds:.4f} s) at {data['timestamp'].strftime('%H:%M:%S.%f')[:-3]}")

    # --- Summarize Command Responses ---
    print("\n--- Command Response Summary ---")
    command_responses = [e for e in parsed_data if e['type'] == 'command_response']
    if not command_responses:
        print("No command responses found.")
    else:
        for response in command_responses:
            print(f"[{response['timestamp'].strftime('%H:%M:%S.%f')[:-3]}] Command Type: 0x{response['original_cmd_type']:02X}, Target ID: {response['original_target_id']}, Status: {response['status_text']}")

    # --- NEW: Summarize Sequence Status ---
    print("\n--- Sequence Status Summary ---")
    sequence_status_entries = [e for e in parsed_data if e['type'] == 'sequence_status']
    if not sequence_status_entries:
        print("No sequence status updates found.")
    else:
        # Group by sequence type (Startup/Shutdown) and show the progression
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
                print(f"    [{entry['timestamp'].strftime('%H:%M:%S.%f')[:-3]}] {entry['sequence_type_str']}: {entry['step_code_str']} [{entry['status_code_str']}]")
        
        if shutdown_history:
            print("\n  ** Shutdown Sequence History **")
            for entry in shutdown_history:
                print(f"    [{entry['timestamp'].strftime('%H:%M:%S.%f')[:-3]}] {entry['sequence_type_str']}: {entry['step_code_str']} [{entry['status_code_str']}]")


if __name__ == "__main__":
    log_file = 'Work.log'
    analyze_log(log_file)
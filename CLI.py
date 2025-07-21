import serial
import struct
import time
import serial.tools.list_ports
import sys
import threading
import argparse
import math
import queue
import logging
import os

# --- ANSI Color Codes ---
RESET = "\033[0m"
BOLD = "\033[1m"
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
MAGENTA = "\033[95m"
CYAN = "\033[96m"
WHITE = "\033[97m"

# --- Custom Colored Formatter for Console Output ---
class ColoredFormatter(logging.Formatter):
    FORMAT = "%(asctime)s - %(levelname)s - %(message)s"
    
    LOG_COLORS = {
        logging.DEBUG: CYAN,
        logging.INFO: BLUE, # General info messages, including sensor data
        logging.WARNING: YELLOW,
        logging.ERROR: RED,
        logging.CRITICAL: BOLD + RED
    }

    def format(self, record):
        log_fmt = self.FORMAT
        # Apply color based on log level
        color = self.LOG_COLORS.get(record.levelno, RESET)
        formatter = logging.Formatter(color + log_fmt + RESET, datefmt='%Y-%m-%d %H:%M:%S')
        return formatter.format(record)

# --- Configure Logging ---
logger = logging.getLogger()
logger.setLevel(logging.DEBUG) # CHANGED: Set root logger to DEBUG to allow all messages to pass

# File handler (no colors) - Logs ALL debug/info/warning/error/critical messages
file_handler = logging.FileHandler('Work.log', mode='a')
file_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
file_handler.setFormatter(file_formatter)
file_handler.setLevel(logging.DEBUG) # CHANGED: Set file handler to DEBUG
logger.addHandler(file_handler)

# Stream handler (with colors for console) - Logs INFO, WARNING, ERROR, CRITICAL
stream_handler = logging.StreamHandler(sys.stdout)
stream_formatter = ColoredFormatter()
stream_handler.setFormatter(stream_formatter)
stream_handler.setLevel(logging.INFO) # CHANGED: Set stream handler to INFO
logger.addHandler(stream_handler)


# Redirect stderr to logger
class StdErrToLogger:
    def __init__(self, logger):
        self.logger = logger

    def write(self, message):
        if message.strip():
            self.logger.error(message.strip()) # Log stderr as ERROR level

    def flush(self):
        pass

sys.stderr = StdErrToLogger(logger)

# --- Constants (matched to Arduino SensorManager.cpp/h) ---
COMMAND_START_BYTE = 0xFC
COMMAND_END_BYTE = 0xFD
CMD_TYPE_SET_MOTOR = 0x02
CMD_TYPE_SET_RELAY = 0x01
CMD_TARGET_MOTOR_ID = 0
CMD_TARGET_RELAY_START = 0
MAX_COMMAND_PAYLOAD_SIZE = 32

# Sensor data packet constants
PRESSURE_PACKET_START_BYTE = 0xAA
PRESSURE_PACKET_END_BYTE = 0x55
PRESSURE_ID_START = 0
NUM_IDS_PRESSURE = 6

LOADCELL_PACKET_START_BYTE = 0xBB
LOADCELL_PACKET_END_BYTE = 0x66
LOADCELL_ID_START = 6
NUM_IDS_LOADCELL = 3

FLOW_PACKET_START_BYTE = 0xCC
FLOW_PACKET_END_BYTE = 0xDD
FLOW_SENSOR_ID = 9

TEMP_PACKET_START_BYTE = 0xEE
TEMP_PACKET_END_BYTE = 0xFF
TEMP_ID_START = 10
NUM_IDS_TEMP = 4

MOTOR_RPM_ID = 14
MOTOR_RPM_PACKET_START_BYTE = 0xF0
MOTOR_RPM_PACKET_END_BYTE = 0xF1

# --- Timing Packet Constants (matched to Arduino SensorManager.cpp/h) ---
TIMING_PACKET_START_BYTE = 0xDE
TIMING_PACKET_END_BYTE = 0xAD
TIMING_SENSOR_OPERATION_ID = 0x01 # Used as sensor_id_in_payload for individual sensor timing
TIMING_CATEGORY_CYCLE_ID = 0x02 # Used as sensor_id_in_payload for category timing

# --- Command Response (ACK/NACK) Constants (matched to Arduino) ---
RESPONSE_PACKET_START_BYTE = 0xF2
RESPONSE_PACKET_END_BYTE = 0xF3
RESPONSE_ID_COMMAND_ACK = 0x01 # ID for command acknowledgments/errors

# Status Codes (matched to Arduino)
STATUS_OK = 0x00
STATUS_ERROR_INVALID_TARGET_ID      = 0xE1
STATUS_ERROR_INVALID_STATE_VALUE    = 0xE2
STATUS_ERROR_INVALID_PAYLOAD_SIZE   = 0xE3
STATUS_ERROR_INVALID_COMMAND_TYPE   = 0xE4
STATUS_ERROR_HARDWARE_FAILURE       = 0xE5 # Corresponds to Arduino's ERROR_INVALID_COMMAND_EXC
STATUS_ERROR_UNKNOWN_ISSUE          = 0xE6 # Corresponds to Arduino's ERROR_UNKNOWN_ISSUE


# Define the structure of the appended SensorTiming data
# struct SensorTiming { byte sensor_id; unsigned long start_micros; unsigned long end_micros; unsigned long duration_micros; };
# Python struct format: <BIII (1 byte, 3 unsigned 4-byte integers)
SENSOR_TIMING_STRUCT_SIZE = 1 + 4 + 4 + 4  # 13 bytes
# SENSOR_TIMING_STRUCT_FORMAT = '<BIII' # We will no longer use this for concatenation
# Instead, we'll use the individual type codes for timing:
TIMING_FIELD_FORMAT = 'BIII' # Just the type codes, no leading '<'

SENSOR_TIMING_STRUCT_FIELDS = ['timing_sensor_id', 'start_micros', 'end_micros', 'duration_micros']


# Define expected payload structures and sizes for ALL packet types
PACKET_TYPES = {
    PRESSURE_PACKET_START_BYTE: {
        'name': 'Pressure',
        'end_byte': PRESSURE_PACKET_END_BYTE,
        'payload_size': 4 + SENSOR_TIMING_STRUCT_SIZE, # float + timing struct
        # CHANGED: Direct format string, removed redundant '<'
        'format': '<f' + TIMING_FIELD_FORMAT, # Now becomes '<fBIII'
        'fields': ['pressure'] + SENSOR_TIMING_STRUCT_FIELDS,
        'ids': list(range(PRESSURE_ID_START, PRESSURE_ID_START + NUM_IDS_PRESSURE))
    },
    LOADCELL_PACKET_START_BYTE: {
        'name': 'LoadCell',
        'end_byte': LOADCELL_PACKET_END_BYTE,
        'payload_size': 4 + SENSOR_TIMING_STRUCT_SIZE, # float + timing struct
        # CHANGED: Direct format string, removed redundant '<'
        'format': '<f' + TIMING_FIELD_FORMAT, # Now becomes '<fBIII'
        'fields': ['weight_grams'] + SENSOR_TIMING_STRUCT_FIELDS,
        'ids': list(range(LOADCELL_ID_START, LOADCELL_ID_START + NUM_IDS_LOADCELL))
    },
    FLOW_PACKET_START_BYTE: {
        'name': 'Flow',
        'end_byte': FLOW_PACKET_END_BYTE,
        'payload_size': 4 + SENSOR_TIMING_STRUCT_SIZE, # float + timing struct
        # CHANGED: Direct format string, removed redundant '<'
        'format': '<f' + TIMING_FIELD_FORMAT, # Now becomes '<fBIII'
        'fields': ['flow_rate_lpm'] + SENSOR_TIMING_STRUCT_FIELDS,
        'ids': [FLOW_SENSOR_ID]
    },
    TEMP_PACKET_START_BYTE: {
        'name': 'Temperature',
        'end_byte': TEMP_PACKET_END_BYTE,
        'payload_size': 8 + SENSOR_TIMING_STRUCT_SIZE, # 2 floats + timing struct
        # CHANGED: Direct format string, removed redundant '<'
        'format': '<ff' + TIMING_FIELD_FORMAT, # Now becomes '<ffBIII'
        'fields': ['temp_c', 'temp_f'] + SENSOR_TIMING_STRUCT_FIELDS,
        'ids': list(range(TEMP_ID_START, TEMP_ID_START + NUM_IDS_TEMP))
    },
    MOTOR_RPM_PACKET_START_BYTE: {
        'name': 'MotorRPM',
        'end_byte': MOTOR_RPM_PACKET_END_BYTE,
        'payload_size': 4 + SENSOR_TIMING_STRUCT_SIZE, # float + timing struct
        # CHANGED: Direct format string, removed redundant '<'
        'format': '<f' + TIMING_FIELD_FORMAT, # Now becomes '<fBIII'
        'fields': ['rpm'] + SENSOR_TIMING_STRUCT_FIELDS,
        'ids': [MOTOR_RPM_ID]
    },
    # This entry is ONLY for category timing packets, which are still sent separately
    TIMING_PACKET_START_BYTE: {
        'name': 'Timing',
        'end_byte': TIMING_PACKET_END_BYTE,
        'payload_size': SENSOR_TIMING_STRUCT_SIZE, # Only the timing struct
        # CHANGED: Direct format string, kept single '<'
        'format': '<' + TIMING_FIELD_FORMAT, # Now becomes '<BIII'
        'fields': SENSOR_TIMING_STRUCT_FIELDS,
        'ids': [TIMING_SENSOR_OPERATION_ID, TIMING_CATEGORY_CYCLE_ID] # Timing Type IDs
    },
    # Command Response Packet Type
    RESPONSE_PACKET_START_BYTE: {
        'name': 'CommandResponse',
        'end_byte': RESPONSE_PACKET_END_BYTE,
        'payload_size': 3, # originalCommandType (1B), originalTargetId (1B), statusCode (1B)
        'format': '<BBB',
        'fields': ['original_cmd_type', 'original_target_id', 'status_code'],
        'ids': [RESPONSE_ID_COMMAND_ACK] # The ID field in the packet is RESPONSE_ID_COMMAND_ACK
    }
}

# Mapping from Packet ID to Type Info (for sensor data packets only)
ID_TO_PACKET_INFO = {}

# Parsing State Machine
STATE_WAITING_FOR_START = 0
STATE_READING_HEADER = 1
STATE_READING_PAYLOAD = 2
STATE_READING_END = 3

class SerialPacketReceiver:
    def __init__(self):
        self.state = STATE_WAITING_FOR_START
        self.current_packet_type_info = None
        self.current_start_byte = None
        self.current_id = None
        self.payload_size = 0
        self.payload_buffer = b''
        self.latest_sensor_data = {} # Stores sensor values (e.g., pressure, rpm)
        self.latest_timing_data = {} # Stores ALL timing data (individual sensor and category cycles)
        self.relay_states = {i: 0 for i in range(CMD_TARGET_RELAY_START, CMD_TARGET_RELAY_START + 4)}
        self.response_queue = queue.Queue() # Queue to pass command responses to the main thread

    def process_byte(self, byte_data):
        """Processes a single incoming byte."""
        if self.state == STATE_WAITING_FOR_START:
            if byte_data in PACKET_TYPES:
                self.state = STATE_READING_HEADER
                self.current_start_byte = byte_data
                self.current_packet_type_info = PACKET_TYPES[byte_data]
                self.payload_buffer = b''
            # else: ignore unexpected byte

        elif self.state == STATE_READING_HEADER:
            if len(self.payload_buffer) == 0:  # Reading ID
                self.current_id = byte_data
                self.payload_buffer += bytes([byte_data])
            elif len(self.payload_buffer) == 1:  # Reading Size
                self.payload_size = byte_data
                self.payload_buffer += bytes([byte_data])
                
                expected_size = self.current_packet_type_info['payload_size']
                expected_ids = self.current_packet_type_info.get('ids')

                # Validate payload size
                if self.payload_size != expected_size:
                    logger.warning(f"Packet {self.current_packet_type_info['name']} (start {self.current_start_byte:02X}) - Declared size ({self.payload_size}) != expected ({expected_size}). Discarding. Payload buffer: {self.payload_buffer.hex()}")
                    self._reset_state()
                # Validate ID for non-timing/non-response packets
                elif self.current_start_byte not in [TIMING_PACKET_START_BYTE, RESPONSE_PACKET_START_BYTE] and \
                     expected_ids is not None and self.current_id not in expected_ids:
                    logger.warning(f"Packet {self.current_packet_type_info['name']} (start {self.current_start_byte:02X}) - Unexpected ID ({self.current_id}). Discarding. Payload buffer: {self.payload_buffer.hex()}")
                    self._reset_state()
                # Validate ID for timing packets (which use TIMING_SENSOR_OPERATION_ID or TIMING_CATEGORY_CYCLE_ID as their 'id' field)
                elif self.current_start_byte == TIMING_PACKET_START_BYTE and \
                     self.current_id not in [TIMING_SENSOR_OPERATION_ID, TIMING_CATEGORY_CYCLE_ID]:
                    logger.warning(f"Timing Packet (start {self.current_start_byte:02X}) - Unexpected Timing Type ID ({self.current_id}). Discarding. Payload buffer: {self.payload_buffer.hex()}")
                    self._reset_state()
                # Validate ID for response packets (which use RESPONSE_ID_COMMAND_ACK as their 'id' field)
                elif self.current_start_byte == RESPONSE_PACKET_START_BYTE and \
                     self.current_id != RESPONSE_ID_COMMAND_ACK:
                    logger.warning(f"Response Packet (start {self.current_start_byte:02X}) - Unexpected Response ID ({self.current_id}). Discarding. Payload buffer: {self.payload_buffer.hex()}")
                    self._reset_state()
                elif self.payload_size == 0:
                    self.state = STATE_READING_END
                    self.payload_buffer = b''
                else:
                    self.state = STATE_READING_PAYLOAD
                    self.payload_buffer = b''

        elif self.state == STATE_READING_PAYLOAD:
            self.payload_buffer += bytes([byte_data])
            if len(self.payload_buffer) == self.payload_size:
                self.state = STATE_READING_END

        elif self.state == STATE_READING_END:
            expected_end_byte = self.current_packet_type_info['end_byte']
            if byte_data == expected_end_byte:
                self._process_complete_packet()
            else:
                logger.warning(f"Protocol Error for packet starting with {self.current_start_byte:02X}. Expected End Byte {expected_end_byte:02X}, got {byte_data:02X}. Discarding. Incomplete payload: {self.payload_buffer.hex()}")
            self._reset_state()

    def _process_complete_packet(self):
        """Handles a successfully received and validated packet."""
        packet_info = self.current_packet_type_info
        try:
            all_values = struct.unpack(packet_info['format'], self.payload_buffer)
            
            if packet_info['name'] == 'Timing': # This block is for Category Timing packets
                # Timing packets only contain the SensorTiming struct
                timing_data_values = all_values
                (timing_sensor_id_in_payload, start_micros, end_micros, duration_micros) = timing_data_values
                
                timing_type_id = self.current_id # This is TIMING_CATEGORY_CYCLE_ID or TIMING_SENSOR_OPERATION_ID (if sent separately)
                
                category_name = "Unknown"
                if timing_sensor_id_in_payload == PRESSURE_ID_START: category_name = "Pressure"
                elif timing_sensor_id_in_payload == LOADCELL_ID_START: category_name = "LoadCell"
                elif timing_sensor_id_in_payload == FLOW_SENSOR_ID: category_name = "Flow"
                elif timing_sensor_id_in_payload == TEMP_ID_START: category_name = "Temperature"
                elif timing_sensor_id_in_payload == MOTOR_RPM_ID: category_name = "MotorRPM"

                self.latest_timing_data[('category', category_name)] = {
                    'start': start_micros,
                    'end': end_micros,
                    'duration': duration_micros,
                    'source_id': timing_sensor_id_in_payload # Store the ID of the category's first sensor
                }
                # CHANGED: Log all received timing packets immediately at DEBUG level
                logger.debug(f"[RAW TIMING] Type=0x{timing_type_id:02X}, CategoryID={timing_sensor_id_in_payload}, Duration={duration_micros} us")

            elif packet_info['name'] == 'CommandResponse': # Handle Command Response packets
                original_cmd_type, original_target_id, status_code = all_values
                # Log all received command responses immediately at DEBUG level
                status_messages = {
                    STATUS_OK: "OK",
                    STATUS_ERROR_INVALID_TARGET_ID: "Invalid Target ID",
                    STATUS_ERROR_INVALID_STATE_VALUE: "Invalid State Value",
                    STATUS_ERROR_INVALID_PAYLOAD_SIZE: "Invalid Payload Size",
                    STATUS_ERROR_INVALID_COMMAND_TYPE: "Invalid Command Type",
                    STATUS_ERROR_HARDWARE_FAILURE: "Hardware Failure",
                    STATUS_ERROR_UNKNOWN_ISSUE: "Unknown Issue"
                }
                status_text = status_messages.get(status_code, f"UNKNOWN STATUS {status_code:02X}")
                # CHANGED: Log at DEBUG level
                logger.debug(f"[RAW RESPONSE] CmdType=0x{original_cmd_type:02X}, TargetID={original_target_id}, Status=0x{status_code:02X} ({status_text})")
                
                # Put the response into the queue for the main thread to process
                self.response_queue.put({
                    'type': 'response',
                    'original_cmd_type': original_cmd_type,
                    'original_target_id': original_target_id,
                    'status_code': status_code
                })

            else: # Regular sensor data packet (now includes embedded timing)
                # Separate sensor data from timing data
                sensor_data_values = all_values[:-len(SENSOR_TIMING_STRUCT_FIELDS)]
                timing_data_values = all_values[-len(SENSOR_TIMING_STRUCT_FIELDS):]

                # Extract timing fields
                (timing_sensor_id_in_payload, start_micros, end_micros, duration_micros) = timing_data_values

                # Store sensor data
                self.latest_sensor_data[self.current_id] = {
                    'type': packet_info['name'],
                    'timestamp': time.time(),
                    'values': sensor_data_values,
                    'fields': packet_info['fields'][:-len(SENSOR_TIMING_STRUCT_FIELDS)] # Only sensor-specific fields
                }
                
                # Store individual sensor timing data
                self.latest_timing_data[('individual', self.current_id)] = {
                    'start': start_micros,
                    'end': end_micros,
                    'duration': duration_micros, # Store raw microseconds
                    'source_id': timing_sensor_id_in_payload # This should match self.current_id for individual timings
                }
                # CHANGED: Log all received sensor packets immediately at DEBUG level
                logger.debug(f"[RAW SENSOR] {packet_info['name']} ID {self.current_id}: Values={sensor_data_values}, Timing Duration={duration_micros} us")

        except struct.error as e:
            logger.error(f"Unpacking {packet_info['name']} packet ID {self.current_id}: {e}. Raw payload: {self.payload_buffer.hex()}")
        except Exception as e:
            logger.error(f"Processing packet {packet_info['name']} ID {self.current_id}: {e}. Raw payload: {self.payload_buffer.hex()}")

    def _reset_state(self):
        self.state = STATE_WAITING_FOR_START
        self.payload_buffer = b''
        self.current_packet_type_info = None
        self.current_start_byte = None
        self.current_id = None
        self.payload_size = 0

    def get_all_sensor_ids_and_types(self):
        """Returns a list of (sensor_id, sensor_type_name) for all known sensors."""
        all_sensors_info = []
        for start_byte, info in PACKET_TYPES.items():
            # Skip timing and command/response packets, only interested in sensor data here
            if start_byte in [TIMING_PACKET_START_BYTE, COMMAND_START_BYTE, RESPONSE_PACKET_START_BYTE]:
                continue
            sensor_type_name = info['name']
            for sensor_id in info.get('ids', []):
                all_sensors_info.append((sensor_id, sensor_type_name))
        # Sort by ID for consistent display order
        return sorted(all_sensors_info, key=lambda x: x[0])

    def max_header_len_for_display(self):
        """Calculates the maximum header length needed for display, considering all sensors, relays, and timing."""
        max_len = 20 # Default minimum width

        # Sensor data headers (including space for timing)
        for sensor_id, sensor_type in self.get_all_sensor_ids_and_types():
            # Estimate max length for sensor data + timing string for layout
            # e.g., "[Pressure ID 0] pressure: 12.34 bar (Time: 1234 us / 0.0012 s)"
            # This is a rough estimate, actual length depends on values
            header_str = f"[{sensor_type} ID {sensor_id}]"
            # Add a typical length for the data and timing part
            estimated_data_timing_len = 40 # e.g., "pressure: 12.34 bar (Time: 1234 us / 0.0012 s)"
            max_len = max(max_len, len(header_str) + estimated_data_timing_len)
            
        # Timing data headers (for category timings)
        for timing_key in self.latest_timing_data.keys():
            timing_type, identifier = timing_key
            if timing_type == 'category': # Only category timings have a distinct header here
                header_str = f"[Timing {timing_type.capitalize()} {identifier}]"
                # Add typical length for category timing data
                estimated_category_timing_len = 60 # e.g., "Start: 1234567 us, End: 1234567 us, Duration: 1234 us (0.0012 s)"
                max_len = max(max_len, len(header_str) + estimated_category_timing_len)

        # Relay state headers
        if self.relay_states:
            for relay_id in self.relay_states.keys():
                header_str = f"[Relay ID {relay_id}]"
                max_len = max(max_len, len(header_str) + 15) # "state: OPEN/CLOSE"
            
        return max_len

    def print_all_latest_data(self, status_message=None):
        """Prints the latest stored data for all known sensors, relay states, and timing info."""
        max_header_len = self.max_header_len_for_display() # Use the new helper method
        
        # Clear screen for a dashboard-like view
        os.system('cls' if os.name == 'nt' else 'clear')

        # Use print() for the dashboard-like display, logger.info for general CLI messages
        print(BOLD + BLUE + "-" * (max_header_len + 50) + RESET)
        print(BOLD + BLUE + f"{'--- SENSOR DATA ---':<{max_header_len + 50}}" + RESET)

        # Iterate through all known sensor IDs and print their data
        for sensor_id, packet_type in self.get_all_sensor_ids_and_types():
            header = f"[{packet_type} ID {sensor_id}]"
            
            output_parts = [f"{header:<{max_header_len}} "] # Start with header

            if sensor_id in self.latest_sensor_data:
                data = self.latest_sensor_data[sensor_id]
                values = data['values']
                fields = data['fields']
                
                # Apply specific formatting based on the packet type and field name
                for i, field_name in enumerate(fields):
                    value = values[i]
                    if packet_type == 'Pressure' and field_name == 'pressure':
                        output_parts.append(f"{field_name}: {value:.2f} bar ")
                    elif packet_type == 'LoadCell' and field_name == 'weight_grams':
                        output_parts.append(f"{field_name}: {value:.3f} g ")
                    elif packet_type == 'Flow' and field_name == 'flow_rate_lpm':
                        output_parts.append(f"{field_name}: {value:.3f} LPM ")
                    elif packet_type == 'Temperature':
                        if field_name == 'temp_c':
                            output_parts.append(f"{field_name}: {'ERR (Open TC)' if math.isnan(value) else f'{value:.1f} C'} ")
                        elif field_name == 'temp_f':
                            output_parts.append(f"{field_name}: {'ERR (Open TC)' if math.isnan(value) else f'{value:.1f} F'} ")
                    elif packet_type == 'MotorRPM' and field_name == 'rpm':
                        output_parts.append(f"{field_name}: {value:.1f} RPM ")
                    else:
                        output_parts.append(f"{field_name}: {value:.3f} ") # Default for any other type/field
                
                # Append individual sensor timing data if available
                timing_key = ('individual', sensor_id)
                if timing_key in self.latest_timing_data:
                    timing_data = self.latest_timing_data[timing_key]
                    duration_seconds = timing_data['duration'] / 1_000_000.0 # Convert micros to seconds
                    # Display both microseconds and seconds
                    output_parts.append(f"(Time: {timing_data['duration']:,} us / {duration_seconds:.4f} s)")  
                
                print("".join(output_parts).strip()) # Use print() for sensor data lines
            else:
                # If no data, print "No data"
                print(f"{header:<{max_header_len}} No data") # Use print()

        # Print relay states
        print(BOLD + BLUE + f"\n{'--- RELAY STATES ---':<{max_header_len + 50}}" + RESET)
        for relay_id in sorted(self.relay_states.keys()):
            state = self.relay_states[relay_id]
            header = f"[Relay ID {relay_id}]"
            output = f"{header:<{max_header_len}} state: {GREEN + 'OPEN' + RESET if state else RED + 'CLOSE' + RESET}"
            print(output) # Use print()

        # Print timing data (now primarily for category cycles)
        print(BOLD + BLUE + f"\n{'--- TIMING DATA (microseconds / seconds) ---':<{max_header_len + 50}}" + RESET)
        # Filter for category timings and sort them
        sorted_category_timing_keys = sorted([k for k in self.latest_timing_data.keys() if k[0] == 'category'], key=lambda x: x[1])

        if not sorted_category_timing_keys:
            print(f"{'No category timing data received yet.':<{max_header_len + 50}}") # Use print()
        else:
            for timing_key in sorted_category_timing_keys:
                timing_type, identifier = timing_key # identifier is the category name (e.g., "Pressure")
                timing_data = self.latest_timing_data[timing_key]
                
                duration_seconds = timing_data['duration'] / 1_000_000.0 # Convert micros to seconds

                header = f"[Timing {timing_type.capitalize()} {identifier}]"
                output = (f"{header:<{max_header_len}} "
                          f"Start: {timing_data['start']:,} us, "
                          f"End: {timing_data['end']:,} us, "
                          f"Duration: {timing_data['duration']:,} us ({duration_seconds:.4f} s)")
                print(output) # Use print()
            
        # NEW: Print ephemeral status message
        if status_message:
            print(BOLD + MAGENTA + f"\n{'--- STATUS ---':<{max_header_len + 50}}" + RESET)
            print(BOLD + MAGENTA + f"{status_message:<{max_header_len + 50}}" + RESET)


# --- (Rest of your code, including auto_detect_arduino_port, send_motor_control_command,
# send_relay_control_command, serial_reader_thread, command_input_thread, print_help_message, main) ---
def auto_detect_arduino_port():
    """Detects Arduino Mega port, returns port name or None if not found/ambiguous."""
    arduino_ports = []
    search_terms = ['Arduino', 'USB-SERIAL', 'VID:PID=2341', 'VID:PID=1A86', 'VID:PID=0403', 'usbmodem']
    logger.info("Searching for Arduino port...")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        port_description = port.description.lower()
        port_hwid = port.hwid.lower() if port.hwid else ''
        for term in search_terms:
            if term.lower() in port_description or term.lower() in port_hwid:
                arduino_ports.append(port.device)
                break
    if len(arduino_ports) == 1:
        logger.info(f"Automatically selected port: {arduino_ports[0]}")
        return arduino_ports[0]
    elif len(arduino_ports) > 1:
        logger.info(f"Found multiple potential Arduino ports: {', '.join(arduino_ports)}")
        logger.info("Please specify the port using the --port argument.")
        return None
    else:
        logger.info("No Arduino port found.")
        return None

def send_motor_control_command(ser, motor_id, throttle, enable=1):
    """
    Send a motor control command to the Arduino.
    :param ser: Open serial connection
    :param motor_id: Target motor ID (0 for CMD_TARGET_MOTOR_ID)
    :param throttle: Motor throttle (0-100%)
    :param enable: Motor enable state (0 for disabled, 1 for enabled, default: 1)
    """
    try:
        if not (0 <= throttle <= 100):
            logger.error("Throttle must be between 0 and 100.")
            return
        if enable not in (0, 1):
            logger.error("Enable must be 0 (disabled) or 1 (enabled).")
            return
        
        payload = struct.pack('>BB', enable, throttle)
        payload_size = len(payload)
        if payload_size > MAX_COMMAND_PAYLOAD_SIZE:
            logger.error(f"Payload size {payload_size} exceeds max {MAX_COMMAND_PAYLOAD_SIZE}.")
            return
        packet = bytearray()
        packet.append(COMMAND_START_BYTE)
        packet.append(CMD_TYPE_SET_MOTOR)
        packet.append(motor_id)
        packet.append(payload_size)
        packet.extend(payload)
        packet.append(COMMAND_END_BYTE)
        ser.write(packet)
        # logger.info(f"Sent motor control command: ID={motor_id}, Enable={enable}, Throttle={throttle}%")
        # Removed direct logging here, main loop will handle status via ACK/NACK
    except serial.SerialException as e:
        logger.error(f"Serial error when sending motor command: {e}")
    except Exception as e:
        logger.error(f"Error sending motor command: {e}")

def send_relay_control_command(ser, relay_id, state): # Removed receiver as it's not updated here directly
    """
    Send a relay control command to the Arduino.
    :param ser: Open serial connection
    :param relay_id: Target relay ID (0, 1, 2, 3)
    :param state: Relay state (0 for OFF, 1 for ON)
    """
    try:
        if state not in (0, 1):
            logger.error("Relay state must be 0 (OFF) or 1 (ON).")
            return
        if relay_id not in range(CMD_TARGET_RELAY_START, CMD_TARGET_RELAY_START + 4):
            logger.error(f"Relay ID {relay_id} is out of valid range ({CMD_TARGET_RELAY_START}-{CMD_TARGET_RELAY_START + 3}).")
            return

        payload = struct.pack('>B', state)
        payload_size = len(payload)
        if payload_size > MAX_COMMAND_PAYLOAD_SIZE:
            logger.error(f"Payload size {payload_size} exceeds max {MAX_COMMAND_PAYLOAD_SIZE}.")
            return
        packet = bytearray()
        packet.append(COMMAND_START_BYTE)
        packet.append(CMD_TYPE_SET_RELAY)
        packet.append(relay_id)
        packet.append(payload_size)
        packet.extend(payload)
        packet.append(COMMAND_END_BYTE)
        ser.write(packet)
        # logger.info(f"Sent relay control command: ID={relay_id}, State={'ON' if state else 'OFF'}")
        # Removed direct logging here, main loop will handle status via ACK/NACK
    except serial.SerialException as e:
        logger.error(f"Serial error when sending relay command: {e}")
    except Exception as e:
        logger.error(f"Error sending relay command: {e}")

def serial_reader_thread(ser, receiver):
    """Reads serial data in a separate thread."""
    logger.info("Serial reading thread started.")
    while ser.is_open:
        try:
            byte_data = ser.read(1)
            if byte_data:
                receiver.process_byte(byte_data[0])
        except serial.SerialException as e:
            logger.error(f"Serial reading thread error: {e}")
            break
        except Exception as e:
            logger.error(f"Unexpected error in serial reading thread: {e}")
            break
    logger.info("Serial reading thread finished.")

def command_input_thread(command_queue):
    """Reads user input in a separate thread and puts commands into the queue."""
    logger.info("Command input thread started.")
    while True:
        try:
            command = input("CLI> ").strip()
            command_queue.put(command)
        except EOFError:
            command_queue.put('q')
            break
        except Exception as e:
            logger.error(f"Error in command input thread: {e}")
            break
    logger.info("Command input thread finished.")

def print_help_message(max_header_len):
    """Prints the available commands and their syntax."""
    # CHANGED: Use print() instead of logger.info() for help message to keep it off Work.log
    print(BOLD + BLUE + f"\n{'--- COMMANDS ---':<{max_header_len + 50}}" + RESET)
    print("  'm <throttle>'         : Set motor throttle (0-100%). E.g., 'm 50'")
    print("  'r <id> <state>'       : Set relay state (id: 0-3, state: 0=OFF, 1=ON). E.g., 'r 0 1'")
    print("  'u <interval>'         : Set sensor data update interval in seconds (e.g., 'u 0.5')")
    print("  'h' or 'help'          : Display this help message.")
    print("  'q'                    : Quit the application.")
    print(BOLD + BLUE + "-" * (max_header_len + 50) + RESET)


def main():
    # Build ID_TO_PACKET_INFO lookup table for non-timing packets
    for start_byte, info in PACKET_TYPES.items():
        # Only map sensor data packets by their ID
        if start_byte not in [TIMING_PACKET_START_BYTE, COMMAND_START_BYTE, RESPONSE_PACKET_START_BYTE]:
            for sensor_id in info.get('ids', []):
                if sensor_id in ID_TO_PACKET_INFO:
                    existing_start_byte = ID_TO_PACKET_INFO[sensor_id]
                    existing_type_name = PACKET_TYPES[existing_start_byte]['name']
                    logger.warning(f"Duplicate sensor ID {sensor_id} found! Defined for {existing_type_name} (start {existing_start_byte:02X}) and {info['name']} (start {start_byte:02X}).")
                ID_TO_PACKET_INFO[sensor_id] = start_byte

    # Argument parser
    parser = argparse.ArgumentParser(description='Monitor sensor data and send motor/relay commands to Arduino Mega.')
    parser.add_argument('--port', help='Serial port name (e.g., COM3). If not specified, attempts auto-detection.')
    parser.add_argument('-b', '--baudrate', type=int, default=115200, help='Serial baud rate (default: 115200)')
    parser.add_argument('-t', '--timeout', type=float, default=0.1, help='Serial read timeout in seconds (default: 0.1)')
    parser.add_argument('-u', '--update-interval', type=float, default=1.0, help='Initial interval to print sensor data (default: 1.0)')
    args = parser.parse_args()

    SERIAL_PORT = args.port if args.port else auto_detect_arduino_port()
    if SERIAL_PORT is None:
        logger.error("No suitable serial port found or specified. Exiting.")
        sys.exit(1)

    BAUDRATE = args.baudrate
    READ_TIMEOUT = args.timeout
    UPDATE_INTERVAL = args.update_interval

    receiver = SerialPacketReceiver()
    command_queue = queue.Queue()
    serial_thread = None
    input_thread = None
    ser = None

    # NEW: Command timeout and status message variables
    COMMAND_ACK_TIMEOUT_SECONDS = 3.0 # How long to wait for an ACK/NACK
    STATUS_MESSAGE_DISPLAY_DURATION = 4.0 # How long ephemeral messages stay on screen

    pending_command = { # Tracks the last command sent that's awaiting an ACK
        'type': None, # CMD_TYPE_SET_MOTOR or CMD_TYPE_SET_RELAY
        'target_id': None, # Motor ID or Relay ID
        'desired_state': None, # For relay: 0/1; for motor: (enable, throttle) tuple
        'sent_time': None # Timestamp when command was sent
    }
    status_message = None
    status_message_expiry_time = 0.0

    try:
        logger.info(f"Opening serial port {SERIAL_PORT} at {BAUDRATE} baud...")
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=READ_TIMEOUT)
        logger.info("Serial port opened successfully.")
        time.sleep(2) # Give Arduino time to initialize and send initial messages

        serial_thread = threading.Thread(target=serial_reader_thread, args=(ser, receiver), daemon=True)
        serial_thread.start()

        input_thread = threading.Thread(target=command_input_thread, args=(command_queue,), daemon=True)
        input_thread.start()

        logger.info(f"Connected to Arduino Mega on {SERIAL_PORT}")
        print_help_message(max(20, receiver.max_header_len_for_display())) # CHANGED: Call print_help_message directly
        last_print_time = time.time()

        while True:
            current_time = time.time()

            # --- Process incoming command responses from Arduino ---
            try:
                response = receiver.response_queue.get_nowait()
                if response['type'] == 'response' and pending_command['sent_time'] is not None:
                    # Check if this response matches our pending command
                    if (response['original_cmd_type'] == pending_command['type'] and
                        response['original_target_id'] == pending_command['target_id']):
                        
                        # Command matched, process status
                        status_code = response['status_code']
                        status_messages = {
                            STATUS_OK: "OK",
                            STATUS_ERROR_INVALID_TARGET_ID: "ERROR: Invalid Target ID",
                            STATUS_ERROR_INVALID_STATE_VALUE: "ERROR: Invalid State Value",
                            STATUS_ERROR_INVALID_PAYLOAD_SIZE: "ERROR: Invalid Payload Size",
                            STATUS_ERROR_INVALID_COMMAND_TYPE: "ERROR: Invalid Command Type",
                            STATUS_ERROR_HARDWARE_FAILURE: "ERROR: Hardware Failure (Command Execution Failed)", # Added for clarity
                            STATUS_ERROR_UNKNOWN_ISSUE: "ERROR: Unknown Issue" # Added for clarity
                        }
                        status_text = status_messages.get(status_code, f"UNKNOWN STATUS {status_code:02X}")

                        if status_code == STATUS_OK:
                            status_message = GREEN + f"Command successful: {status_text}" + RESET
                            # NEW: Update CLI state ONLY on STATUS_OK for relays
                            if pending_command['type'] == CMD_TYPE_SET_RELAY:
                                receiver.relay_states[pending_command['target_id']] = pending_command['desired_state']
                        else:
                            status_message = RED + f"Command failed: {status_text}" + RESET
                        
                        pending_command = {'type': None, 'target_id': None, 'desired_state': None, 'sent_time': None} # Clear pending
                        status_message_expiry_time = current_time + STATUS_MESSAGE_DISPLAY_DURATION
                    else:
                        # Response received, but it doesn't match the pending command (e.g., an old response)
                        # This could be logged at DEBUG level as it's not an error in itself
                        logger.debug(f"Received unmatched response: CmdType={response['original_cmd_type']:02X}, TargetID={response['original_target_id']}, Status={response['status_code']:02X}")
            except queue.Empty:
                pass # No response in queue

            # --- Check for command timeout ---
            if pending_command['sent_time'] is not None and \
               current_time - pending_command['sent_time'] > COMMAND_ACK_TIMEOUT_SECONDS:
                status_message = YELLOW + f"Command timed out for CmdType={pending_command['type']:02X}, TargetID={pending_command['target_id']}. No ACK received." + RESET
                pending_command = {'type': None, 'target_id': None, 'desired_state': None, 'sent_time': None} # Clear pending
                status_message_expiry_time = current_time + STATUS_MESSAGE_DISPLAY_DURATION


            # --- Print sensor data and status message ---
            if current_time - last_print_time >= UPDATE_INTERVAL:
                # Clear status message if expired
                if status_message and current_time >= status_message_expiry_time:
                    status_message = None
                    status_message_expiry_time = 0.0

                receiver.print_all_latest_data(status_message=status_message)
                last_print_time = current_time

            # --- Process user input commands ---
            try:
                command = command_queue.get_nowait()
                parts = command.split()
                if not parts:
                    continue
                
                cmd_type = parts[0].lower()

                if cmd_type == 'q':
                    break
                elif cmd_type == 'h' or cmd_type == 'help':
                    print_help_message(max(20, receiver.max_header_len_for_display()))
                elif cmd_type == 'm' and len(parts) == 2:
                    try:
                        throttle = int(parts[1])
                        # Set pending command before sending
                        pending_command = {
                            'type': CMD_TYPE_SET_MOTOR,
                            'target_id': CMD_TARGET_MOTOR_ID,
                            'desired_state': (1, throttle), # (enable, throttle)
                            'sent_time': current_time
                        }
                        send_motor_control_command(ser, CMD_TARGET_MOTOR_ID, throttle, enable=1)
                        status_message = BOLD + CYAN + f"Sending motor command: Throttle={throttle}%..." + RESET
                        status_message_expiry_time = current_time + STATUS_MESSAGE_DISPLAY_DURATION
                    except ValueError:
                        logger.error("Invalid throttle. Use an integer number (0-100).")
                elif cmd_type == 'r' and len(parts) == 3:
                    try:
                        relay_id = int(parts[1])
                        state = int(parts[2])
                        # Set pending command before sending
                        pending_command = {
                            'type': CMD_TYPE_SET_RELAY,
                            'target_id': relay_id,
                            'desired_state': state,
                            'sent_time': current_time
                        }
                        send_relay_control_command(ser, relay_id, state) # Removed receiver argument
                        status_message = BOLD + CYAN + f"Sending relay command: ID={relay_id}, State={'ON' if state else 'OFF'}..." + RESET
                        status_message_expiry_time = current_time + STATUS_MESSAGE_DISPLAY_DURATION
                    except ValueError:
                        logger.error("Invalid relay command. Use integers for relay_id (0-3) and state (0 or 1).")
                elif cmd_type == 'u' and len(parts) == 2:
                    try:
                        new_interval = float(parts[1])
                        if new_interval > 0:
                            UPDATE_INTERVAL = new_interval
                            logger.info(f"Update interval set to {UPDATE_INTERVAL:.1f} seconds.")
                        else:
                            logger.error("Update interval must be a positive number.")
                    except ValueError:
                        logger.error("Invalid update interval. Use a positive number (e.g., 'u 0.5').")
                else:
                    logger.error(f"Unknown command: '{command}'. Type 'h' for help.")
            except queue.Empty:
                pass # No user input command in queue
            
            time.sleep(0.01) # Small sleep to prevent busy-waiting

    except serial.SerialException as e:
        logger.error(f"Could not open serial port {SERIAL_PORT}: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        logger.info("Exiting due to user interrupt.")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        sys.exit(1)
    finally:
        if ser and ser.is_open:
            # Send a command to stop the motor when exiting
            send_motor_control_command(ser, CMD_TARGET_MOTOR_ID, 0, enable=0)
            time.sleep(0.1) # Give Arduino time to process the stop command
            ser.close()
            logger.info("Serial port closed.")

if __name__ == "__main__":
    main()
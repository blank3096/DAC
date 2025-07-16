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

# Configure logging (rest of your logging config)
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[
        logging.FileHandler('Work.log', mode='a'),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger()

# Redirect stderr to logger (your StdErrToLogger class)
class StdErrToLogger:
    def __init__(self, logger):
        self.logger = logger

    def write(self, message):
        if message.strip():
            self.logger.error(message.strip())

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

# --- NEW: Timing Packet Constants (matched to Arduino SensorManager.cpp/h) ---
TIMING_PACKET_START_BYTE = 0xDE
TIMING_PACKET_END_BYTE = 0xAD
TIMING_SENSOR_OPERATION_ID = 0x01
TIMING_CATEGORY_CYCLE_ID = 0x02

# Define expected payload structures and sizes for ALL packet types
PACKET_TYPES = {
    PRESSURE_PACKET_START_BYTE: {
        'name': 'Pressure',
        'end_byte': PRESSURE_PACKET_END_BYTE,
        'payload_size': 4,
        'format': '<f',
        'fields': ['pressure'],
        'ids': list(range(PRESSURE_ID_START, PRESSURE_ID_START + NUM_IDS_PRESSURE))
    },
    LOADCELL_PACKET_START_BYTE: {
        'name': 'LoadCell',
        'end_byte': LOADCELL_PACKET_END_BYTE,
        'payload_size': 4,
        'format': '<f',
        'fields': ['weight_grams'],
        'ids': list(range(LOADCELL_ID_START, LOADCELL_ID_START + NUM_IDS_LOADCELL))
    },
    FLOW_PACKET_START_BYTE: {
        'name': 'Flow',
        'end_byte': FLOW_PACKET_END_BYTE,
        'payload_size': 4,
        'format': '<f',
        'fields': ['flow_rate_lpm'],
        'ids': [FLOW_SENSOR_ID]
    },
    TEMP_PACKET_START_BYTE: {
        'name': 'Temperature',
        'end_byte': TEMP_PACKET_END_BYTE,
        'payload_size': 8,
        'format': '<ff',
        'fields': ['temp_c', 'temp_f'],
        'ids': list(range(TEMP_ID_START, TEMP_ID_START + NUM_IDS_TEMP))
    },
    MOTOR_RPM_PACKET_START_BYTE: {
        'name': 'MotorRPM',
        'end_byte': MOTOR_RPM_PACKET_END_BYTE,
        'payload_size': 4,
        'format': '<f',
        'fields': ['rpm'],
        'ids': [MOTOR_RPM_ID]
    },
    TIMING_PACKET_START_BYTE: {
        'name': 'Timing',
        'end_byte': TIMING_PACKET_END_BYTE,
        'payload_size': 13,
        'format': '<BIII',
        'fields': ['sensor_id_in_payload', 'start_micros', 'end_micros', 'duration_micros'],
        'ids': [TIMING_SENSOR_OPERATION_ID, TIMING_CATEGORY_CYCLE_ID]
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
        self.latest_sensor_data = {}
        self.latest_timing_data = {}
        self.relay_states = {i: 0 for i in range(CMD_TARGET_RELAY_START, CMD_TARGET_RELAY_START + 4)}

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

                if self.payload_size != expected_size:
                    logger.warning(f"Packet {self.current_packet_type_info['name']} (start {self.current_start_byte:02X}) - Declared size ({self.payload_size}) != expected ({expected_size}). Discarding. Payload buffer: {self.payload_buffer.hex()}")
                    self._reset_state()
                elif self.current_start_byte != TIMING_PACKET_START_BYTE and \
                     expected_ids is not None and self.current_id not in expected_ids:
                    logger.warning(f"Packet {self.current_packet_type_info['name']} (start {self.current_start_byte:02X}) - Unexpected ID ({self.current_id}). Discarding. Payload buffer: {self.payload_buffer.hex()}")
                    self._reset_state()
                elif self.current_start_byte == TIMING_PACKET_START_BYTE and \
                     self.current_id not in [TIMING_SENSOR_OPERATION_ID, TIMING_CATEGORY_CYCLE_ID]:
                    logger.warning(f"Timing Packet (start {self.current_start_byte:02X}) - Unexpected Timing Type ID ({self.current_id}). Discarding. Payload buffer: {self.payload_buffer.hex()}")
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
            values = struct.unpack(packet_info['format'], self.payload_buffer)
            
            if packet_info['name'] == 'Timing':
                sensor_id_in_payload, start_micros, end_micros, duration_micros = values
                timing_type_id = self.current_id

                if timing_type_id == TIMING_SENSOR_OPERATION_ID:
                    self.latest_timing_data[('individual', sensor_id_in_payload)] = {
                        'start': start_micros,
                        'end': end_micros,
                        'duration': duration_micros
                    }
                elif timing_type_id == TIMING_CATEGORY_CYCLE_ID:
                    category_name = "Unknown"
                    if sensor_id_in_payload == PRESSURE_ID_START: category_name = "Pressure"
                    elif sensor_id_in_payload == LOADCELL_ID_START: category_name = "LoadCell"
                    elif sensor_id_in_payload == FLOW_SENSOR_ID: category_name = "Flow"
                    elif sensor_id_in_payload == TEMP_ID_START: category_name = "Temperature"
                    elif sensor_id_in_payload == MOTOR_RPM_ID: category_name = "MotorRPM"

                    self.latest_timing_data[('category', category_name)] = {
                        'start': start_micros,
                        'end': end_micros,
                        'duration': duration_micros
                    }
                logger.debug(f"Received Timing Packet: Type={timing_type_id:02X}, SensorID={sensor_id_in_payload}, Duration={duration_micros} us")

            else: # Regular sensor data packet
                self.latest_sensor_data[self.current_id] = {
                    'type': packet_info['name'],
                    'timestamp': time.time(),
                    'values': values,
                    'fields': packet_info['fields']
                }
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
            # Skip timing and command packets, only interested in sensor data here
            if start_byte in [TIMING_PACKET_START_BYTE, COMMAND_START_BYTE]:
                continue
            sensor_type_name = info['name']
            for sensor_id in info.get('ids', []):
                all_sensors_info.append((sensor_id, sensor_type_name))
        # Sort by ID for consistent display order
        return sorted(all_sensors_info, key=lambda x: x[0])

    def max_header_len_for_display(self):
        """Calculates the maximum header length needed for display, considering all sensors, relays, and timing."""
        max_len = 20 # Default minimum width

        # Sensor data headers
        for sensor_id, sensor_type in self.get_all_sensor_ids_and_types():
            header_str = f"[{sensor_type} ID {sensor_id}]"
            max_len = max(max_len, len(header_str))
        
        # Timing data headers
        for timing_key in self.latest_timing_data.keys():
            timing_type, identifier = timing_key
            header_str = f"[Timing {timing_type.capitalize()} {identifier}]"
            max_len = max(max_len, len(header_str))

        # Relay state headers
        if self.relay_states:
            for relay_id in self.relay_states.keys():
                header_str = f"[Relay ID {relay_id}]"
                max_len = max(max_len, len(header_str))
        
        return max_len

    def print_all_latest_data(self):
        """Prints the latest stored data for all known sensors, relay states, and timing info."""
        max_header_len = self.max_header_len_for_display() # Use the new helper method
        
        # Clear screen for a dashboard-like view
        os.system('cls' if os.name == 'nt' else 'clear')

        logger.info("-" * (max_header_len + 50))
        logger.info(f"{'--- SENSOR DATA ---':<{max_header_len + 50}}")

        # Iterate through all known sensor IDs and print their data
        for sensor_id, packet_type in self.get_all_sensor_ids_and_types():
            header = f"[{packet_type} ID {sensor_id}]"
            
            if sensor_id in self.latest_sensor_data:
                data = self.latest_sensor_data[sensor_id]
                values = data['values']
                fields = data['fields']
                output = f"{header:<{max_header_len}} "
                
                # Apply specific formatting based on the packet type and field name
                for i, field_name in enumerate(fields):
                    value = values[i]
                    if packet_type == 'Pressure' and field_name == 'pressure':
                        output += f"{field_name}: {value:.2f} bar "
                    elif packet_type == 'LoadCell' and field_name == 'weight_grams':
                        output += f"{field_name}: {value:.3f} g "
                    elif packet_type == 'Flow' and field_name == 'flow_rate_lpm':
                        output += f"{field_name}: {value:.3f} LPM "
                    elif packet_type == 'Temperature':
                        if field_name == 'temp_c':
                            output += f"{field_name}: {'ERR (Open TC)' if math.isnan(value) else f'{value:.1f} C'} "
                        elif field_name == 'temp_f':
                            output += f"{field_name}: {'ERR (Open TC)' if math.isnan(value) else f'{value:.1f} F'} "
                    elif packet_type == 'MotorRPM' and field_name == 'rpm':
                        output += f"{field_name}: {value:.1f} RPM "
                    else:
                        output += f"{field_name}: {value:.3f} " # Default for any other type/field
                logger.info(output.strip())
            else:
                # If no data, print "No data"
                logger.info(f"{header:<{max_header_len}} No data")

        # Print relay states (unchanged)
        logger.info(f"\n{'--- RELAY STATES ---':<{max_header_len + 50}}")
        for relay_id in sorted(self.relay_states.keys()):
            state = self.relay_states[relay_id]
            header = f"[Relay ID {relay_id}]"
            output = f"{header:<{max_header_len}} state: {'OPEN' if state else 'CLOSE'}"
            logger.info(output)

        # Print timing data (unchanged)
        logger.info(f"\n{'--- TIMING DATA (microseconds) ---':<{max_header_len + 50}}")
        sorted_timing_keys = sorted(self.latest_timing_data.keys(), key=lambda x: (0 if x[0] == 'category' else 1, x[1]))

        if not sorted_timing_keys:
            logger.info(f"{'No timing data received yet.':<{max_header_len + 50}}")
        else:
            for timing_key in sorted_timing_keys:
                timing_type, identifier = timing_key
                timing_data = self.latest_timing_data[timing_key]
                
                header = f"[Timing {timing_type.capitalize()} {identifier}]"
                output = (f"{header:<{max_header_len}} "
                          f"Start: {timing_data['start']:,} us, "
                          f"End: {timing_data['end']:,} us, "
                          f"Duration: {timing_data['duration']:,} us")
                logger.info(output)

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

def send_motor_control_command(ser, motor_id, throttle, enable=1, forward=1):
    """
    Send a motor control command to the Arduino.
    :param ser: Open serial connection
    :param motor_id: Target motor ID (0 for CMD_TARGET_MOTOR_ID)
    :param throttle: Motor throttle (0-100%)
    :param enable: Motor enable state (0 for disabled, 1 for enabled, default: 1)
    :param forward: Motor direction (0 for reverse, 1 for forward, default: 1)
    """
    try:
        if not (0 <= throttle <= 100):
            logger.error("Throttle must be between 0 and 100.")
            return
        if enable not in (0, 1):
            logger.error("Enable must be 0 (disabled) or 1 (enabled).")
            return
        if forward not in (0, 1):
            logger.error("Direction must be 0 (reverse) or 1 (forward).")
            return
        payload = struct.pack('>BBB', enable, forward, throttle)
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
        logger.info(f"Sent motor control command: ID={motor_id}, Enable={enable}, Forward={forward}, Throttle={throttle}%")
    except serial.SerialException as e:
        logger.error(f"Serial error when sending motor command: {e}")
    except Exception as e:
        logger.error(f"Error sending motor command: {e}")

def send_relay_control_command(ser, relay_id, state, receiver):
    """
    Send a relay control command to the Arduino and update relay state.
    :param ser: Open serial connection
    :param relay_id: Target relay ID (0, 1, 2, 3)
    :param state: Relay state (0 for OFF, 1 for ON)
    :param receiver: SerialPacketReceiver instance to update relay_states
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
        receiver.relay_states[relay_id] = state # Update local state immediately
        logger.info(f"Sent relay control command: ID={relay_id}, State={'ON' if state else 'OFF'}")
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
    logger.info(f"\n{'--- COMMANDS ---':<{max_header_len + 50}}")
    logger.info("  'm <throttle>'       : Set motor throttle (0-100%). E.g., 'm 50'")
    logger.info("  'r <id> <state>'     : Set relay state (id: 0-3, state: 0=OFF, 1=ON). E.g., 'r 0 1'")
    logger.info("  'u <interval>'       : Set sensor data update interval in seconds (e.g., 'u 0.5')")
    logger.info("  'h' or 'help'        : Display this help message.")
    logger.info("  'q'                  : Quit the application.")
    logger.info("-" * (max_header_len + 50))


def main():
    # Build ID_TO_PACKET_INFO lookup table for non-timing packets
    for start_byte, info in PACKET_TYPES.items():
        if start_byte == TIMING_PACKET_START_BYTE:
            continue
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

    try:
        logger.info(f"Opening serial port {SERIAL_PORT} at {BAUDRATE} baud...")
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=READ_TIMEOUT)
        logger.info("Serial port opened successfully.")
        time.sleep(2)

        serial_thread = threading.Thread(target=serial_reader_thread, args=(ser, receiver), daemon=True)
        serial_thread.start()

        input_thread = threading.Thread(target=command_input_thread, args=(command_queue,), daemon=True)
        input_thread.start()

        logger.info(f"Connected to Arduino Mega on {SERIAL_PORT}")
        print_help_message(20)
        last_print_time = time.time()

        while True:
            current_time = time.time()
            if current_time - last_print_time >= UPDATE_INTERVAL:
                receiver.print_all_latest_data()
                last_print_time = current_time

            try:
                command = command_queue.get_nowait()
                parts = command.split()
                if not parts:
                    continue
                
                cmd_type = parts[0].lower()

                if cmd_type == 'q':
                    break
                elif cmd_type == 'h' or cmd_type == 'help':
                    # Corrected call: Use the newly implemented method
                    print_help_message(max(20, receiver.max_header_len_for_display()))
                elif cmd_type == 'm' and len(parts) == 2:
                    try:
                        throttle = int(parts[1])
                        send_motor_control_command(ser, CMD_TARGET_MOTOR_ID, throttle, enable=1, forward=1)
                    except ValueError:
                        logger.error("Invalid throttle. Use an integer number (0-100).")
                elif cmd_type == 'r' and len(parts) == 3:
                    try:
                        relay_id = int(parts[1])
                        state = int(parts[2])
                        send_relay_control_command(ser, relay_id, state, receiver)
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
                pass
            time.sleep(0.01)

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
            send_motor_control_command(ser, CMD_TARGET_MOTOR_ID, 0, enable=0, forward=1)
            time.sleep(0.1)
            ser.close()
            logger.info("Serial port closed.")

if __name__ == "__main__":
    main()
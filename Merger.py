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

# Configure logging to write to Work.log and console with timestamps
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

# Redirect stderr to logger
class StdErrToLogger:
    def __init__(self, logger):
        self.logger = logger

    def write(self, message):
        if message.strip():  # Avoid logging empty lines
            self.logger.error(message.strip())

    def flush(self):
        pass

sys.stderr = StdErrToLogger(logger)

# Constants (matched to SensorManager.cpp)
COMMAND_START_BYTE = 0xFC  # Start byte for command packets
COMMAND_END_BYTE = 0xFD    # End byte for command packets
CMD_TYPE_SET_MOTOR = 0x02  # Command type for motor control
CMD_TYPE_SET_RELAY = 0x01  # Command type for relay control
CMD_TARGET_MOTOR_ID = 0    # Target ID for motor
CMD_TARGET_RELAY_START = 0 # Starting ID for relays (0, 1, 2, 3)
MAX_COMMAND_PAYLOAD_SIZE = 32  # Maximum payload size

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

# Define expected payload structures and sizes
PACKET_TYPES = {
    PRESSURE_PACKET_START_BYTE: {
        'name': 'Pressure',
        'end_byte': PRESSURE_PACKET_END_BYTE,
        'payload_size': 4,  # 1 float * 4 bytes/float
        'format': '<f',     # little-endian, 1 float (pressure)
        'fields': ['pressure'],
        'ids': list(range(PRESSURE_ID_START, PRESSURE_ID_START + NUM_IDS_PRESSURE))
    },
    LOADCELL_PACKET_START_BYTE: {
        'name': 'LoadCell',
        'end_byte': LOADCELL_PACKET_END_BYTE,
        'payload_size': 4,  # 1 float * 4 bytes/float
        'format': '<f',
        'fields': ['weight_grams'],
        'ids': list(range(LOADCELL_ID_START, LOADCELL_ID_START + NUM_IDS_LOADCELL))
    },
    FLOW_PACKET_START_BYTE: {
        'name': 'Flow',
        'end_byte': FLOW_PACKET_END_BYTE,
        'payload_size': 4,  # 1 float * 4 bytes/float
        'format': '<f',
        'fields': ['flow_rate_lpm'],
        'ids': [FLOW_SENSOR_ID]
    },
    TEMP_PACKET_START_BYTE: {
        'name': 'Temperature',
        'end_byte': TEMP_PACKET_END_BYTE,
        'payload_size': 8,  # 2 floats * 4 bytes/float
        'format': '<ff',
        'fields': ['temp_c', 'temp_f'],
        'ids': list(range(TEMP_ID_START, TEMP_ID_START + NUM_IDS_TEMP))
    },
    MOTOR_RPM_PACKET_START_BYTE: {
        'name': 'MotorRPM',
        'end_byte': MOTOR_RPM_PACKET_END_BYTE,
        'payload_size': 4,  # 1 float * 4 bytes/float
        'format': '<f',
        'fields': ['rpm'],
        'ids': [MOTOR_RPM_ID]
    }
}

# Mapping from Packet ID to Type Info
ID_TO_PACKET_INFO = {}

# Parsing State Machine
STATE_WAITING_FOR_START = 0
STATE_READING_HEADER = 1  # ID, Size
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
                    logger.warning(f"Packet {self.current_packet_type_info['name']} (start {self.current_start_byte:02X}) - Declared size ({self.payload_size}) != expected ({expected_size}). Discarding.")
                    self._reset_state()
                elif expected_ids is not None and self.current_id not in expected_ids:
                    logger.warning(f"Packet {self.current_packet_type_info['name']} (start {self.current_start_byte:02X}) - Unexpected ID ({self.current_id}). Discarding.")
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
                logger.warning(f"Protocol Error for packet starting with {self.current_start_byte:02X}. Expected End Byte {expected_end_byte:02X}, got {byte_data:02X}. Discarding.")
            self._reset_state()

    def _process_complete_packet(self):
        """Handles a successfully received and validated packet."""
        packet_info = self.current_packet_type_info
        try:
            values = struct.unpack(packet_info['format'], self.payload_buffer)
            self.latest_sensor_data[self.current_id] = {
                'type': packet_info['name'],
                'timestamp': time.time(),
                'values': values,
                'fields': packet_info['fields']
            }
        except struct.error as e:
            logger.error(f"Unpacking {packet_info['name']} packet ID {self.current_id}: {e}")
        except Exception as e:
            logger.error(f"Processing packet {packet_info['name']} ID {self.current_id}: {e}")

    def _reset_state(self):
        self.state = STATE_WAITING_FOR_START
        self.payload_buffer = b''
        self.current_packet_type_info = None
        self.current_start_byte = None
        self.current_id = None
        self.payload_size = 0

    def print_all_latest_data(self):
        """Prints the latest stored data for all known sensors."""
        if not self.latest_sensor_data:
            return
        sorted_ids = sorted(self.latest_sensor_data.keys())
        max_header_len = max(len(f"[{data['type']} ID {sid}]") for sid, data in self.latest_sensor_data.items())
        logger.info("-" * (max_header_len + 50))
        for sensor_id in sorted_ids:
            data = self.latest_sensor_data[sensor_id]
            packet_type = data['type']
            values = data['values']
            fields = data['fields']
            header = f"[{packet_type} ID {sensor_id}]"
            output = f"{header:<{max_header_len}} "
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
                    output += f"{field_name}: {value:.3f} "
            logger.info(output.strip())

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

def send_motor_control_command(ser, motor_id, throttle):
    """
    Send a motor control command to the Arduino.
    :param ser: Open serial connection
    :param motor_id: Target motor ID (0 for CMD_TARGET_MOTOR_ID)
    :param throttle: Motor throttle (0-100%)
    """
    try:
        if not (0 <= throttle <= 100):
            logger.error("Throttle must be between 0 and 100")
            return
        payload = struct.pack('>B', throttle)
        payload_size = len(payload)
        if payload_size > MAX_COMMAND_PAYLOAD_SIZE:
            logger.error(f"Payload size {payload_size} exceeds max {MAX_COMMAND_PAYLOAD_SIZE}")
            return
        packet = bytearray()
        packet.append(COMMAND_START_BYTE)
        packet.append(CMD_TYPE_SET_MOTOR)
        packet.append(motor_id)
        packet.append(payload_size)
        packet.extend(payload)
        packet.append(COMMAND_END_BYTE)
        ser.write(packet)
        logger.info(f"Sent motor control command: ID={motor_id}, Throttle={throttle}%")
    except serial.SerialException as e:
        logger.error(f"Serial error: {e}")
    except Exception as e:
        logger.error(f"Error: {e}")

def send_relay_control_command(ser, relay_id, state):
    """
    Send a relay control command to the Arduino.
    :param ser: Open serial connection
    :param relay_id: Target relay ID (0, 1, 2, 3)
    :param state: Relay state (0 for OFF, 1 for ON)
    """
    try:
        if state not in (0, 1):
            logger.error("State must be 0 (OFF) or 1 (ON)")
            return
        payload = struct.pack('>B', state)
        payload_size = len(payload)
        if payload_size > MAX_COMMAND_PAYLOAD_SIZE:
            logger.error(f"Payload size {payload_size} exceeds max {MAX_COMMAND_PAYLOAD_SIZE}")
            return
        packet = bytearray()
        packet.append(COMMAND_START_BYTE)
        packet.append(CMD_TYPE_SET_RELAY)
        packet.append(relay_id)
        packet.append(payload_size)
        packet.extend(payload)
        packet.append(COMMAND_END_BYTE)
        ser.write(packet)
        logger.info(f"Sent relay control command: ID={relay_id}, State={'ON' if state else 'OFF'}")
    except serial.SerialException as e:
        logger.error(f"Serial error: {e}")
    except Exception as e:
        logger.error(f"Error: {e}")

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
            command = input().strip()
            command_queue.put(command)
        except EOFError:
            command_queue.put('q')  # Signal exit on EOF
            break
        except Exception as e:
            logger.error(f"Error in command input thread: {e}")
            break
    logger.info("Command input thread finished.")

def main():
    # Build ID_TO_PACKET_INFO lookup table
    for start_byte, info in PACKET_TYPES.items():
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
    parser.add_argument('-u', '--update-interval', type=float, default=1.0, help='Interval to print sensor data (default: 1.0)')
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
        time.sleep(2)  # Allow Arduino to reset

        serial_thread = threading.Thread(target=serial_reader_thread, args=(ser, receiver), daemon=True)
        serial_thread.start()

        input_thread = threading.Thread(target=command_input_thread, args=(command_queue,), daemon=True)
        input_thread.start()

        logger.info(f"Connected to Arduino Mega on {SERIAL_PORT}")
        logger.info("Monitoring sensor data. Enter commands (or 'q' to quit):")
        logger.info("  Motor: 'm <throttle>' (e.g., 'm 50' for 50% throttle)")
        logger.info("  Relay: 'r <id> <state>' (e.g., 'r 0 1' for relay 0 ON)")
        last_print_time = time.time()

        while True:
            current_time = time.time()
            if current_time - last_print_time >= UPDATE_INTERVAL:
                receiver.print_all_latest_data()
                last_print_time = current_time

            try:
                # Check for commands in the queue (non-blocking)
                command = command_queue.get_nowait()
                if command.lower() == 'q':
                    break
                parts = command.split()
                if not parts:
                    logger.info("Please enter a command or 'q' to quit")
                    continue
                cmd_type = parts[0].lower()
                if cmd_type == 'm' and len(parts) == 2:
                    try:
                        throttle = int(parts[1])
                        send_motor_control_command(ser, CMD_TARGET_MOTOR_ID, throttle)
                    except ValueError:
                        logger.error("Invalid input. Use a number for throttle")
                elif cmd_type == 'r' and len(parts) == 3:
                    try:
                        relay_id = int(parts[1])
                        state = int(parts[2])
                        if relay_id not in (0, 1, 2, 3):
                            logger.error("Relay ID must be 0, 1, 2, or 3")
                        else:
                            send_relay_control_command(ser, relay_id, state)
                    except ValueError:
                        logger.error("Invalid input. Use numbers for relay_id and state")
                else:
                    logger.error("Invalid command. Use 'm <throttle>' or 'r <id> <state>'")
            except queue.Empty:
                pass  # No command available, continue loop
            time.sleep(0.05)  # Prevent CPU overload

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
            ser.close()
            logger.info("Serial port closed.")

if __name__ == "__main__":
    main()
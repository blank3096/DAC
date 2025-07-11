# live_sensor_monitor.py
import serial
import struct
import time
import sys
import threading
import argparse
import serial.tools.list_ports
import math # Import math for isnan check

# --- Protocol Constants (Must match Arduino SensorManager.h/cpp) ---
PRESSURE_PACKET_START_BYTE = 0xAA
PRESSURE_PACKET_END_BYTE = 0x55

LOADCELL_PACKET_START_BYTE = 0xBB
LOADCELL_PACKET_END_BYTE = 0x66

FLOW_PACKET_START_BYTE = 0xCC
FLOW_PACKET_END_BYTE = 0xDD

TEMP_PACKET_START_BYTE = 0xEE
TEMP_PACKET_END_BYTE = 0xFF


# --- Define expected payload structures and sizes ---
# Format strings for struct.unpack: '<' is little-endian (Arduino AVR is little-endian)
# 'f' is float (4 bytes)
PACKET_TYPES = {
    PRESSURE_PACKET_START_BYTE: {
        'name': 'Pressure',
        'end_byte': PRESSURE_PACKET_END_BYTE,
        'payload_size': 4, # *** CHANGED: 1 float * 4 bytes/float ***
        'format': '<f',   # *** CHANGED: little-endian, 1 float (pressure) ***
        'fields': ['pressure'], # *** CHANGED: Names for the unpacked values ***
        'ids': list(range(0, 6)) # Expected IDs for pressure sensors (0-5)
    },
    LOADCELL_PACKET_START_BYTE: {
        'name': 'LoadCell',
        'end_byte': LOADCELL_PACKET_END_BYTE,
        'payload_size': 4,  # 1 float * 4 bytes/float
        'format': '<f',    # little-endian, 1 float (weight_grams)
        'fields': ['weight_grams'],
        'ids': list(range(6, 8)) # Expected IDs for load cells (6, 7, assuming pressure is 0-5)
    },
    FLOW_PACKET_START_BYTE: {
        'name': 'Flow',
        'end_byte': FLOW_PACKET_END_BYTE,
        'payload_size': 4,  # 1 float * 4 bytes/float
        'format': '<f',
        'fields': ['flow_rate_lpm'],
        'ids': [8] # Assuming a single flow sensor with ID 8
    },
     TEMP_PACKET_START_BYTE: {
        'name': 'Temperature',
        'end_byte': TEMP_PACKET_END_BYTE,
        'payload_size': 8,  # 2 floats * 4 bytes/float
        'format': '<ff',
        'fields': ['temp_c', 'temp_f'],
        'ids': list(range(9, 11)) # Assuming two temp sensors with IDs 9 and 10
    },
    # Add other sensor types here
    # OTHER_PACKET_START_BYTE: { ... }
}

# Mapping from Packet ID back to Type Info for easier lookup (Built in main)
ID_TO_PACKET_INFO = {}


# --- Parsing State Machine ---
STATE_WAITING_FOR_START = 0
STATE_READING_HEADER = 1 # ID, Size
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
        # self._lock = threading.Lock() # Lock only needed if accessing data outside thread without care

        # --- Data Storage for Live Monitoring ---
        # Dictionary to store the latest data for each sensor ID
        # Format: {id: {'type': '...', 'timestamp': ..., 'values': (...), 'fields': [...]}}
        self.latest_sensor_data = {}


    def process_byte(self, byte_data):
        """Processes a single incoming byte."""
        # print(f"Processing byte: {byte_data:02X}") # Debug byte processing

        if self.state == STATE_WAITING_FOR_START:
            if byte_data in PACKET_TYPES:
                self.state = STATE_READING_HEADER
                self.current_start_byte = byte_data # Store the start byte
                self.current_packet_type_info = PACKET_TYPES[byte_data]
                self.payload_buffer = b'' # Reset buffer
                # print(f"--- Detected Start Byte {byte_data:02X} for {self.current_packet_type_info['name']} ---") # Debug
            # else: ignore unexpected byte

        elif self.state == STATE_READING_HEADER:
            # Header is 2 bytes: ID (1 byte) and Payload Size (1 byte)
            if len(self.payload_buffer) == 0: # Reading ID
                self.current_id = byte_data
                self.payload_buffer += bytes([byte_data]) # Add ID to buffer (optional)
            elif len(self.payload_buffer) == 1: # Reading Size
                 self.payload_size = byte_data
                 self.payload_buffer += bytes([byte_data]) # Add Size to buffer (optional)

                 # Check if declared size matches expected size for this type
                 # Also check if the ID is one we expect for this packet type
                 expected_size = self.current_packet_type_info['payload_size']
                 expected_ids = self.current_packet_type_info.get('ids')

                 if self.payload_size != expected_size:
                     print(f"Warning: Packet {self.current_packet_type_info['name']} (start {self.current_start_byte:02X}) - Declared size ({self.payload_size}) != expected ({expected_size}). Discarding.", file=sys.stderr)
                     self._reset_state()
                 elif expected_ids is not None and self.current_id not in expected_ids:
                     print(f"Warning: Packet {self.current_packet_type_info['name']} (start {self.current_start_byte:02X}) - Unexpected ID ({self.current_id}) for this type. Discarding.", file=sys.stderr)
                     self._reset_state()
                 elif self.payload_size == 0:
                     # If payload size is 0, skip reading payload and go straight to end byte
                     self.state = STATE_READING_END
                     self.payload_buffer = b'' # Clear buffer after header processing
                 else:
                     self.state = STATE_READING_PAYLOAD
                     self.payload_buffer = b'' # Clear buffer to store only actual payload data

        elif self.state == STATE_READING_PAYLOAD:
            self.payload_buffer += bytes([byte_data])
            if len(self.payload_buffer) == self.payload_size:
                self.state = STATE_READING_END

        elif self.state == STATE_READING_END:
            expected_end_byte = self.current_packet_type_info['end_byte']
            if byte_data == expected_end_byte:
                # --- Packet Complete and Validated! ---
                self._process_complete_packet()
            else:
                # Protocol error - unexpected byte where end byte should be
                print(f"Warning: Protocol Error for packet starting with {self.current_start_byte:02X}. Expected End Byte {expected_end_byte:02X}, but got {byte_data:02X}. Discarding packet.", file=sys.stderr)

            # Reset state for the next packet regardless of success or failure
            self._reset_state()


    def _process_complete_packet(self):
        """Handles a successfully received and validated packet."""
        packet_info = self.current_packet_type_info
        try:
            # Unpack the binary data according to the format string
            values = struct.unpack(packet_info['format'], self.payload_buffer)

            # --- Store the data ---
            self.latest_sensor_data[self.current_id] = {
                'type': packet_info['name'],
                'timestamp': time.time(), # Store reception timestamp
                'values': values,
                'fields': packet_info['fields']
            }

            # Optional: Print the data as it arrives (can be noisy for fast updates)
            # print(f"[Received - {packet_info['name']} ID {self.current_id}]", end=" ")
            # for i, field_name in enumerate(packet_info['fields']):
            #      print(f"{field_name}: {values[i]:.3f}", end=" ")
            # print()

        except struct.error as e:
            print(f"Error unpacking {packet_info['name']} packet ID {self.current_id}: {e}", file=sys.stderr)
        except Exception as e:
            print(f"Error processing packet {packet_info['name']} ID {self.current_id}: {e}", file=sys.stderr)


    def _reset_state(self):
         self.state = STATE_WAITING_FOR_START
         self.payload_buffer = b''
         self.current_packet_type_info = None
         self.current_start_byte = None
         self.current_id = None
         self.payload_size = 0

    def get_latest_data(self, sensor_id):
        """Retrieve the latest data for a given sensor ID."""
        return self.latest_sensor_data.get(sensor_id) # Use .get to avoid KeyError

    def print_all_latest_data(self):
        """Prints the latest stored data for all known sensors."""
        # print("\n--- Latest Sensor Data ---")
        if not self.latest_sensor_data:
            # print("No data received yet.") # Avoid printing this repeatedly if no data
            return

        # Sort by ID for consistent output order
        sorted_ids = sorted(self.latest_sensor_data.keys())

        # Find the maximum length of sensor type name + ID for alignment
        max_header_len = 0
        for sensor_id in sorted_ids:
            data = self.latest_sensor_data[sensor_id]
            header = f"[{data['type']} ID {sensor_id}]"
            max_header_len = max(max_header_len, len(header))


        print("-" * (max_header_len + 50)) # Print a separator line
        for sensor_id in sorted_ids:
            data = self.latest_sensor_data[sensor_id]
            packet_type = data['type']
            # timestamp = data['timestamp'] # Not printed in summary
            values = data['values']
            fields = data['fields']

            header = f"[{packet_type} ID {sensor_id}]"
            print(f"{header:<{max_header_len}}", end=" ") # Print header left-aligned

            for i, field_name in enumerate(fields):
                 # Add specific formatting or units based on field_name or sensor_id
                 value = values[i] # Get the specific value
                 if packet_type == 'Pressure' and field_name == 'pressure': # Formatting for Pressure (now only pressure)
                     print(f"{field_name}: {value:.2f} bar", end=" ")
                 elif packet_type == 'LoadCell' and field_name == 'weight_grams':
                      print(f"{field_name}: {value:.3f} g", end=" ")
                 elif packet_type == 'Flow' and field_name == 'flow_rate_lpm': # Formatting for Flow
                      print(f"{field_name}: {value:.3f} LPM", end=" ")
                 elif packet_type == 'Temperature': # Formatting for Temp
                     if field_name == 'temp_c':
                          # Check for NaN (open thermocouple)
                          if math.isnan(value): # Use math.isnan for clarity
                              print(f"{field_name}: ERR (Open TC)", end=" ")
                          else:
                              print(f"{field_name}: {value:.1f} C", end=" ")
                     elif field_name == 'temp_f':
                          if math.isnan(value):
                               print(f"{field_name}: ERR (Open TC)", end=" ")
                          else:
                              print(f"{field_name}: {value:.1f} F", end=" ")
                     else: # Default float formatting for temp fields if names change
                           print(f"{field_name}: {value:.1f}", end=" ")
                 else: # Default formatting for other types or unknown fields
                      print(f"{field_name}: {value:.3f}", end=" ")

            print() # Newline for next sensor

        # print("--------------------------") # Separator handled by the first line


# --- Function to automatically detect Arduino port ---
def auto_detect_arduino_port():
    """
    Lists serial ports and attempts to find one that looks like an Arduino.
    Returns the port name (string) or None if not found or ambiguous.
    """
    arduino_ports = []
    # List of common keywords/patterns found in Arduino port descriptions or HWIDs
    search_terms = ['Arduino', 'USB-SERIAL', 'VID:PID=2341', 'VID:PID=1A86', 'VID:PID=0403', 'usbmodem']

    print("Searching for Arduino port...")
    ports = serial.tools.list_ports.comports() # Get a list of all available ports

    for port in ports:
        # Check description and hardware ID (hwid) for search terms
        port_description = port.description.lower()
        port_hwid = port.hwid.lower()

        found_match = False
        for term in search_terms:
            if term.lower() in port_description or term.lower() in port_hwid:
                arduino_ports.append(port.device)
                found_match = True
                break # Found a match for this port, move to the next port

    if len(arduino_ports) == 1:
        print(f"Automatically selected port: {arduino_ports[0]}")
        return arduino_ports[0]
    elif len(arduino_ports) > 1:
        print(f"Found multiple potential Arduino ports: {', '.join(arduino_ports)}")
        print("Please specify the port using the command-line argument.")
        return None # Indicate ambiguity
    else:
        print("No Arduino port found.")
        return None # Indicate not found


# --- Threading function to continuously read from serial ---
def serial_reader_thread(ser, receiver):
    """Function to run in a separate thread, reading serial."""
    print("Serial reading thread started.")
    while ser.is_open: # Keep reading as long as port is open
        try:
            byte_data = ser.read(1) # Read one byte
            if byte_data: # Check if read was successful (not timed out)
                receiver.process_byte(byte_data[0]) # Process the byte

        except serial.SerialException as e:
            print(f"Serial reading thread error: {e}", file=sys.stderr)
            break # Exit the thread on error
        except Exception as e:
             print(f"Unexpected error in serial reading thread: {e}", file=sys.stderr)
             break # Exit thread

    print("Serial reading thread finished.")


# --- Main part to run the live monitor ---
if __name__ == "__main__":
    # Build the ID_TO_PACKET_INFO lookup table
    for start_byte, info in PACKET_TYPES.items():
        for sensor_id in info.get('ids', []):
            if sensor_id in ID_TO_PACKET_INFO:
                existing_start_byte = ID_TO_PACKET_INFO[sensor_id]
                existing_type_name = PACKET_TYPES[existing_start_byte]['name']
                print(f"Warning: Duplicate sensor ID {sensor_id} found! Defined for {existing_type_name} (start {existing_start_byte:02X}) and {info['name']} (start {start_byte:02X}). This will cause ambiguity!", file=sys.stderr)
            ID_TO_PACKET_INFO[sensor_id] = start_byte


    # Create an argument parser
    parser = argparse.ArgumentParser(description='Live monitor for sensor data from an Arduino via serial.')
    parser.add_argument(
        'port',
        nargs='?', # '?' means 0 or 1 argument is accepted
        default=None, # Default value is None if not provided
        help='Serial port name (e.g., COM3). If not specified, attempts auto-detection.'
    )
    parser.add_argument(
        '-b', '--baudrate',
        type=int,
        default=115200,
        help='Serial baud rate (default: 115200)'
    )
    parser.add_argument(
        '-t', '--timeout',
        type=float,
        default=0.1,
        help='Serial read timeout in seconds (default: 0.1)'
    )
    parser.add_argument(
        '-u', '--update-interval',
        type=float,
        default=1.0,
        help='Interval to print latest data summary (in seconds, default: 1.0)'
    )

    args = parser.parse_args()

    # --- Determine the serial port to use ---
    SERIAL_PORT_TO_USE = args.port

    if SERIAL_PORT_TO_USE is None:
        # User didn't provide a port, try auto-detection
        detected_port = auto_detect_arduino_port()
        if detected_port:
            SERIAL_PORT_TO_USE = detected_port
        else:
            # Auto-detection failed or was ambiguous
            print("Error: No suitable serial port found or specified. Exiting.", file=sys.stderr)
            sys.exit(1)


    BAUD_RATE = args.baudrate
    READ_TIMEOUT = args.timeout
    UPDATE_INTERVAL = args.update_interval


    receiver = SerialPacketReceiver()
    serial_thread = None
    ser = None # Define ser outside try block

    try:
        print(f"Attempting to open serial port {SERIAL_PORT_TO_USE} at {BAUD_RATE} baud...")
        ser = serial.Serial(SERIAL_PORT_TO_USE, BAUD_RATE, timeout=READ_TIMEOUT)
        print("Serial port opened successfully.")
        time.sleep(2) # Give Arduino time to reset and reach setup()


        # Start the serial reading in a separate thread
        serial_thread = threading.Thread(target=serial_reader_thread, args=(ser, receiver), daemon=True)
        serial_thread.start()

        print("Press Ctrl+C to exit.")
        last_print_time = time.time() # Timer for printing summary

        # Main thread loop - prints summary periodically
        while True:
            current_time = time.time()
            if current_time - last_print_time >= UPDATE_INTERVAL:
                receiver.print_all_latest_data()
                last_print_time = current_time

            time.sleep(0.05) # Sleep for 50ms

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT_TO_USE}: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nExiting due to user interrupt.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed.")

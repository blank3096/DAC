# Arduino Sensor & Motor Control CLI

This project provides a Command Line Interface (CLI) in Python for monitoring various sensors connected to an Arduino Mega and controlling a DC motor and relays. It utilizes a custom binary serial protocol for efficient and robust communication between the Python host and the Arduino embedded system.

## 🚀 Project Overview

This system is designed for real-time interaction with an Arduino Mega, allowing users to:

- **Monitor Sensor Data:** Receive and display live readings from pressure, load cell, flow, and temperature sensors, as well as motor RPM.
    
- **Control Actuators:** Send commands to control a DC motor's throttle, direction, and enable state, and toggle relays (ON/OFF).
    
- **Performance Monitoring:** Track the execution time of sensor reading and processing cycles on the Arduino.
    

The communication is built on a custom binary protocol, ensuring compact data transmission and efficient parsing on both ends.

## ✨ Features

- **Multi-Sensor Monitoring:** Supports pressure (6), load cell (3), flow (1), temperature (4), and motor RPM sensors.
    
- **Real-time Data Display:** Python CLI provides a live, updating dashboard of sensor values.
    
- **Actuator Control:** Send commands for motor throttle, direction, enable, and individual relay states.
    
- **Binary Protocol:** Efficient, low-overhead communication between Python and Arduino.
    
- **Robust Packet Parsing:** State-machine based packet receiver on both Python and Arduino for reliable data integrity.
    
- **Threaded Operations:** Non-blocking serial reading and command input in Python for a responsive CLI.
    
- **Dynamic Logging:** Configurable logging to console and file for debugging and record-keeping.
    
- **Auto-Port Detection:** Python script can automatically detect the Arduino's serial port.
    
- **Performance Timing:** Arduino measures and reports the execution time of sensor reading blocks and full sensor categories.
    
- **Unified Sensor Display:** All sensors are listed in the CLI, showing "No data" if no readings have been received.
    
- **Safe Motor Stop:** Motor is automatically stopped and disabled when the Python CLI exits.
    

## ⚙️ Hardware Requirements

- **Arduino Mega 2560:** The microcontroller board.
    
- **Pressure Sensors (6):** Analog current output (e.g., 4-20mA, requiring shunt resistors).
    
- **HX711 Load Cell Amplifiers (3):** For load cell integration.
    
- **Load Cells (3):** Compatible with HX711.
    
- **Flow Sensor (1):** Pulse output type.
    
- **MAX6675 Thermocouple Amplifiers (4):** For K-type thermocouples.
    
- **K-Type Thermocouples (4):** Compatible with MAX6675.
    
- **Relay Module (4-channel):** For controlling external devices.
    
- **DC Motor:** With a motor driver (e.g., L298N, VNH2SP30) that accepts PWM, Enable, and Direction signals.
    
- **Motor Speed Sensor:** Hall effect sensor or encoder providing pulse output.
    
- **USB Cable:** To connect Arduino to the computer.
    
- **Wiring:** Jumper wires, breadboard as needed.
    

## 💻 Software Requirements

### Arduino Side:

- **Arduino IDE:** Version 1.8.x or newer.
    
- **Libraries:**
    
    - `HX711.h`: For load cells. (Install via Arduino IDE Library Manager: Search "HX711 by bogde")
        
    - `max6675.h`: For MAX6675 thermocouples. (Install via Arduino IDE Library Manager: Search "MAX6675 library by Adafruit")
        

### Python Side:

- **Python 3.x:** (Tested with Python 3.8+)
    
- **`pyserial` library:**
    
    ```
    pip install pyserial
    ```
    

## 🚀 Getting Started

### 1. Arduino Setup

1. **Open Arduino IDE:**
    
    - Open `your_main_sketch.ino` in the Arduino IDE.
        
    - Ensure `SensorManager.h` and `SensorManager.cpp` are in the same directory as `your_main_sketch.ino` (or in a subfolder named `SensorManager`).
        
2. **Install Libraries:**
    
    - Go to `Sketch > Include Library > Manage Libraries...`
        
    - Search for and install "HX711 by bogde".
        
    - Search for and install "MAX6675 library by Adafruit".
        
3. **Configure Board:**
    
    - Go to `Tools > Board` and select "Arduino Mega 2560".
        
    - Go to `Tools > Port` and select the serial port connected to your Arduino Mega.
        
4. **Upload Code:**
    
    - Click the "Upload" button (right arrow icon) to compile and upload the code to your Arduino Mega.
        
    - The Arduino will reset and begin sending sensor data and listening for commands.
        

### 2. Python CLI Setup

1. **Save the Python Script:** Save the provided Python code as `arduino_cli.py` (or any other `.py` filename) on your computer.
    
2. **Open Terminal/Command Prompt:** Navigate to the directory where you saved `arduino_cli.py`.
    
3. **Run the Script:**
    
    ```
    python arduino_cli.py
    ```
    
    - The script will attempt to auto-detect your Arduino's serial port.
        
    - If multiple Arduinos are found or auto-detection fails, you may need to specify the port manually:
        
        ```
        python arduino_cli.py --port COM3  # On Windows
        python arduino_cli.py --port /dev/ttyACM0 # On Linux
        python arduino_cli.py --port /dev/cu.usbmodemXXXX # On macOS
        ```
        
    - You can also change the baud rate (default: 115400) or initial update interval:
        
        ```
        python arduino_cli.py --port COM3 --baudrate 9600 --update-interval 0.5
        ```
        

## 🎮 Usage

Once the Python CLI is running, you will see a continuous, updating display of sensor data. The `CLI>` prompt indicates it's ready for your commands.

### Available Commands:

- `m <throttle>`: Sets the motor throttle percentage.
    
    - `<throttle>`: An integer from `0` to `100`.
        
    - Example: `m 50` (sets motor to 50% throttle)
        
    - Example: `m 0` (stops the motor, explicitly disables PWM and sets pin low)
        
- `r <id> <state>`: Sets the state of a specific relay.
    
    - `<id>`: The relay ID (integer `0`, `1`, `2`, or `3`).
        
    - `<state>`: `0` for OFF (CLOSE), `1` for ON (OPEN).
        
    - Example: `r 0 1` (turns Relay 0 ON)
        
    - Example: `r 2 0` (turns Relay 2 OFF)
        
- `u <interval>`: Sets the sensor data update interval in seconds.
    
    - `<interval>`: A positive floating-point number.
        
    - Example: `u 0.2` (updates every 0.2 seconds for faster refresh)
        
    - Example: `u 5.0` (updates every 5.0 seconds for slower refresh)
        
- `h` or `help`: Displays the help message with all available commands.
    
- `q`: Quits the application. The motor will be stopped and disabled automatically upon exit.
    

### Example Output:

```
--- SENSOR DATA ---
[Pressure ID 0]      pressure: 1.23 bar
[Pressure ID 1]      pressure: No data
[Pressure ID 2]      pressure: 5.67 bar
[Pressure ID 3]      pressure: 8.90 bar
[Pressure ID 4]      pressure: No data
[Pressure ID 5]      pressure: 12.34 bar
[LoadCell ID 6]      weight_grams: 100.567 g
[LoadCell ID 7]      weight_grams: No data
[LoadCell ID 8]      weight_grams: 50.123 g
[Flow ID 9]          flow_rate_lpm: 0.750 LPM
[Temperature ID 10]  temp_c: 25.1 C temp_f: 77.2 F
[Temperature ID 11]  temp_c: ERR (Open TC) temp_f: ERR (Open TC)
[Temperature ID 12]  temp_c: 30.5 C temp_f: 86.9 F
[Temperature ID 13]  temp_c: 28.0 C temp_f: 82.4 F
[MotorRPM ID 14]     rpm: 1200.5 RPM

--- RELAY STATES ---
[Relay ID 0]         state: OPEN
[Relay ID 1]         state: CLOSE
[Relay ID 2]         state: CLOSE
[Relay ID 3]         state: OPEN

--- TIMING DATA (microseconds) ---
[Timing Category Pressure] Start: 123,456 us, End: 123,678 us, Duration: 222 us
[Timing Individual Pressure 0] Start: 123,456 us, End: 123,490 us, Duration: 34 us
[Timing Individual LoadCell 6] Start: 123,500 us, End: 123,550 us, Duration: 50 us
CLI>
```

## 🧑‍💻 Codebase Deep Dive

### Python CLI (`arduino_cli.py`)

The Python script manages serial communication, data parsing, and user interaction.

#### 1. Logging & Error Handling

- **`logging.basicConfig`**: Configures a robust logging system that writes messages to both the console and a `Work.log` file. This is crucial for debugging and keeping a record of operations.
    
- **`StdErrToLogger` Class**: Redirects standard error output (e.g., from `pyserial` or other libraries) to the logger, ensuring all error messages are captured in the log file with timestamps.
    

#### 2. Constants & Packet Definitions

- **`PACKET_TYPES` Dictionary**: This is the central definition for all expected incoming binary packets from the Arduino.
    
    - Each key is a `START_BYTE` (e.g., `0xAA` for pressure).
        
    - Each value is a dictionary containing:
        
        - `'name'`: Human-readable name (e.g., 'Pressure').
            
        - `'end_byte'`: The corresponding end byte for the packet.
            
        - `'payload_size'`: The expected number of bytes in the payload.
            
        - `'format'`: A `struct` module format string (e.g., `'<f'` for little-endian float, `'<BIII'` for byte and three unsigned integers). This defines how the binary payload should be unpacked.
            
        - `'fields'`: A list of string names for the values in the payload, used for structured output.
            
        - `'ids'`: A list of expected sensor IDs for this packet type.
            
- **`ID_TO_PACKET_INFO` Dictionary**: A lookup table created at startup to quickly find the `START_BYTE` (and thus the `PACKET_TYPES` info) given a sensor `ID`. This is primarily used internally by the `SerialPacketReceiver`.
    

#### 3. `SerialPacketReceiver` Class

This class implements a state machine to parse the incoming byte stream from the Arduino, ensuring that complete and valid packets are processed.

- **`__init__`**: Initializes the state machine variables (`self.state`, `self.payload_buffer`, etc.) and dictionaries to store the `latest_sensor_data` and `latest_timing_data`.
    
- **`process_byte(byte_data)`**: The core of the state machine. It processes one byte at a time:
    
    - `STATE_WAITING_FOR_START`: Looks for a `START_BYTE`.
        
    - `STATE_READING_HEADER`: Reads the `ID` and `SIZE` bytes following the start byte. It performs validation checks on the received `payload_size` and `ID` against the `PACKET_TYPES` definitions.
        
    - `STATE_READING_PAYLOAD`: Accumulates the payload bytes into `self.payload_buffer` until `payload_size` bytes are received.
        
    - `STATE_READING_END`: Checks for the `END_BYTE`. If valid, it calls `_process_complete_packet()`. If not, it logs an error and resets the state.
        
- **`_process_complete_packet()`**: Called when a full, valid packet is received.
    
    - It uses `struct.unpack()` with the `packet_info['format']` to convert the binary `payload_buffer` into Python values.
        
    - It then stores the unpacked data into `self.latest_sensor_data` or `self.latest_timing_data` based on the packet type.
        
    - Includes robust `try-except` blocks to catch `struct.error` (unpacking issues) and other general exceptions, logging the raw payload for debugging.
        
- **`_reset_state()`**: Resets all state machine variables to `STATE_WAITING_FOR_START` after a packet is processed (or discarded due to error).
    
- **`get_all_sensor_ids_and_types()`**: A helper method that dynamically generates a sorted list of all expected sensor IDs and their corresponding types from the `PACKET_TYPES` dictionary. This is crucial for the unified display.
    
- **`max_header_len_for_display()`**: Calculates the maximum length of any sensor, relay, or timing data header string. This ensures that all output lines can be consistently padded for clean, columnar display.
    
- **`print_all_latest_data()`**:
    
    - Clears the console screen (`os.system('cls'/'clear')`) for a dashboard-like effect.
        
    - Iterates through _all_ known sensor IDs (using `get_all_sensor_ids_and_types()`).
        
    - For each sensor, it checks if data is present in `self.latest_sensor_data`.
        
    - If data is present, it formats and prints the values with appropriate units.
        
    - If no data is present, it prints "No data" for that sensor, ensuring a consistent layout.
        
    - Also prints the `relay_states` and `latest_timing_data` in dedicated sections.
        

#### 4. Command Sending Functions

- **`send_motor_control_command(ser, motor_id, throttle, enable, forward)`**:
    
    - Validates input parameters (`throttle`, `enable`, `forward`).
        
    - Uses `struct.pack('>BBB', ...)` to create a compact binary payload for motor control (big-endian, 3 bytes).
        
    - Constructs the full command packet with `COMMAND_START_BYTE`, `CMD_TYPE_SET_MOTOR`, `motor_id`, `payload_size`, the `payload`, and `COMMAND_END_BYTE`.
        
    - Sends the packet over the serial port (`ser.write(packet)`).
        
- **`send_relay_control_command(ser, relay_id, state, receiver)`**:
    
    - Similar to motor control, validates input and packs the state into a 1-byte payload.
        
    - Constructs and sends the relay command packet.
        
    - **Crucially, it immediately updates `receiver.relay_states[relay_id] = state`** to reflect the command locally, providing instant feedback in the CLI.
        

#### 5. Threading

- **`serial_reader_thread(ser, receiver)`**: Runs in a separate daemon thread. Continuously reads single bytes from the serial port and feeds them to `receiver.process_byte()`. This prevents serial reading from blocking the main thread or user input.
    
- **`command_input_thread(command_queue)`**: Runs in another separate daemon thread. It continuously waits for user input at the `CLI>` prompt and places the entered commands into a `queue.Queue`. This ensures user input is non-blocking.
    

#### 6. `main()` Function

- **`ID_TO_PACKET_INFO` Build**: Initializes the lookup table for sensor IDs.
    
- **`argparse`**: Handles command-line arguments (`--port`, `--baudrate`, `--timeout`, `--update-interval`).
    
- **Auto-detection**: Calls `auto_detect_arduino_port()` to simplify initial setup.
    
- **Serial Connection**: Establishes the `pyserial` connection.
    
- **Thread Spawning**: Starts the `serial_reader_thread` and `command_input_thread`.
    
- **Main Loop**:
    
    - Periodically calls `receiver.print_all_latest_data()` based on `UPDATE_INTERVAL`.
        
    - Non-blockingly checks `command_queue` for user commands (`command_queue.get_nowait()`).
        
    - Parses and executes user commands (`m`, `r`, `u`, `h`, `q`).
        
    - Includes `try-except` blocks for graceful shutdown (`KeyboardInterrupt`) and general error handling.
        
- **`finally` Block**: Ensures the serial port is always closed on exit, and importantly, sends a motor stop command (`send_motor_control_command(ser, CMD_TARGET_MOTOR_ID, 0, enable=0, forward=1)`) to ensure safety.
    

### Arduino Code (`SensorManager.h`, `SensorManager.cpp`, `your_main_sketch.ino`)

The Arduino code manages sensor readings, actuator control, and binary communication with the Python host.

#### 1. Constants and Global Variables

- **`SensorManager.h`**: Declares all constants (pin assignments, calibration factors, protocol bytes) and global variables (sensor objects, state machine variables, timing data arrays) using `extern`.
    
- **`SensorManager.cpp`**: Defines the actual values for all constants and initializes global variables.
    
    - **`PRESSURE_SHUNT_OHMS`**: This new array holds the individual shunt resistor values for each pressure sensor (`A0` to `A10`).
        
    - **`pressure_ma_factor`**: This array is calculated in `setupPressureSensors()` using the individual `PRESSURE_SHUNT_OHMS` values, making the `mA` calculation specific to each sensor.
        

#### 2. Data Structures (`struct`s)

- **`PressureSensorValues`**, **`LoadCellValues`**, **`FlowMeterValues`**, **`TemperatureSensorValues`**, **`MotorRPMValue`**: Simple `struct`s to hold the processed sensor data before being packed into binary packets.
    
- **`SensorTiming`**: A new `struct` to hold timing information (`sensor_id`, `start_micros`, `end_micros`, `duration_micros`). This is used for sending performance metrics.
    

#### 3. Sensor Calculation Functions

Functions like `calculatePressureSensorValues()`, `calculateLoadCellValues()`, `calculateFlowMeterValues()`, `calculateTemperatureSensorValues()`, and `calculateMotorRPM()` encapsulate the logic for reading raw sensor data and converting it into meaningful engineering units.

#### 4. Command Handling

- **`handleCommand(commandType, targetId, payload, payloadSize)`**: The central function for processing incoming commands from the Python CLI. It uses a `switch` statement to dispatch to specific helper functions based on `commandType`. It performs basic validation on `payloadSize` and `targetId`.
    
- **`setRelayState(relayIndex, state)`**: Controls the digital output pins for relays.
    
- **`setMotorEnable(state)`**: Controls the motor driver's enable pin.
    
- **`setMotorDirection(direction)`**: Controls the motor driver's direction pin.
    
- **`setMotorThrottle(throttlePercent)`**:
    
    - Sets the PWM duty cycle for the motor.
        
    - **Special Case for 0% Throttle:** This function now includes specific low-level register manipulation (`TCCR4A &= ~(_BV(COM4A1) | _BV(COM4A0));` and `PORTH &= ~_BV(3);`) to explicitly **disconnect the PWM timer from Pin 6 and set Pin 6 to a guaranteed LOW state** when `throttlePercent` is 0. This ensures the motor is truly off and not just at a minimal PWM duty cycle.
        
    - For non-zero throttle, it re-enables the PWM output mode on Pin 6 and sets the `OCR4A` register for the desired duty cycle.
        

#### 5. Timing Functions

- **`startSensorTimer(sensorId, startTimeVar)`**: Records the current `micros()` timestamp at the beginning of a sensor operation.
    
- **`endSensorTimer(sensorId, startTime, description)`**: Calculates the duration of a sensor operation, stores it in the appropriate `SensorTiming` array (e.g., `pressureTimingData`), prints it to serial, and then calls `sendTimingPacket()`.
    
- **`sendTimingPacket(timing_id, data_ptr)`**: A wrapper around `sendBinaryPacket` specifically for sending `SensorTiming` structs.
    

#### 6. `sendBinaryPacket(start_byte, id, data_ptr, data_size, end_byte)` - **Crucial Explanation**

This is the core function for sending structured binary data from the Arduino to the Python CLI.

**Packet Structure:** The function constructs a packet with the following format: `[START_BYTE (1 byte)] [ID (1 byte)] [PAYLOAD_SIZE (1 byte)] [PAYLOAD_BYTES (data_size bytes)] [END_BYTE (1 byte)]`

**Parameter Breakdown:**

- **`start_byte`**: A single byte that marks the beginning of a new packet (e.g., `0xAA` for Pressure, `0xDE` for Timing).
    
- **`id`**: A single byte identifying the specific sensor or type of data within the packet (e.g., `PRESSURE_ID_START + 0` for the first pressure sensor, `TIMING_SENSOR_OPERATION_ID` for individual timing).
    
- **`data_ptr`**: A `const void*` pointer to the memory location where the actual data (the "payload") is stored. This is typically a pointer to a `struct` (e.g., `&pressureData`, `&currentSensorTiming`).
    
- **`data_size`**: The `size_t` (unsigned integer) indicating the number of bytes that the `data_ptr` points to. This is usually obtained using `sizeof(your_struct_variable)`.
    
- **`end_byte`**: A single byte that marks the end of the packet.
    

**Why `data_size` Appears "Twice" in `Serial.write`:**

You correctly observed these two lines:

```
  Serial.write((byte)data_size);              // Line A
  Serial.write((const byte*)data_ptr, (size_t)data_size); // Line B
```

They serve **distinct and critical purposes**:

- **Line A (`Serial.write((byte)data_size);`)**:
    
    - This sends the **value** of `data_size` as a **single byte** into the serial stream.
        
    - **Purpose:** This byte is part of the **packet header**. It tells the _receiving Python script_ exactly how many bytes to expect in the upcoming payload. This is vital for the Python script's state machine to know when it has finished reading the payload and should look for the `END_BYTE`.
        
- **Line B (`Serial.write((const byte*)data_ptr, (size_t)data_size);`)**:
    
    - This calls a different overload of the `Serial.write()` function. This overload takes a pointer to a buffer (`(const byte*)data_ptr`) and a **length argument** (`(size_t)data_size`).
        
    - **Purpose:** This line sends the **actual payload bytes** from the memory location pointed to by `data_ptr`. The `(size_t)data_size` here is _not_ sent as data; it's an instruction to the `Serial.write` function, telling it _how many_ bytes to read from the `data_ptr` buffer and transmit.
        

**Binary Mapping (Packing and Unpacking):**

- **Arduino (Packing):** When you call `sendBinaryPacket`, you pass a pointer to a `struct` (e.g., `&pressureData`). The `sizeof(pressureData)` gives the `data_size`. The Arduino then takes the raw bytes directly from the memory occupied by this `struct` and sends them.
    
- **Python (Unpacking):** On the Python side, the `SerialPacketReceiver` collects these raw bytes into `self.payload_buffer`. It then uses `struct.unpack(packet_info['format'], self.payload_buffer)`. The `packet_info['format']` (e.g., `'<f'`, `'<BIII'`) explicitly tells Python how to interpret these raw bytes (e.g., as a little-endian float, or as a byte followed by three unsigned 32-bit integers). This is the "binary mapping" in action, converting raw bytes back into structured data.
    

#### 7. State Machines / Round-Robin Sensor Polling (`your_main_sketch.ino`)

The `loop()` function in `your_main_sketch.ino` implements a non-blocking, round-robin approach to reading sensors and handling serial communication:

- **Serial Receive State Machine**: The `while (Serial.available() > 0)` loop and the `switch (currentRxState)` block continuously process incoming bytes from the Python CLI, reconstructing commands byte-by-byte.
    
- **Sensor Polling**: Each sensor type (Pressure, Load Cell, Flow, Temp, Motor RPM) has its own `if (currentMillis - lastSensorProcessTime >= MIN_SENSOR_INTERVAL_MS)` block. This ensures that:
    
    - Sensors are read at defined intervals (e.g., every 10ms for pressure, 500ms for temperature).
        
    - The `loop()` function remains non-blocking, allowing other tasks (like serial command processing) to run concurrently.
        
    - Sensors within a category (like multiple pressure sensors) are processed in a round-robin fashion.
        
- **Timing Integration**: `startSensorTimer()` and `endSensorTimer()` calls are strategically placed around sensor reading and data sending operations to measure their execution times, which are then transmitted back to the Python CLI. Category timers track the total time to cycle through all sensors of a specific type.
    

#### 8. Interrupt Service Routines (ISRs)

- **`flow_increase_pulse()`**: Attached to an external interrupt pin (Pin 2 for flow sensor). Increments `flow_pulse` every time a pulse is detected from the flow sensor.
    
- **`motor_count_pulse()`**: Attached to an external interrupt pin (Pin 3 for motor speed sensor). Increments `motor_pulse_count` for each pulse from the motor encoder.
    
    - These ISRs are kept minimal to ensure quick execution and avoid blocking the main loop. The actual calculation of flow rate and RPM happens in the main loop's sensor polling blocks.
        

## ⚠️ Troubleshooting

- **"No suitable serial port found"**:
    
    - Ensure your Arduino is connected to your computer.
        
    - Check if the Arduino IDE can see the port (`Tools > Port`).
        
    - Manually specify the port using `--port COMx` or `--port /dev/ttyACM0`.
        
- **"Serial reading thread error" / Disconnections**:
    
    - The Arduino might have reset or been unplugged. Re-run the Python script.
        
    - Ensure the baud rates match on both Arduino (115200 in `Serial.begin()`) and Python (`--baudrate` argument).
        
- **"Unpacking ... packet ID ... struct.error"**:
    
    - This indicates a mismatch in the binary protocol. The Python script expected a certain payload size or format but received something different.
        
    - Check the `PACKET_TYPES` definitions in Python and the `sendBinaryPacket` calls/struct definitions in Arduino (`SensorManager.cpp`).
        
    - The logged `Raw payload: ...` hex string is crucial for debugging this.
        
- **"ERR (Open TC)" for Temperature**:
    
    - The MAX6675 library returns `NAN` (Not A Number) if the thermocouple is disconnected or faulty. Check your thermocouple wiring.
        
- **Motor not responding / behaving erratically**:
    
    - Verify `MOTOR_PWM_PIN`, `MOTOR_ENABLE_PIN`, `MOTOR_DIRECTION_PIN` are correctly wired to your motor driver.
        
    - Check `setMotorThrottle` logic and PWM setup (Timer4 configuration).
        
    - Ensure the motor driver is powered correctly.
        

## 💡 Future Enhancements

- **Python GUI**: Implement a graphical user interface (e.g., using PyQt, Tkinter, or Kivy) for a more intuitive control and visualization experience.
    
- **Data Logging to CSV**: Add an option in the Python CLI to log all incoming sensor data to a CSV file for later analysis.
    
- **Command Acknowledgment**: Implement acknowledgment packets from Arduino to Python for commands, so the CLI knows if a command was successfully received and processed.
    
- **Error Packets**: Define specific error packets from Arduino to Python to report sensor faults (e.g., HX711 not ready, MAX6675 communication errors) more explicitly.
    
- **Configuration Commands**: Add commands to change Arduino-side parameters (e.g., sensor polling intervals, calibration factors) from the CLI.
    
- **Advanced CLI Features**: Integrate libraries like `prompt_toolkit` for command history, autocompletion, and better line editing in the Python CLI.
    
- **PID Control (Arduino)**: Implement PID loops on the Arduino for motor speed control or other process control based on sensor feedback.
    
- **EEPROM Storage (Arduino)**: Store calibration factors or last-known states in Arduino's EEPROM so they persist across power cycles.
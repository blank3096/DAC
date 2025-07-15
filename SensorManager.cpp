#include "SensorManager.h" // Include your own header first
#include <Arduino.h>     // Include Arduino.h here too
#include <math.h>        // Required for isnan()
// Add includes for other libraries needed for implementation (e.g., Wire.h)


// =======================================================
// === Definitions of Constants and Global Variables =====
// =======================================================

// --- Common Constants ---
const long ANALOG_REFERENCE_mV = 5000;
// const float SHUNT_OHM = 217.9; // REMOVED: Now per-sensor
const float PERCENT_SLOPE = 6.25;
const float PERCENT_OFFSET = -25.0;


// --- Pressure Sensor Constants ---
const int PRESSURE_SENSOR_PINS[6] = {A0, A2, A4, A6, A8, A10}; // As per table
const float PRESSURE_MAX[6] = {16.0, 16.0, 25.0, 25.0, 40.0, 40.0}; // Using original pressure max values for now
const int NUM_PRESSURE_SENSORS = sizeof(PRESSURE_SENSOR_PINS) / sizeof(PRESSURE_SENSOR_PINS[0]);

const float MV_FACTOR = (float)ANALOG_REFERENCE_mV / 1024.0f;
// float MA_FACTOR = 1000.0f / SHUNT_OHM; // REMOVED: Now per-sensor, calculated in setup
float pressure_scale_factor[6];
// NEW: Individual shunt resistances for each pressure sensor
const float PRESSURE_SHUNT_OHMS[6] = {220.7f, 220.9f, 218.3f, 219.4f, 220.1f, 218.3f};
// NEW: Individual mA factors calculated from shunt resistances
float pressure_ma_factor[6];


// --- Load Cell Constants ---
// Pins for THREE Load Cells (As per table)
const byte LOADCELL_DOUT_PINS[3] = {28, 30, 32}; // Data Output lines (DT)
const byte LOADCELL_CLK_PINS[3] = {29, 31, 33};   // Clock lines (SCK)
const float LOADCELL_CALIBRATION_FACTORS[3] = {145.4f, 150.0f, 160.0f}; // Calibration factors for 3 sensors
const int NUM_LOADCELL_SENSORS = sizeof(LOADCELL_DOUT_PINS) / sizeof(LOADCELL_DOUT_PINS[0]); // Should be 3

// Array of HX711 objects - SIZE IS 3
HX711 scales[3];


// --- Flow Sensor Constants ---
const int FLOW_SENSOR_PIN_MEGA = 2; // Pin 2 on Mega is External Interrupt 0 (As per table)
const int FLOW_PPL = 4215;


// --- Temperature Sensor (MAX6675) Constants ---
// Pins for FOUR MAX6675 sensors (As per table, interpreting ranges)
const int THERMO_SHARED_CLK_PIN = 26; // D22 - Shared Clock
const int THERMO_SHARED_DO_PIN  = 50; // D50 - Shared Data Out (MISO)

const int THERMO_CS_PINS[4]  = { 22, 23, 24, 25 }; // D22-D25 - Unique Chip Selects
const int NUM_TEMP_SENSORS = sizeof(THERMO_CS_PINS) / sizeof(THERMO_CS_PINS[0]); // Should be 4

const float FAHRENHEIT_SLOPE = 9.0f / 5.0f;
const float FAHRENHEIT_OFFSET = 32.0f;

MAX6675 thermocouples[4] = { // Defined here (array of objects)
    MAX6675(THERMO_SHARED_CLK_PIN, THERMO_CS_PINS[0], THERMO_SHARED_DO_PIN),
    MAX6675(THERMO_SHARED_CLK_PIN, THERMO_CS_PINS[1], THERMO_SHARED_DO_PIN),
    MAX6675(THERMO_SHARED_CLK_PIN, THERMO_CS_PINS[2], THERMO_SHARED_DO_PIN),
    MAX6675(THERMO_SHARED_CLK_PIN, THERMO_CS_PINS[3], THERMO_SHARED_DO_PIN)
};


// --- Relay Constants ---
// Pins for the FOUR Relays (As per table)
const int RELAY_PINS[4] = {34, 36, 38, 40};
const int NUM_RELAYS = sizeof(RELAY_PINS) / sizeof(RELAY_PINS[0]);


// --- DC Motor Constants ---
// Pins for DC Motor (As per table)
const int MOTOR_PWM_PIN = 6;       // D6 (Timer4) <-- CHANGED TO D6
const int MOTOR_ENABLE_PIN = 37;    // D37
const int MOTOR_DIRECTION_PIN = 38; // D38
const int MOTOR_SPEED_SENSE_PIN = 3;  // D3 (INT1)

const int MOTOR_PULSES_PER_REVOLUTION = 12; // As per code snippet
const float PULSES_PER_SEC_TO_RPM_FACTOR = 60.0f / MOTOR_PULSES_PER_REVOLUTION;


// Add definitions for other sensor constants here
/*
const int OTHER_SENSOR_INPUTS[2] = { ... };
const int NUM_OTHER_SENSORS = sizeof(OTHER_SENSOR_INPUTS) / sizeof(OTHER_SENSOR_INPUTS[0]);
*/


// --- Binary Protocol Constants (Sensor Data Output) ---
const byte PRESSURE_PACKET_START_BYTE = 0xAA;
const byte PRESSURE_PACKET_END_BYTE = 0x55;
const byte LOADCELL_PACKET_START_BYTE = 0xBB;
const byte LOADCELL_PACKET_END_BYTE = 0x66;
const byte FLOW_PACKET_START_BYTE = 0xCC;
const byte FLOW_PACKET_END_BYTE = 0xDD;
const byte TEMP_PACKET_START_BYTE = 0xEE;
const byte TEMP_PACKET_END_BYTE = 0xFF;
const byte MOTOR_RPM_PACKET_START_BYTE = 0xF0;
const byte MOTOR_RPM_PACKET_END_BYTE = 0xF1;


// --- Binary Protocol Constants (Incoming Commands) ---
const byte COMMAND_START_BYTE = 0xFC; // Start byte for commands
const byte COMMAND_END_BYTE = 0xFD;   // End byte for commands

// Command Type definitions
const byte CMD_TYPE_SET_RELAY = 0x01; // Command to set a relay state
const byte CMD_TYPE_SET_MOTOR = 0x02; // Command to set motor state (enable, direction, throttle)

// Target ID definitions for commands
const byte CMD_TARGET_RELAY_START = 0; // Relays targeted by their 0-3 index
const byte CMD_TARGET_MOTOR_ID = 0;    // Motor targeted by ID 0


// Define ID ranges and number of IDs for each sensor type (Output Packets)
const byte PRESSURE_ID_START = 0;
const byte NUM_IDS_PRESSURE = NUM_PRESSURE_SENSORS; // 6

const byte LOADCELL_ID_START = PRESSURE_ID_START + NUM_IDS_PRESSURE; // 6
const byte NUM_IDS_LOADCELL = NUM_LOADCELL_SENSORS; // 3

const byte FLOW_SENSOR_ID = LOADCELL_ID_START + NUM_IDS_LOADCELL; // 9
const byte NUM_IDS_FLOW = 1;

const byte TEMP_ID_START = FLOW_SENSOR_ID + NUM_IDS_FLOW;     // 10
const byte NUM_IDS_TEMP = NUM_TEMP_SENSORS; // 4

const byte MOTOR_RPM_ID = TEMP_ID_START + NUM_IDS_TEMP;      // 14
const byte NUM_IDS_MOTOR_RPM = 1;


// Add constants for other sensor types here
/*
const byte OTHER_PACKET_START_BYTE = 0x01;
const byte OTHER_PACKET_END_BYTE = 0x02;
const byte OTHER_ID_START = MOTOR_RPM_ID + NUM_IDS_MOTOR_RPM; // Offset from previous group
const byte NUM_IDS_OTHER = NUM_OTHER_SENSORS;
*/


// --- Serial Receive State Machine Variables and Constants ---
// Define states for parsing incoming commands

// Global state variable, initialized in setup()
RxState currentRxState = RX_WAITING_FOR_START;

// Variables to store components of the current command being received
byte rxCommandType = 0;
byte rxTargetId = 0;
byte rxPayloadSize = 0; // The size indicated in the packet
byte rxPayloadBytesRead = 0; // How many payload bytes we've read so far

// Buffer to hold the incoming command payload
byte rxPayloadBuffer[MAX_COMMAND_PAYLOAD_SIZE];


// --- State Machine / Round-Robin Variables and Constants (Sensor Reading) ---
int currentPressureSensorIndex = 0;
unsigned long lastPressureSensorProcessTime = 0;
const unsigned long MIN_PRESSURE_INTERVAL_MS = 10;

int currentLoadCellIndex = 0; // Cycles 0, 1, 2
unsigned long lastLoadCellProcessTime = 0;
const unsigned long MIN_LOADCELL_CHECK_INTERVAL_MS = 150;

volatile long flow_pulse = 0;
long flow_pulseLast = 0;
unsigned long lastFlowProcessTime = 0;
const unsigned long FLOW_CALCULATION_INTERVAL_MS = 1000;
unsigned long elapsed_time = 0; // Corrected to unsigned long

int currentTempSensorIndex = 0; // Cycles 0, 1, 2, 3
unsigned long lastTempProcessTime = 0;
const unsigned long MIN_TEMP_INTERVAL_MS = 500;

volatile unsigned long motor_pulse_count = 0;
unsigned long motor_last_pulse_count = 0;
unsigned long lastMotorCalcTime = 0;
const unsigned long MOTOR_CALCULATION_INTERVAL_MS = 500;


// --- NEW: Global Arrays to Store Timing Data ---
SensorTiming pressureTimingData[MAX_TIMED_PRESSURE_SENSORS];
SensorTiming loadCellTimingData[MAX_TIMED_LOADCELL_SENSORS];
SensorTiming tempTimingData[MAX_TIMED_TEMP_SENSORS];
SensorTiming flowTimingData[MAX_TIMED_FLOW_SENSORS]; // New for flow sensor
SensorTiming motorTimingData[MAX_TIMED_MOTOR_SENSORS]; // New for motor RPM sensor


// --- NEW: Packet Constants for Timing Data ---
const byte TIMING_PACKET_START_BYTE = 0xDE; // Choose a unique byte
const byte TIMING_PACKET_END_BYTE = 0xAD;   // Choose a unique byte
const byte TIMING_SENSOR_OPERATION_ID = 0x01; // ID for individual sensor timings
const byte TIMING_CATEGORY_CYCLE_ID = 0x02;   // ID for full category cycle times

// --- NEW: Timing variables for category cycles ---
unsigned long pressureCategoryStartTime = 0;
unsigned long loadCellCategoryStartTime = 0;
unsigned long tempCategoryStartTime = 0;
unsigned long flowCategoryStartTime = 0; // New for flow sensor
unsigned long motorCategoryStartTime = 0; // New for motor RPM sensor


// Add state variables and constants for other sensor types here
/*
int currentOtherSensorIndex = 0;
unsigned long lastOtherSensorProcessTime = 0;
const unsigned long MIN_OTHER_INTERVAL_MS = 50;
*/


// --- Timing Helper Functions (UPDATED) ---
// This function starts a timer for a specific sensor operation.
// It takes the sensor's ID and a pointer to a variable where the start time will be stored.
void startSensorTimer(byte sensorId, unsigned long* startTimeVar) {
    if (startTimeVar != nullptr) {
        *startTimeVar = micros(); // Store the start time in the provided variable
    }
}

// This function ends a timer for a specific sensor operation, calculates duration,
// stores it in the respective array, and prints/sends the information.
void endSensorTimer(byte sensorId, unsigned long startTime, const char* description) {
    unsigned long endTime = micros();
    unsigned long duration = endTime - startTime;

    // Determine which array to store the timing in based on sensorId ranges
    if (sensorId >= PRESSURE_ID_START && sensorId < PRESSURE_ID_START + NUM_PRESSURE_SENSORS) {
        byte index = sensorId - PRESSURE_ID_START;
        if (index < MAX_TIMED_PRESSURE_SENSORS) {
            pressureTimingData[index] = {sensorId, startTime, endTime, duration};
        }
    } else if (sensorId >= LOADCELL_ID_START && sensorId < LOADCELL_ID_START + NUM_LOADCELL_SENSORS) {
        byte index = sensorId - LOADCELL_ID_START;
        if (index < MAX_TIMED_LOADCELL_SENSORS) {
            loadCellTimingData[index] = {sensorId, startTime, endTime, duration};
        }
    } 
    // else if (sensorId == FLOW_SENSOR_ID) { // Specific check for the single flow sensor ID
    //     // Flow sensor index will always be 0 as there's only one
    //     if (0 < MAX_TIMED_FLOW_SENSORS) { // Ensure bounds
    //         flowTimingData[0] = {sensorId, startTime, endTime, duration};
    //     }
    // } 
    else if (sensorId >= TEMP_ID_START && sensorId < TEMP_ID_START + NUM_IDS_TEMP) {
        byte index = sensorId - TEMP_ID_START;
        if (index < MAX_TIMED_TEMP_SENSORS) {
            tempTimingData[index] = {sensorId, startTime, endTime, duration};
        }
    } else if (sensorId == MOTOR_RPM_ID) { // Specific check for the single motor RPM sensor ID
        // Motor RPM sensor index will always be 0 as there's only one
        if (0 < MAX_TIMED_MOTOR_SENSORS) { // Ensure bounds
            motorTimingData[0] = {sensorId, startTime, endTime, duration};
        }
    }
    // Add more else if blocks for other sensor types if you add them

    Serial.print(F("Time for ")); Serial.print(description); Serial.print(F(" (ID ")); Serial.print(sensorId); Serial.print(F("): "));
    Serial.print(duration); Serial.println(F(" us"));

    // Send this individual sensor's timing data
    SensorTiming currentSensorTiming = {sensorId, startTime, endTime, duration};
    sendTimingPacket(TIMING_SENSOR_OPERATION_ID, &currentSensorTiming);
}

// --- NEW: Function to send timing data packet ---
void sendTimingPacket(byte timing_id, const SensorTiming* data_ptr) {
    sendBinaryPacket(TIMING_PACKET_START_BYTE, timing_id, (const void*)data_ptr, sizeof(SensorTiming), TIMING_PACKET_END_BYTE);
}


// =======================================================
// === Function Definitions (Code Bodies) ================
// =======================================================

// --- Calculation Function for Pressure Sensor ---
PressureSensorValues calculatePressureSensorValues(int raw_pressure_int, int index) {
  if (index < 0 || index >= NUM_PRESSURE_SENSORS) { return {-1.0f}; }
  float raw_pressure_f = (float)raw_pressure_int;
  float mV = raw_pressure_f * MV_FACTOR;
  float volts = mV / 1000.0f;
  // Use the specific mA factor for this sensor
  float mA = volts * pressure_ma_factor[index]; // CHANGED: Using array for MA_FACTOR
  float percent = mA * PERCENT_SLOPE + PERCENT_OFFSET;
  float pressure = percent * pressure_scale_factor[index];
  return {pressure};
}

// --- Calculation Function for Load Cell Sensor ---
LoadCellValues calculateLoadCellValues(float raw_weight_float) {
    return {raw_weight_float};
}

// --- Calculation Function for Flow Sensor ---
FlowMeterValues calculateFlowMeterValues(long delta_pulse, unsigned long elapsed_time) {
    // Prevent division by zero if elapsed_time happens to be 0
    if (elapsed_time == 0) {
        return {0.0f}; // Return 0 LPM if no time has elapsed
    }
    // The corrected formula, ensuring floating-point arithmetic throughout
    float lpm = ((float)delta_pulse * 60000.0f) / (FLOW_PPL * (float)elapsed_time);
    return {lpm};
}
// --- Calculation Function for Temperature Sensor (MAX6675) ---
TemperatureSensorValues calculateTemperatureSensorValues(int index) {
    if (index < 0 || index >= NUM_TEMP_SENSORS) { return {NAN, NAN}; } // Corrected to return NAN for invalid index
    MAX6675& currentThermocouple = thermocouples[index];
    double celsius = currentThermocouple.readCelsius();
    TemperatureSensorValues values;
    if (isnan(celsius)) { values.temp_c = NAN; values.temp_f = NAN; }
    else {
        double fahrenheit = celsius * FAHRENHEIT_SLOPE + FAHRENHEIT_OFFSET;
        values.temp_c = (float)celsius;
        values.temp_f = (float)fahrenheit;
    }
    return values;
}

// --- Calculation Function for Motor RPM ---
MotorRPMValue calculateMotorRPM(unsigned long currentPulseCount, unsigned long previousPulseCount, unsigned long interval_ms) {
    unsigned long delta_pulse = currentPulseCount - previousPulseCount;
    if (interval_ms == 0) { return {0.0f}; } // Avoid division by zero
    float frequency_hz = (float)delta_pulse / (float)interval_ms * 1000.0f;
    float rpm = frequency_hz * PULSES_PER_SEC_TO_RPM_FACTOR;
    return {rpm};
}

// Add calculation functions for other sensor types here
/*
OtherSensorValues calculateOtherSensorValues(...) { ... }
*/


// --- GENERIC Function to send any data block in binary format ---
void sendBinaryPacket(byte start_byte, byte id, const void* data_ptr, size_t data_size, byte end_byte) {
   if (data_ptr == nullptr || data_size == 0) return;

  Serial.write(start_byte);
  Serial.write(id);

  if (data_size > 255) {
    Serial.print(F("Warning: Packet ID ")); Serial.print(id); Serial.print(F(" data size (")); Serial.print(data_size); Serial.println(F(" bytes) exceeds 1-byte limit. Skipping packet."));
    return;
   }
  Serial.write((byte)data_size);

  Serial.write((const byte*)data_ptr, (size_t)data_size);
  Serial.write(end_byte);
}


// --- Command Handling Helper Functions ---

// Helper function to set a relay state
void setRelayState(byte relayIndex, byte state) {
    if (relayIndex >= NUM_RELAYS) {
        Serial.print(F("Error: Invalid relay index received: ")); Serial.println(relayIndex);
        // Optional: Send error packet back to GUI
        return;
    }
    if (state > 1) { // Assuming state is 0 (OFF) or 1 (ON)
        Serial.print(F("Error: Invalid relay state received: ")); Serial.println(state);
        // Optional: Send error packet back to GUI
        return;
    }
    digitalWrite(RELAY_PINS[relayIndex], (state == 1) ? LOW : HIGH);
    Serial.print(F("Set Relay ")); Serial.print(relayIndex); Serial.print(F(" to ")); Serial.println((state == 1) ? F("ON") : F("OFF"));
    // Optional: Send acknowledgment packet back to GUI
}

// Helper function to set motor enable state
void setMotorEnable(byte state) {
    if (state > 1) { // Assuming state is 0 (OFF) or 1 (ON)
        Serial.print(F("Error: Invalid motor enable state received: ")); Serial.println(state);
        return;
    }
    digitalWrite(MOTOR_ENABLE_PIN, (state == 1) ? HIGH : LOW);
    Serial.print(F("Set Motor Enable to: ")); Serial.println((state == 1) ? F("ON") : F("OFF"));
}

// Helper function to set motor direction
void setMotorDirection(byte direction) {
     if (direction > 1) { // Assuming direction is 0 (Reverse) or 1 (Forward)
        Serial.print(F("Error: Invalid motor direction received: ")); Serial.println(direction);
        return;
    }
    digitalWrite(MOTOR_DIRECTION_PIN, (direction == 1) ? HIGH : LOW); // Assuming HIGH is Forward based on setup
     Serial.print(F("Set Motor Direction to: ")); Serial.println((direction == 1) ? F("Forward") : F("Reverse"));
}

// Helper function to set motor throttle (PWM duty cycle)
void setMotorThrottle(byte throttlePercent) {
    // Clamp the received percentage to 0-100
    if (throttlePercent > 100) {
        Serial.print(F("Warning: Clamping received throttle "));
        Serial.print(throttlePercent); Serial.println(F(" to 100%."));
        throttlePercent = 100;
    }

    // Map the 0-100% duty cycle value to the timer's range (0-199 for Timer4 @ 10kHz)
    // Note: OCR4A is 16-bit, but map returns long. Cast to int or uint16_t.

    int dutyCycleValue = map(throttlePercent, 0, 100, 0, 199);

    // Set the duty cycle on Timer4's A channel (controls MOTOR_PWM_PIN = Pin 6)
    OCR4A = dutyCycleValue; // Assign to Timer4 Output Compare Register A

    Serial.print(F("Set Motor Throttle to: "));
     Serial.print(throttlePercent);
     Serial.print(F("% (Timer value: "));
      Serial.print(dutyCycleValue); Serial.println(F(")"));
}


// --- Main Command Handling Function ---
// Processes a complete, valid command packet
void handleCommand(byte commandType, byte targetId, const byte* payload, byte payloadSize) {
    Serial.print(F("Received Command: Type=")); Serial.print(commandType);
    Serial.print(F(", Target ID=")); Serial.print(targetId);
    Serial.print(F(", Payload Size=")); Serial.println(payloadSize);

    switch (commandType) {
        case CMD_TYPE_SET_RELAY:
            // Expected payload size is 1 byte (state)
            if (payloadSize == 1) {
                 // Target ID should be within the relay index range (0 to NUM_RELAYS-1)
                if (targetId >= CMD_TARGET_RELAY_START && targetId < CMD_TARGET_RELAY_START + NUM_RELAYS) {
                    setRelayState(targetId - CMD_TARGET_RELAY_START, payload[0]); // Adjust targetId to 0-3 index
                } else {
                    Serial.print(F("Error: CMD_TYPE_SET_RELAY received with invalid target ID: ")); Serial.println(targetId);
                }
            } else {
                Serial.print(F("Error: CMD_TYPE_SET_RELAY received with invalid payload size: ")); Serial.println(payloadSize);
            }
            break;

        case CMD_TYPE_SET_MOTOR:
            // Expected payload size is 3 bytes (enable, direction, throttle)
             if (payloadSize == 3) {
                 // Target ID should be the motor ID
                 if (targetId == CMD_TARGET_MOTOR_ID) {
                     byte enableState = payload[0];
                     byte directionState = payload[1];
                     byte throttlePercent = payload[2];

                     // Call motor control helpers
                     setMotorEnable(enableState);
                     setMotorDirection(directionState);
                     setMotorThrottle(throttlePercent);

                 } else {
                    Serial.print(F("Error: CMD_TYPE_SET_MOTOR received with invalid target ID: ")); Serial.println(targetId);
                 }
             } else {
                Serial.print(F("Error: CMD_TYPE_SET_MOTOR received with invalid payload size: ")); Serial.println(payloadSize);
             }
             break;

        // Add cases for other command types here
        /*
        case CMD_TYPE_OTHER_COMMAND:
           // ... process payload based on command type and target ID ...
           break;
        */

        default:
            // Received an unknown command type
            Serial.print(F("Error: Received unknown command type: ")); Serial.println(commandType);
            break;
    }
    // Optional: Send a generic acknowledgment packet back to GUI after processing a command
    // sendBinaryPacket(ACK_PACKET_START_BYTE, ACK_ID, nullptr, 0, ACK_PACKET_END_BYTE);
}


// --- Modular Setup Functions ---
void setupPressureSensors() {
  Serial.println(F("Setting up Pressure Sensors..."));
  for (int i = 0; i < NUM_PRESSURE_SENSORS; i++) {
    pressure_scale_factor[i] = PRESSURE_MAX[i] / 100.0f;
    // NEW: Calculate individual mA factor for each sensor
    pressure_ma_factor[i] = 1000.0f / PRESSURE_SHUNT_OHMS[i]; // CHANGED: Calculate per sensor
    // Analog pins are inputs by default, no pinMode needed unless using as digital
  }
  Serial.print(NUM_PRESSURE_SENSORS); Serial.println(F(" Pressure Sensors setup complete."));
}

void setupLoadCells() {
  Serial.println(F("Setting up Load Cells..."));
  for (int i = 0; i < NUM_LOADCELL_SENSORS; i++) { // Loop runs 3 times now
    Serial.print(F("Load Cell ")); Serial.print(i + 1);
    Serial.print(F(" on pins DOUT:")); Serial.print(LOADCELL_DOUT_PINS[i]);
    Serial.print(F(" CLK:")); Serial.println(LOADCELL_CLK_PINS[i]);

    scales[i].begin(LOADCELL_DOUT_PINS[i], LOADCELL_CLK_PINS[i]);
    delay(100); // Give HX711 a moment

    scales[i].set_scale(LOADCELL_CALIBRATION_FACTORS[i]);
    // Add user prompt and delay here if needed for initial load taring
    // Serial.print(F("Load Cell ")); Serial.print(i + 1); Serial.println(F(": Please place initial load on the scale NOW. Taring in 5 seconds...")); delay(5000);

    scales[i].tare();

    Serial.print(F("Load Cell ")); Serial.print(i + 1); Serial.println(F(" tared."));
    delay(50); // Small delay between taring
  }
  Serial.print(NUM_LOADCELL_SENSORS); Serial.println(F(" Load Cells setup complete."));
}

void setupFlowSensors() {
  Serial.println(F("Setting up Flow Sensor..."));
  pinMode(FLOW_SENSOR_PIN_MEGA, INPUT); // Or INPUT_PULLUP if needed
  // Pin 2 is External Interrupt 0 on Mega
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN_MEGA), flow_increase_pulse, RISING);

  flow_pulse = 0;
  flow_pulseLast = 0;
  lastFlowProcessTime = millis();

  Serial.println(F("Flow Sensor setup complete."));
}

void setupTemperatureSensors() {
  Serial.println(F("Setting up Temperature Sensors (MAX6675)..."));
  // MAX6675 objects are defined globally, initialized with pins.
  // delay(500); // Optional power-up delay

  currentTempSensorIndex = 0;
  lastTempProcessTime = millis();

  Serial.print(NUM_TEMP_SENSORS); Serial.println(F(" Temperature Sensors setup complete."));
}

void setupRelays() {
    Serial.println(F("Setting up Relays..."));
    for (int i = 0; i < NUM_RELAYS; i++) {
        pinMode(RELAY_PINS[i], OUTPUT);
        digitalWrite(RELAY_PINS[i], LOW); // Ensure relays are off initially
    }
    Serial.print(NUM_RELAYS); Serial.println(F(" Relays setup complete."));
}

void setupDCMotor() {
    Serial.println(F("Setting up DC Motor..."));

    // Configure Control Pins
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR_DIRECTION_PIN, OUTPUT);
    pinMode(MOTOR_PWM_PIN, OUTPUT); // PWM pin needs to be an output

    // digitalWrite(MOTOR_ENABLE_PIN, HIGH);    // Start with motor disabled
    // digitalWrite(MOTOR_DIRECTION_PIN, HIGH); // Set default direction (e.g., Forward)
    //analogWrite(MOTOR_PWM_PIN, 0);         // Set initial PWM to 0 (stopped)

    // --- Setup PWM on Pin 6 using Timer4 for 10kHz frequency ---
    // Pin 6 is OC4A, controlled by Timer4.
    // Mode 14: Fast PWM, TOP=ICR4. WGM43:42:41:40 = 1110
    // Non-inverting mode: COM4A1=1, COM4A0=0
    // Prescaler 8: CS41=1
    // Frequency = F_CPU / (Prescaler * (1 + TOP)) = 16,000,000 / (8 * (1 + 199)) = 10,000 Hz

    TCCR4A = _BV(COM4A1) | _BV(WGM41); // COM4A1 non-inverting, WGM41 for Mode 14
    TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS41); // WGM43, WGM42 for Mode 14, CS41 for prescaler 8
    ICR4 = 199; // Sets the TOP value for the timer
    OCR4A = 0;  // Set initial duty cycle to 0% (Pin 6)

    // --- Setup for RPM Reading ---
    pinMode(MOTOR_SPEED_SENSE_PIN, INPUT_PULLUP); // Use internal pull-up resistor
    // Pin 3 is External Interrupt 1 on Mega
    attachInterrupt(digitalPinToInterrupt(MOTOR_SPEED_SENSE_PIN), motor_count_pulse, RISING); // Attach ISR

    // Initialize motor speed sense state variables
    motor_pulse_count = 0;
    motor_last_pulse_count = 0;
    lastMotorCalcTime = millis();

    Serial.println(F("DC Motor setup complete. Driver Disabled, Motor Stopped."));
    // To enable the motor and set a speed, use the control signal handling logic in loop().
}


// Add setup functions for other sensor types here
/*
void setupOtherSensors() { ... }
*/

// --- Flow sensor Interrupt Service Routine (ISR) ---
void flow_increase_pulse() {
  flow_pulse++;
}

// --- Motor Speed Sense Interrupt Service Routine (ISR) ---
void motor_count_pulse() {
  motor_pulse_count++;
}


// --- Test Function Definitions ---

// Function to run a test batch of all sensor processing blocks once
void testTimingBatchAllTypes() {
  Serial.println(F("\n--- Running Timing Test: Batch All Types ---"));
  // This function calls the logic for processing one item of each type back-to-back.
  // It includes sensor reads that happen within the processing blocks (like analogRead, readCelsius, get_units).
  // It bypasses the State Machine's timers for measurement purposes.

  // Measure time for one Pressure sensor block execution (simulating a read)
  int raw_p = analogRead(PRESSURE_SENSOR_PINS[0]); // Use sensor 0's pin for read
  unsigned long testStartTime;
  startSensorTimer(PRESSURE_ID_START + 0, &testStartTime); // Use new timing
  PressureSensorValues pData = calculatePressureSensorValues(raw_p, 0); // Calculate for sensor 0
  sendBinaryPacket(PRESSURE_PACKET_START_BYTE, PRESSURE_ID_START, &pData, sizeof(pData), PRESSURE_PACKET_END_BYTE); // Send for sensor 0
  endSensorTimer(PRESSURE_ID_START + 0, testStartTime, "One Pressure Sensor Block");


  // Measure time for one Load Cell block execution (simulating a read)
  // testStartTime; // Reuse variable - no need to declare again
  HX711& testScale = scales[0]; // Get Load Cell 0
  float raw_weight = testScale.get_units(); // Read from Load Cell 0 (will wait for ready)
  startSensorTimer(LOADCELL_ID_START + 0, &testStartTime); // Use new timing
  LoadCellValues loadCellData = calculateLoadCellValues(raw_weight);
  byte loadCell_id = LOADCELL_ID_START + 0; // ID for load cell 0
  sendBinaryPacket(LOADCELL_PACKET_START_BYTE, loadCell_id, &loadCellData, sizeof(loadCellData), LOADCELL_PACKET_END_BYTE);
  endSensorTimer(LOADCELL_ID_START + 0, testStartTime, "One Load Cell Block (incl. get_units wait)");


  // Measure time for one Flow Sensor block execution (simulating update)
  // testStartTime; // Reuse variable - no need to declare again
  long dummyCurrentPulse = flow_pulse + 100; // Assume 100 pulses happened since flow_pulse was 0 in setup
  long dummyLastPulse = 0; // Since flow_pulseLast was 0 in setup
  long delta_pulse = dummyCurrentPulse - dummyLastPulse;
  unsigned long dummyElapsedTime = FLOW_CALCULATION_INTERVAL_MS; // Simulate an interval
  
  startSensorTimer(FLOW_SENSOR_ID, &testStartTime); // Use new timing
  FlowMeterValues fData = calculateFlowMeterValues(delta_pulse, dummyElapsedTime);
  sendBinaryPacket(FLOW_PACKET_START_BYTE, FLOW_SENSOR_ID, &fData, sizeof(fData), FLOW_PACKET_END_BYTE);
  endSensorTimer(FLOW_SENSOR_ID, testStartTime, "One Flow Sensor Calc + Send");


  // Measure time for one Temp Sensor block execution (simulating a read)
  // testStartTime; // Reuse variable - no need to declare again
  startSensorTimer(TEMP_ID_START + 0, &testStartTime); // Use new timing
  TemperatureSensorValues tData = calculateTemperatureSensorValues(0); // Calculate for sensor 0
  sendBinaryPacket(TEMP_PACKET_START_BYTE, TEMP_ID_START, &tData, sizeof(tData), TEMP_PACKET_END_BYTE);
  endSensorTimer(TEMP_ID_START + 0, testStartTime, "One Temp Sensor Block (incl. readCelsius wait)");

  // Measure time for one Motor RPM block execution (simulating update)
  // testStartTime; // Reuse variable - no need to declare again
  startSensorTimer(MOTOR_RPM_ID, &testStartTime); // Use new timing function

  // unsigned long currentMillis_motor = micros(); // Use micros for start time consistency - not needed here, testStartTime already has it
  unsigned long interval_ms = MOTOR_CALCULATION_INTERVAL_MS; // Dummy interval
  unsigned long currentPulseCount_motor = motor_pulse_count + 50; // Assume 50 pulses
  unsigned long motor_last_pulse_count_test = 0; // Assume 0 at start of interval

  MotorRPMValue mData = calculateMotorRPM(currentPulseCount_motor, motor_last_pulse_count_test, interval_ms);
  byte motor_id = MOTOR_RPM_ID;
  sendBinaryPacket(MOTOR_RPM_PACKET_START_BYTE, motor_id, &mData, sizeof(mData), MOTOR_RPM_PACKET_END_BYTE);
  endSensorTimer(MOTOR_RPM_ID, testStartTime, "Motor RPM Block");


  // Add tests for other sensor types here
  /*
  OtherSensorValues dummyOtherData = { ... };
  startTimer();
  OtherSensorValues processedOtherData = calculateOtherSensorValues(dummyOtherData);
  byte other_id = OTHER_ID_START + 0; // ID for other sensor 0
  sendBinaryPacket(OTHER_PACKET_START_BYTE, other_id, &processedOtherData, sizeof(processedOtherData), OTHER_PACKET_END_BYTE);
  printElapsedTime("One Other Sensor Block");
  */

  Serial.println(F("--- Timing Test Batch Complete ---"));
}
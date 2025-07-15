#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h> // Always include Arduino.h in .h files
#include "HX711.h"   // Include necessary library headers
#include "max6675.h" // Include MAX6675 library header
#include <math.h>    // For isnan() prototype

// --- Common Constants ---
extern const long ANALOG_REFERENCE_mV;
extern const float SHUNT_OHM;
extern const float PERCENT_SLOPE;
extern const float PERCENT_OFFSET;


// --- Pressure Sensor Constants ---
extern const int PRESSURE_SENSOR_PINS[6];
extern const float PRESSURE_MAX[6];
extern const int NUM_PRESSURE_SENSORS;
extern const float MV_FACTOR;
extern const float MA_FACTOR;
extern float pressure_scale_factor[6];


// --- Load Cell Constants ---
extern const byte LOADCELL_DOUT_PINS[3];
extern const byte LOADCELL_CLK_PINS[3];
extern const float LOADCELL_CALIBRATION_FACTORS[3];

extern const int NUM_LOADCELL_SENSORS;

extern HX711 scales[3];


// --- Flow Sensor Constants ---
extern const int FLOW_SENSOR_PIN_MEGA; // Pin 2 (INT0)
extern const int FLOW_PPL;


// --- Temperature Sensor (MAX6675) Constants ---
extern const int THERMO_SHARED_CLK_PIN; // D22
extern const int THERMO_SHARED_DO_PIN;  // D50
extern const int THERMO_CS_PINS[4];

extern const int NUM_TEMP_SENSORS;

extern const float FAHRENHEIT_SLOPE;
extern const float FAHRENHEIT_OFFSET;

extern MAX6675 thermocouples[4];


// --- Relay Constants ---
extern const int RELAY_PINS[4];
extern const int NUM_RELAYS;


// --- DC Motor Constants ---
// Define pins for DC Motor
extern const int MOTOR_PWM_PIN;       // Now D6 (Timer4)
extern const int MOTOR_ENABLE_PIN;    // D37
extern const int MOTOR_DIRECTION_PIN; // D38
extern const int MOTOR_SPEED_SENSE_PIN; // D3 (INT1)

extern const int MOTOR_PULSES_PER_REVOLUTION; // Pulses per revolution from driver
extern const float PULSES_PER_SEC_TO_RPM_FACTOR;


// Add constants for other sensor types here
/*
extern const int OTHER_SENSOR_INPUTS[2]; // Example for other sensors
extern const int NUM_OTHER_SENSORS;
*/


// --- Binary Protocol Constants (Sensor Data Output) ---
extern const byte PRESSURE_PACKET_START_BYTE;
extern const byte PRESSURE_PACKET_END_BYTE;
extern const byte LOADCELL_PACKET_START_BYTE;
extern const byte LOADCELL_PACKET_END_BYTE;
extern const byte FLOW_PACKET_START_BYTE;
extern const byte FLOW_PACKET_END_BYTE;
extern const byte TEMP_PACKET_START_BYTE;
extern const byte TEMP_PACKET_END_BYTE;
extern const byte MOTOR_RPM_PACKET_START_BYTE;
extern const byte MOTOR_RPM_PACKET_END_BYTE;


// --- Binary Protocol Constants (Incoming Commands) ---
extern const byte COMMAND_START_BYTE; // Start byte for commands
extern const byte COMMAND_END_BYTE;   // End byte for commands

// Command Type definitions
extern const byte CMD_TYPE_SET_RELAY; // Command to set a relay state
extern const byte CMD_TYPE_SET_MOTOR; // Command to set motor state (enable, direction, throttle)

// Target ID definitions for commands (usually map 1-to-1 with sensor IDs or use 0-indexed)
extern const byte CMD_TARGET_RELAY_START; // Usually 0 (for Relay 0)
extern const byte CMD_TARGET_MOTOR_ID;    // Usually 0 (for the single motor)


// Define ID ranges and number of IDs for each sensor type (Output Packets)
extern const byte PRESSURE_ID_START;
extern const byte NUM_IDS_PRESSURE;
extern const byte LOADCELL_ID_START;
extern const byte NUM_IDS_LOADCELL;
extern const byte FLOW_SENSOR_ID;
extern const byte NUM_IDS_FLOW;
extern const byte TEMP_ID_START;
extern const byte NUM_IDS_TEMP;
extern const byte MOTOR_RPM_ID;
extern const byte NUM_IDS_MOTOR_RPM;

//-------Struct for sensor data ------
struct PressureSensorValues { float pressure; };
struct LoadCellValues { float weight_grams; };
struct FlowMeterValues { float flow_rate_lpm; };
struct TemperatureSensorValues { float temp_c; float temp_f; };
struct MotorRPMValue { float rpm; };

// --- NEW: Timing Data Structure ---
struct SensorTiming {
    byte sensor_id;
    unsigned long start_micros;
    unsigned long end_micros;
    unsigned long duration_micros;
};

// --- NEW: Global Arrays to Store Timing Data ---
const byte MAX_TIMED_PRESSURE_SENSORS = 6;
const byte MAX_TIMED_LOADCELL_SENSORS = 3;
const byte MAX_TIMED_TEMP_SENSORS = 4;
const byte MAX_TIMED_FLOW_SENSORS = 1; // For the single flow sensor

extern SensorTiming pressureTimingData[MAX_TIMED_PRESSURE_SENSORS];
extern SensorTiming loadCellTimingData[MAX_TIMED_LOADCELL_SENSORS];
extern SensorTiming tempTimingData[MAX_TIMED_TEMP_SENSORS];
extern SensorTiming flowTimingData[MAX_TIMED_FLOW_SENSORS]; // New for flow sensor

// --- NEW: Packet Constants for Timing Data ---
extern const byte TIMING_PACKET_START_BYTE;
extern const byte TIMING_PACKET_END_BYTE;
extern const byte TIMING_SENSOR_OPERATION_ID; // For individual sensor timings (e.g., Pressure ID 0-5)
extern const byte TIMING_CATEGORY_CYCLE_ID;   // For category cycle times (e.g., "All Pressures")


// --- Serial Receive State Machine Variables and Constants ---
// Define states for parsing incoming commands
enum RxState {
  RX_WAITING_FOR_START,
  RX_READING_TYPE,
  RX_READING_TARGET_ID,
  RX_READING_SIZE,
  RX_READING_PAYLOAD,
  RX_READING_END // Or just transition directly to processing from READING_PAYLOAD
};

// Global state variable, initialized in setup()
extern RxState currentRxState;

// Variables to store components of the current command being received
extern byte rxCommandType;
extern byte rxTargetId;
extern byte rxPayloadSize; // The size indicated in the packet
extern byte rxPayloadBytesRead; // How many payload bytes we've read so far

// Buffer to hold the incoming command payload
const byte MAX_COMMAND_PAYLOAD_SIZE = 16; // Choose a reasonable size larger than max needed (Motor is 3 bytes)
extern byte rxPayloadBuffer[MAX_COMMAND_PAYLOAD_SIZE];


// --- State Machine / Round-Robin Variables and Constants (Sensor Reading) ---
extern int currentPressureSensorIndex;
extern unsigned long lastPressureSensorProcessTime;
extern const unsigned long MIN_PRESSURE_INTERVAL_MS;

extern int currentLoadCellIndex; // Cycles 0, 1, 2
extern unsigned long lastLoadCellProcessTime;
extern const unsigned long MIN_LOADCELL_CHECK_INTERVAL_MS;

extern volatile long flow_pulse;
extern long flow_pulseLast;
extern unsigned long lastFlowProcessTime;
extern const unsigned long FLOW_CALCULATION_INTERVAL_MS;
extern unsigned long elapsed_time; // Corrected to unsigned long

extern int currentTempSensorIndex; // Cycles 0, 1, 2, 3
extern unsigned long lastTempProcessTime;
extern const unsigned long MIN_TEMP_INTERVAL_MS;

extern volatile unsigned long motor_pulse_count;
extern unsigned long motor_last_pulse_count;
extern unsigned long lastMotorCalcTime;
extern const unsigned long MOTOR_CALCULATION_INTERVAL_MS;

// --- NEW: Timing variables for category cycles ---
extern unsigned long pressureCategoryStartTime;
extern unsigned long loadCellCategoryStartTime;
extern unsigned long tempCategoryStartTime;
extern unsigned long flowCategoryStartTime; // New for flow sensor

// Add state variables and constants for other sensor types here
/*
extern int currentOtherSensorIndex;
unsigned long lastOtherSensorProcessTime;
extern const unsigned long MIN_OTHER_INTERVAL_MS;
*/


// --- Timing Helper Functions ---
// Removed _timerStartTime as it's replaced by individual sensor timing variables
void startSensorTimer(byte sensorId, unsigned long* startTimeVar); // Updated prototype
void endSensorTimer(byte sensorId, unsigned long startTime, const char* description); // Updated prototype
void sendTimingPacket(byte timing_id, const SensorTiming* data_ptr); // New function to send timing data


// --- Function Prototypes (Declarations) ---
// Sensor calculation functions
PressureSensorValues calculatePressureSensorValues(int raw_pressure_int, int index);
LoadCellValues calculateLoadCellValues(float raw_weight_float);
FlowMeterValues calculateFlowMeterValues(long delta_pulse, unsigned long elapsed_time); // Corrected parameters
TemperatureSensorValues calculateTemperatureSensorValues(int index);
MotorRPMValue calculateMotorRPM(unsigned long currentPulseCount, unsigned long previousPulseCount, unsigned long interval_ms);

// Add prototypes for other sensor calculation functions here
/*
OtherSensorValues calculateOtherSensorValues(...);
*/

// GENERIC Function to send any data block in binary format
void sendBinaryPacket(byte start_byte, byte id, const void* data_ptr, size_t data_size, byte end_byte);

// --- Command Handling Functions ---
// Main function to process a complete, valid command packet
void handleCommand(byte commandType, byte targetId, const byte* payload, byte payloadSize);

// Helper functions to perform specific actions
void setRelayState(byte relayIndex, byte state);
// Add other specific relay control functions if needed (e.g., toggle)

// Helper functions for motor control actions
void setMotorEnable(byte state); // state: 0=OFF, 1=ON
void setMotorDirection(byte direction); // direction: 0=Reverse, 1=Forward
void setMotorThrottle(byte throttlePercent); // throttlePercent: 0-100


// Modular Setup Functions
void setupPressureSensors();
void setupLoadCells();
void setupFlowSensors();
void setupTemperatureSensors();
void setupRelays();
void setupDCMotor();


// Add prototypes for other modular setup functions here
/*
void setupOtherSensors();
*/

// Interrupt Service Routines (ISRs)
void flow_increase_pulse();
void motor_count_pulse();


// --- Test Functions ---
void testTimingBatchAllTypes();


#endif // End of include guard
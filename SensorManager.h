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
// Define arrays for pins for each of the THREE Load Cells - SIZE IS 3
extern const byte LOADCELL_DOUT_PINS[3];
extern const byte LOADCELL_CLK_PINS[3];
extern const float LOADCELL_CALIBRATION_FACTORS[3]; // Calibration factors for 3 sensors

extern const int NUM_LOADCELL_SENSORS; // Will be 3

// HX711 objects - Array of objects - SIZE IS 3
extern HX711 scales[3];


// --- Flow Sensor Constants ---
extern const int FLOW_SENSOR_PIN_MEGA; // Pin 2 (INT0)

extern const float FLOW_PPL;
extern const float PULSES_TO_LPM_FACTOR;


// --- Temperature Sensor (MAX6675) Constants ---
// Define shared CLK and DO pins
extern const int THERMO_SHARED_CLK_PIN; // D22
extern const int THERMO_SHARED_DO_PIN;  // D50

// Define unique CS pins for each of the FOUR MAX6675 sensors - SIZE IS 4
extern const int THERMO_CS_PINS[4];

extern const int NUM_TEMP_SENSORS; // Will be 4

// Pre-calculated constants for Fahrenheit conversion (needed in calc function)
extern const float FAHRENHEIT_SLOPE;
extern const float FAHRENHEIT_OFFSET;

// Array of MAX6675 objects - SIZE IS 4
extern MAX6675 thermocouples[4];


// --- Relay Constants ---
// Define pins for the FOUR Relays - SIZE IS 4
extern const int RELAY_PINS[4];
extern const int NUM_RELAYS;


// --- DC Motor Constants ---
// Define pins for DC Motor
extern const int MOTOR_PWM_PIN;       // D11 (Timer1)
extern const int MOTOR_ENABLE_PIN;    // D37
extern const int MOTOR_DIRECTION_PIN; // D38
extern const int MOTOR_SPEED_SENSE_PIN; // D3 (INT1)

// Define motor specific constants
extern const int MOTOR_PULSES_PER_REVOLUTION; // Pulses per revolution from driver

// Pre-calculated factor for motor RPM calculation
extern const float PULSES_PER_SEC_TO_RPM_FACTOR;


// Add constants for other sensor types here
/*
extern const int OTHER_SENSOR_INPUTS[2]; // Example for other sensors
extern const int NUM_OTHER_SENSORS;
*/


// --- Binary Protocol Constants ---
extern const byte PRESSURE_PACKET_START_BYTE;
extern const byte PRESSURE_PACKET_END_BYTE;
extern const byte LOADCELL_PACKET_START_BYTE;
extern const byte LOADCELL_PACKET_END_BYTE;
extern const byte FLOW_PACKET_START_BYTE;
extern const byte FLOW_PACKET_END_BYTE;
extern const byte TEMP_PACKET_START_BYTE;
extern const byte TEMP_PACKET_END_BYTE;
extern const byte MOTOR_RPM_PACKET_START_BYTE; // New start byte for Motor RPM
extern const byte MOTOR_RPM_PACKET_END_BYTE;   // New end byte for Motor RPM


// Define ID ranges and number of IDs for each sensor type
extern const byte PRESSURE_ID_START; // 0
extern const byte NUM_IDS_PRESSURE; // 6

extern const byte LOADCELL_ID_START; // 6
extern const byte NUM_IDS_LOADCELL; // 3

extern const byte FLOW_SENSOR_ID;    // 9
extern const byte NUM_IDS_FLOW;      // 1

extern const byte TEMP_ID_START;     // 10
extern const byte NUM_IDS_TEMP;      // 4

extern const byte MOTOR_RPM_ID;      // 14
extern const byte NUM_IDS_MOTOR_RPM; // 1


// Add constants for other sensor types here (packet markers, ID starts, num IDs)
/*
extern const byte OTHER_PACKET_START_BYTE;
extern const byte OTHER_PACKET_END_BYTE;
extern const byte OTHER_ID_START;
extern const byte NUM_IDS_OTHER;
*/


// --- Data Structures for Sensor Values ---
struct PressureSensorValues {
  float pressure;
};

struct LoadCellValues {
  float weight_grams;
};

struct FlowMeterValues {
  float flow_rate_lpm;
};

struct TemperatureSensorValues {
  float temp_c;
  float temp_f;
};

// New struct for Motor RPM
struct MotorRPMValue {
  float rpm;
};

// Add structs for other sensor types here
/*
struct OtherSensorValues { // ... define fields ... };
*/


// --- State Machine / Round-Robin Variables and Constants ---
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

extern int currentTempSensorIndex; // Cycles 0, 1, 2, 3
extern unsigned long lastTempProcessTime;
extern const unsigned long MIN_TEMP_INTERVAL_MS;


// Motor Speed Sense State (for the single sensor)
extern volatile unsigned long motor_pulse_count; // MUST be volatile
extern unsigned long motor_last_pulse_count; // Stores pulse count at the start of the last interval
extern unsigned long lastMotorCalcTime; // Time when the last RPM was calculated/sent
// The interval for how often to CALCULATE and SEND the Motor RPM
extern const unsigned long MOTOR_CALCULATION_INTERVAL_MS;


// Add state variables and constants for other sensor types here
/*
extern int currentOtherSensorIndex;
extern unsigned long lastOtherSensorProcessTime;
extern const unsigned long MIN_OTHER_INTERVAL_MS;
*/


// --- Timing Helper Functions ---
extern unsigned long _timerStartTime;

void startTimer();
void printElapsedTime(const char* description);


// --- Function Prototypes (Declarations) ---
PressureSensorValues calculatePressureSensorValues(int raw_pressure_int, int index);
LoadCellValues calculateLoadCellValues(float raw_weight_float);
FlowMeterValues calculateFlowMeterValues(long currentPulseCount, long previousPulseCount);
TemperatureSensorValues calculateTemperatureSensorValues(int index);
// New motor RPM calculation prototype
MotorRPMValue calculateMotorRPM(unsigned long currentPulseCount, unsigned long previousPulseCount, unsigned long interval_ms);


// Add prototypes for other sensor calculation functions here
/*
OtherSensorValues calculateOtherSensorValues(...);
*/

// GENERIC Function to send any data block in binary format
void sendBinaryPacket(byte start_byte, byte id, const void* data_ptr, size_t data_size, byte end_byte);


// Modular Setup Functions
void setupPressureSensors();
void setupLoadCells(); // Updated to handle 3 sensors
void setupFlowSensors();
void setupTemperatureSensors(); // Updated to handle 4 sensors
void setupRelays(); // New setup for relays
void setupDCMotor(); // New setup for DC Motor


// Add prototypes for other modular setup functions here
/*
void setupOtherSensors();
*/

// Flow sensor Interrupt Service Routine (ISR) prototype
void flow_increase_pulse();

// New Motor Speed Sense Interrupt Service Routine (ISR) prototype
void motor_count_pulse();


// --- Test Functions ---
void testTimingBatchAllTypes();


#endif // End of include guard
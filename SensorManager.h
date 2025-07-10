#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h> // Always include Arduino.h in .h files
#include "HX711.h"   // Include necessary library headers
#include "max6675.h" // Include MAX6675 library header
#include <math.h>    // For isnan() prototype (used in calculation function)

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
extern const byte LOADCELL_DOUT_PINS[2];
extern const byte LOADCELL_CLK_PINS[2];
extern const float LOADCELL_CALIBRATION_FACTORS[2];
extern const int NUM_LOADCELL_SENSORS;
extern HX711 scales[2];


// --- Flow Sensor Constants ---
extern const int FLOW_SENSOR_PIN_MEGA; // Pin 2 recommended on Mega
extern const float FLOW_PPL;
extern const float PULSES_TO_LPM_FACTOR;


// --- Temperature Sensor (MAX6675) Constants ---
extern const int THERMO_DO_PINS[2];
extern const int THERMO_CS_PINS[2];
extern const int THERMO_CLK_PINS[2];
extern const int NUM_TEMP_SENSORS;
extern const float FAHRENHEIT_SLOPE;
extern const float FAHRENHEIT_OFFSET;
extern MAX6675 thermocouples[2];


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

// Define ID ranges and number of IDs for each sensor type
extern const byte PRESSURE_ID_START;
extern const byte NUM_IDS_PRESSURE;
extern const byte LOADCELL_ID_START;
extern const byte NUM_IDS_LOADCELL;
extern const byte FLOW_SENSOR_ID;
extern const byte NUM_IDS_FLOW;
extern const byte TEMP_ID_START;
extern const byte NUM_IDS_TEMP;

// Add constants for other sensor types here (packet markers, ID starts, num IDs)
/*
extern const byte OTHER_PACKET_START_BYTE;
extern const byte OTHER_PACKET_END_BYTE;
extern const byte OTHER_ID_START;
extern const byte NUM_IDS_OTHER;
*/


// --- Data Structures for Sensor Values ---
struct PressureSensorValues {
  float volts;
  float mA;
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

// Add structs for other sensor types here
/*
struct OtherSensorValues { // ... define fields ... };
*/


// --- State Machine / Round-Robin Variables and Constants ---
extern int currentPressureSensorIndex;
extern unsigned long lastPressureSensorProcessTime;
extern const unsigned long MIN_PRESSURE_INTERVAL_MS;

extern int currentLoadCellIndex;
extern unsigned long lastLoadCellProcessTime;
extern const unsigned long MIN_LOADCELL_CHECK_INTERVAL_MS;

extern volatile long flow_pulse;
extern long flow_pulseLast;
extern unsigned long lastFlowProcessTime;
extern const unsigned long FLOW_CALCULATION_INTERVAL_MS;

extern int currentTempSensorIndex;
extern unsigned long lastTempProcessTime;
extern const unsigned long MIN_TEMP_INTERVAL_MS;

// Add state variables and constants for other sensor types here
/*
extern int currentOtherSensorIndex;
extern unsigned long lastOtherSensorProcessTime;
extern const unsigned long MIN_OTHER_INTERVAL_MS;
*/


// --- Timing Helper Functions ---
extern unsigned long _timerStartTime; // Global variable to store the timer start time

void startTimer(); // Records the current micros()
void printElapsedTime(const char* description); // Prints elapsed time since startTimer()


// --- Function Prototypes (Declarations) ---
PressureSensorValues calculatePressureSensorValues(int raw_pressure_int, int index);
LoadCellValues calculateLoadCellValues(float raw_weight_float);
FlowMeterValues calculateFlowMeterValues(long currentPulseCount, long previousPulseCount);
TemperatureSensorValues calculateTemperatureSensorValues(int index);

// Add prototypes for other sensor calculation functions here
/*
OtherSensorValues calculateOtherSensorValues(...);
*/

// GENERIC Function to send any data block in binary format
void sendBinaryPacket(byte start_byte, byte id, const void* data_ptr, size_t data_size, byte end_byte);


// Modular Setup Functions
void setupPressureSensors();
void setupLoadCells();
void setupFlowSensors();
void setupTemperatureSensors();

// Add prototypes for other modular setup functions here
/*
void setupOtherSensors();
*/

// Flow sensor Interrupt Service Routine (ISR) prototype
void flow_increase_pulse();


// --- Test Functions ---
// Function to run a test batch of all sensor processing blocks once
void testTimingBatchAllTypes();


#endif // End of include guard
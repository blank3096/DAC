#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h> // Always include Arduino.h in .h files for types like byte, unsigned long etc.
#include "HX711.h"   // Include necessary library headers if their types are used in declarations
// Add includes for other sensor libraries here (e.g., Wire.h for I2C, etc.)

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

// --- Pre-calculated constants for faster floating-point math (Pressure - Method 1 style) ---
extern const float MA_FACTOR;
extern float pressure_scale_factor[6];


// --- Load Cell Constants ---
extern const byte LOADCELL_DOUT_PINS[2];
extern const byte LOADCELL_CLK_PINS[2];
extern const float LOADCELL_CALIBRATION_FACTORS[2];
extern const int NUM_LOADCELL_SENSORS;

extern HX711 scales[2];


// --- Flow Sensor Constants ---
extern const int FLOW_SENSOR_PIN; // The pin for the flow sensor's pulse output
extern const float FLOW_PPL; // Pulses per liter calibration factor for the flow sensor

// Pre-calculated factor for flow calculation
extern const float PULSES_TO_LPM_FACTOR;

// Add constants for other sensor types here (pins, calibration, etc.)
/*
extern const int TEMP_SENSOR_PINS[2];
extern const int NUM_TEMP_SENSORS;
*/


// --- Binary Protocol Constants ---
extern const byte PRESSURE_PACKET_START_BYTE;
extern const byte PRESSURE_PACKET_END_BYTE;
extern const byte LOADCELL_PACKET_START_BYTE;
extern const byte LOADCELL_PACKET_END_BYTE;
extern const byte FLOW_PACKET_START_BYTE; // New start byte for flow data
extern const byte FLOW_PACKET_END_BYTE;   // New end byte for flow data

// Define ID ranges for each sensor type
extern const byte PRESSURE_ID_START; // Usually 0
extern const byte LOADCELL_ID_START; // Usually PRESSURE_ID_START + NUM_PRESSURE_SENSORS
extern const byte FLOW_SENSOR_ID;    // Unique ID for the single flow sensor (e.g., 8)

// Add constants for other sensor types here (packet markers, ID starts)
/*
extern const byte TEMP_PACKET_START_BYTE;
extern const byte TEMP_PACKET_END_BYTE;
extern const byte TEMP_ID_START;
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

struct FlowMeterValues { // New struct for flow sensor data
  float flow_rate_lpm;
};

// Add structs for other sensor types here (Temp, Other)
/*
struct TemperatureSensorValues { float temp_c; };
struct OtherSensorValues { // ... define fields ... };
*/


// --- State Machine / Round-Robin Variables and Constants ---

// Pressure Sensor State
extern int currentPressureSensorIndex;
extern unsigned long lastPressureSensorProcessTime;
extern const unsigned long MIN_PRESSURE_INTERVAL_MS;

// Load Cell Sensor State
extern int currentLoadCellIndex;
extern unsigned long lastLoadCellProcessTime;
extern const unsigned long MIN_LOADCELL_CHECK_INTERVAL_MS;

// Flow Sensor State (for the single sensor)
extern volatile long flow_pulse; // MUST be volatile because it's written in an ISR
extern long flow_pulseLast;      // Stores pulse count at the start of the last interval
extern unsigned long lastFlowProcessTime; // Time when the last flow rate was calculated/sent
// The interval for how often to CALCULATE and SEND the flow rate
extern const unsigned long FLOW_CALCULATION_INTERVAL_MS;

// Add state variables and constants for other sensor types here
/*
extern int currentTempSensorIndex;
extern unsigned long lastTempSensorProcessTime;
extern const unsigned long MIN_TEMP_INTERVAL_MS;
*/


// --- Function Prototypes (Declarations) ---
PressureSensorValues calculatePressureSensorValues(int raw_pressure_int, int index);
LoadCellValues calculateLoadCellValues(float raw_weight_float);
FlowMeterValues calculateFlowMeterValues(long currentPulseCount, long previousPulseCount); // New flow calc prototype


// Add prototypes for other sensor calculation functions here
/*
TemperatureSensorValues calculateTemperatureSensorValues(...);
OtherSensorValues calculateOtherSensorValues(...);
*/

// GENERIC Function to send any data block in binary format
void sendBinaryPacket(byte start_byte, byte id, const void* data_ptr, size_t data_size, byte end_byte);


// Modular Setup Functions
void setupPressureSensors();
void setupLoadCells();
void setupFlowSensors(); // New flow sensor setup prototype

// Add prototypes for other modular setup functions here
/*
void setupTemperatureSensors();
void setupOtherSensors();
*/

// Flow sensor Interrupt Service Routine (ISR)
void flow_increase_pulse(); // New ISR prototype


#endif // End of include guard
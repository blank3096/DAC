#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h> // Always include Arduino.h in .h files for types like byte, unsigned long etc.
#include "HX711.h"   // Include necessary library headers if their types are used in declarations
// Add includes for other sensor libraries here (e.g., Wire.h for I2C, OneWire.h, etc.)

// --- Common Constants ---
extern const long ANALOG_REFERENCE_mV;
extern const float SHUNT_OHM;
extern const float PERCENT_SLOPE;
extern const float PERCENT_OFFSET;


// --- Pressure Sensor Constants ---
extern const int PRESSURE_SENSOR_PINS[6];
extern const float PRESSURE_MAX[6];
extern const int NUM_PRESSURE_SENSORS; // Defined by array size, but good to declare

// MV_FACTOR calculation (Method 1 style) - ensure float division
extern const float MV_FACTOR;

// --- Pre-calculated constants for faster floating-point math (Pressure - Method 1 style) ---
extern const float MA_FACTOR;

// This array is modified in setup, so it must be defined in the .cpp and declared extern here.
extern float pressure_scale_factor[6];


// --- Load Cell Constants ---
extern const byte LOADCELL_DOUT_PINS[2];
extern const byte LOADCELL_CLK_PINS[2];
extern const float LOADCELL_CALIBRATION_FACTORS[2];
extern const int NUM_LOADCELL_SENSORS; // Defined by array size

// HX711 objects - Array of objects defined in the .cpp, declared extern here
extern HX711 scales[2];


// --- Binary Protocol Constants ---
extern const byte PRESSURE_PACKET_START_BYTE;
extern const byte PRESSURE_PACKET_END_BYTE;
extern const byte LOADCELL_PACKET_START_BYTE;
extern const byte LOADCELL_PACKET_END_BYTE;

// Define ID ranges for each sensor type
extern const byte PRESSURE_ID_START; // Usually 0
extern const byte LOADCELL_ID_START; // Usually PRESSURE_ID_START + NUM_PRESSURE_SENSORS

// Add constants for other sensor types here (pins, calibration, etc.)
/*
extern const int FLOW_METER_PINS[2];
extern const int NUM_FLOW_METERS;
extern const byte FLOW_PACKET_START_BYTE;
extern const byte FLOW_PACKET_END_BYTE;
extern const byte FLOW_ID_START;

extern const int TEMP_SENSOR_PINS[2];
extern const int NUM_TEMP_SENSORS;
extern const byte TEMP_PACKET_START_BYTE;
extern const byte TEMP_PACKET_END_BYTE;
extern const byte TEMP_ID_START;
// ... etc for other sensor types
*/


// --- Data Structures for Sensor Values ---
// Define these structs here so both the .ino and the .cpp know about them
struct PressureSensorValues {
  float volts;
  float mA;
  float pressure;
};

struct LoadCellValues {
  float weight_grams;
};

// Add structs for other sensor types here (Flow, Temp, Other)
/*
struct FlowMeterValues { float flow_rate_lpm; };
struct TemperatureSensorValues { float temp_c; };
struct OtherSensorValues { // ... define fields ... };
*/


// --- State Machine / Round-Robin Variables and Constants ---

// These are variables modified in loop(), so they must be defined in the .cpp and declared extern here
extern int currentPressureSensorIndex;
extern unsigned long lastPressureSensorProcessTime;
extern const unsigned long MIN_PRESSURE_INTERVAL_MS;

extern int currentLoadCellIndex;
extern unsigned long lastLoadCellProcessTime;
extern const unsigned long MIN_LOADCELL_CHECK_INTERVAL_MS;

// Add state variables and constants for other sensor types here
/*
extern int currentFlowMeterIndex;
extern unsigned long lastFlowMeterProcessTime;
const unsigned long MIN_FLOW_INTERVAL_MS; // Note: const here, so needs definition in .cpp

extern int currentTempSensorIndex;
extern unsigned long lastTempSensorProcessTime;
const unsigned long MIN_TEMP_INTERVAL_MS; // Note: const here
// ... etc
*/


// --- Function Prototypes (Declarations) ---
// Declare each function defined in SensorManager.cpp so the .ino file knows they exist
PressureSensorValues calculatePressureSensorValues(int raw_pressure_int, int index);
LoadCellValues calculateLoadCellValues(float raw_weight_float);

// Add prototypes for other sensor calculation functions here
/*
FlowMeterValues calculateFlowMeterValues(...);
TemperatureSensorValues calculateTemperatureSensorValues(...);
OtherSensorValues calculateOtherSensorValues(...);
*/

// GENERIC Function to send any data block in binary format
void sendBinaryPacket(byte start_byte, byte id, const void* data_ptr, size_t data_size, byte end_byte);


// Modular Setup Functions
void setupPressureSensors();
void setupLoadCells();

// Add prototypes for other modular setup functions here
/*
void setupFlowMeters();
void setupTemperatureSensors();
void setupOtherSensors();
*/


#endif //End of include guard

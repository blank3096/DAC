#include "SensorManager.h" // Include your own header first
#include <Arduino.h>     // Include Arduino.h here too, as this is a compilation unit


// =======================================================
// === Definitions of Constants and Global Variables =====
// === (Matching extern declarations in SensorManager.h) ===
// =======================================================

// --- Common Constants ---
const long ANALOG_REFERENCE_mV = 5000;
const float SHUNT_OHM = 217.9;
const float PERCENT_SLOPE = 6.25;
const float PERCENT_OFFSET = -25.0;


// --- Pressure Sensor Constants ---
const int PRESSURE_SENSOR_PINS[6] = {A0, A1, A2, A3, A4, A5};
const float PRESSURE_MAX[6] = {16.0, 16.0, 25.0, 25.0, 40.0, 40.0};
const int NUM_PRESSURE_SENSORS = sizeof(PRESSURE_SENSOR_PINS) / sizeof(PRESSURE_SENSOR_PINS[0]); // Defined here

const float MV_FACTOR = (float)ANALOG_REFERENCE_mV / 1024.0f; // Defined here

// --- Pre-calculated constants for faster floating-point math (Pressure - Method 1 style) ---
const float MA_FACTOR = 1000.0f / SHUNT_OHM;

float pressure_scale_factor[6]; // Defined here (not extern in *this* file)


// --- Load Cell Constants ---
const byte LOADCELL_DOUT_PINS[2] = {12, 10};
const byte LOADCELL_CLK_PINS[2] = {11, 9};
const float LOADCELL_CALIBRATION_FACTORS[2] = {145.4f, 150.0f};
const int NUM_LOADCELL_SENSORS = sizeof(LOADCELL_DOUT_PINS) / sizeof(LOADCELL_DOUT_PINS[0]); // Defined here

HX711 scales[2]; // Defined here (array of objects)


// --- Binary Protocol Constants ---
const byte PRESSURE_PACKET_START_BYTE = 0xAA;
const byte PRESSURE_PACKET_END_BYTE = 0x55;
const byte LOADCELL_PACKET_START_BYTE = 0xBB;
const byte LOADCELL_PACKET_END_BYTE = 0x66;

const byte PRESSURE_ID_START = 0;
const byte LOADCELL_ID_START = PRESSURE_ID_START + NUM_PRESSURE_SENSORS; // Defined here


// Add definitions for other sensor constants here (pins, calibration, etc.)
/*
const int FLOW_METER_PINS[2] = { ... };
const int NUM_FLOW_METERS = sizeof(FLOW_METER_PINS) / sizeof(FLOW_METER_PINS[0]);
const byte FLOW_PACKET_START_BYTE = 0xCC;
const byte FLOW_PACKET_END_BYTE = 0{DD;
const byte FLOW_ID_START = LOADCELL_ID_START + NUM_LOADCELL_SENSORS; // Offset from previous group

const int TEMP_SENSOR_PINS[2] = { ... };
const int NUM_TEMP_SENSORS = sizeof(TEMP_SENSOR_PINS) / sizeof(TEMP_SENSOR_PINS[0]);
const byte TEMP_PACKET_START_BYTE = 0xEE;
const byte TEMP_PACKET_END_BYTE = 0xFF;
const byte TEMP_ID_START = FLOW_ID_START + NUM_FLOW_METERS; // Offset from previous group
// ... etc
*/


// --- State Machine / Round-Robin Variables and Constants ---

// These are variables modified in loop(), so defined here
int currentPressureSensorIndex = 0;
unsigned long lastPressureSensorProcessTime = 0;
const unsigned long MIN_PRESSURE_INTERVAL_MS = 10;

int currentLoadCellIndex = 0;
unsigned long lastLoadCellProcessTime = 0;
const unsigned long MIN_LOADCELL_CHECK_INTERVAL_MS = 150;

// Add definitions for state variables and constants for other sensor types here
/*
int currentFlowMeterIndex = 0;
unsigned long lastFlowMeterProcessTime = 0;
const unsigned long MIN_FLOW_INTERVAL_MS = 50; // Defined here

int currentTempSensorIndex = 0;
unsigned long lastTempSensorProcessTime = 0;
const unsigned long MIN_TEMP_INTERVAL_MS = 500; // Defined here
// ... etc
*/


// =======================================================
// === Function Definitions (Code Bodies) ================
// === (Matching prototypes in SensorManager.h) ==========
// =======================================================

// --- Calculation Function for Pressure Sensor ---
PressureSensorValues calculatePressureSensorValues(int raw_pressure_int, int index) {
  // Access constants and scale_factor array defined above in this .cpp file
  if (index < 0 || index >= NUM_PRESSURE_SENSORS) return {0.0f, 0.0f, 0.0f};

  float raw_pressure_f = (float)raw_pressure_int;
  float mV = raw_pressure_f * MV_FACTOR;
  float volts = mV / 1000.0f;
  float mA = volts * MA_FACTOR;
  float percent = mA * PERCENT_SLOPE + PERCENT_OFFSET;
  float pressure = percent * pressure_scale_factor[index];

  return {volts, mA, pressure};
}

// --- Calculation Function for Load Cell Sensor ---
LoadCellValues calculateLoadCellValues(float raw_weight_float) {
    return {raw_weight_float};
}

// Add calculation functions for other sensor types here
/*
FlowMeterValues calculateFlowMeterValues(...) { ... }
TemperatureSensorValues calculateTemperatureSensorValues(...) { ... }
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


// --- Modular Setup Functions ---

void setupPressureSensors() {
  Serial.println(F("Setting up Pressure Sensors..."));
  for (int i = 0; i < NUM_PRESSURE_SENSORS; i++) {
    pressure_scale_factor[i] = PRESSURE_MAX[i] / 100.0f;
  }
  Serial.print(NUM_PRESSURE_SENSORS); Serial.println(F(" Pressure Sensors setup complete."));
}

void setupLoadCells() {
  Serial.println(F("Setting up Load Cells..."));
  for (int i = 0; i < NUM_LOADCELL_SENSORS; i++) {
    Serial.print(F("Load Cell ")); Serial.print(i + 1);
    Serial.print(F(" on pins DOUT:")); Serial.print(LOADCELL_DOUT_PINS[i]);
    Serial.print(F(" CLK:")); Serial.println(LOADCELL_CLK_PINS[i]);

    scales[i].begin(LOADCELL_DOUT_PINS[i], LOADCELL_CLK_PINS[i]);
    delay(100); // Give HX711 a moment

    scales[i].set_scale(LOADCELL_CALIBRATION_FACTORS[i]);
    scales[i].tare();

    Serial.print(F("Load Cell ")); Serial.print(i + 1); Serial.println(F(" tared."));
    delay(50); // Small delay between taring
  }
  Serial.print(NUM_LOADCELL_SENSORS); Serial.println(F(" Load Cells setup complete."));
}

// Add setup functions for other sensor types here
/*
void setupFlowMeters() { ... }
void setupTemperatureSensors() { ... }
void setupOtherSensors() { ... }
*/

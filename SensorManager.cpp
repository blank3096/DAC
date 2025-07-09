#include "SensorManager.h" // Include your own header first
#include <Arduino.h>     // Include Arduino.h here too
#include <math.h>        // Required for isnan()


// =======================================================
// === Definitions of Constants and Global Variables =====
// =======================================================

// --- Common Constants ---
const long ANALOG_REFERENCE_mV = 5000;
const float SHUNT_OHM = 217.9;
const float PERCENT_SLOPE = 6.25;
const float PERCENT_OFFSET = -25.0;


// --- Pressure Sensor Constants ---
const int PRESSURE_SENSOR_PINS[6] = {A0, A1, A2, A3, A4, A5};
const float PRESSURE_MAX[6] = {16.0, 16.0, 25.0, 25.0, 40.0, 40.0};
const int NUM_PRESSURE_SENSORS = sizeof(PRESSURE_SENSOR_PINS) / sizeof(PRESSURE_SENSOR_PINS[0]);

const float MV_FACTOR = (float)ANALOG_REFERENCE_mV / 1024.0f;
const float MA_FACTOR = 1000.0f / SHUNT_OHM;
float pressure_scale_factor[6];


// --- Load Cell Constants ---
const byte LOADCELL_DOUT_PINS[2] = {12, 10};
const byte LOADCELL_CLK_PINS[2] = {11, 9};
const float LOADCELL_CALIBRATION_FACTORS[2] = {145.4f, 150.0f};
const int NUM_LOADCELL_SENSORS = sizeof(LOADCELL_DOUT_PINS) / sizeof(LOADCELL_DOUT_PINS[0]);
HX711 scales[2];


// --- Flow Sensor Constants ---
// Pin 2 on Mega is External Interrupt 0
const int FLOW_SENSOR_PIN_MEGA = 2; // *** REMEMBER TO WIRE FLOW SENSOR TO PIN 2 ***
const float FLOW_PPL = 4215;
const float PULSES_TO_LPM_FACTOR = 60.0f / FLOW_PPL;


// --- Temperature Sensor (MAX6675) Constants ---
// Example pins for two MAX6675 sensors on Mega (adjust as needed)
// Ensure these are NOT conflicting with other sensor pins (like SPI pins if using hardware SPI)
const int THERMO_DO_PINS[2]  = { 4,  8 }; // Data Out pins
const int THERMO_CS_PINS[2]  = { 5,  7 }; // Chip Select pins (each sensor needs a unique CS)
const int THERMO_CLK_PINS[2] = { 6,  6 }; // Clock pins (can often share the CLK pin)
const int NUM_TEMP_SENSORS = sizeof(THERMO_DO_PINS) / sizeof(THERMO_DO_PINS[0]); // Should be 2

const float FAHRENHEIT_SLOPE = 9.0f / 5.0f; // Defined here
const float FAHRENHEIT_OFFSET = 32.0f;     // Defined here

MAX6675 thermocouples[2] = { // Defined here (array of objects)
    MAX6675(THERMO_CLK_PINS[0], THERMO_CS_PINS[0], THERMO_DO_PINS[0]),
    MAX6675(THERMO_CLK_PINS[1], THERMO_CS_PINS[1], THERMO_DO_PINS[1])
};


// Add definitions for other sensor constants here
/*
const int OTHER_SENSOR_INPUTS[2] = { ... };
const int NUM_OTHER_SENSORS = sizeof(OTHER_SENSOR_INPUTS) / sizeof(OTHER_SENSOR_INPUTS[0]);
*/


// --- Binary Protocol Constants ---
const byte PRESSURE_PACKET_START_BYTE = 0xAA;
const byte PRESSURE_PACKET_END_BYTE = 0x55;
const byte LOADCELL_PACKET_START_BYTE = 0xBB;
const byte LOADCELL_PACKET_END_BYTE = 0x66;
const byte FLOW_PACKET_START_BYTE = 0xCC;
const byte FLOW_PACKET_END_BYTE = 0xDD;
const byte TEMP_PACKET_START_BYTE = 0xEE; // Example byte pair
const byte TEMP_PACKET_END_BYTE = 0xFF;   // Example byte pair


// Define ID ranges and number of IDs for each sensor type
const byte PRESSURE_ID_START = 0;
const byte NUM_IDS_PRESSURE = NUM_PRESSURE_SENSORS;

const byte LOADCELL_ID_START = PRESSURE_ID_START + NUM_IDS_PRESSURE; // IDs 6, 7
const byte NUM_IDS_LOADCELL = NUM_LOADCELL_SENSORS;

const byte FLOW_SENSOR_ID = LOADCELL_ID_START + NUM_IDS_LOADCELL; // Single ID for the flow sensor (e.g., 8)
const byte NUM_IDS_FLOW = 1;

const byte TEMP_ID_START = FLOW_SENSOR_ID + NUM_IDS_FLOW;     // Temp IDs start after flow (e.g., 8 + 1 = 9)
const byte NUM_IDS_TEMP = NUM_TEMP_SENSORS;

// Add constants for other sensor types here (packet markers, ID starts, num IDs)
/*
const byte OTHER_PACKET_START_BYTE = 0x01; // Example byte pair
const byte OTHER_PACKET_END_BYTE = 0x02;   // Example byte pair
const byte OTHER_ID_START = TEMP_ID_START + NUM_IDS_TEMP; // Offset from previous group
const byte NUM_IDS_OTHER = NUM_OTHER_SENSORS;
*/


// --- State Machine / Round-Robin Variables and Constants ---

// Pressure Sensor State
int currentPressureSensorIndex = 0;
unsigned long lastPressureSensorProcessTime = 0;
const unsigned long MIN_PRESSURE_INTERVAL_MS = 10;

// Load Cell Sensor State
int currentLoadCellIndex = 0;
unsigned long lastLoadCellProcessTime = 0;
const unsigned long MIN_LOADCELL_CHECK_INTERVAL_MS = 150;

// Flow Sensor State
volatile long flow_pulse = 0;
long flow_pulseLast = 0;
unsigned long lastFlowProcessTime = 0;
const unsigned long FLOW_CALCULATION_INTERVAL_MS = 1000;

// Temperature Sensor State (for the two sensors)
int currentTempSensorIndex = 0; // Which temp sensor to process next
unsigned long lastTempProcessTime = 0; // Time last temp sensor was processed
// MAX6675 requires >= 250ms between reads. Use a value >= 250ms for interval.
const unsigned long MIN_TEMP_INTERVAL_MS = 500; // Process one temp sensor every 500ms (approx)


// Add state variables and constants for other sensor types here
/*
int currentOtherSensorIndex = 0;
unsigned long lastOtherSensorProcessTime = 0;
const unsigned long MIN_OTHER_INTERVAL_MS = 50; // Example interval
*/


// =======================================================
// === Function Definitions (Code Bodies) ================
// =======================================================

// --- Calculation Function for Pressure Sensor ---
PressureSensorValues calculatePressureSensorValues(int raw_pressure_int, int index) {
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

// --- Calculation Function for Flow Sensor ---
// Takes current total pulse count and the count from the previous interval start
FlowMeterValues calculateFlowMeterValues(long currentPulseCount, long previousPulseCount) {
    // Calculate pulses accumulated during the last interval
    long delta_pulse = currentPulseCount - previousPulseCount;

    // Calculate flow rate using the pre-calculated factor
    float lpm = (float)delta_pulse * PULSES_TO_LPM_FACTOR;

    return {lpm};
}

// --- Calculation Function for Temperature Sensor (MAX6675) ---
// Takes the sensor index, reads the specific sensor, and returns values
TemperatureSensorValues calculateTemperatureSensorValues(int index) {
    if (index < 0 || index >= NUM_TEMP_SENSORS) {
        // Invalid index, return zero values
        return {0.0f, 0.0f};
    }

    // Get the specific MAX6675 object for this index
    MAX6675& currentThermocouple = thermocouples[index];

    // Read temperature from the sensor
    // This call will block until the MAX6675 conversion is ready (at least 250ms after last conversion started).
    // The State Machine timer MIN_TEMP_INTERVAL_MS should account for this.
    double celsius = currentThermocouple.readCelsius(); // Library returns double

    TemperatureSensorValues values;

    // Check for errors (isnan indicates open thermocouple or communication problem)
    if (isnan(celsius)) {
        // Handle error condition - perhaps send a specific error value or NaN
        // Sending NaN directly in binary might be interpreted correctly by some systems.
        values.temp_c = NAN; // Not a Number
        values.temp_f = NAN; // Not a Number
        Serial.print(F("Warning: MAX6675 ID ")); // Debug print on serial
        Serial.print(TEMP_ID_START + index);
        Serial.println(F(" - Thermocouple open!"));

    } else {
        // Convert Celsius to Fahrenheit using pre-calculated factors
        double fahrenheit = celsius * FAHRENHEIT_SLOPE + FAHRENHEIT_OFFSET;

        // Store the calculated values in the struct
        values.temp_c = (float)celsius;   // Convert double back to float for the struct
        values.temp_f = (float)fahrenheit; // Convert double back to float for the struct
    }

    return values;
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

void setupFlowSensors() {
  Serial.println(F("Setting up Flow Sensor..."));
  pinMode(FLOW_SENSOR_PIN_MEGA, INPUT);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN_MEGA), flow_increase_pulse, RISING);

  // Initialize flow sensor state variables
  flow_pulse = 0;
  flow_pulseLast = 0;
  lastFlowProcessTime = millis(); // Initialize timer

  Serial.println(F("Flow Sensor setup complete."));
}

void setupTemperatureSensors() {
  Serial.println(F("Setting up Temperature Sensors (MAX6675)..."));
  // The MAX6675 library's begin() function is often just setting up pins.
  // The important delay is after power-up and between reads.
  // Our loop timing will handle the 250ms delay between reads.
  // No specific initialization needed for the objects other than their definition in global scope.

  // Power-up delay for the first reading might be needed if chip power state is uncertain
  // delay(500); // Optional: Delay after chip power-up if needed

  // Initialize temp sensor state variables
  currentTempSensorIndex = 0;
  lastTempProcessTime = millis(); // Initialize timer

  Serial.print(NUM_TEMP_SENSORS); Serial.println(F(" Temperature Sensors setup complete."));
}


// Add setup functions for other sensor types here
/*
void setupOtherSensors() { ... }
*/

// --- Flow sensor Interrupt Service Routine (ISR) ---
void flow_increase_pulse() {
  flow_pulse++;
}
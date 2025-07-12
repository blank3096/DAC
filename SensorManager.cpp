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
const byte LOADCELL_DOUT_PINS[2] = {5, 10}; // Using D31 for LC0, D10 for LC1 (example, adjust if used)
const byte LOADCELL_CLK_PINS[2] = {4, 9};   // Using D30 for LC0, D9 for LC1 (example, adjust if used)
const float LOADCELL_CALIBRATION_FACTORS[2] = {145.4f, 150.0f};
const int NUM_LOADCELL_SENSORS = sizeof(LOADCELL_DOUT_PINS) / sizeof(LOADCELL_DOUT_PINS[0]);
HX711 scales[2];


// --- Flow Sensor Constants ---
const int FLOW_SENSOR_PIN_MEGA = 2; // Pin 2 on Mega is External Interrupt 0
const float FLOW_PPL = 4215;
const float PULSES_TO_LPM_FACTOR = 60.0f / FLOW_PPL;


// --- Temperature Sensor (MAX6675) Constants ---
// Example pins for two MAX6675 sensors on Mega (adjust as needed based on your wiring)
// Using pins 3, 6, 7 for TS0 and 8, 11, 12 for TS1 (example, verify availability)
const int THERMO_DO_PINS[2]  = { 3,  8 }; // Data Out pins
const int THERMO_CS_PINS[2]  = { 6, 11 }; // Chip Select pins (each sensor needs a unique CS)
const int THERMO_CLK_PINS[2] = { 7, 12 }; // Clock pins (using separate CLK pins is safer)
const int NUM_TEMP_SENSORS = sizeof(THERMO_DO_PINS) / sizeof(THERMO_DO_PINS[0]); // Should be 2

const float FAHRENHEIT_SLOPE = 9.0f / 5.0f;
const float FAHRENHEIT_OFFSET = 32.0f;

MAX6675 thermocouples[2] = { // Defined here (array of objects)
    MAX6675(THERMO_CLK_PINS[0], THERMO_CS_Pconst int THERMO_DO_PINS[2]  = { 3,  8 }; // Data Out pins
const int THERMO_CS_PINS[2]  = { 6, 11 }; // Chip Select pins (each sensor needs a unique CS)
const int THERMO_CLK_PINS[2] = { 7, 12 }; // Clock pins (using separate CLK pins is safer)INS[0], THERMO_DO_PINS[0]),
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
const byte TEMP_PACKET_START_BYTE = 0xEE;
const byte TEMP_PACKET_END_BYTE = 0xFF;


// Define ID ranges and number of IDs for each sensor type
const byte PRESSURE_ID_START = 0;
const byte NUM_IDS_PRESSURE = NUM_PRESSURE_SENSORS;

const byte LOADCELL_ID_START = PRESSURE_ID_START + NUM_IDS_PRESSURE;
const byte NUM_IDS_LOADCELL = NUM_LOADCELL_SENSORS;

const byte FLOW_SENSOR_ID = LOADCELL_ID_START + NUM_IDS_LOADCELL;
const byte NUM_IDS_FLOW = 1;

const byte TEMP_ID_START = FLOW_SENSOR_ID + NUM_IDS_FLOW;
const byte NUM_IDS_TEMP = NUM_TEMP_SENSORS;

// Add constants for other sensor types here
/*
const byte OTHER_PACKET_START_BYTE = 0x01;
const byte OTHER_PACKET_END_BYTE = 0x02;
const byte OTHER_ID_START = TEMP_ID_START + NUM_IDS_TEMP;
const byte NUM_IDS_OTHER = NUM_OTHER_SENSORS;
*/


// --- State Machine / Round-Robin Variables and Constants ---
int currentPressureSensorIndex = 0;
unsigned long lastPressureSensorProcessTime = 0;
const unsigned long MIN_PRESSURE_INTERVAL_MS = 10;

int currentLoadCellIndex = 0;
unsigned long lastLoadCellProcessTime = 0;
const unsigned long MIN_LOADCELL_CHECK_INTERVAL_MS = 150;

volatile long flow_pulse = 0;
long flow_pulseLast = 0;
unsigned long lastFlowProcessTime = 0;
const unsigned long FLOW_CALCULATION_INTERVAL_MS = 1000;

int currentTempSensorIndex = 0;
unsigned long lastTempProcessTime = 0;
const unsigned long MIN_TEMP_INTERVAL_MS = 500;


// Add state variables and constants for other sensor types here
/*
int currentOtherSensorIndex = 0;
unsigned long lastOtherSensorProcessTime = 0;
const unsigned long MIN_OTHER_INTERVAL_MS = 50;
*/


// --- Timing Helper Functions ---
unsigned long _timerStartTime = 0; // Defined here

void startTimer() {
  _timerStartTime = micros();
}

void printElapsedTime(const char* description) {
  unsigned long elapsed = micros() - _timerStartTime;
  Serial.print(F("Time for "));
  Serial.print(description);
  Serial.print(F(": "));
  Serial.print(elapsed);
  Serial.println(F(" us"));
}


// =======================================================
// === Function Definitions (Code Bodies) ================
// =======================================================

// --- Calculation Function for Pressure Sensor ---
// Now returns only the pressure value in the struct
PressureSensorValues calculatePressureSensorValues(int raw_pressure_int, int index) {
  // Access constants and scale_factor array defined above in this .cpp file
  if (index < 0 || index >= NUM_PRESSURE_SENSORS) {
      // Return a struct with a clear invalid/error value, e.g., negative pressure if not possible
      // Or NAN if preferred, but requires math.h
      return {-1.0f}; // Example: return -1.0f for invalid index
  }

  float raw_pressure_f = (float)raw_pressure_int;
  float mV = raw_pressure_f * MV_FACTOR;
  float volts = mV / 1000.0f;
  float mA = volts * MA_FACTOR;
  float percent = mA * PERCENT_SLOPE + PERCENT_OFFSET;
  float pressure = percent * pressure_scale_factor[index];

  return {pressure}; // Only return the pressure value
}

// --- Calculation Function for Load Cell Sensor ---
LoadCellValues calculateLoadCellValues(float raw_weight_float) {
    return {raw_weight_float};
}

// --- Calculation Function for Flow Sensor ---
FlowMeterValues calculateFlowMeterValues(long currentPulseCount, long previousPulseCount) {
    long delta_pulse = currentPulseCount - previousPulseCount;
    float lpm = (float)delta_pulse * PULSES_TO_LPM_FACTOR;
    return {lpm};
}

// --- Calculation Function for Temperature Sensor (MAX6675) ---
TemperatureSensorValues calculateTemperatureSensorValues(int index) {
    if (index < 0 || index >= NUM_TEMP_SENSORS) {
        return {0.0f, 0.0f};
    }

    MAX6675& currentThermocouple = thermocouples[index];
    double celsius = currentThermocouple.readCelsius(); // Library returns double

    TemperatureSensorValues values;

    if (isnan(celsius)) {
        values.temp_c = NAN;
        values.temp_f = NAN;
        // Serial.print(F("Warning: MAX6675 ID ")); Serial.print(TEMP_ID_START + index); Serial.println(F(" - Thermocouple open!")); // Debug print
    } else {
        double fahrenheit = celsius * FAHRENHEIT_SLOPE + FAHRENHEIT_OFFSET;
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
   if (data_ptr == nullptr || data_size == 0) return; // Corrected condition

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

    scales[i].set_scale(LOADCELL_CALIBRATION_FACTORS[i]); // Apply calibration factor


    Serial.print(F("Load Cell ")); Serial.print(i + 1);
    Serial.println(F(": Please place initial load on the scale NOW."));
    Serial.println(F("Taring in 5 seconds..."));
    delay(5000); // Wait for 5 seconds to allow user to place the load
    // --- END ADDED ---

    scales[i].tare(); // Tare the scale - this sets the current reading (with load) to zero

    Serial.print(F("Load Cell ")); Serial.print(i + 1); Serial.println(F(" tared with initial load."));
    delay(50); // Small delay between taring different sensors
  }
  Serial.print(NUM_LOADCELL_SENSORS); Serial.println(F(" Load Cells setup complete."));
}
void setupFlowSensors() {
  Serial.println(F("Setting up Flow Sensor..."));
  pinMode(FLOW_SENSOR_PIN_MEGA, INPUT);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN_MEGA), flow_increase_pulse, RISING);

  flow_pulse = 0;
  flow_pulseLast = 0;
  lastFlowProcessTime = millis();

  Serial.println(F("Flow Sensor setup complete."));
}

void setupTemperatureSensors() {
  Serial.println(F("Setting up Temperature Sensors (MAX6675)..."));
  // Objects are defined globally, initialized with pins.
  // delay(500); // Optional power-up delay

  currentTempSensorIndex = 0;
  lastTempProcessTime = millis();

  Serial.print(NUM_TEMP_SENSORS); Serial.println(F(" Temperature Sensors setup complete."));
}



// --- Flow sensor Interrupt Service Routine (ISR) ---
void flow_increase_pulse() {
  flow_pulse++;
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
  startTimer();
  PressureSensorValues pData = calculatePressureSensorValues(raw_p, 0); // Calculate for sensor 0
  sendBinaryPacket(PRESSURE_PACKET_START_BYTE, PRESSURE_ID_START, &pData, sizeof(pData), PRESSURE_PACKET_END_BYTE); // Send for sensor 0
  printElapsedTime("One Pressure Sensor Block");

  // Measure time for one Load Cell block execution (simulating a read)
  // This will likely block for ~100ms waiting for the sensor to be ready if it just tared/read.
  startTimer();
  // Get the first Load Cell object
  HX711& testScale = scales[0];
  // Read, Calculate, Send for load cell 0.
  float raw_weight = testScale.get_units(); // This will wait for is_ready()
  LoadCellValues loadCellData = calculateLoadCellValues(raw_weight);
  byte loadCell_id = LOADCELL_ID_START + 0; // ID for load cell 0
  sendBinaryPacket(LOADCELL_PACKET_START_BYTE, loadCell_id, &loadCellData, sizeof(loadCellData), LOADCELL_PACKET_END_BYTE);
  printElapsedTime("One Load Cell Block (incl. get_units wait)");


  // Measure time for one Flow Sensor block execution (simulating update)
  // Simulate delta pulse for calculation test
  long dummyCurrentPulse = flow_pulse + 100; // Assume 100 pulses happened since flow_pulse was 0 in setup
  long dummyLastPulse = 0; // Since flow_pulseLast was 0 in setup
  startTimer();
  FlowMeterValues fData = calculateFlowMeterValues(dummyCurrentPulse, dummyLastPulse);
  sendBinaryPacket(FLOW_PACKET_START_BYTE, FLOW_SENSOR_ID, &fData, sizeof(fData), FLOW_PACKET_END_BYTE);
  printElapsedTime("One Flow Sensor Calc + Send");


  // Measure time for one Temp Sensor block execution (simulating a read)
  // This will call readCelsius() and potentially block for ~250ms if not ready.
  startTimer();
  TemperatureSensorValues tData = calculateTemperatureSensorValues(0); // Calculate for sensor 0
  sendBinaryPacket(TEMP_PACKET_START_BYTE, TEMP_ID_START, &tData, sizeof(tData), TEMP_PACKET_END_BYTE);
  printElapsedTime("One Temp Sensor Block (incl. readCelsius wait)");

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
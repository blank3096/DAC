#include "SensorManager.h" // Include your own header first
#include <Arduino.h>     // Include Arduino.h here too, as this is a compilation unit
// Add includes for other libraries needed for implementation (e.g., Wire.h)


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
const int NUM_PRESSURE_SENSORS = sizeof(PRESSURE_SENSOR_PINS) / sizeof(PRESSURE_SENSOR_PINS[0]);

const float MV_FACTOR = (float)ANALOG_REFERENCE_mV / 1024.0f;

// --- Pre-calculated constants for faster floating-point math (Pressure - Method 1 style) ---
const float MA_FACTOR = 1000.0f / SHUNT_OHM;

float pressure_scale_factor[6];


// --- Load Cell Constants ---
const byte LOADCELL_DOUT_PINS[2] = {12, 10};
const byte LOADCELL_CLK_PINS[2] = {11, 9};
const float LOADCELL_CALIBRATION_FACTORS[2] = {145.4f, 150.0f};
const int NUM_LOADCELL_SENSORS = sizeof(LOADCELL_DOUT_PINS) / sizeof(LOADCELL_DOUT_PINS[0]);

HX711 scales[2];


// --- Flow Sensor Constants ---
const int FLOW_SENSOR_PIN = 25; // Digital pin connected to the flow sensor output (needs to be interrupt capable!)
// On Mega, pins 2, 3, 18, 19, 20, 21 are external interrupt pins. Pin 25 is NOT a hardware interrupt pin.
// Let's change this to one of the Mega's interrupt pins for reliable pulse counting.
// Using Pin 2 for example. Update your wiring accordingly.
// const int FLOW_SENSOR_PIN = 2; // *** REMEMBER TO WIRE TO PIN 2 ***

// NOTE: If you *must* use pin 25 and it's NOT an interrupt pin, you cannot use attachInterrupt.
// You would have to use pin change interrupts (more complex) or poll the pin VERY rapidly in loop()
// (which would block other tasks). Using a hardware interrupt pin is highly recommended for pulse counting.
// Let's use Pin 2 for this code example based on Mega's capability.
const int FLOW_SENSOR_PIN_MEGA = 2; // Use Pin 2 on Mega for external interrupt 0

const float FLOW_PPL = 4215; // Pulses per liter calibration factor

// Pre-calculated factor for flow calculation
const float PULSES_TO_LPM_FACTOR = 60.0f / FLOW_PPL; // Use float literal 60.0f


// --- Binary Protocol Constants ---
const byte PRESSURE_PACKET_START_BYTE = 0xAA;
const byte PRESSURE_PACKET_END_BYTE = 0x55;
const byte LOADCELL_PACKET_START_BYTE = 0xBB;
const byte LOADCELL_PACKET_END_BYTE = 0x66;
const byte FLOW_PACKET_START_BYTE = 0xCC; // Example byte pair
const byte FLOW_PACKET_END_BYTE = 0xDD;   // Example byte pair


// Define ID ranges for each sensor type
const byte PRESSURE_ID_START = 0;
const byte NUM_IDS_PRESSURE = NUM_PRESSURE_SENSORS; // Number of IDs for this type

const byte LOADCELL_ID_START = PRESSURE_ID_START + NUM_IDS_PRESSURE; // IDs 6, 7
const byte NUM_IDS_LOADCELL = NUM_LOADCELL_SENSORS;

const byte FLOW_SENSOR_ID = LOADCELL_ID_START + NUM_IDS_LOADCELL; // Single ID for the flow sensor (e.g., 8)
const byte NUM_IDS_FLOW = 1; // Just one flow sensor in this example


// Add definitions for other sensor constants here (packet markers, ID starts)
/*
const int TEMP_SENSOR_PINS[2] = { ... };
const int NUM_TEMP_SENSORS = ...;
const byte TEMP_PACKET_START_BYTE = 0xEE;
const byte TEMP_PACKET_END_BYTE = 0xFF;
const byte TEMP_ID_START = FLOW_SENSOR_ID + NUM_IDS_FLOW;
const byte NUM_IDS_TEMP = NUM_TEMP_SENSORS;
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

// Flow Sensor State (for the single sensor)
volatile long flow_pulse = 0; // Defined here and initialized
long flow_pulseLast = 0;      // Defined here and initialized
unsigned long lastFlowProcessTime = 0; // Defined here and initialized
// How often to CALCULATE and SEND the flow rate for the single sensor
const unsigned long FLOW_CALCULATION_INTERVAL_MS = 1000; // Example: every 1 second


// Add definitions for state variables and constants for other sensor types here
/*
int currentTempSensorIndex = 0;
unsigned long lastTempSensorProcessTime = 0;
const unsigned long MIN_TEMP_INTERVAL_MS = 500;
*/


// =======================================================
// === Function Definitions (Code Bodies) ================
// === (Matching prototypes in SensorManager.h) ==========
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
// Takes the current total pulse count and the count from the previous interval start
FlowMeterValues calculateFlowMeterValues(long currentPulseCount, long previousPulseCount) {
    // Calculate pulses accumulated during the last interval
    long delta_pulse = currentPulseCount - previousPulseCount;

    // Calculate flow rate using the pre-calculated factor
    float lpm = (float)delta_pulse * PULSES_TO_LPM_FACTOR;

    return {lpm};
}


// Add calculation functions for other sensor types here
/*
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

void setupFlowSensors() {
  Serial.println(F("Setting up Flow Sensor..."));
  // Configure the pin as an input
  pinMode(FLOW_SENSOR_PIN_MEGA, INPUT);

  // Attach the interrupt to the pin.
  // digitalPinToInterrupt(pin) maps the digital pin number to the specific interrupt number on the chip.
  // flow_increase_pulse is the ISR function to call.
  // RISING means trigger the interrupt on the rising edge of the pulse.
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN_MEGA), flow_increase_pulse, RISING);

  // Initialize flow sensor state variables
  flow_pulse = 0; // Reset the counter
  flow_pulseLast = 0; // Reset the last count
  lastFlowProcessTime = millis(); // Initialize the timer

  Serial.println(F("Flow Sensor setup complete."));
}


// Add setup functions for other sensor types here
/*
void setupTemperatureSensors() { ... }
void setupOtherSensors() { ... }
*/

// --- Flow sensor Interrupt Service Routine (ISR) ---
// This function is called by the microcontroller hardware whenever the interrupt condition occurs on the configured pin.
// It MUST be very short and fast!
void flow_increase_pulse() {
  // Simply increment the volatile pulse counter.
  // Incrementing a 'long' might take a few cycles, but disabling/re-enabling interrupts in the ISR itself
  // is generally discouraged unless absolutely necessary and you know what you're doing.
  // Safely reading the 'long' in the main loop is the standard approach.
  flow_pulse++;
}
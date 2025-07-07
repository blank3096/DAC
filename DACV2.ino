#include "Arduino.h"
#include "SensorManager.h" // Include your custom header file

void setup() {
  // 1. Basic Serial Communication Setup (usually done first)
  Serial.begin(115200); // High baud rate

  // Add setup for other serial ports, I2C, SPI, etc. if needed before specific sensors
  // Example: Wire.begin(); // For I2C sensors

  // Give Serial Monitor time to connect
  delay(2000);
  Serial.println(F("--- System Setup Starting ---"));

  // 2. Call Modular Setup Functions for each sensor type
  setupPressureSensors();
  setupLoadCells();

  // Call setup functions for other sensor types here
  /*
  setupFlowMeters();
  setupTemperatureSensors();
  setupOtherSensors();
  */

  Serial.println(F("--- System Setup Complete. Starting loop ---"));
}

void loop() {
  unsigned long currentMillis = millis(); // Get the current time *once* per loop iteration

  // --- State Machine Logic for Pressure Sensors ---
  // Process one pressure sensor when its interval has passed
  if (currentMillis - lastPressureSensorProcessTime >= MIN_PRESSURE_INTERVAL_MS) {
    lastPressureSensorProcessTime = currentMillis; // Update timer

    // Read, Calculate, Send for the current pressure sensor
    int raw_pressure_int = analogRead(PRESSURE_SENSOR_PINS[currentPressureSensorIndex]);
    PressureSensorValues pressureData = calculatePressureSensorValues(raw_pressure_int, currentPressureSensorIndex);
    byte pressure_id = PRESSURE_ID_START + currentPressureSensorIndex; // Use ID offset + index (0-5)
    sendBinaryPacket(PRESSURE_PACKET_START_BYTE, pressure_id, &pressureData, sizeof(pressureData), PRESSURE_PACKET_END_BYTE);

    // Move to the next pressure sensor
    currentPressureSensorIndex++;
    if (currentPressureSensorIndex >= NUM_PRESSURE_SENSORS) {
      currentPressureSensorIndex = 0; // Wrap around
    }
  }

  // --- State Machine Logic for Load Cells ---
  // Attempt to process one load cell when its interval has passed
   if (currentMillis - lastLoadCellProcessTime >= MIN_LOADCELL_CHECK_INTERVAL_MS) {
       lastLoadCellProcessTime = currentMillis; // Update timer

       // Get the HX711 object for the current load cell
       // Access the scales array which is defined in SensorManager.cpp
       // and declared extern in SensorManager.h
       HX711& currentScale = scales[currentLoadCellIndex];

       // Check if the HX711 is ready (CRUCIAL to avoid blocking)
       if (currentScale.is_ready()) {
           // Read, Calculate, Send for the current load cell (if ready)
           float raw_weight = currentScale.get_units(); // Might block briefly if ready state is marginal
           LoadCellValues loadCellData = calculateLoadCellValues(raw_weight);
           byte loadCell_id = LOADCELL_ID_START + currentLoadCellIndex; // Use ID offset + index (6, 7)
           sendBinaryPacket(LOADCELL_PACKET_START_BYTE, loadCell_id, &loadCellData, sizeof(loadCellData), LOADCELL_PACKET_END_BYTE);

           // Move to the next load cell ONLY if a successful read happened
           currentLoadCellIndex++;
           if (currentLoadCellIndex >= NUM_LOADCELL_SENSORS) {
             currentLoadCellIndex = 0; // Wrap around
           }
       }
       // If not ready, the timer was reset, but the index was NOT incremented.
       // The next time this block fires, we check the same sensor again.
   }

  // --- Add State Machine Logic blocks for other sensor types here ---
  /*
  // Example for Flow Meters:
  unsigned long currentMillis_Flow = millis();
  if (currentMillis_Flow - lastFlowMeterProcessTime >= MIN_FLOW_INTERVAL_MS) {
    lastFlowMeterProcessTime = currentMillis_Flow;
    // Read raw flow data for currentFlowMeterIndex (e.g., count pulses, analogRead)
    // FlowMeterValues flowData = calculateFlowMeterValues(...);
    // byte flow_id = FLOW_ID_START + currentFlowMeterIndex; // Define FLOW_ID_START
    // sendBinaryPacket(FLOW_PACKET_START_BYTE, flow_id, &flowData, sizeof(flowData), FLOW_PACKET_END_BYTE);
    // currentFlowMeterIndex++; if (currentFlowMeterIndex >= NUM_FLOW_METERS) currentFlowMeterIndex = 0;
  }
  // ... etc for Temperature, Other sensors
  */

  // --- Incoming Control Signal Handling Block ---
  // Implement checking Serial.available() and parsing logic here.
  // This runs every loop() iteration, providing responsiveness.
  // (See previous explanation for details)
  // while (Serial.available() > 0) { ... read and parse command ... }


}

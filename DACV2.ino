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
  setupFlowSensors();
  setupTemperatureSensors(); // Add the new temperature sensor setup

  // Call setup functions for other sensor types here
  /*
  setupOtherSensors(); // If you have other types defined
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

  // --- State Machine Logic for Flow Sensor ---
  // Calculate and send flow rate periodically for the single flow sensor
   if (currentMillis - lastFlowProcessTime >= FLOW_CALCULATION_INTERVAL_MS) {
       lastFlowProcessTime = currentMillis; // Update timer

       // Safely read the current pulse count from the volatile variable
       long currentPulseCount;
       noInterrupts(); // Disable interrupts
       currentPulseCount = flow_pulse; // Read the volatile counter
       interrupts();   // Re-enable interrupts

       // Calculate delta and update last count
       long delta_pulse = currentPulseCount - flow_pulseLast;
       flow_pulseLast = currentPulseCount; // Update for the next interval

       // Calculate flow rate (LPM), package, and send
       FlowMeterValues flowData = calculateFlowMeterValues(currentPulseCount, flow_pulseLast); // Pass delta internally in calc function
       byte flow_id = FLOW_SENSOR_ID; // Unique ID for this flow sensor (e.g., 8)
       sendBinaryPacket(
         FLOW_PACKET_START_BYTE,     // Start byte for flow packets
         flow_id,                  // Unique ID for this flow sensor
         &flowData,                  // Pointer to the calculated data struct
         sizeof(flowData),           // Size of the data struct
         FLOW_PACKET_END_BYTE        // End byte for flow packets
       );
   }

  // --- State Machine Logic for Temperature Sensors (MAX6675) ---
  // Process one temperature sensor when its interval has passed
  if (currentMillis - lastTempProcessTime >= MIN_TEMP_INTERVAL_MS) {
    lastTempProcessTime = currentMillis; // Update timer

    // Read, Calculate, Send for the current temperature sensor (currentTempSensorIndex)
    // Note: The calculate function will handle the MAX6675's internal 250ms delay if needed.
    TemperatureSensorValues tempData = calculateTemperatureSensorValues(currentTempSensorIndex);

    // Use ID offset + index for temperature sensors (e.g., 9, 10)
    byte temp_id = TEMP_ID_START + currentTempSensorIndex;

    sendBinaryPacket(
      TEMP_PACKET_START_BYTE, // Start byte for temperature packets
      temp_id,                // ID for this specific sensor (9 or 10)
      &tempData,              // Pointer to the calculated data struct
      sizeof(tempData),       // Size of the data struct
      TEMP_PACKET_END_BYTE    // End byte for temperature packets
    );

    // Move to the next temperature sensor
    currentTempSensorIndex++;
    if (currentTempSensorIndex >= NUM_TEMP_SENSORS) {
      currentTempSensorIndex = 0; // Wrap around
    }
  }


  // --- Add State Machine Logic blocks for other sensor types here ---
  /*
  // Example for Other sensors:
  unsigned long currentMillis_Other = millis();
  if (currentMillis_Other - lastOtherSensorProcessTime >= MIN_OTHER_INTERVAL_MS) {
    lastOtherSensorProcessTime = currentMillis_Other;
    // Read raw other data for currentOtherSensorIndex
    // OtherSensorValues otherData = calculateOtherSensorValues(...);
    // byte other_id = OTHER_ID_START + currentOtherSensorIndex;
    // sendBinaryPacket(OTHER_PACKET_START_BYTE, other_id, &otherData, sizeof(otherData), OTHER_PACKET_END_BYTE);
    // currentOtherSensorIndex++; if (currentOtherSensorIndex >= NUM_OTHER_SENSORS) currentOtherSensorIndex = 0;
  }
  */

  // --- Incoming Control Signal Handling Block ---
  // Implement checking Serial.available() and parsing logic here.
  // This runs every loop() iteration, providing responsiveness.
  // (See previous explanation for details)
  // while (Serial.available() > 0) { ... read and parse command ... }


}
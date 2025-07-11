#include "Arduino.h"
#include "SensorManager.h" // Include your custom header file

void setup() {
  Serial.begin(115200);

  // Add setup for other serial ports, I2C, SPI, etc. if needed
  // Example: Wire.begin(); // For I2C sensors

  delay(2000); // Give Serial Monitor time to connect
  Serial.println(F("--- System Setup Starting ---"));

  setupPressureSensors();
  setupLoadCells();
  setupFlowSensors();
  setupTemperatureSensors();

  // Call setup functions for other sensor types here
  /*
  setupOtherSensors();
  */

  Serial.println(F("--- System Setup Complete. Starting loop ---"));

  // --- Run timing tests once after setup ---
  delay(1000); // Short delay before tests
  Serial.println(F("\n--- Running Initial Timing Tests ---"));

  // Test 1: Time for one full iteration of the State Machine loop (should be very fast if no blocks fire)
  // Note: This will fluctuate depending on which sensor blocks _do_ fire in that specific loop iteration.
  // A more reliable test is Test Batch.
  /*
  startTimer();
  // Let loop() run one time... this is tricky because loop runs continuously.
  // We'll measure the time _around_ the code in loop, but the loop itself never finishes.
  // The best way to measure the loop _overhead_ is to have an empty loop or very short timed tasks.
  // Let's skip this test and rely on the batch tests and per-block timings.
  */

  // Test 2: Time for a batch execution of ONE block from EACH sensor type.
  // This simulates the time needed to process one sample from each type back-to-back.
  testTimingBatchAllTypes();

  Serial.println(F("\n--- Initial Timing Tests Complete. Entering Main Loop ---"));
  delay(1000); // Delay before starting the continuous loop
}

void loop() {
  // --- Measure the time for this specific loop iteration ---
  startTimer(); // Start timer at the beginning of loop

  unsigned long currentMillis = millis(); // Get the current time

  // --- State Machine Logic for Pressure Sensors ---
  if (currentMillis - lastPressureSensorProcessTime >= MIN_PRESSURE_INTERVAL_MS) {
    startTimer(); // Start timer for THIS block
    lastPressureSensorProcessTime = currentMillis; // Update timer

    int raw_pressure_int = analogRead(PRESSURE_SENSOR_PINS[currentPressureSensorIndex]);
    // calculatePressureSensorValues now returns PressureSensorValues { float pressure; }
    PressureSensorValues pressureData = calculatePressureSensorValues(raw_pressure_int, currentPressureSensorIndex);
    byte pressure_id = PRESSURE_ID_START + currentPressureSensorIndex; // Use ID offset + index (0-5)
    sendBinaryPacket(PRESSURE_PACKET_START_BYTE, pressure_id, &pressureData, sizeof(pressureData), PRESSURE_PACKET_END_BYTE); // sizeof(pressureData) is now 4

    currentPressureSensorIndex++;
    if (currentPressureSensorIndex >= NUM_PRESSURE_SENSORS) {
      currentPressureSensorIndex = 0; // Wrap around
    }
    printElapsedTime("Pressure Sensor Block"); // Print time for THIS block
  }

  // --- State Machine Logic for Load Cells ---djfk
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

  // --- State Machine Logic for Flow Sensor ---
   if (currentMillis - lastFlowProcessTime >= FLOW_CALCULATION_INTERVAL_MS) {
       startTimer(); // Start timer for THIS block
       lastFlowProcessTime = currentMillis; // Update timer
       long currentPulseCount;
       noInterrupts();
       currentPulseCount = flow_pulse;
       interrupts();
       long delta_pulse = currentPulseCount - flow_pulseLast;
       flow_pulseLast = currentPulseCount;
       FlowMeterValues flowData = calculateFlowMeterValues(currentPulseCount, flow_pulseLast);
       byte flow_id = FLOW_SENSOR_ID;
       sendBinaryPacket(FLOW_PACKET_START_BYTE, flow_id, &flowData, sizeof(flowData), FLOW_PACKET_END_BYTE);
       printElapsedTime("Flow Sensor Block"); // Print time for THIS block
   }

  // --- State Machine Logic for Temperature Sensors (MAX6675) ---
  if (currentMillis - lastTempProcessTime >= MIN_TEMP_INTERVAL_MS) {
    startTimer(); // Start timer for THIS block
    lastTempProcessTime = currentMillis; // Update timer
    TemperatureSensorValues tempData = calculateTemperatureSensorValues(currentTempSensorIndex);
    byte temp_id = TEMP_ID_START + currentTempSensorIndex;
    sendBinaryPacket(TEMP_PACKET_START_BYTE, temp_id, &tempData, sizeof(tempData), TEMP_PACKET_END_BYTE);
    currentTempSensorIndex++;
    if (currentTempSensorIndex >= NUM_TEMP_SENSORS) {
      currentTempSensorIndex = 0;
    }
    printElapsedTime("Temp Sensor Block (incl. readCelsius wait)"); // Print time for THIS block
  }

  // --- Add State Machine Logic blocks for other sensor types here ---
  /*
  unsigned long currentMillis_Other = millis();
  if (currentMillis_Other - lastOtherSensorProcessTime >= MIN_OTHER_INTERVAL_MS) {
    startTimer(); // Start timer for THIS block
    lastOtherSensorProcessTime = currentMillis_Other;
    // Read, Calculate, Send for currentOtherSensorIndex
    // OtherSensorValues otherData = calculateOtherSensorValues(...);
    // byte other_id = OTHER_ID_START + currentOtherSensorIndex;
    // sendBinaryPacket(OTHER_PACKET_START_BYTE, other_id, &otherData, sizeof(otherData), OTHER_PACKET_END_BYTE);
    // currentOtherSensorIndex++; if (currentOtherSensorIndex >= NUM_OTHER_SENSORS) currentOtherSensorIndex = 0;
    printElapsedTime("Other Sensor Block"); // Print time for THIS block
  }
  */

  // --- Incoming Control Signal Handling Block ---
  // Implement checking Serial.available() and parsing logic here.
  // startTimer(); // If you want to time the duration of processing all available bytes
  // while (Serial.available() > 0) { ... read and parse command ... }
  // If the while loop finishes and it actually read bytes: printElapsedTime("Control Signal Handling");

  // There is no overall loop timing print as it's not meaningful here.

}
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
  setupTemperatureSensors(); // Will now set up 4 sensors

  // Call setup functions for other sensor types here
  /*
  setupOtherSensors();
  */

  Serial.println(F("--- System Setup Complete. Starting loop ---"));

  // --- Run timing tests once after setup ---
  delay(1000); // Short delay before tests
  Serial.println(F("\n--- Running Initial Timing Tests ---"));

  testTimingBatchAllTypes();

  Serial.println(F("\n--- Initial Timing Tests Complete. Entering Main Loop ---"));
  delay(1000); // Delay before starting the continuous loop
}

void loop() {
  unsigned long currentMillis = millis(); // Get the current time

  // --- State Machine Logic for Pressure Sensors ---
  if (currentMillis - lastPressureSensorProcessTime >= MIN_PRESSURE_INTERVAL_MS) {
    startTimer();
    lastPressureSensorProcessTime = currentMillis;

    int raw_pressure_int = analogRead(PRESSURE_SENSOR_PINS[currentPressureSensorIndex]);
    // calculatePressureSensorValues now returns PressureSensorValues { float pressure; }
    PressureSensorValues pressureData = calculatePressureSensorValues(raw_pressure_int, currentPressureSensorIndex);
    byte pressure_id = PRESSURE_ID_START + currentPressureSensorIndex;
    sendBinaryPacket(PRESSURE_PACKET_START_BYTE, pressure_id, &pressureData, sizeof(pressureData), PRESSURE_PACKET_END_BYTE); // sizeof(pressureData) is 4

    currentPressureSensorIndex++;
    if (currentPressureSensorIndex >= NUM_PRESSURE_SENSORS) {
      currentPressureSensorIndex = 0;
    }
    printElapsedTime("Pressure Sensor Block");
  }

  // --- State Machine Logic for Load Cells ---
   if (currentMillis - lastLoadCellProcessTime >= MIN_LOADCELL_CHECK_INTERVAL_MS) {
       lastLoadCellProcessTime = currentMillis;

       HX711& currentScale = scales[currentLoadCellIndex];

       if (currentScale.is_ready()) {
           startTimer(); // Timer inside is_ready block
           float raw_weight = currentScale.get_units();
           LoadCellValues loadCellData = calculateLoadCellValues(raw_weight);
           byte loadCell_id = LOADCELL_ID_START + currentLoadCellIndex;
           sendBinaryPacket(LOADCELL_PACKET_START_BYTE, loadCell_id, &loadCellData, sizeof(loadCellData), LOADCELL_PACKET_END_BYTE);

           currentLoadCellIndex++;
           if (currentLoadCellIndex >= NUM_LOADCELL_SENSORS) {
             currentLoadCellIndex = 0;
           }
            printElapsedTime("Load Cell Block (read+calc+send)"); // Name reflects action
       }
       // If not ready, the timer was reset, and the index was NOT moved.
       // The next time this block fires, we check the same sensor again.
       // No 'else' branch timing print here, as it's covered by the loop's overall execution.
   }

  // --- State Machine Logic for Flow Sensor ---
   if (currentMillis - lastFlowProcessTime >= FLOW_CALCULATION_INTERVAL_MS) {
       startTimer();
       lastFlowProcessTime = currentMillis;

       long currentPulseCount;
       noInterrupts();
       currentPulseCount = flow_pulse;
       interrupts();

       long delta_pulse = currentPulseCount - flow_pulseLast;
       flow_pulseLast = currentPulseCount;

       FlowMeterValues flowData = calculateFlowMeterValues(currentPulseCount, flow_pulseLast);
       byte flow_id = FLOW_SENSOR_ID;
       sendBinaryPacket(FLOW_PACKET_START_BYTE, flow_id, &flowData, sizeof(flowData), FLOW_PACKET_END_BYTE);
       printElapsedTime("Flow Sensor Block");
   }

  // --- State Machine Logic for Temperature Sensors (MAX6675) ---
  if (currentMillis - lastTempProcessTime >= MIN_TEMP_INTERVAL_MS) {
    startTimer();
    lastTempProcessTime = currentMillis;

    TemperatureSensorValues tempData = calculateTemperatureSensorValues(currentTempSensorIndex);

    byte temp_id = TEMP_ID_START + currentTempSensorIndex;
    sendBinaryPacket(TEMP_PACKET_START_BYTE, temp_id, &tempData, sizeof(tempData), TEMP_PACKET_END_BYTE);

    currentTempSensorIndex++; // Will now cycle through 0, 1, 2, 3
    if (currentTempSensorIndex >= NUM_TEMP_SENSORS) { // NUM_TEMP_SENSORS is now 4
      currentTempSensorIndex = 0;
    }
    printElapsedTime("Temp Sensor Block (incl. readCelsius wait)");
  }

 }

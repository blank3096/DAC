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
  setupRelays();
  setupDCMotor();

  // Call setup functions for other sensor types here
  /*
  setupOtherSensors();
  */

  // Initialize the Serial Receive State Machine
  currentRxState = RX_WAITING_FOR_START; // <-- Initialize state variable

  Serial.println(F("--- System Setup Complete. Starting loop ---"));

  // --- Run timing tests once after setup ---
  delay(1000); // Short delay before tests
  Serial.println(F("\n--- Running Initial Timing Tests ---"));

  //testTimingBatchAllTypes(); // Call the test function

  Serial.println(F("\n--- Initial Timing Tests Complete. Entering Main Loop ---"));
  delay(1000); // Delay before starting the continuous loop
}

void loop() {
  unsigned long currentMillis = millis(); // Get the current time

  // --- Serial Receive State Machine Block ---
  // This block runs every loop() iteration to process any incoming serial bytes
  while (Serial.available() > 0) {
    byte incomingByte = Serial.read(); // Read the next byte from the buffer

    switch (currentRxState) {
      case RX_WAITING_FOR_START:
        if (incomingByte == COMMAND_START_BYTE) {
          currentRxState = RX_READING_TYPE;
          // Reset variables for the new command packet
          rxCommandType = 0;
          rxTargetId = 0;
          rxPayloadSize = 0;
          rxPayloadBytesRead = 0;
          // No need to clear rxPayloadBuffer here, it will be overwritten
          // Serial.println("RX: Found Start"); // Debug
        }
        // If not the start byte, stay in this state and discard the byte.
        break;

      case RX_READING_TYPE:
        rxCommandType = incomingByte;
        currentRxState = RX_READING_TARGET_ID;
        // Serial.print("RX: Read Type: "); Serial.println(rxCommandType); // Debug
        break;

      case RX_READING_TARGET_ID:
        rxTargetId = incomingByte;
        currentRxState = RX_READING_SIZE;
        // Serial.print("RX: Read Target ID: "); Serial.println(rxTargetId); // Debug
        break;

      case RX_READING_SIZE:
        rxPayloadSize = incomingByte; // This is the expected size of JUST the payload
        rxPayloadBytesRead = 0; // Reset payload counter

        // Validate the received payload size against the expected maximum
        if (rxPayloadSize > MAX_COMMAND_PAYLOAD_SIZE) {
             Serial.print(F("RX Error: Payload size too large (")); Serial.print(rxPayloadSize); Serial.print(F("). Max is ")); Serial.print(MAX_COMMAND_PAYLOAD_SIZE); Serial.println(F(". Resetting."));
             currentRxState = RX_WAITING_FOR_START; // Discard packet
        } else if (rxPayloadSize == 0) {
             // If payload size is 0, we're done reading payload. Go straight to END.
             currentRxState = RX_READING_END;
             // Serial.println("RX: Read Size 0, going to End"); // Debug
        } else {
            // Expecting payload bytes next
            currentRxState = RX_READING_PAYLOAD;
            // Serial.print("RX: Read Size: "); Serial.println(rxPayloadSize); // Debug
        }
        break;

      case RX_READING_PAYLOAD:
        // Store the incoming byte in the buffer
        rxPayloadBuffer[rxPayloadBytesRead] = incomingByte;
        rxPayloadBytesRead++;

        // Check if we have read all the expected payload bytes
        if (rxPayloadBytesRead == rxPayloadSize) {
          currentRxState = RX_READING_END;
          // Serial.println("RX: Payload Complete, going to End"); // Debug
        }
        break;

      case RX_READING_END:
        // We expect the Command End Byte
        if (incomingByte == COMMAND_END_BYTE) {
          // --- COMMAND PACKET SUCCESSFULLY RECEIVED AND VALIDATED! ---
          // Process the command
          // Serial.println("RX: Found End. Processing command..."); // Debug
          handleCommand(rxCommandType, rxTargetId, rxPayloadBuffer, rxPayloadSize); // Pass the stored components
        } else {
          // Protocol error! Unexpected byte where END byte should be.
          Serial.print(F("RX Error: Expected End Byte (")); Serial.print(COMMAND_END_BYTE, HEX); Serial.print(F(") but got (")); Serial.print(incomingByte, HEX); Serial.println(F("). Resetting."));
          // Discard the corrupted packet.
        }

        // --- Reset the state for the next packet regardless of success or failure ---
        currentRxState = RX_WAITING_FOR_START;
        // Serial.println("RX: State Reset to WAITING_FOR_START"); // Debug
        break;
    } // End switch(currentRxState)
  } // End while(Serial.available())


  // --- State Machine Logic for Pressure Sensors ---
  if (currentMillis - lastPressureSensorProcessTime >= MIN_PRESSURE_INTERVAL_MS) {
    unsigned long individualSensorStartTime = 0; // Local variable for individual sensor timing

    // Start category timer if this is the first sensor in the cycle
    if (currentPressureSensorIndex == 0) {
        pressureCategoryStartTime = micros();
    }

    // Start timer for the individual sensor operation
    byte pressure_id = PRESSURE_ID_START + currentPressureSensorIndex;
    startSensorTimer(pressure_id, &individualSensorStartTime); // Pass ID and address of local var

    // Perform sensor reading and calculation
    int raw_pressure_int = analogRead(PRESSURE_SENSOR_PINS[currentPressureSensorIndex]);
    PressureSensorValues pressureData = calculatePressureSensorValues(raw_pressure_int, currentPressureSensorIndex);

    // Send data (existing)
    sendBinaryPacket(PRESSURE_PACKET_START_BYTE, pressure_id, &pressureData, sizeof(pressureData), PRESSURE_PACKET_END_BYTE); // sizeof(pressureData) is 4

    // End timer for the individual sensor operation
    endSensorTimer(pressure_id, individualSensorStartTime, "Pressure Sensor Block"); // Pass start time back

    currentPressureSensorIndex++;
    if (currentPressureSensorIndex >= NUM_PRESSURE_SENSORS) {
      currentPressureSensorIndex = 0; // Wrap around
      // End category timer when the last sensor in the category is processed
      unsigned long categoryDuration = micros() - pressureCategoryStartTime;
      Serial.print(F("All Pressure Sensors Cycle Time: ")); Serial.print(categoryDuration); Serial.println(F(" us"));
      // Send category cycle time (optional, define a specific ID for this)
      SensorTiming categoryTiming = {TIMING_CATEGORY_CYCLE_ID, pressureCategoryStartTime, micros(), categoryDuration};
      sendTimingPacket(TIMING_CATEGORY_CYCLE_ID, &categoryTiming); // Use a distinct ID for cycle times
    }
    lastPressureSensorProcessTime = currentMillis; // Update for next interval
  }

  // --- State Machine Logic for Load Cells ---
   if (currentMillis - lastLoadCellProcessTime >= MIN_LOADCELL_CHECK_INTERVAL_MS) {
       lastLoadCellProcessTime = currentMillis; // Update timestamp for next check

       HX711& currentScale = scales[currentLoadCellIndex]; // Get reference to current scale

       if (currentScale.is_ready()) { // Check if data is available
           unsigned long individualSensorStartTime = 0; // Local variable for individual sensor timing

           // Start category timer if this is the first sensor in the cycle
           if (currentLoadCellIndex == 0) {
               loadCellCategoryStartTime = micros();
           }

           // Start timer for the individual sensor operation
           byte loadCell_id = LOADCELL_ID_START + currentLoadCellIndex;
           startSensorTimer(loadCell_id, &individualSensorStartTime);

           // Perform sensor reading and calculation
           float raw_weight = currentScale.get_units(); // Might block briefly
           LoadCellValues loadCellData = calculateLoadCellValues(raw_weight);

           // Send data (existing)
           sendBinaryPacket(LOADCELL_PACKET_START_BYTE, loadCell_id, &loadCellData, sizeof(loadCellData), LOADCELL_PACKET_END_BYTE);

           // End timer for the individual sensor operation
           endSensorTimer(loadCell_id, individualSensorStartTime, "Load Cell Block (read+calc+send)");

           currentLoadCellIndex++; // Move to the next load cell ONLY if a successful read happened
           if (currentLoadCellIndex >= NUM_LOADCELL_SENSORS) {
             currentLoadCellIndex = 0;
             // End category timer when the last sensor in the category is processed
             unsigned long categoryDuration = micros() - loadCellCategoryStartTime;
             Serial.print(F("All Load Cell Sensors Cycle Time: ")); Serial.print(categoryDuration); Serial.println(F(" us"));
             SensorTiming categoryTiming = {TIMING_CATEGORY_CYCLE_ID, loadCellCategoryStartTime, micros(), categoryDuration};
             sendTimingPacket(TIMING_CATEGORY_CYCLE_ID, &categoryTiming);
           }
       }
   }

  // --- State Machine Logic for Flow Sensor ---
   if (currentMillis - lastFlowProcessTime >= FLOW_CALCULATION_INTERVAL_MS) {
       unsigned long individualSensorStartTime = 0; // Local variable for individual sensor timing

       // For a single flow sensor, individual timing IS the category timing
       flowCategoryStartTime = micros(); // Start of the flow sensor's processing cycle

       byte flow_id = FLOW_SENSOR_ID;
       startSensorTimer(flow_id, &individualSensorStartTime); // Start timer for the flow sensor's operation

       elapsed_time = currentMillis - lastFlowProcessTime;
       Serial.print("the current elapsed time is ");
       Serial.print(elapsed_time);


       lastFlowProcessTime = currentMillis;

       long currentPulseCount;
       noInterrupts();
       currentPulseCount = flow_pulse;
       interrupts();

       Serial.print("the current pulse count ");
       Serial.print(currentPulseCount);

       long delta_pulse = currentPulseCount - flow_pulseLast;
       Serial.print("the current delta pulse ");
       Serial.print(delta_pulse);

       FlowMeterValues flowData = calculateFlowMeterValues(delta_pulse,elapsed_time);
       flow_pulseLast = currentPulseCount;

       sendBinaryPacket(FLOW_PACKET_START_BYTE, flow_id, &flowData, sizeof(flowData), FLOW_PACKET_END_BYTE);

       // End timer for the flow sensor's operation
       endSensorTimer(flow_id, individualSensorStartTime, "Flow Sensor Block");

       // End category timer (same as individual for single sensor)
       unsigned long categoryDuration = micros() - flowCategoryStartTime;
       Serial.print(F("All Flow Sensors Cycle Time: ")); Serial.print(categoryDuration); Serial.println(F(" us"));
       SensorTiming categoryTiming = {TIMING_CATEGORY_CYCLE_ID, flowCategoryStartTime, micros(), categoryDuration};
       sendTimingPacket(TIMING_CATEGORY_CYCLE_ID, &categoryTiming);
   }

  // --- State Machine Logic for Temperature Sensors (MAX6675) ---
  if (currentMillis - lastTempProcessTime >= MIN_TEMP_INTERVAL_MS) {
    unsigned long individualSensorStartTime = 0; // Local variable for individual sensor timing

    // Start category timer if this is the first sensor in the cycle
    if (currentTempSensorIndex == 0) {
        tempCategoryStartTime = micros();
    }

    // Start timer for the individual sensor operation
    byte temp_id = TEMP_ID_START + currentTempSensorIndex;
    startSensorTimer(temp_id, &individualSensorStartTime);

    // Perform sensor reading and calculation
    TemperatureSensorValues tempData = calculateTemperatureSensorValues(currentTempSensorIndex); // Cycles through 0, 1, 2, 3

    // Send data (existing)
    sendBinaryPacket(TEMP_PACKET_START_BYTE, temp_id, &tempData, sizeof(tempData), TEMP_PACKET_END_BYTE);

    // End timer for the individual sensor operation
    endSensorTimer(temp_id, individualSensorStartTime, "Temp Sensor Block (incl. readCelsius wait)");

    currentTempSensorIndex++; // Will now cycle through 0, 1, 2, 3
    if (currentTempSensorIndex >= NUM_TEMP_SENSORS) {
      currentTempSensorIndex = 0; // Wrap around
      // End category timer when the last sensor in the category is processed
      unsigned long categoryDuration = micros() - tempCategoryStartTime;
      Serial.print(F("All Temp Sensors Cycle Time: ")); Serial.print(categoryDuration); Serial.println(F(" us"));
      SensorTiming categoryTiming = {TIMING_CATEGORY_CYCLE_ID, tempCategoryStartTime, micros(), categoryDuration};
      sendTimingPacket(TIMING_CATEGORY_CYCLE_ID, &categoryTiming);
    }
    lastTempProcessTime = currentMillis;
  }

   // --- State Machine Logic for Motor RPM ---
   // This block currently uses the generic startTimer() and printElapsedTime().
   // If you want it to also send a SensorTiming packet, you'd adapt it like the others.
   if (currentMillis - lastMotorCalcTime >= MOTOR_CALCULATION_INTERVAL_MS) {
       unsigned long individualMotorStartTime = 0; // Local variable for motor timing
       byte motor_id = MOTOR_RPM_ID;
       startSensorTimer(motor_id, &individualMotorStartTime); // Use new timing function

       unsigned long interval_ms = currentMillis - lastMotorCalcTime; // Actual interval duration
       lastMotorCalcTime = currentMillis; // Update timer

       // --- Perform Motor RPM Calculation and Sending ---

       // 1. Safely read the current pulse count from the volatile variable
       unsigned long currentPulseCount;
       noInterrupts(); // Disable interrupts
       currentPulseCount = motor_pulse_count; // Read the volatile counter
       interrupts();   // Re-enable interrupts

       // 2. Calculate the number of pulses since the last check
       unsigned long delta_pulse = currentPulseCount - motor_last_pulse_count;

       // 3. Update motor_last_pulse_count for the next calculation interval
       motor_last_pulse_count = currentPulseCount;

       // 4. Calculate RPM
       MotorRPMValue mData = calculateMotorRPM(currentPulseCount, motor_last_pulse_count, interval_ms);

       // 5. Send the data using the GENERIC sender
       sendBinaryPacket(
         MOTOR_RPM_PACKET_START_BYTE, // Start byte for motor rpm packets
         MOTOR_RPM_ID,                // Unique ID for this motor RPM (14)
         &mData,                      // Pointer to the calculated data struct
         sizeof(mData),               // Size of the data struct
         MOTOR_RPM_PACKET_END_BYTE     // End byte for motor rpm packets
       );

       endSensorTimer(motor_id, individualMotorStartTime, "Motor RPM Block"); // Use new timing function
   }
}
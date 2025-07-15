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

 // testTimingBatchAllTypes();

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
    startTimer();
    lastPressureSensorProcessTime = currentMillis;

    int raw_pressure_int = analogRead(PRESSURE_SENSOR_PINS[currentPressureSensorIndex]);
    // calculatePressureSensorValues returns PressureSensorValues { float pressure; }
    PressureSensorValues pressureData = calculatePressureSensorValues(raw_pressure_int, currentPressureSensorIndex);
    byte pressure_id = PRESSURE_ID_START + currentPressureSensorIndex; // Use ID offset + index (0-5)
    sendBinaryPacket(PRESSURE_PACKET_START_BYTE, pressure_id, &pressureData, sizeof(pressureData), PRESSURE_PACKET_END_BYTE); // sizeof(pressureData) is 4

    currentPressureSensorIndex++;
    if (currentPressureSensorIndex >= NUM_PRESSURE_SENSORS) {
      currentPressureSensorIndex = 0; // Wrap around
    }
    printElapsedTime("Pressure Sensor Block");
  }

  // --- State Machine Logic for Load Cells ---
   if (currentMillis - lastLoadCellProcessTime >= MIN_LOADCELL_CHECK_INTERVAL_MS) {
       lastLoadCellProcessTime = currentMillis;

       HX711& currentScale = scales[currentLoadCellIndex]; // Cycles through 0, 1, 2

       if (currentScale.is_ready()) {
           startTimer(); // Timer inside is_ready block
           float raw_weight = currentScale.get_units(); // Might block briefly
           LoadCellValues loadCellData = calculateLoadCellValues(raw_weight);
           byte loadCell_id = LOADCELL_ID_START + currentLoadCellIndex; // Use ID offset + index (6, 7, 8)
           sendBinaryPacket(LOADCELL_PACKET_START_BYTE, loadCell_id, &loadCellData, sizeof(loadCellData), LOADCELL_PACKET_END_BYTE);

           currentLoadCellIndex++; // Move to the next load cell ONLY if a successful read happened
           if (currentLoadCellIndex >= NUM_LOADCELL_SENSORS) { // NUM_LOADCELL_SENSORS is now 3
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
       
       elapsed_time = currentMillis - lastFlowProcessTime;
      
       lastFlowProcessTime = currentMillis;

       long currentPulseCount;
       noInterrupts();
       currentPulseCount = flow_pulse;
       interrupts();

       long delta_pulse = currentPulseCount - flow_pulseLast;
       FlowMeterValues flowData = calculateFlowMeterValues(delta_pulse,elapsed_time);
       flow_pulseLast = currentPulseCount;
       byte flow_id = FLOW_SENSOR_ID; 
       sendBinaryPacket(FLOW_PACKET_START_BYTE, flow_id, &flowData, sizeof(flowData), FLOW_PACKET_END_BYTE);
       printElapsedTime("Flow Sensor Block");
   }

  // --- State Machine Logic for Temperature Sensors (MAX6675) ---
  if (currentMillis - lastTempProcessTime >= MIN_TEMP_INTERVAL_MS) {
    startTimer();
    lastTempProcessTime = currentMillis;

    TemperatureSensorValues tempData = calculateTemperatureSensorValues(currentTempSensorIndex); // Cycles through 0, 1, 2, 3

    byte temp_id = TEMP_ID_START + currentTempSensorIndex; // Use ID offset + index (10, 11, 12, 13)
    sendBinaryPacket(TEMP_PACKET_START_BYTE, temp_id, &tempData, sizeof(tempData), TEMP_PACKET_END_BYTE);

    currentTempSensorIndex++; // Will now cycle through 0, 1, 2, 3
    if (currentTempSensorIndex >= NUM_TEMP_SENSORS) { // NUM_TEMP_SENSORS is now 4
      currentTempSensorIndex = 0;
    }
    printElapsedTime("Temp Sensor Block (incl. readCelsius wait)");
  }

   // --- State Machine Logic for Motor RPM ---
   if (currentMillis - lastMotorCalcTime >= MOTOR_CALCULATION_INTERVAL_MS) {
       startTimer(); // Start timer for THIS block
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

       printElapsedTime("Motor RPM Block"); // Print time for THIS block
   }


 
}
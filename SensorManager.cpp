#include "SensorManager.h" // Include your own header first
// No need to include <Arduino.h> again if SensorManager.h already includes it
// and you're compiling as C++. If you get errors, you might need to add it back.
// #include <Arduino.h>
#include <math.h>          // Required for isnan()


// =======================================================
// === Definitions of Constants and Global Variables =====
// =======================================================

// --- Common Constants ---
const long ANALOG_REFERENCE_mV = 5000;
const float PERCENT_SLOPE = 6.25;
const float PERCENT_OFFSET = -25.0;
volatile byte RELAY_STATUS_BYTE = STATUS_OK; // Changed to volatile byte, initialized with STATUS_OK

// --- Pressure Sensor Constants ---
const int PRESSURE_SENSOR_PINS[6] = {A0, A2, A4, A6, A8, A10};
const float PRESSURE_MAX[6] = {16.0, 16.0, 25.0, 25.0, 40.0, 40.0};
const int NUM_PRESSURE_SENSORS = sizeof(PRESSURE_SENSOR_PINS) / sizeof(PRESSURE_SENSOR_PINS[0]);

const float MV_FACTOR = (float)ANALOG_REFERENCE_mV / 1024.0f;
float pressure_scale_factor[6];
const float PRESSURE_SHUNT_OHMS[6] = {220.7f, 220.9f, 218.3f, 219.4f, 220.1f, 218.3f};
float pressure_ma_factor[6];


// --- Load Cell Constants ---
const byte LOADCELL_DOUT_PINS[3] = {26, 30, 32};
const byte LOADCELL_CLK_PINS[3] = {29, 31, 33};
const float LOADCELL_CALIBRATION_FACTORS[3] = {145.4f, 150.0f, 160.0f};
const int NUM_LOADCELL_SENSORS = sizeof(LOADCELL_DOUT_PINS) / sizeof(LOADCELL_DOUT_PINS[0]);

HX711 scales[3]; // Definition


// --- Flow Sensor Constants ---
const int FLOW_SENSOR_PIN_MEGA = 2;
const int FLOW_PPL = 4215;


// --- Temperature Sensor (MAX6675) Constants ---
const int THERMO_SHARED_CLK_PIN = 27;
const int THERMO_SHARED_DO_PIN  = 50;
const int THERMO_CS_PINS[4]  = { 22, 23, 24, 25 };
const int NUM_TEMP_SENSORS = sizeof(THERMO_CS_PINS) / sizeof(THERMO_CS_PINS[0]);

const float FAHRENHEIT_SLOPE = 9.0f / 5.0f;
const float FAHRENHEIT_OFFSET = 32.0f;

MAX6675 thermocouples[4] = { // Definition
    MAX6675(THERMO_SHARED_CLK_PIN, THERMO_CS_PINS[0], THERMO_SHARED_DO_PIN),
    MAX6675(THERMO_SHARED_CLK_PIN, THERMO_CS_PINS[1], THERMO_SHARED_DO_PIN),
    MAX6675(THERMO_SHARED_CLK_PIN, THERMO_CS_PINS[2], THERMO_SHARED_DO_PIN),
    MAX6675(THERMO_SHARED_CLK_PIN, THERMO_CS_PINS[3], THERMO_SHARED_DO_PIN)
};


// --- Relay Constants ---
const int RELAY_PINS[4] = {34, 36, 38, 40};
const int NUM_RELAYS = sizeof(RELAY_PINS) / sizeof(RELAY_PINS[0]);
// Removed individual RELAY_ID_BYTE constants as they are not used in the protocol
// const byte RELAY1_ID_BYTE = 0xA1;
// const byte RELAY2_ID_BYTE = 0xA2;
// const byte RELAY3_ID_BYTE = 0xA3;
// const byte RELAY4_ID_BYTE = 0xA4;



// --- DC Motor Constants ---
const int MOTOR_PWM_PIN = 6;
const int MOTOR_ENABLE_PIN = 37;
const int MOTOR_SPEED_SENSE_PIN = 3;
const int MOTOR_PULSES_PER_REVOLUTION = 12;
const float PULSES_PER_SEC_TO_RPM_FACTOR = 60.0f / MOTOR_PULSES_PER_REVOLUTION;


// --- Binary Protocol Constants (Sensor Data Output) ---
const byte PRESSURE_PACKET_START_BYTE = 0xAA;
const byte PRESSURE_PACKET_END_BYTE = 0x55;
const byte LOADCELL_PACKET_START_BYTE = 0xBB;
const byte LOADCELL_PACKET_END_BYTE = 0x66;
const byte FLOW_PACKET_START_BYTE = 0xCC;
const byte FLOW_PACKET_END_BYTE = 0xDD;
const byte TEMP_PACKET_START_BYTE = 0xEE;
const byte TEMP_PACKET_END_BYTE = 0xFF;
const byte MOTOR_RPM_PACKET_START_BYTE = 0xF0;
const byte MOTOR_RPM_PACKET_END_BYTE = 0xF1;


// --- Binary Protocol Constants (Incoming Commands) ---
const byte COMMAND_START_BYTE = 0xFC;
const byte COMMAND_END_BYTE = 0xFD;
const byte CMD_TYPE_SET_RELAY = 0x01;
const byte CMD_TYPE_SET_MOTOR = 0x02;
const byte CMD_TARGET_RELAY_START = 0;
const byte CMD_TARGET_MOTOR_ID = 0;

// --- Error handling ACK / NACK constants (Aligned with Python CLI's STATUS_ constants) ---
const byte RESPONSE_PACKET_START_BYTE = 0xF2; // Matches ACK_START_BYTE from .h
const byte RESPONSE_PACKET_END_BYTE = 0xF3;    // Matches ACK_END_BYTE from .h
const byte RESPONSE_ID_COMMAND_ACK = 0x01;     // The ID byte for command responses

const byte STATUS_OK                  = 0x00; // Matches SUCCESS_BYTE from .h
const byte STATUS_ERROR_INVALID_TARGET_ID = 0xE1; // Matches ERROR_INVALID_TARGET_ID from .h
const byte STATUS_ERROR_INVALID_STATE_VALUE = 0xE2; // Matches ERROR_INVALID_STATE_VALUE from .h
const byte STATUS_ERROR_INVALID_PAYLOAD_SIZE = 0xE3; // Matches ERROR_INVALID_PAYLOAD_SIZE from .h
const byte STATUS_ERROR_INVALID_COMMAND_TYPE = 0xE4; // Matches ERROR_INVALID_COMMAND_TYPE from .h
const byte STATUS_ERROR_HARDWARE_FAILURE = 0xE5; // Matches ERROR_INVALID_COMMAND_EXC from .h
const byte STATUS_ERROR_UNKNOWN_ISSUE      = 0xE6; // Matches ERROR_UNKNOWN_ISSUE from .h


// Define ID ranges and number of IDs for each sensor type (Output Packets)
const byte PRESSURE_ID_START = 0;
const byte NUM_IDS_PRESSURE = NUM_PRESSURE_SENSORS;

const byte LOADCELL_ID_START = PRESSURE_ID_START + NUM_IDS_PRESSURE;
const byte NUM_IDS_LOADCELL = NUM_LOADCELL_SENSORS;

const byte FLOW_SENSOR_ID = LOADCELL_ID_START + NUM_IDS_LOADCELL;
const byte NUM_IDS_FLOW = 1;

const byte TEMP_ID_START = FLOW_SENSOR_ID + NUM_IDS_FLOW;
const byte NUM_IDS_TEMP = NUM_TEMP_SENSORS;

const byte MOTOR_RPM_ID = TEMP_ID_START + NUM_IDS_TEMP;
const byte NUM_IDS_MOTOR_RPM = 1;


// --- Serial Receive State Machine Variables and Constants ---
RxState currentRxState = RX_WAITING_FOR_START; // Definition

byte rxCommandType = 0;
byte rxTargetId = 0;
byte rxPayloadSize = 0;
byte rxPayloadBytesRead = 0;

// const byte MAX_COMMAND_PAYLOAD_SIZE = 16; // Definition (moved from .h to avoid duplication)
byte rxPayloadBuffer[MAX_COMMAND_PAYLOAD_SIZE]; // Definition


// --- State Machine / Round-Robin Variables and Constants (Sensor Reading) ---
int currentPressureSensorIndex = 0;
unsigned long lastPressureSensorProcessTime = 0;
const unsigned long MIN_PRESSURE_INTERVAL_MS = 5;

int currentLoadCellIndex = 0;
unsigned long lastLoadCellProcessTime = 0;
const unsigned long MIN_LOADCELL_CHECK_INTERVAL_MS = 10;

volatile long flow_pulse = 0;
long flow_pulseLast = 0;
unsigned long lastFlowProcessTime = 0;
const unsigned long FLOW_CALCULATION_INTERVAL_MS = 1000;
unsigned long elapsed_time = 0;

int currentTempSensorIndex = 0;
unsigned long lastTempProcessTime = 0;
const unsigned long MIN_TEMP_INTERVAL_MS = 260;

volatile unsigned long motor_pulse_count = 0;
unsigned long motor_last_pulse_count = 0;
unsigned long lastMotorCalcTime = 0;
const unsigned long MOTOR_CALCULATION_INTERVAL_MS = 500;


//--- global areas to store ACK and NACK INFO (REMOVED ErrorandStatus struct and its arrays)
// ErrorandStatus RELAY_RESPONSE[MAX_CONTROLLED_RELAYS]; // REMOVED
// ErrorandStatus MOTOR_RESPONSE[MAX_CONTROLLED_MOTORS]; // REMOVED


// --- NEW: Global Arrays to Store Timing Data ---
SensorTiming pressureTimingData[MAX_TIMED_PRESSURE_SENSORS]; // Definition
SensorTiming loadCellTimingData[MAX_TIMED_LOADCELL_SENSORS]; // Definition
SensorTiming tempTimingData[MAX_TIMED_TEMP_SENSORS];          // Definition
SensorTiming flowTimingData[MAX_TIMED_FLOW_SENSORS];          // Definition
SensorTiming motorTimingData[MAX_TIMED_MOTOR_SENSORS];        // Definition


// --- NEW: Global variable for category timing ---
SensorTiming categoryTiming; // DEFINITION HERE (no 'extern')

// --- NEW: Packet Constants for Timing Data (already defined above, just for clarity) ---
const byte TIMING_PACKET_START_BYTE = 0xDE;
const byte TIMING_PACKET_END_BYTE = 0xAD;
const byte TIMING_SENSOR_OPERATION_ID = 0x01;
const byte TIMING_CATEGORY_CYCLE_ID = 0x02;


// --- NEW: Timing variables for category cycles ---
unsigned long pressureCategoryStartTime = 0;
unsigned long loadCellCategoryStartTime = 0;
unsigned long tempCategoryStartTime = 0;
unsigned long flowCategoryStartTime = 0;
unsigned long motorCategoryStartTime = 0;

// --- NEW: Global variables for sequence management ---
FlowMeterValues latestFlowMeterValues; // Global to store latest flow data
volatile bool isStartupSequenceActive = false;
volatile bool isShutdownSequenceActive = false;

// NEW: Enums for sequence states
StartupState currentStartupState = STARTUP_IDLE;
ShutdownState currentShutdownState = SHUTDOWN_IDLE;

unsigned long sequenceStepStartTime = 0; // To track start time for non-blocking delays (like 200ms wait)


// --- Timing Helper Functions (Definitions) ---
void startSensorTimer(byte sensorId, unsigned long* startTimeVar) {
    if (startTimeVar != nullptr) {
        *startTimeVar = micros();
    }
}

void endSensorTimer(byte sensorId, unsigned long startTime, const char* description) {
    unsigned long endTime = micros();
    unsigned long duration = endTime - startTime;

    if (sensorId >= PRESSURE_ID_START && sensorId < PRESSURE_ID_START + NUM_PRESSURE_SENSORS) {
        byte index = sensorId - PRESSURE_ID_START;
        if (index < MAX_TIMED_PRESSURE_SENSORS) {
            pressureTimingData[index] = {sensorId, startTime, endTime, duration};
        }
    } else if (sensorId >= LOADCELL_ID_START && sensorId < LOADCELL_ID_START + NUM_LOADCELL_SENSORS) {
        byte index = sensorId - LOADCELL_ID_START;
        if (index < MAX_TIMED_LOADCELL_SENSORS) {
            loadCellTimingData[index] = {sensorId, startTime, endTime, duration};
        }
    } else if (sensorId == FLOW_SENSOR_ID) {
        if (0 < MAX_TIMED_FLOW_SENSORS) { // This condition is always true if MAX_TIMED_FLOW_SENSORS is > 0
            flowTimingData[0] = {sensorId, startTime, endTime, duration};
        }
    } else if (sensorId >= TEMP_ID_START && sensorId < TEMP_ID_START + NUM_IDS_TEMP) {
        byte index = sensorId - TEMP_ID_START;
        if (index < MAX_TIMED_TEMP_SENSORS) {
            tempTimingData[index] = {sensorId, startTime, endTime, duration};
        }
    } else if (sensorId == MOTOR_RPM_ID) {
        if (0 < MAX_TIMED_MOTOR_SENSORS) { // This condition is always true if MAX_TIMED_MOTOR_SENSORS is > 0
            motorTimingData[0] = {sensorId, startTime, endTime, duration};
        }
    }

    // Serial.print(F("Time for ")); Serial.print(description); Serial.print(F(" (ID ")); Serial.print(sensorId); Serial.print(F("): "));
    // Serial.print(duration); Serial.println(F(" us"));

}

void sendTimingPacket(byte timing_id, const SensorTiming* data_ptr) {
    sendBinaryPacket(TIMING_PACKET_START_BYTE, timing_id, (const void*)data_ptr, sizeof(SensorTiming), TIMING_PACKET_END_BYTE);
}


// =======================================================
// === Function Definitions (Code Bodies) ================
// =======================================================

// --- Calculation Function for Pressure Sensor ---
PressureSensorValues calculatePressureSensorValues(int raw_pressure_int, int index) {
    if (index < 0 || index >= NUM_PRESSURE_SENSORS) { return {-1.0f}; }
    float raw_pressure_f = (float)raw_pressure_int;
    float mV = raw_pressure_f * MV_FACTOR;
    float volts = mV / 1000.0f;
    float mA = volts * pressure_ma_factor[index];
    float percent = mA * PERCENT_SLOPE + PERCENT_OFFSET;
    float pressure = percent * pressure_scale_factor[index];
    return {pressure};
}

// --- Calculation Function for Load Cell Sensor ---
LoadCellValues calculateLoadCellValues(float raw_weight_float) {
    return {raw_weight_float};
}

// --- Calculation Function for Flow Sensor ---
FlowMeterValues calculateFlowMeterValues(long delta_pulse, unsigned long elapsed_time) {
    if (elapsed_time == 0) {
        return {0.0f};
    }
    float lpm = ((float)delta_pulse * 60000.0f) / (FLOW_PPL * (float)elapsed_time);
    return {lpm};
}
// --- Calculation Function for Temperature Sensor (MAX6675) ---
TemperatureSensorValues calculateTemperatureSensorValues(int index) {
    if (index < 0 || index >= NUM_TEMP_SENSORS) { return {NAN, NAN}; }
    MAX6675& currentThermocouple = thermocouples[index];
    double celsius = currentThermocouple.readCelsius();
    TemperatureSensorValues values;
    if (isnan(celsius)) { values.temp_c = NAN; values.temp_f = NAN; }
    else {
        double fahrenheit = celsius * FAHRENHEIT_SLOPE + FAHRENHEIT_OFFSET;
        values.temp_c = (float)celsius;
        values.temp_f = (float)fahrenheit;
    }
    return values;
}

// --- Calculation Function for Motor RPM ---
MotorRPMValue calculateMotorRPM(unsigned long currentPulseCount, unsigned long previousPulseCount, unsigned long interval_ms) {
    unsigned long delta_pulse = currentPulseCount - previousPulseCount;
    if (interval_ms == 0) { return {0.0f}; }
    float frequency_hz = (float)delta_pulse / (float)interval_ms * 1000.0f;
    float rpm = frequency_hz * PULSES_PER_SEC_TO_RPM_FACTOR;
    return {rpm};
}


// --- GENERIC Function to send any data block in binary format ---
// Now includes an optional timing_data_ptr. If provided, timing data is appended to the payload.
void sendBinaryPacket(byte start_byte, byte id, const void* data_ptr, size_t data_size, byte end_byte, const SensorTiming* timing_data_ptr = nullptr) {

    bool timing_presence = false;
    // Ensure main data pointer is valid and has size
    if (data_ptr == nullptr || data_size == 0) return;
    size_t total_payload_size = data_size;

    if (timing_data_ptr != nullptr) {
        total_payload_size += sizeof(SensorTiming); // Add size of timing data if present
        timing_presence = true;
    }

    if (total_payload_size > 255) {
    // Optional: Print a warning to serial if the packet is too large
    // Serial.print(F("Warning: Packet ID ")); Serial.print(id); Serial.print(F(" total data size (")); Serial.print(total_payload_size); Serial.println(F(" bytes) exceeds 1-byte limit. Skipping packet."));
    return; // Abort if packet is too large
    }

    Serial.write(start_byte); // Write the packet start byte
    Serial.write(id);          // Write the packet ID (which will be the sensor ID for sensor data packets)

    Serial.write((byte)total_payload_size); // Write the total size of the payload

    Serial.write((const byte*)data_ptr, (size_t)data_size); // Write the original sensor data bytes

    // If timing data is provided, append it to the packet
    if (timing_presence == true) {
        Serial.write((const byte*)timing_data_ptr, sizeof(SensorTiming)); // Write the appended timing data bytes
    }

    Serial.write(end_byte); // Write the packet end byte
}

// --- Dedicated function to send ACK/NACK responses (REQUIRED for Python CLI) ---
// This function creates the specific 3-byte payload expected by the Python CLI
// (originalCmdType, originalTargetId, statusCode)
void sendResponsePacket(byte originalCmdType, byte originalTargetId, byte statusCode) {
    byte payload[3];
    payload[0] = originalCmdType;
    payload[1] = originalTargetId;
    payload[2] = statusCode;

    Serial.write(RESPONSE_PACKET_START_BYTE);
    Serial.write(RESPONSE_ID_COMMAND_ACK); // This is the fixed ID for command responses
    Serial.write(sizeof(payload));         // Payload size (always 3)
    Serial.write(payload, sizeof(payload));
    Serial.write(RESPONSE_PACKET_END_BYTE);

    // Optional: For debugging on Arduino Serial Monitor (disable in production)
    // Serial.print(F("Sent ACK/NACK: CmdType=0x")); Serial.print(originalCmdType, HEX);
    // Serial.print(F(", TargetID=")); Serial.print(originalTargetId);
    // Serial.print(F(", Status=0x")); Serial.println(statusCode, HEX);
}

// --- NEW: Function to send sequence status updates ---
// This function sends a structured packet for sequence progress/status
void sendSequenceStatusPacket(byte sequenceType, byte stepCode, byte statusCode) {
    // Payload: sequenceType (1B), stepCode (1B), statusCode (1B)
    byte payload[3];
    payload[0] = sequenceType; // e.g., SEQUENCE_TYPE_STARTUP, SEQUENCE_TYPE_SHUTDOWN
    payload[1] = stepCode;     // e.g., SEQ_STARTUP_STEP_OPEN_FUEL_VALVE, SEQ_SHUTDOWN_COMPLETE
    payload[2] = statusCode;   // e.g., STATUS_OK, STATUS_ERROR_HARDWARE_FAILURE

    Serial.write(SEQUENCE_STATUS_PACKET_START_BYTE);
    Serial.write(RESPONSE_ID_COMMAND_ACK); // Using a placeholder ID for the packet header, could be 0x01
                                            // The actual sequence type is in the payload[0]
    Serial.write(sizeof(payload));         // Payload size (always 3)
    Serial.write(payload, sizeof(payload));
    Serial.write(SEQUENCE_STATUS_PACKET_END_BYTE);

    // Optional: For debugging on Arduino Serial Monitor
    // Serial.print(F("Sent Seq Status: Type=0x")); Serial.print(sequenceType, HEX);
    // Serial.print(F(", Step=0x")); Serial.print(stepCode, HEX);
    // Serial.print(F(", Status=0x")); Serial.println(statusCode, HEX);
}


// --- Command Handling Helper Functions ---

void setRelayState(byte relayIndex, byte state) {
    // Reset the success flag before attempting the operation
    RELAY_STATUS_BYTE = STATUS_ERROR_HARDWARE_FAILURE; // Default to failure, set to OK on success

    // No need for index/state validation here, handleCommand already does it
    // but we keep it minimal for safety if called elsewhere.
    if (relayIndex >= NUM_RELAYS || state > 1) {
        return; // Invalid parameters, flag remains failure
    }

    // Determine the actual HIGH/LOW value for digitalWrite
    // IMPORTANT: Adjust LOW/HIGH based on whether your relay module is active LOW or active HIGH
    // For active LOW modules (common): state 1 (ON) -> LOW signal, state 0 (OFF) -> HIGH signal
    int desiredPinLevel = (state == 1) ? LOW : HIGH;  

    digitalWrite(RELAY_PINS[relayIndex], desiredPinLevel);
    
    // --- PROBE THE STATE OF THE RELAY PIN ---
    // Read back the actual state of the pin
    int actualPinLevel = digitalRead(RELAY_PINS[relayIndex]);

    // Compare desired state with actual state
    if (actualPinLevel == desiredPinLevel) {
        RELAY_STATUS_BYTE = STATUS_OK; // Set to OK if the state matches
    } else {
        RELAY_STATUS_BYTE = STATUS_ERROR_HARDWARE_FAILURE; // Set to hardware failure if there's a mismatch
        // Optional: Debug print if the state didn't match
        // Serial.print(F("WARN: Relay ")); Serial.print(relayIndex);
        // Serial.print(F(" desired ")); Serial.print(desiredPinLevel);
        // Serial.print(F(" but actual ")); Serial.println(actualPinLevel);
    }    
    // Serial.print(F("Set Relay ")); Serial.print(relayIndex); Serial.print(F(" to ")); Serial.println((state == 1) ? F("ON") : F("OFF"));
}

void setMotorEnable(byte state) {
    if (state > 1) {
        Serial.print(F("Error: Invalid motor enable state received: ")); Serial.println(state);
        return;
    }
    digitalWrite(MOTOR_ENABLE_PIN, (state == 1) ? HIGH : LOW);
    // Serial.print(F("Set Motor Enable to: ")); Serial.println((state == 1) ? F("ON") : F("OFF"));
}

void setMotorThrottle(byte throttlePercent) {
    if (throttlePercent > 100) {
        throttlePercent = 100;
    }

    if (throttlePercent == 0) {
        TCCR4A &= ~(_BV(COM4A1) | _BV(COM4A0));
        PORTH &= ~_BV(3);
    } else {
        TCCR4A |= _BV(COM4A1);
        TCCR4A &= ~_BV(COM4A0);
        int dutyCycleValue = map(throttlePercent, 0, 100, 0, ICR4);
        OCR4A = dutyCycleValue;
    }
}


// --- Main Command Handling Function ---
void handleCommand(byte commandType, byte targetId, const byte* payload, byte payloadSize) {
    // Serial.print(F("Received Command: Type=")); Serial.print(commandType);
    // Serial.print(F(", Target ID=")); Serial.print(targetId);
    // Serial.print(F(", Payload Size=")); Serial.println(payloadSize);

    byte statusCode = STATUS_OK; // Initialize status code to OK. It will be updated if an error occurs.

    switch (commandType) {
        case CMD_TYPE_SET_RELAY:
            // Check if the payload size is correct for a SET_RELAY command (1 byte for state)
            if (payloadSize == 1) {
                // Check if the targetId is within the valid range for relays
                if (targetId >= CMD_TARGET_RELAY_START && targetId < CMD_TARGET_RELAY_START + NUM_RELAYS) {
                    byte state = payload[0]; // Extract the desired state (0 or 1)

                    // Validate the state value (must be 0 or 1)
                    if (state > 1) {  
                        statusCode = STATUS_ERROR_INVALID_STATE_VALUE;
                        // Optional: Serial.println(F("Error: Invalid relay state value."));
                    } else {
                        // Call the function to set the relay state and perform hardware verification.
                        // setRelayState will update the global RELAY_STATUS_BYTE flag.
                        setRelayState(targetId, state); // Pass targetId directly as it's 0-indexed

                        // Check the result of the setRelayState operation via the global flag.
                        if (RELAY_STATUS_BYTE == STATUS_OK) {  
                            statusCode = STATUS_OK; // Relay state was successfully set and verified.
                            // Optional: Serial.println(F("Relay command successful."));
                        } else {
                            // If RELAY_STATUS_BYTE is not STATUS_OK, it indicates a hardware failure
                            statusCode = STATUS_ERROR_HARDWARE_FAILURE;  
                            // Optional: Serial.println(F("Error: Relay hardware verification failed."));
                        }
                    }
                } else {
                    // Invalid target ID for a SET_RELAY command.
                    statusCode = STATUS_ERROR_INVALID_TARGET_ID;  
                    // Optional: Serial.print(F("Error: CMD_TYPE_SET_RELAY received with invalid target ID: ")); Serial.println(targetId);
                }
            } else {
                // Invalid payload size for a SET_RELAY command.
                statusCode = STATUS_ERROR_INVALID_PAYLOAD_SIZE;  
                // Optional: Serial.print(F("Error: CMD_TYPE_SET_RELAY received with invalid payload size: ")); Serial.println(payloadSize);
            }
            break;

        case CMD_TYPE_SET_MOTOR:
            if (payloadSize == 2) {
                if (targetId == CMD_TARGET_MOTOR_ID) {
                    byte enableState = payload[0];
                    byte throttlePercent = payload[1];

                    // Validate motor enable and throttle values.
                    if (enableState > 1 || throttlePercent > 100) {
                        statusCode = STATUS_ERROR_INVALID_STATE_VALUE;
                        // Optional: Serial.println(F("Error: Invalid motor enable or throttle value."));
                    } else {
                        setMotorEnable(enableState);
                        setMotorThrottle(throttlePercent);
                        // For motor, without direct hardware read-back (like an encoder),
                        // we assume success if the command was syntactically valid and executed.
                        statusCode = STATUS_OK;
                        // Optional: Serial.println(F("Motor command successful."));
                    }

                } else {
                    // Invalid target ID for a SET_MOTOR command.
                    statusCode = STATUS_ERROR_INVALID_TARGET_ID;
                    // Optional: Serial.print(F("Error: CMD_TYPE_SET_MOTOR received with invalid target ID: ")); Serial.println(targetId);
                }
            } else {
                // Invalid payload size for a SET_MOTOR command.
                statusCode = STATUS_ERROR_INVALID_PAYLOAD_SIZE;
                // Optional: Serial.print(F("Error: CMD_TYPE_SET_MOTOR received with invalid payload size: ")); Serial.println(payloadSize);
            }
            break;

        // NEW: Handle Startup Sequence Command
        case CMD_TYPE_STARTUP_SEQUENCE: {
            // For now, no payload expected for this command
            if (payloadSize != 0) {
                statusCode = STATUS_ERROR_INVALID_PAYLOAD_SIZE;
                Serial.print(F("Startup Sequence Error: Invalid payload size (")); Serial.print(payloadSize); Serial.println(F("). Expected 0."));
                break;
            }
            // Only initiate if no other sequence is active
            if (!isStartupSequenceActive && !isShutdownSequenceActive) {
                sendSequenceStatusPacket(SEQUENCE_TYPE_STARTUP, SEQ_STARTUP_STEP_OPEN_FUEL_VALVE, STATUS_OK); // Initial status
                currentStartupState = STARTUP_STEP_OPEN_FUEL_VALVE; // Set to the first step
                isStartupSequenceActive = true; // Activate the sequence handler
            } else {
                statusCode = STATUS_ERROR_UNKNOWN_ISSUE; // Or a more specific error like "SEQUENCE_ALREADY_ACTIVE"
                Serial.println(F("Startup sequence already active or shutdown active. Ignoring command."));
            }
            break;
        }
        // NEW: Handle Shutdown Sequence Command
        case CMD_TYPE_SHUTDOWN_SEQUENCE: {
            // For now, no payload expected for this command
            if (payloadSize != 0) {
                statusCode = STATUS_ERROR_INVALID_PAYLOAD_SIZE;
                Serial.print(F("Shutdown Sequence Error: Invalid payload size (")); Serial.print(payloadSize); Serial.println(F("). Expected 0."));
                break;
            }
            // Only initiate if no other sequence is active
            if (!isStartupSequenceActive && !isShutdownSequenceActive) {
                sendSequenceStatusPacket(SEQUENCE_TYPE_SHUTDOWN, SEQ_SHUTDOWN_STEP_CLOSE_OXIDIZER_VALVE, STATUS_OK); // Initial status
                currentShutdownState = SHUTDOWN_STEP_CLOSE_OXIDIZER_VALVE; // Set to the first step
                isShutdownSequenceActive = true; // Activate the sequence handler
            } else {
                statusCode = STATUS_ERROR_UNKNOWN_ISSUE; // Or a more specific error like "SEQUENCE_ALREADY_ACTIVE"
                Serial.println(F("Shutdown sequence already active or startup active. Ignoring command."));
            }
            break;
        }

        default:
            // Received an unknown command type.
            statusCode = STATUS_ERROR_INVALID_COMMAND_TYPE;
            // Optional: Serial.print(F("Error: Received unknown command type: ")); Serial.println(commandType);
            break;
    }
    
    // Always send a response packet back to the Python CLI after processing any command.
    // This ensures the Python CLI receives an ACK (OK) or NACK (Error) for every command sent.
    sendResponsePacket(commandType, targetId, statusCode);
}


// --- Modular Setup Functions ---
void setupPressureSensors() {
    Serial.println(F("Setting up Pressure Sensors..."));
    for (int i = 0; i < NUM_PRESSURE_SENSORS; i++) {
        pressure_scale_factor[i] = PRESSURE_MAX[i] / 100.0f;
        pressure_ma_factor[i] = 1000.0f / PRESSURE_SHUNT_OHMS[i];
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
        delay(100);

        scales[i].set_scale(LOADCELL_CALIBRATION_FACTORS[i]);
        scales[i].tare();

        Serial.print(F("Load Cell ")); Serial.print(i + 1); Serial.println(F(" tared."));
        delay(50);
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
    currentTempSensorIndex = 0;
    lastTempProcessTime = millis();

    Serial.print(NUM_TEMP_SENSORS); Serial.println(F(" Temperature Sensors setup complete."));
}

void setupRelays() {
    Serial.println(F("Setting up Relays..."));
    for (int i = 0; i < NUM_RELAYS; i++) {
        pinMode(RELAY_PINS[i], OUTPUT);
        digitalWrite(RELAY_PINS[i], LOW); // Ensure relays are off initially (assuming active HIGH)
    }
    Serial.print(NUM_RELAYS); Serial.println(F(" Relays setup complete."));
}

void setupDCMotor() {
    Serial.println(F("Setting up DC Motor..."));

    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR_PWM_PIN, OUTPUT);

    // Configure Timer 4 for PWM on MOTOR_PWM_PIN (typically pin 6 on Mega)
    // Fast PWM mode, TOP at ICR4, Clear OC4A on Compare Match, Set OC4A at TOP
    TCCR4A = _BV(COM4A1) | _BV(WGM41);
    TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS41); // Prescaler 8 (for 16MHz, gives ~2kHz PWM)
    ICR4 = 199; // Set TOP value for PWM (e.g., 199 for 200 counts, 16MHz/8/200 = 10kHz)
    OCR4A = 0; // Initial duty cycle 0

    digitalWrite(MOTOR_ENABLE_PIN, LOW); // Ensure motor driver is disabled initially

    pinMode(MOTOR_SPEED_SENSE_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MOTOR_SPEED_SENSE_PIN), motor_count_pulse, RISING);

    motor_pulse_count = 0;
    motor_last_pulse_count = 0;
    lastMotorCalcTime = millis();

    Serial.println(F("DC Motor setup complete. Driver Disabled, Motor Stopped."));
}


// --- Flow sensor Interrupt Service Routine (ISR) ---
extern "C" void flow_increase_pulse() { // Defined with extern "C"
    flow_pulse++;
}

// --- Motor Speed Sense Interrupt Service Routine (ISR) ---
extern "C" void motor_count_pulse() { // Defined with extern "C"
    motor_pulse_count++;
}

// --- NEW: Startup Sequence Implementation (Skeleton with State Machine) ---
void startup_sequence() {
    // This function is called by handleCommand() to INITIATE the sequence.
    // The actual sequence steps are handled by runStartupSequence() in the main loop.
    // This ensures the sequence is non-blocking.
    if (!isStartupSequenceActive && !isShutdownSequenceActive) {
        sendSequenceStatusPacket(SEQUENCE_TYPE_STARTUP, SEQ_STARTUP_STEP_OPEN_FUEL_VALVE, STATUS_OK); // Initial status
        currentStartupState = STARTUP_STEP_OPEN_FUEL_VALVE; // Set to the first step
        isStartupSequenceActive = true; // Activate the sequence handler
    } else {
        // Send NACK if already active
        sendResponsePacket(CMD_TYPE_STARTUP_SEQUENCE, 0, STATUS_ERROR_UNKNOWN_ISSUE);
        // Serial.println(F("Startup sequence already active or shutdown active. Ignoring command.")); // For debug
    }
}

void runStartupSequence() {
    if (!isStartupSequenceActive) {
        return; // Do nothing if sequence is not active
    }

    unsigned long currentMillis = millis(); // Get current time for non-blocking operations

    switch (currentStartupState) {
        case STARTUP_STEP_OPEN_FUEL_VALVE:
            sendSequenceStatusPacket(SEQUENCE_TYPE_STARTUP, SEQ_STARTUP_STEP_OPEN_FUEL_VALVE, STATUS_OK);
            setRelayState(0, 1); // Open R0 (fuel valve)
            // You might add a check here for RELAY_STATUS_BYTE if verification is critical for moving to next step
            currentStartupState = STARTUP_STEP_SET_MOTOR_THROTTLE; // Move to next step immediately
            break;

        case STARTUP_STEP_SET_MOTOR_THROTTLE:
            sendSequenceStatusPacket(SEQUENCE_TYPE_STARTUP, SEQ_STARTUP_STEP_SET_MOTOR_THROTTLE, STATUS_OK);
            setMotorEnable(1);
            setMotorThrottle(77);
            currentStartupState = STARTUP_STEP_WAIT_FOR_FLOW; // Move to wait state
            break;

        case STARTUP_STEP_WAIT_FOR_FLOW:
            // This is the non-blocking "wait" for flow to reach target.
            if (latestFlowMeterValues.flow_rate_lpm >= 1.4f) {
                sendSequenceStatusPacket(SEQUENCE_TYPE_STARTUP, SEQ_STARTUP_STEP_WAIT_FOR_FLOW, STATUS_OK); // Indicate condition met
                currentStartupState = STARTUP_STEP_OPEN_OXIDIZER_VALVE; // Move to next step
            } else {
                // Optionally, send a "waiting" status repeatedly
                // sendSequenceStatusPacket(SEQUENCE_TYPE_STARTUP, SEQ_STARTUP_STEP_WAIT_FOR_FLOW, STATUS_UNKNOWN_ISSUE); // Or a custom "WAITING" status
            }
            break;

        case STARTUP_STEP_OPEN_OXIDIZER_VALVE:
            sendSequenceStatusPacket(SEQUENCE_TYPE_STARTUP, SEQ_STARTUP_STEP_OPEN_OXIDIZER_VALVE, STATUS_OK);
            setRelayState(1, 1); // Open R1 (oxidizer valve)
            currentStartupState = STARTUP_STEP_WAIT_200MS; // Move to next step
            sequenceStepStartTime = currentMillis; // Record time when we entered this state for the 200ms wait
            break;

        case STARTUP_STEP_WAIT_200MS:
            // Non-blocking 200ms wait
            if (currentMillis - sequenceStepStartTime >= 200) {
                sendSequenceStatusPacket(SEQUENCE_TYPE_STARTUP, SEQ_STARTUP_STEP_WAIT_200MS, STATUS_OK); // Indicate wait complete
                currentStartupState = STARTUP_STEP_OPEN_IGNITER_VALVE; // Move to next step
            } else {
                // Optionally, send a "waiting" status repeatedly
                // sendSequenceStatusPacket(SEQUENCE_TYPE_STARTUP, SEQ_STARTUP_STEP_WAIT_200MS, STATUS_UNKNOWN_ISSUE); // Or a custom "WAITING" status
            }
            break;

        case STARTUP_STEP_OPEN_IGNITER_VALVE:
            sendSequenceStatusPacket(SEQUENCE_TYPE_STARTUP, SEQ_STARTUP_STEP_OPEN_IGNITER_VALVE, STATUS_OK);
            setRelayState(2, 1); // Open R2 (igniter valve)
            currentStartupState = STARTUP_COMPLETE; // Sequence finished
            break;

        case STARTUP_COMPLETE:
            sendSequenceStatusPacket(SEQUENCE_TYPE_STARTUP, SEQ_STARTUP_COMPLETE, STATUS_OK); // Final completion status
            isStartupSequenceActive = false; // Deactivate the handler
            currentStartupState = STARTUP_IDLE; // Reset state for future use
            break;

        case STARTUP_FAILED: // This state would be reached if you re-introduce timeouts or other failure conditions
            sendSequenceStatusPacket(SEQUENCE_TYPE_STARTUP, SEQ_STARTUP_FAILED, STATUS_ERROR_UNKNOWN_ISSUE); // Final failure status
            isStartupSequenceActive = false; // Deactivate the handler
            currentStartupState = STARTUP_IDLE; // Reset state
            break;

        case STARTUP_IDLE:
            // Should not be here if isStartupSequenceActive is true, but handle defensively
            isStartupSequenceActive = false; // Ensure it's off if somehow entered
            break;
    }
}


// --- NEW: Shutdown Sequence Implementation (Skeleton with State Machine) ---
void shutdown_sequence() {
    // This function is called by handleCommand() to INITIATE the sequence.
    // The actual sequence steps are handled by runShutdownSequence() in the main loop.
    if (!isStartupSequenceActive && !isShutdownSequenceActive) {
        sendSequenceStatusPacket(SEQUENCE_TYPE_SHUTDOWN, SEQ_SHUTDOWN_STEP_CLOSE_OXIDIZER_VALVE, STATUS_OK); // Initial status
        currentShutdownState = SHUTDOWN_STEP_CLOSE_OXIDIZER_VALVE; // Set to the first step
        isShutdownSequenceActive = true; // Activate the sequence handler
    } else {
        // Send NACK if already active
        sendResponsePacket(CMD_TYPE_SHUTDOWN_SEQUENCE, 0, STATUS_ERROR_UNKNOWN_ISSUE);
        // Serial.println(F("Shutdown sequence already active or startup active. Ignoring command.")); // For debug
    }
}

void runShutdownSequence() {
    if (!isShutdownSequenceActive) {
        return; // Do nothing if sequence is not active
    }

    unsigned long currentMillis = millis(); // Get current time for non-blocking operations

    switch (currentShutdownState) {
        case SHUTDOWN_STEP_CLOSE_OXIDIZER_VALVE:
            sendSequenceStatusPacket(SEQUENCE_TYPE_SHUTDOWN, SEQ_SHUTDOWN_STEP_CLOSE_OXIDIZER_VALVE, STATUS_OK);
            setRelayState(1, 0); // Close R1 (oxidizer valve)
            currentShutdownState = SHUTDOWN_STEP_WAIT_200MS; // Move to next step
            sequenceStepStartTime = currentMillis; // Record time for the 200ms wait
            break;

        case SHUTDOWN_STEP_WAIT_200MS:
            // Non-blocking 200ms wait
            if (currentMillis - sequenceStepStartTime >= 200) {
                sendSequenceStatusPacket(SEQUENCE_TYPE_SHUTDOWN, SEQ_SHUTDOWN_STEP_WAIT_200MS, STATUS_OK); // Indicate wait complete
                currentShutdownState = SHUTDOWN_STEP_SET_MOTOR_THROTTLE_ZERO; // Move to next step
            } else {
                // Optionally, send a "waiting" status repeatedly
                // sendSequenceStatusPacket(SEQUENCE_TYPE_SHUTDOWN, SEQ_SHUTDOWN_STEP_WAIT_200MS, STATUS_UNKNOWN_ISSUE); // Or a custom "WAITING" status
            }
            break;

        case SHUTDOWN_STEP_SET_MOTOR_THROTTLE_ZERO:
            sendSequenceStatusPacket(SEQUENCE_TYPE_SHUTDOWN, SEQ_SHUTDOWN_STEP_SET_MOTOR_THROTTLE_ZERO, STATUS_OK);
            setMotorThrottle(0); // Set motor throttle to 0
            setMotorEnable(0);   // Disable motor driver
            currentShutdownState = SHUTDOWN_STEP_WAIT_1000MS; // Move to next step
            sequenceStepStartTime = currentMillis; // Record time for the 1000ms wait
            break;

        case SHUTDOWN_STEP_WAIT_1000MS:
            // Non-blocking 1000ms wait
            if (currentMillis - sequenceStepStartTime >= 1000) {
                sendSequenceStatusPacket(SEQUENCE_TYPE_SHUTDOWN, SEQ_SHUTDOWN_STEP_WAIT_1000MS, STATUS_OK); // Indicate wait complete
                currentShutdownState = SHUTDOWN_STEP_CLOSE_FUEL_VALVE; // Move to next step
            } else {
                // Optionally, send a "waiting" status repeatedly
                // sendSequenceStatusPacket(SEQUENCE_TYPE_SHUTDOWN, SEQ_SHUTDOWN_STEP_WAIT_1000MS, STATUS_UNKNOWN_ISSUE); // Or a custom "WAITING" status
            }
            break;

        case SHUTDOWN_STEP_CLOSE_FUEL_VALVE:
            sendSequenceStatusPacket(SEQUENCE_TYPE_SHUTDOWN, SEQ_SHUTDOWN_STEP_CLOSE_FUEL_VALVE, STATUS_OK);
            setRelayState(0, 0); // Close R0 (fuel valve)
            currentShutdownState = SHUTDOWN_COMPLETE; // Sequence finished
            break;

        case SHUTDOWN_COMPLETE:
            sendSequenceStatusPacket(SEQUENCE_TYPE_SHUTDOWN, SEQ_SHUTDOWN_COMPLETE, STATUS_OK); // Final completion status
            isShutdownSequenceActive = false; // Deactivate the handler
            currentShutdownState = SHUTDOWN_IDLE; // Reset state for future use
            break;

        case SHUTDOWN_FAILED: // This state would be reached if you re-introduce timeouts or other failure conditions
            sendSequenceStatusPacket(SEQUENCE_TYPE_SHUTDOWN, SEQ_SHUTDOWN_FAILED, STATUS_ERROR_UNKNOWN_ISSUE); // Final failure status
            isShutdownSequenceActive = false; // Deactivate the handler
            currentShutdownState = SHUTDOWN_IDLE; // Reset state
            break;

        case SHUTDOWN_IDLE:
            // Should not be here if isShutdownSequenceActive is true, but handle defensively
            isShutdownSequenceActive = false; // Ensure it's off if somehow entered
            break;
    }
}
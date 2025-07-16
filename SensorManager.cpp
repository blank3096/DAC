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
const byte CMD_TARGET_RELAY_START = 1;
const byte CMD_TARGET_MOTOR_ID = 0;


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


// --- NEW: Global Arrays to Store Timing Data ---
SensorTiming pressureTimingData[MAX_TIMED_PRESSURE_SENSORS]; // Definition
SensorTiming loadCellTimingData[MAX_TIMED_LOADCELL_SENSORS]; // Definition
SensorTiming tempTimingData[MAX_TIMED_TEMP_SENSORS];         // Definition
SensorTiming flowTimingData[MAX_TIMED_FLOW_SENSORS];         // Definition
SensorTiming motorTimingData[MAX_TIMED_MOTOR_SENSORS];       // Definition


// --- NEW: Global variable for category timing ---
SensorTiming categoryTiming; // DEFINITION HERE (no 'extern')

// --- NEW: Packet Constants for Timing Data ---
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
        if (0 < MAX_TIMED_FLOW_SENSORS) {
            flowTimingData[0] = {sensorId, startTime, endTime, duration};
        }
    } else if (sensorId >= TEMP_ID_START && sensorId < TEMP_ID_START + NUM_IDS_TEMP) {
        byte index = sensorId - TEMP_ID_START;
        if (index < MAX_TIMED_TEMP_SENSORS) {
            tempTimingData[index] = {sensorId, startTime, endTime, duration};
        }
    } else if (sensorId == MOTOR_RPM_ID) {
        if (0 < MAX_TIMED_MOTOR_SENSORS) {
            motorTimingData[0] = {sensorId, startTime, endTime, duration};
        }
    }

    // Serial.print(F("Time for ")); Serial.print(description); Serial.print(F(" (ID ")); Serial.print(sensorId); Serial.print(F("): "));
    // Serial.print(duration); Serial.println(F(" us"));

    SensorTiming currentSensorTiming = {sensorId, startTime, endTime, duration};
    sendTimingPacket(TIMING_SENSOR_OPERATION_ID, &currentSensorTiming);
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
void sendBinaryPacket(byte start_byte, byte id, const void* data_ptr, size_t data_size, byte end_byte) {
   if (data_ptr == nullptr || data_size == 0) return;

  Serial.write(start_byte);
  Serial.write(id);

  if (data_size > 255) {
    // Serial.print(F("Warning: Packet ID ")); Serial.print(id); Serial.print(F(" data size (")); Serial.print(data_size); Serial.println(F(" bytes) exceeds 1-byte limit. Skipping packet."));
    return;
   }
  Serial.write((byte)data_size);

  Serial.write((const byte*)data_ptr, (size_t)data_size);
  Serial.write(end_byte);
}


// --- Command Handling Helper Functions ---

void setRelayState(byte relayIndex, byte state) {
    if (relayIndex >= NUM_RELAYS) {
        // Serial.print(F("Error: Invalid relay index received: ")); Serial.println(relayIndex);
        return;
    }
    if (state > 1) {
        // Serial.print(F("Error: Invalid relay state received: ")); Serial.println(state);
        return;
    }
    digitalWrite(RELAY_PINS[relayIndex], (state == 1) ? LOW : HIGH);
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

    switch (commandType) {
        case CMD_TYPE_SET_RELAY:
            if (payloadSize == 1) {
                if (targetId >= CMD_TARGET_RELAY_START && targetId < CMD_TARGET_RELAY_START + NUM_RELAYS) {
                    setRelayState(targetId - CMD_TARGET_RELAY_START, payload[0]);
                } else {
                    // Serial.print(F("Error: CMD_TYPE_SET_RELAY received with invalid target ID: ")); Serial.println(targetId);
                }
            } else {
                // Serial.print(F("Error: CMD_TYPE_SET_RELAY received with invalid payload size: ")); Serial.println(payloadSize);
            }
            break;

        case CMD_TYPE_SET_MOTOR:
             if (payloadSize == 2) {
                 if (targetId == CMD_TARGET_MOTOR_ID) {
                     byte enableState = payload[0];
                     byte throttlePercent = payload[1];

                     setMotorEnable(enableState);
                     setMotorThrottle(throttlePercent);

                 } else {
                    // Serial.print(F("Error: CMD_TYPE_SET_MOTOR received with invalid target ID: ")); Serial.println(targetId);
                 }
             } else {
                // Serial.print(F("Error: CMD_TYPE_SET_MOTOR received with invalid payload size: ")); Serial.println(payloadSize);
             }
             break;

        default:
            // Serial.print(F("Error: Received unknown command type: ")); Serial.println(commandType);
            break;
    }
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
        digitalWrite(RELAY_PINS[i], LOW);
    }
    Serial.print(NUM_RELAYS); Serial.println(F(" Relays setup complete."));
}

void setupDCMotor() {
    Serial.println(F("Setting up DC Motor..."));

    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR_PWM_PIN, OUTPUT);

    TCCR4A = _BV(COM4A1) | _BV(WGM41);
    TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS41);
    ICR4 = 199;
    OCR4A = 0;

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


// --- Test Function Definitions ---
void testTimingBatchAllTypes() {
  Serial.println(F("\n--- Running Timing Test: Batch All Types ---"));

  int raw_p = analogRead(PRESSURE_SENSOR_PINS[0]);
  unsigned long testStartTime;
  startSensorTimer(PRESSURE_ID_START + 0, &testStartTime);
  PressureSensorValues pData = calculatePressureSensorValues(raw_p, 0);
  sendBinaryPacket(PRESSURE_PACKET_START_BYTE, PRESSURE_ID_START, &pData, sizeof(pData), PRESSURE_PACKET_END_BYTE);
  endSensorTimer(PRESSURE_ID_START + 0, testStartTime, "One Pressure Sensor Block");


  HX711& testScale = scales[0];
  float raw_weight = testScale.get_units();
  startSensorTimer(LOADCELL_ID_START + 0, &testStartTime);
  LoadCellValues loadCellData = calculateLoadCellValues(raw_weight);
  byte loadCell_id = LOADCELL_ID_START + 0;
  sendBinaryPacket(LOADCELL_PACKET_START_BYTE, loadCell_id, &loadCellData, sizeof(loadCellData), LOADCELL_PACKET_END_BYTE);
  endSensorTimer(LOADCELL_ID_START + 0, testStartTime, "One Load Cell Block (incl. get_units wait)");


  long dummyCurrentPulse = flow_pulse + 100;
  long dummyLastPulse = 0;
  long delta_pulse = dummyCurrentPulse - dummyLastPulse;
  unsigned long dummyElapsedTime = FLOW_CALCULATION_INTERVAL_MS;
  
  startSensorTimer(FLOW_SENSOR_ID, &testStartTime);
  FlowMeterValues fData = calculateFlowMeterValues(delta_pulse, dummyElapsedTime);
  sendBinaryPacket(FLOW_PACKET_START_BYTE, FLOW_SENSOR_ID, &fData, sizeof(fData), FLOW_PACKET_END_BYTE);
  endSensorTimer(FLOW_SENSOR_ID, testStartTime, "One Flow Sensor Calc + Send");


  startSensorTimer(TEMP_ID_START + 0, &testStartTime);
  TemperatureSensorValues tData = calculateTemperatureSensorValues(0);
  sendBinaryPacket(TEMP_PACKET_START_BYTE, TEMP_ID_START, &tData, sizeof(tData), TEMP_PACKET_END_BYTE);
  endSensorTimer(TEMP_ID_START + 0, testStartTime, "One Temp Sensor Block (incl. readCelsius wait)");

  startSensorTimer(MOTOR_RPM_ID, &testStartTime);

  unsigned long interval_ms = MOTOR_CALCULATION_INTERVAL_MS;
  unsigned long currentPulseCount_motor = motor_pulse_count + 50;
  unsigned long motor_last_pulse_count_test = 0;

  MotorRPMValue mData = calculateMotorRPM(currentPulseCount_motor, motor_last_pulse_count_test, interval_ms);
  byte motor_id = MOTOR_RPM_ID;
  sendBinaryPacket(MOTOR_RPM_PACKET_START_BYTE, motor_id, &mData, sizeof(mData), MOTOR_RPM_PACKET_END_BYTE);
  endSensorTimer(MOTOR_RPM_ID, testStartTime, "Motor RPM Block");

  Serial.println(F("--- Timing Test Batch Complete ---"));
}
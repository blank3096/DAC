#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h> // Always include Arduino.h in .h files
#include "HX711.h"   // Include necessary library headers
#include "max6675.h" // Include MAX6675 library header
#include <math.h>    // For isnan() prototype

// --- Common Constants ---
extern const long ANALOG_REFERENCE_mV;
extern const float PERCENT_SLOPE;
extern const float PERCENT_OFFSET;


// --- Pressure Sensor Constants ---
extern const int PRESSURE_SENSOR_PINS[6];
extern const float PRESSURE_MAX[6];
extern const int NUM_PRESSURE_SENSORS;
extern const float MV_FACTOR;
extern float pressure_scale_factor[6];
extern const float PRESSURE_SHUNT_OHMS[6];
extern float pressure_ma_factor[6];


// --- Load Cell Constants ---
extern const byte LOADCELL_DOUT_PINS[3];
extern const byte LOADCELL_CLK_PINS[3];
extern const float LOADCELL_CALIBRATION_FACTORS[3];
extern const int NUM_LOADCELL_SENSORS;
extern HX711 scales[3]; // Declared as extern


// --- Flow Sensor Constants ---
extern const int FLOW_SENSOR_PIN_MEGA; // Pin 2 (INT0)
extern const int FLOW_PPL;


// --- Temperature Sensor (MAX6675) Constants ---
extern const int THERMO_SHARED_CLK_PIN; // D22 - Shared Clock
extern const int THERMO_SHARED_DO_PIN;  // D50 - Shared Data Out (MISO)
extern const int THERMO_CS_PINS[4];
extern const int NUM_TEMP_SENSORS;
extern const float FAHRENHEIT_SLOPE;
extern const float FAHRENHEIT_OFFSET;
extern MAX6675 thermocouples[4]; // Declared as extern


// --- Relay Constants ---
extern const int RELAY_PINS[4];
extern const int NUM_RELAYS;


// --- DC Motor Constants ---
extern const int MOTOR_PWM_PIN;
extern const int MOTOR_ENABLE_PIN;
extern const int MOTOR_SPEED_SENSE_PIN;
extern const int MOTOR_PULSES_PER_REVOLUTION;
extern const float PULSES_PER_SEC_TO_RPM_FACTOR;


// --- Binary Protocol Constants (Sensor Data Output) ---
extern const byte PRESSURE_PACKET_START_BYTE;
extern const byte PRESSURE_PACKET_END_BYTE;
extern const byte LOADCELL_PACKET_START_BYTE;
extern const byte LOADCELL_PACKET_END_BYTE;
extern const byte FLOW_PACKET_START_BYTE;
extern const byte FLOW_PACKET_END_BYTE;
extern const byte TEMP_PACKET_START_BYTE;
extern const byte TEMP_PACKET_END_BYTE;
extern const byte MOTOR_RPM_PACKET_START_BYTE;
extern const byte MOTOR_RPM_PACKET_END_BYTE;


// --- Binary Protocol Constants (Incoming Commands) ---
extern const byte COMMAND_START_BYTE;
extern const byte COMMAND_END_BYTE;
extern const byte CMD_TYPE_SET_RELAY;
extern const byte CMD_TYPE_SET_MOTOR;
extern const byte CMD_TARGET_RELAY_START;
extern const byte CMD_TARGET_MOTOR_ID;


// Define ID ranges and number of IDs for each sensor type (Output Packets)
extern const byte PRESSURE_ID_START;
extern const byte NUM_IDS_PRESSURE;
extern const byte LOADCELL_ID_START;
extern const byte NUM_IDS_LOADCELL;
extern const byte FLOW_SENSOR_ID;
extern const byte NUM_IDS_FLOW;
extern const byte TEMP_ID_START;
extern const byte NUM_IDS_TEMP;
extern const byte MOTOR_RPM_ID;
extern const byte NUM_IDS_MOTOR_RPM;

//-------Struct for sensor data ------
struct PressureSensorValues { float pressure; };
struct LoadCellValues { float weight_grams; };
struct FlowMeterValues { float flow_rate_lpm; };
struct TemperatureSensorValues { float temp_c; float temp_f; };
struct MotorRPMValue { float rpm; };

// --- NEW: Timing Data Structure ---
struct SensorTiming {
    byte sensor_id;
    unsigned long start_micros;
    unsigned long end_micros;
    unsigned long duration_micros;
};

// --- NEW: Global Arrays to Store Timing Data ---
const byte MAX_TIMED_PRESSURE_SENSORS = 6;
const byte MAX_TIMED_LOADCELL_SENSORS = 3;
const byte MAX_TIMED_TEMP_SENSORS = 4;
const byte MAX_TIMED_FLOW_SENSORS = 1;
const byte MAX_TIMED_MOTOR_SENSORS = 1;

extern SensorTiming pressureTimingData[MAX_TIMED_PRESSURE_SENSORS];
extern SensorTiming loadCellTimingData[MAX_TIMED_LOADCELL_SENSORS];
extern SensorTiming tempTimingData[MAX_TIMED_TEMP_SENSORS];
extern SensorTiming flowTimingData[MAX_TIMED_FLOW_SENSORS];
extern SensorTiming motorTimingData[MAX_TIMED_MOTOR_SENSORS];

// --- NEW: Global variable for category timing ---
extern SensorTiming categoryTiming; // Declared as extern


// --- NEW: Packet Constants for Timing Data ---
extern const byte TIMING_PACKET_START_BYTE;
extern const byte TIMING_PACKET_END_BYTE;
extern const byte TIMING_SENSOR_OPERATION_ID;
extern const byte TIMING_CATEGORY_CYCLE_ID;


// --- Serial Receive State Machine Variables and Constants ---
enum RxState {
  RX_WAITING_FOR_START,
  RX_READING_TYPE,
  RX_READING_TARGET_ID,
  RX_READING_SIZE,
  RX_READING_PAYLOAD,
  RX_READING_END
};

extern RxState currentRxState; // Declared as extern

extern byte rxCommandType;
extern byte rxTargetId;
extern byte rxPayloadSize;
extern byte rxPayloadBytesRead;

const byte MAX_COMMAND_PAYLOAD_SIZE = 16;
extern byte rxPayloadBuffer[MAX_COMMAND_PAYLOAD_SIZE]; // Declared as extern


// --- State Machine / Round-Robin Variables and Constants (Sensor Reading) ---
extern int currentPressureSensorIndex;
extern unsigned long lastPressureSensorProcessTime;
extern const unsigned long MIN_PRESSURE_INTERVAL_MS;

extern int currentLoadCellIndex;
extern unsigned long lastLoadCellProcessTime;
extern const unsigned long MIN_LOADCELL_CHECK_INTERVAL_MS;

extern volatile long flow_pulse;
extern long flow_pulseLast;
extern unsigned long lastFlowProcessTime;
extern const unsigned long FLOW_CALCULATION_INTERVAL_MS;
extern unsigned long elapsed_time;

extern int currentTempSensorIndex;
extern unsigned long lastTempProcessTime;
extern const unsigned long MIN_TEMP_INTERVAL_MS;

extern volatile unsigned long motor_pulse_count;
extern unsigned long motor_last_pulse_count;
extern unsigned long lastMotorCalcTime;
extern const unsigned long MOTOR_CALCULATION_INTERVAL_MS;

// --- NEW: Timing variables for category cycles ---
extern unsigned long pressureCategoryStartTime;
extern unsigned long loadCellCategoryStartTime;
extern unsigned long tempCategoryStartTime;
extern unsigned long flowCategoryStartTime;
extern unsigned long motorCategoryStartTime;


// --- Timing Helper Function Prototypes ---
void startSensorTimer(byte sensorId, unsigned long* startTimeVar);
void endSensorTimer(byte sensorId, unsigned long startTime, const char* description);
void sendTimingPacket(byte timing_id, const SensorTiming* data_ptr);


// --- Function Prototypes (Declarations) ---
PressureSensorValues calculatePressureSensorValues(int raw_pressure_int, int index);
LoadCellValues calculateLoadCellValues(float raw_weight_float);
FlowMeterValues calculateFlowMeterValues(long delta_pulse, unsigned long elapsed_time);
TemperatureSensorValues calculateTemperatureSensorValues(int index);
MotorRPMValue calculateMotorRPM(unsigned long currentPulseCount, unsigned long previousPulseCount, unsigned long interval_ms);

void sendBinaryPacket(byte start_byte, byte id, const void* data_ptr, size_t data_size, byte end_byte, const SensorTiming* timing_data_ptr = nullptr);
void handleCommand(byte commandType, byte targetId, const byte* payload, byte payloadSize);
void setRelayState(byte relayIndex, byte state);
void setMotorEnable(byte state);
void setMotorThrottle(byte throttlePercent);

void setupPressureSensors();
void setupLoadCells();
void setupFlowSensors();
void setupTemperatureSensors();
void setupRelays();
void setupDCMotor();

// Interrupt Service Routines (ISRs) - Declared as extern "C" for C++ compatibility
extern "C" void flow_increase_pulse();
extern "C" void motor_count_pulse();

// --- Test Functions ---
void testTimingBatchAllTypes();


#endif // End of include guard
#include <BluetoothSerial.h>

// Bluetooth Serial object
BluetoothSerial SerialBT;

// UART communication with STM32
#define RXD2 16
#define TXD2 17

// Data structures (must match STM32)
typedef struct {
    uint8_t header;
    int16_t pwm_motor;
    int16_t pwm_servo;
    uint8_t checksum;
} ESP_Command_t;

typedef struct {
    uint8_t header;
    int16_t distance;
    int16_t encoder;
    uint8_t checksum;
} Sensor_Data_t;

ESP_Command_t tx_to_stm;
Sensor_Data_t rx_from_stm;

// Control parameters
int16_t motor_pwm = 0;
int16_t servo_angle = 90;

// Obstacle avoidance parameters
const int SAFE_DISTANCE = 50;      // cm - full speed
const int SLOW_DOWN_DISTANCE = 30; // cm - start slowing down
const int STOP_DISTANCE = 15;      // cm - stop completely
const int MAX_PWM = 800;           // Maximum PWM value (80%)

// Robot states
enum RobotState {
    STATE_STOPPED,
    STATE_MOVING,
    STATE_AVOIDING
};
RobotState current_state = STATE_STOPPED;

// Sensor data
int16_t current_distance = 0;
int16_t current_encoder = 0;

// Timing variables
unsigned long last_sensor_time = 0;
unsigned long last_control_time = 0;
unsigned long last_bt_status_time = 0;

// Bluetooth command buffer
String bt_command = "";
bool new_command = false;

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    
    // Initialize Bluetooth
    SerialBT.begin("LineFollower_Robot"); // Bluetooth device name
    Serial.println("Bluetooth Started! Pair with 'LineFollower_Robot'");
    Serial.println("Available Commands:");
    Serial.println("  START - Start autonomous mode");
    Serial.println("  STOP - Stop robot");
    Serial.println("  PWM,500,90 - Set motor PWM and servo angle");
    Serial.println("  MANUAL - Enter manual control mode");
    Serial.println("  AUTO - Enter autonomous mode");
    Serial.println("  STATUS - Get current status");
    
    // Initialize motor to stopped state
    motor_pwm = 0;
    servo_angle = 90;
    sendToSTM32();
    
    Serial.println("ESP32 Master Controller Ready!");
}

void loop() {
    // Handle Bluetooth commands
    handleBluetooth();
    
    unsigned long current_time = millis();
    
    // 1. Receive sensor data from STM32 every 50ms
    if (current_time - last_sensor_time >= 50) {
        receiveSensorData();
        last_sensor_time = current_time;
    }
    
    // 2. Run control logic every 100ms (only in autonomous mode)
    if (current_time - last_control_time >= 100) {
        if (current_state == STATE_MOVING || current_state == STATE_AVOIDING) {
            calculateObstacleAvoidance();
            sendToSTM32();
        }
        last_control_time = current_time;
    }
    
    // 3. Send status via Bluetooth every 2 seconds
    if (current_time - last_bt_status_time >= 2000) {
        sendBluetoothStatus();
        last_bt_status_time = current_time;
    }
    
    delay(10);
}

// Handle Bluetooth communication
void handleBluetooth() {
    // Check for incoming Bluetooth commands
    if (SerialBT.available()) {
        char c = SerialBT.read();
        if (c == '\n') {
            new_command = true;
        } else {
            bt_command += c;
        }
    }
    
    // Process complete command
    if (new_command) {
        bt_command.trim();
        bt_command.toUpperCase();
        Serial.println("BT Command: " + bt_command);
        
        if (bt_command == "START") {
            handleStart();
        }
        else if (bt_command == "STOP") {
            handleStop();
        }
        else if (bt_command == "MANUAL") {
            current_state = STATE_STOPPED;
            SerialBT.println("MANUAL MODE - Use PWM,value,angle to control");
            Serial.println("Manual mode activated");
        }
        else if (bt_command == "AUTO") {
            current_state = STATE_MOVING;
            SerialBT.println("AUTONOMOUS MODE - Robot driving automatically");
            Serial.println("Autonomous mode activated");
        }
        else if (bt_command == "STATUS") {
            sendBluetoothStatus();
        }
        else if (bt_command.startsWith("PWM,")) {
            handleManualControl(bt_command);
        }
        else {
            SerialBT.println("Unknown command. Available: START, STOP, PWM,value,angle, MANUAL, AUTO, STATUS");
        }
        
        bt_command = "";
        new_command = false;
    }
}

// Handle manual control command "PWM,500,90"
void handleManualControl(String command) {
    int firstComma = command.indexOf(',');
    int secondComma = command.indexOf(',', firstComma + 1);
    
    if (firstComma != -1 && secondComma != -1) {
        String pwmStr = command.substring(firstComma + 1, secondComma);
        String angleStr = command.substring(secondComma + 1);
        
        motor_pwm = pwmStr.toInt();
        servo_angle = angleStr.toInt();
        
        // Constrain values
        motor_pwm = constrain(motor_pwm, 0, 800);
        servo_angle = constrain(servo_angle, 0, 180);
        
        // Send command to STM32
        sendToSTM32();
        
        String response = "Manual control - Motor: " + String(motor_pwm) + 
                         ", Servo: " + String(servo_angle);
        SerialBT.println(response);
        Serial.println(response);
    } else {
        SerialBT.println("Invalid PWM format. Use: PWM,motor_value,servo_angle");
    }
}

// Receive sensor data from STM32
void receiveSensorData() {
    if (Serial2.available() >= sizeof(Sensor_Data_t)) {
        uint8_t buffer[sizeof(Sensor_Data_t)];
        Serial2.readBytes(buffer, sizeof(Sensor_Data_t));
        
        memcpy(&rx_from_stm, buffer, sizeof(Sensor_Data_t));
        
        // Validate data
        if (rx_from_stm.header == 0x5A) {
            uint8_t checksum = calculateChecksum(buffer, sizeof(Sensor_Data_t)-1);
            if (checksum == rx_from_stm.checksum) {
                current_distance = rx_from_stm.distance;
                current_encoder = rx_from_stm.encoder;
                
                // Debug output
                Serial.print("Sensor - Distance: ");
                Serial.print(current_distance);
                Serial.print("cm, Encoder: ");
                Serial.println(current_encoder);
            }
        }
    }
}

// Calculate obstacle avoidance logic
void calculateObstacleAvoidance() {
    if (current_distance > SAFE_DISTANCE) {
        // Safe distance - full speed ahead
        motor_pwm = MAX_PWM;
        servo_angle = 90; // Straight
        current_state = STATE_MOVING;
    }
    else if (current_distance > SLOW_DOWN_DISTANCE) {
        // Slow down zone - reduce speed linearly
        float ratio = (float)(current_distance - SLOW_DOWN_DISTANCE) / 
                     (SAFE_DISTANCE - SLOW_DOWN_DISTANCE);
        motor_pwm = (int16_t)(MAX_PWM * ratio);
        servo_angle = 90; // Straight
        current_state = STATE_MOVING;
        
        // Avoid going too slow
        if (motor_pwm < 100) motor_pwm = 100;
    }
    else if (current_distance > STOP_DISTANCE) {
        // Very close - minimal speed
        motor_pwm = 100;
        servo_angle = 90;
        current_state = STATE_AVOIDING;
    }
    else {
        // Too close - STOP!
        motor_pwm = 0;
        servo_angle = 90;
        current_state = STATE_STOPPED;
        
        SerialBT.println("OBSTACLE TOO CLOSE - EMERGENCY STOP!");
        Serial.println("OBSTACLE TOO CLOSE - EMERGENCY STOP!");
    }
    
    // Additional avoidance logic for turning
    if (current_distance < 20 && current_distance > STOP_DISTANCE) {
        // If obstacle is very close but not stopping distance, try to turn
        servo_angle = 60; // Turn left
        SerialBT.println("Obstacle detected - turning left");
        Serial.println("Obstacle detected - turning left");
    }
}

// Send commands to STM32
void sendToSTM32() {
    tx_to_stm.header = 0xA5;
    tx_to_stm.pwm_motor = motor_pwm;
    tx_to_stm.pwm_servo = servo_angle;
    tx_to_stm.checksum = calculateChecksum((uint8_t*)&tx_to_stm, sizeof(ESP_Command_t)-1);
    
    Serial2.write((uint8_t*)&tx_to_stm, sizeof(ESP_Command_t));
    
    // Debug output
    Serial.print("Command to STM32 - Motor: ");
    Serial.print(motor_pwm);
    Serial.print(", Servo: ");
    Serial.println(servo_angle);
}

// Calculate checksum (XOR of all bytes)
uint8_t calculateChecksum(uint8_t* data, uint8_t len) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// Send status via Bluetooth
void sendBluetoothStatus() {
    String state_str;
    switch(current_state) {
        case STATE_STOPPED: state_str = "STOPPED"; break;
        case STATE_MOVING: state_str = "MOVING"; break;
        case STATE_AVOIDING: state_str = "AVOIDING"; break;
    }
    
    String status = "=== STATUS ===";
    status += "\nState: " + state_str;
    status += "\nDistance: " + String(current_distance) + " cm";
    status += "\nEncoder: " + String(current_encoder);
    status += "\nMotor PWM: " + String(motor_pwm);
    status += "\nServo Angle: " + String(servo_angle);
    status += "\n=================";
    
    SerialBT.println(status);
}

// Print system status to Serial
void printStatus() {
    Serial.println("=== SYSTEM STATUS ===");
    Serial.print("State: ");
    switch(current_state) {
        case STATE_STOPPED: Serial.println("STOPPED"); break;
        case STATE_MOVING: Serial.println("MOVING"); break;
        case STATE_AVOIDING: Serial.println("AVOIDING"); break;
    }
    Serial.print("Distance: "); Serial.print(current_distance); Serial.println(" cm");
    Serial.print("Encoder: "); Serial.println(current_encoder);
    Serial.print("Motor PWM: "); Serial.println(motor_pwm);
    Serial.print("Servo Angle: "); Serial.println(servo_angle);
    Serial.println("====================");
}

// ==================== BLUETOOTH COMMAND HANDLERS ====================

void handleStart() {
    current_state = STATE_MOVING;
    motor_pwm = MAX_PWM;
    servo_angle = 90;
    sendToSTM32();
    
    SerialBT.println("Robot STARTED - Autonomous mode activated");
    Serial.println("Robot STARTED - Autonomous mode");
}

void handleStop() {
    current_state = STATE_STOPPED;
    motor_pwm = 0;
    servo_angle = 90;
    sendToSTM32();
    
    SerialBT.println("Robot STOPPED");
    Serial.println("Robot STOPPED");
}

#include <WiFi.h>
#include <WebServer.h>

// WiFi credentials
const char* ssid = "Your_WiFi_SSID";
const char* password = "Your_WiFi_Password";

// UART communication with STM32
#define RXD2 16
#define TXD2 17

// Web server on port 80
WebServer server(80);

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

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    
    // Connect to WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());
    
    // Setup web server routes
    server.on("/", handleRoot);
    server.on("/control", handleControl);
    server.on("/data", handleData);
    server.on("/start", handleStart);
    server.on("/stop", handleStop);
    
    server.begin();
    Serial.println("HTTP server started");
    
    // Initialize motor to stopped state
    motor_pwm = 0;
    servo_angle = 90;
    sendToSTM32();
    
    Serial.println("ESP32 Master Controller Ready!");
    Serial.println("Send HTTP GET requests to:");
    Serial.println("  /start - Start robot");
    Serial.println("  /stop - Stop robot");
    Serial.println("  /control?pwm=500&angle=90 - Manual control");
    Serial.println("  /data - Get sensor data");
}

void loop() {
    server.handleClient();
    
    unsigned long current_time = millis();
    
    // 1. Receive sensor data from STM32 every 50ms
    if (current_time - last_sensor_time >= 50) {
        receiveSensorData();
        last_sensor_time = current_time;
    }
    
    // 2. Run control logic every 100ms
    if (current_time - last_control_time >= 100) {
        if (current_state != STATE_STOPPED) {
            calculateObstacleAvoidance();
            sendToSTM32();
        }
        last_control_time = current_time;
    }
    
    // 3. Print status every 2 seconds
    static unsigned long last_status_time = 0;
    if (current_time - last_status_time >= 2000) {
        printStatus();
        last_status_time = current_time;
    }
    
    delay(10);
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
        
        Serial.println("OBSTACLE TOO CLOSE - EMERGENCY STOP!");
    }
    
    // Additional avoidance logic for turning
    if (current_distance < 20 && current_distance > STOP_DISTANCE) {
        // If obstacle is very close but not stopping distance, try to turn
        servo_angle = 60; // Turn left
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

// Print system status
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

// ==================== WEB SERVER HANDLERS ====================

void handleRoot() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<title>Robot Line Follower Control</title>";
    html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
    html += "<style>";
    html += "body { font-family: Arial; margin: 40px; background: #f0f0f0; }";
    html += ".container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }";
    html += ".status { background: #e8f4fd; padding: 15px; border-radius: 5px; margin: 10px 0; }";
    html += ".control { background: #f8f8f8; padding: 15px; border-radius: 5px; margin: 10px 0; }";
    html += "button { padding: 10px 20px; margin: 5px; font-size: 16px; cursor: pointer; }";
    html += ".start { background: #4CAF50; color: white; border: none; }";
    html += ".stop { background: #f44336; color: white; border: none; }";
    html += ".manual { background: #2196F3; color: white; border: none; }";
    html += "input[type=\"range\"] { width: 300px; }";
    html += "</style>";
    html += "<script>";
    html += "function updateStatus() {";
    html += "    fetch('/data')";
    html += "        .then(response => response.json())";
    html += "        .then(data => {";
    html += "            document.getElementById('status').innerHTML = ";
    html += "                'Distance: ' + data.distance + ' cm<br>' +";
    html += "                'Encoder: ' + data.encoder + '<br>' +";
    html += "                'Motor PWM: ' + data.motor_pwm + '<br>' +";
    html += "                'Servo Angle: ' + data.servo_angle + '<br>' +";
    html += "                'State: ' + data.state;";
    html += "        });";
    html += "}";
    html += "function sendManualControl() {";
    html += "    const pwm = document.getElementById('pwm').value;";
    html += "    const angle = document.getElementById('angle').value;";
    html += "    window.location.href = '/control?pwm=' + pwm + '&angle=' + angle;";
    html += "}";
    html += "setInterval(updateStatus, 2000);";
    html += "updateStatus();";
    html += "</script>";
    html += "</head><body>";
    html += "<div class=\"container\">";
    html += "<h1>Robot Line Follower Control</h1>";
    html += "<div class=\"status\">";
    html += "<h3>System Status</h3>";
    html += "<div id=\"status\">Loading...</div>";
    html += "</div>";
    html += "<div class=\"control\">";
    html += "<h3>Basic Control</h3>";
    html += "<button class=\"start\" onclick=\"window.location.href='/start'\">START</button>";
    html += "<button class=\"stop\" onclick=\"window.location.href='/stop'\">STOP</button>";
    html += "</div>";
    html += "<div class=\"control\">";
    html += "<h3>Manual Control</h3>";
    html += "<label>Motor PWM (0-800): </label>";
    html += "<input type=\"range\" id=\"pwm\" min=\"0\" max=\"800\" value=\"0\">";
    html += "<span id=\"pwmValue\">0</span><br><br>";
    html += "<label>Servo Angle (0-180): </label>";
    html += "<input type=\"range\" id=\"angle\" min=\"0\" max=\"180\" value=\"90\">";
    html += "<span id=\"angleValue\">90</span><br><br>";
    html += "<button class=\"manual\" onclick=\"sendManualControl()\">Send Command</button>";
    html += "</div>";
    html += "</div>";
    html += "<script>";
    html += "document.getElementById('pwm').oninput = function() {";
    html += "    document.getElementById('pwmValue').innerHTML = this.value;";
    html += "};";
    html += "document.getElementById('angle').oninput = function() {";
    html += "    document.getElementById('angleValue').innerHTML = this.value;";
    html += "};";
    html += "</script>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
}

void handleControl() {
    if (server.hasArg("pwm")) {
        motor_pwm = server.arg("pwm").toInt();
        // Constrain values
        motor_pwm = constrain(motor_pwm, 0, 800);
    }
    if (server.hasArg("angle")) {
        servo_angle = server.arg("angle").toInt();
        servo_angle = constrain(servo_angle, 0, 180);
    }
    
    // Send command to STM32 immediately
    sendToSTM32();
    current_state = STATE_MOVING;
    
    String response = "Manual control set - Motor: " + String(motor_pwm) + 
                     ", Servo: " + String(servo_angle);
    server.send(200, "text/plain", response);
    
    Serial.println("Manual control: Motor=" + String(motor_pwm) + 
                   ", Servo=" + String(servo_angle));
}

void handleData() {
    String state_str;
    switch(current_state) {
        case STATE_STOPPED: state_str = "STOPPED"; break;
        case STATE_MOVING: state_str = "MOVING"; break;
        case STATE_AVOIDING: state_str = "AVOIDING"; break;
    }
    
    String json = "{";
    json += "\"distance\":" + String(current_distance) + ",";
    json += "\"encoder\":" + String(current_encoder) + ",";
    json += "\"motor_pwm\":" + String(motor_pwm) + ",";
    json += "\"servo_angle\":" + String(servo_angle) + ",";
    json += "\"state\":\"" + state_str + "\"";
    json += "}";
    
    server.send(200, "application/json", json);
}

void handleStart() {
    current_state = STATE_MOVING;
    motor_pwm = MAX_PWM;
    servo_angle = 90;
    sendToSTM32();
    
    server.send(200, "text/plain", "Robot STARTED - Autonomous mode activated");
    Serial.println("Robot STARTED - Autonomous mode");
}

void handleStop() {
    current_state = STATE_STOPPED;
    motor_pwm = 0;
    servo_angle = 90;
    sendToSTM32();
    
    server.send(200, "text/plain", "Robot STOPPED");
    Serial.println("Robot STOPPED");
}

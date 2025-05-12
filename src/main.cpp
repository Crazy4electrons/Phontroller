#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <LittleFS.h>
#include <Servo.h>
#include <ArduinoJson.h>

// --- Configuration Constants ---
// @brief SSID for the Wi-Fi Access Point created by the ESP8266.
const char* WIFI_SSID = "Car_AP";
// @brief Password for the Wi-Fi Access Point. Minimum 8 characters.
const char* WIFI_PASS = "knjokies";
// @brief Port for the HTTP web server (serving index.html and other files).
const uint16_t WEBSERVER_PORT = 80;
// @brief Port for the WebSocket server (real-time communication with web client).
const uint16_t WEBSOCKET_PORT = 81;

// --- Pin Definitions ---
// @brief Analog pin for reading battery voltage.
const int PIN_BATTERY = A0;
// @brief Digital pin for the ultrasonic sensors' shared Trigger pin.
const int PIN_TRIG = D0;
// @brief Digital pin for the motor driver's Enable pin (used for PWM speed control).
const int PIN_MOTOR_A_ENABLE = D1;
// @brief Digital pin for the FRONT ultrasonic sensor's Echo pin.
const int PIN_ECHO_FRONT = D2;
// @brief Digital pin for the REAR ultrasonic sensor's Echo pin. (Previously PIN_IR_SENSE)
const int PIN_ECHO_BACK = D3;
// @brief Digital pin for the steering servo motor.
const int PIN_SERVO = D4;
// @brief Digital pin for controlling the front lights.
const int PIN_FRONT_LIGHTS = D8;
// @brief Digital pin for Motor A input 1 (part of H-bridge for direction control).
const int PIN_MOTOR_A1 = D7;
// @brief Digital pin for Motor A input 2 (part of H-bridge for direction control).
const int PIN_MOTOR_A2 = D6;
// @brief Digital pin for controlling the back lights.
const int PIN_BACK_LIGHTS = D5;

// --- Hardware Configuration ---
// @brief Resistance of the first resistor in the battery voltage divider (Ohms).
const float R1_VOLTAGE_DIVIDER = 10000.0;
// @brief Resistance of the second resistor in the battery voltage divider (Ohms).
const float R2_VOLTAGE_DIVIDER = 15000.0;
// @brief Servo position for straight steering (degrees).
const int SERVO_CENTER_POS = 90;
// @brief Servo position for full left steering (degrees).
const int SERVO_LEFT_POS = 0;
// @brief Servo position for full right steering (degrees).
const int SERVO_RIGHT_POS = 180;
// @brief Maximum PWM value for motor speed control (0-255).
const int MAX_MOTOR_SPEED = 255;
// @brief Distance threshold in CM for obstacle safety stop.
const long OBSTACLE_STOP_THRESHOLD_CM = 10;


// --- Timing Constants ---
// @brief Interval in milliseconds for reading sensors and broadcasting status.
const unsigned long SENSOR_READ_INTERVAL_MS = 500;
// @brief Duration in milliseconds to apply reverse power during a HARD_STOP for braking.
const unsigned long HARD_STOP_REVERSE_MS = 100;
// @brief Duration in milliseconds for the brake lights to stay on during/after a HARD_STOP.
const unsigned long HARD_STOP_BRAKE_LIGHT_MS = 200;

// --- Enums ---
// @brief Represents the possible directions/states for the back motor.
enum class MotorDirection {
    FORWARD,
    REVERSE,
    OFF,
    HARD_STOP
};

// @brief Represents the different modes for the car's lights.
enum class LightMode {
    OFF = 0,
    FRONT_ONLY = 1,
    FRONT_AND_BACK = 2,
    AUTO = 3
};

// --- Global Objects ---
// @brief Asynchronous web server instance.
AsyncWebServer server(WEBSERVER_PORT);
// @brief WebSocket server instance for real-time communication.
WebSocketsServer webSocket(WEBSOCKET_PORT);
// @brief Servo object for controlling steering.
Servo steeringServo;
// @brief Dynamic JSON document for creating status messages.
// @details Size (1024) should be adjusted based on the expected JSON payload size.
DynamicJsonDocument jsonDoc(1024);

// --- Car State Class ---
// @brief Holds the current operational state of the car.
class CarState {
public:
    LightMode currentLightMode = LightMode::OFF;
    bool engineStarted = false;
    int batteryPercent = 0; // Stored as percentage (0-100).
    long frontObstacleDistanceCm = -1; // Front distance in cm. -1 indicates invalid reading.
    long backObstacleDistanceCm = -1;  // Back distance in cm. -1 indicates invalid reading.
    bool autoLightsEnabled = false; // Tracks if lights are in AUTO mode.
    MotorDirection currentMotorDirection = MotorDirection::OFF;
    int currentMotorSpeed = 0; // Stored as PWM value (0-MAX_MOTOR_SPEED).
    int currentServoPosition = SERVO_CENTER_POS;

    // @brief Generates a JSON string representing the car's current status.
    // @return String containing the JSON status.
    String getStatusJson() {
        jsonDoc.clear(); // Clear previous data
        jsonDoc["LightsStatus"] = static_cast<int>(currentLightMode);
        jsonDoc["EngineStatus"] = engineStarted;
        jsonDoc["FrontObstacleDistance"] = frontObstacleDistanceCm;
        jsonDoc["BackObstacleDistance"] = backObstacleDistanceCm;
        jsonDoc["BatteryStatus"] = batteryPercent;

        String output;
        serializeJson(jsonDoc, output);
        return output;
    }

    // @brief Updates the battery percentage state based on a voltage reading.
    // @param voltage The measured battery voltage.
    void updateBatteryLevel(float voltage) {
        // Example mapping: 3.0V = 0%, 4.2V = 100% (adjust thresholds to your battery)
        const float minVoltage = 3.0;
        const float maxVoltage = 4.2;
        if (voltage <= minVoltage) {
            batteryPercent = 0;
        } else if (voltage >= maxVoltage) {
            batteryPercent = 100;
        } else {
            batteryPercent = static_cast<int>(((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0);
        }
    }
};

// @brief Global instance of the CarState.
CarState myCar;

// --- Global Variables ---
// @brief Stores the millis() value of the last sensor reading time.
unsigned long lastSensorReadMillis = 0;
// @brief Stores the ID of the last WebSocket client that sent a message.
uint8_t lastConnectedClientId = 0;

// --- Sensor Reading Function (Generic) ---

// @brief Measures distance using an ultrasonic sensor with the shared trigger.
// @param echoPin The digital pin connected to the sensor's Echo pin.
// @return Distance in centimeters, or -1 if timeout occurred.
long measureDistance(int echoPin) {
    // Trigger pulse is handled once for both sensors in the loop.
    // Just measure the echo pulse for the specified pin.
    // Set a timeout for pulseIn to prevent blocking if echo is lost.
    long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout (~5m range)
    if (duration == 0) {
        return -1; // Indicate timeout or error
    }
    // Calculating the distance: Speed of sound is ~0.034 cm/us. Distance = (duration * speed) / 2.
    long distance = duration * 0.034 / 2;
    return distance; // Distance in cm
}


// @brief Reads the battery voltage via the analog pin and voltage divider.
// @return The calculated battery voltage.
float readBatteryVoltage() {
    int sensorValue = analogRead(PIN_BATTERY);
    // Convert ADC reading (0-1023) to voltage at the ADC pin (assuming 0-3.3V range).
    float voltageAtAdc = sensorValue * (3.3 / 1023.0);
    // Apply voltage divider formula to get actual battery voltage. V_battery = V_adc * ((R1 + R2) / R2)
    float batteryVoltage = voltageAtAdc * ((R1_VOLTAGE_DIVIDER + R2_VOLTAGE_DIVIDER) / R2_VOLTAGE_DIVIDER);
    return batteryVoltage;
}

// --- Control Functions ---

// @brief Sets the car's light mode.
// @param mode The desired LightMode.
void setLights(LightMode mode) {
    myCar.currentLightMode = mode;
    myCar.autoLightsEnabled = (mode == LightMode::AUTO);

    if (!myCar.autoLightsEnabled) {
        digitalWrite(PIN_FRONT_LIGHTS, LOW);
        digitalWrite(PIN_BACK_LIGHTS, LOW);
    }

    switch (mode) {
        case LightMode::FRONT_ONLY:
            digitalWrite(PIN_FRONT_LIGHTS, HIGH);
            break;
        case LightMode::FRONT_AND_BACK:
            digitalWrite(PIN_FRONT_LIGHTS, HIGH);
            digitalWrite(PIN_BACK_LIGHTS, HIGH);
            break;
        case LightMode::OFF:
            break;
        case LightMode::AUTO:
            // Lights will be controlled by setBackMotor when auto mode is active.
            break;
    }
}

// @brief Controls the back motor direction and speed.
// @param direction The desired MotorDirection.
// @param speed The desired speed (0-MAX_MOTOR_SPEED). Defaults to MAX_MOTOR_SPEED.
void setBackMotor(MotorDirection direction, int speed = MAX_MOTOR_SPEED) {
    // Prevent motor movement if engine is off, unless explicitly turning off/braking.
    if (!myCar.engineStarted && direction != MotorDirection::OFF && direction != MotorDirection::HARD_STOP) {
       direction = MotorDirection::OFF; // Force off if engine isn't started
    }

    // --- Obstacle safety checks ---
    // Stop forward motion if front obstacle is too close (and reading is valid).
    if (direction == MotorDirection::FORWARD && myCar.frontObstacleDistanceCm >= 0 && myCar.frontObstacleDistanceCm < OBSTACLE_STOP_THRESHOLD_CM) {
       Serial.println("Obstacle detected ahead! Stopping forward motion.");
       direction = MotorDirection::HARD_STOP;
    }
    // Stop reverse motion if back obstacle is too close (and reading is valid).
    if (direction == MotorDirection::REVERSE && myCar.backObstacleDistanceCm >= 0 && myCar.backObstacleDistanceCm < OBSTACLE_STOP_THRESHOLD_CM) {
       Serial.println("Obstacle detected behind! Stopping reverse motion.");
       direction = MotorDirection::HARD_STOP;
    }

    MotorDirection previousDirection = myCar.currentMotorDirection; // Store previous direction for HARD_STOP logic.
    myCar.currentMotorDirection = direction;
    myCar.currentMotorSpeed = (direction == MotorDirection::OFF || direction == MotorDirection::HARD_STOP) ? 0 : speed;

    // --- Motor Driver Pin Control ---
    bool brakeLightsOn = false;

    switch (direction) {
        case MotorDirection::FORWARD:
            digitalWrite(PIN_MOTOR_A1, HIGH);
            digitalWrite(PIN_MOTOR_A2, LOW);
            analogWrite(PIN_MOTOR_A_ENABLE, myCar.currentMotorSpeed);
            if (myCar.autoLightsEnabled) {
                digitalWrite(PIN_FRONT_LIGHTS, HIGH);
                digitalWrite(PIN_BACK_LIGHTS, LOW);
            }
            break;

        case MotorDirection::REVERSE:
            digitalWrite(PIN_MOTOR_A1, LOW);
            digitalWrite(PIN_MOTOR_A2, HIGH);
            analogWrite(PIN_MOTOR_A_ENABLE, myCar.currentMotorSpeed);
             if (myCar.autoLightsEnabled) {
                digitalWrite(PIN_FRONT_LIGHTS, LOW);
                digitalWrite(PIN_BACK_LIGHTS, HIGH);
            }
            break;

        case MotorDirection::OFF:
            digitalWrite(PIN_MOTOR_A1, LOW);
            digitalWrite(PIN_MOTOR_A2, LOW);
            analogWrite(PIN_MOTOR_A_ENABLE, 0);
             if (myCar.autoLightsEnabled) {
                digitalWrite(PIN_FRONT_LIGHTS, LOW);
                digitalWrite(PIN_BACK_LIGHTS, LOW);
            }
            break;

        case MotorDirection::HARD_STOP:
            // Apply reverse polarity briefly for braking action.
            // WARNING: This uses blocking delay. Consider a non-blocking timer for critical applications.
            if (previousDirection == MotorDirection::FORWARD) {
                digitalWrite(PIN_MOTOR_A1, LOW);
                digitalWrite(PIN_MOTOR_A2, HIGH);
                analogWrite(PIN_MOTOR_A_ENABLE, MAX_MOTOR_SPEED);
                delay(HARD_STOP_REVERSE_MS);
            } else if (previousDirection == MotorDirection::REVERSE) {
                digitalWrite(PIN_MOTOR_A1, HIGH);
                digitalWrite(PIN_MOTOR_A2, LOW);
                 analogWrite(PIN_MOTOR_A_ENABLE, MAX_MOTOR_SPEED);
                delay(HARD_STOP_REVERSE_MS);
            }
            // Ensure motors are off after braking attempt.
            digitalWrite(PIN_MOTOR_A1, LOW);
            digitalWrite(PIN_MOTOR_A2, LOW);
            analogWrite(PIN_MOTOR_A_ENABLE, 0);
            brakeLightsOn = true;
            break;
    }

    // --- Brake Light Logic ---
    // Brake lights are controlled if not in AUTO mode, or if in HARD_STOP.
    if (!myCar.autoLightsEnabled || direction == MotorDirection::HARD_STOP) {
        long delayTime = millis();
           if (brakeLightsOn) {
               digitalWrite(PIN_BACK_LIGHTS, HIGH);
               // WARNING: Blocking delay for brake light duration.
                delay(HARD_STOP_BRAKE_LIGHT_MS);
                if(delayTime + HARD_STOP_BRAKE_LIGHT_MS < millis()) {
                   // If the delay time has passed, turn off the brake lights.
                   digitalWrite(PIN_BACK_LIGHTS, LOW);
                }
                digitalWrite(PIN_BACK_LIGHTS, LOW);
           } else {
               // If not braking and not in AUTO, turn off back lights unless explicitly in FRONT_AND_BACK mode.
               if (myCar.currentLightMode != LightMode::FRONT_AND_BACK && !myCar.autoLightsEnabled) {
                   digitalWrite(PIN_BACK_LIGHTS, LOW);
               }
           }
    }
}

// @brief Sets the steering servo position.
// @param position The desired servo angle (0-180 degrees).
void setSteering(int position) {
    // Constrain position to valid servo range.
    position = constrain(position, SERVO_LEFT_POS, SERVO_RIGHT_POS);
    // Only allow steering if engine is started, unless centering.
    if (!myCar.engineStarted && position != SERVO_CENTER_POS) {
        position = SERVO_CENTER_POS; // Force center if engine off
    }
    myCar.currentServoPosition = position;
    steeringServo.write(position);
}

// @brief Stops all motors and centers the steering.
void stopAllMotors() {
    setBackMotor(MotorDirection::OFF);
    setSteering(SERVO_CENTER_POS);
}

// @brief Toggles the engine state (started/stopped).
void toggleEngine() {
    myCar.engineStarted = !myCar.engineStarted;
    Serial.printf("Engine toggled: %s\n", myCar.engineStarted ? "ON" : "OFF");
    if (!myCar.engineStarted) {
        stopAllMotors(); // Ensure motors are off when engine is turned off
    }
    // Send immediate status update after engine toggle.
    String statusPayload = myCar.getStatusJson();
    if (lastConnectedClientId != 0 && webSocket.clientIsConnected(lastConnectedClientId)) {
           webSocket.sendTXT(lastConnectedClientId, statusPayload);
    } else {
           webSocket.broadcastTXT(statusPayload);
    }
}

// --- WebSocket Event Handler ---
// @brief Handles incoming WebSocket events (connect, disconnect, text, binary, etc.).
// @param num The client ID.
// @param type The type of WebSocket event.
// @param payload The message payload (if any).
// @param length The length of the payload.
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    lastConnectedClientId = num; // Store the most recent client ID

    switch (type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            // Stop motors if no clients are connected.
            if (webSocket.connectedClients() == 0) { stopAllMotors(); }
            break;

        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
            // Send current status to newly connected client.
            String statusPayload = myCar.getStatusJson();
            webSocket.sendTXT(num, statusPayload);
            break;
        }

        case WStype_TEXT: {
            String command = String((char*)payload);
            command.trim();

            // --- Command Parsing ---
            if (command.equalsIgnoreCase("enginestart")) {
                toggleEngine();
            } else if (command.equalsIgnoreCase("stop")) {
                stopAllMotors();
            } else if (command.equalsIgnoreCase("stopacc") || command.equalsIgnoreCase("brake")) {
                setBackMotor(MotorDirection::HARD_STOP);
            } else if (command.equalsIgnoreCase("forward")) {
                 setBackMotor(MotorDirection::FORWARD, MAX_MOTOR_SPEED);
             } else if (command.equalsIgnoreCase("backward") || command.equalsIgnoreCase("reverse")) {
                 setBackMotor(MotorDirection::REVERSE, MAX_MOTOR_SPEED);
             } else if (command.equalsIgnoreCase("left")) {
                 setSteering(SERVO_LEFT_POS);
             } else if (command.equalsIgnoreCase("right")) {
                 setSteering(SERVO_RIGHT_POS);
             } else if (command.equalsIgnoreCase("center")) {
                 setSteering(SERVO_CENTER_POS);
             } else if (command.startsWith("F")) { // Steering angle command, e.g., F90
                 int angle = command.substring(1).toInt();
                 setSteering(angle);
             } else if (command.startsWith("B")) { // Speed/Direction command, e.g., B75 (forward 50%), B25 (reverse 50%)
                 int value = command.substring(1).toInt();
                 value = constrain(value, 0, 100); // Expect 0-100 input range

                 if (value == 50) { // Center means stop
                     setBackMotor(MotorDirection::OFF);
                 } else if (value > 50) { // Forward: Map 51-100 to 0-MAX_MOTOR_SPEED
                     int speed = map(value, 51, 100, 0, MAX_MOTOR_SPEED);
                     setBackMotor(MotorDirection::FORWARD, speed);
                 } else { // Reverse (value < 50): Map 49-0 to 0-MAX_MOTOR_SPEED
                     int speed = map(value, 49, 0, 0, MAX_MOTOR_SPEED);
                     setBackMotor(MotorDirection::REVERSE, speed);
                 }
             } else if (command.equalsIgnoreCase("L0")) { // Light modes
                 setLights(LightMode::OFF);
             } else if (command.equalsIgnoreCase("L1")) {
                 setLights(LightMode::FRONT_ONLY);
             } else if (command.equalsIgnoreCase("L2")) {
                 setLights(LightMode::FRONT_AND_BACK);
             } else if (command.equalsIgnoreCase("L3")) {
                 setLights(LightMode::AUTO);
             } else if (command.equalsIgnoreCase("status")) { // Explicit status request
                 String statusPayload = myCar.getStatusJson();
                 webSocket.sendTXT(num, statusPayload);
             }
             else {
                 Serial.printf("Unknown command: %s\n", command.c_str());
             }

             // Send updated status after processing command (unless it was just a status request or engine toggle, which already sends).
             if (!command.equalsIgnoreCase("status") && !command.equalsIgnoreCase("enginestart")) {
                 String statusPayload = myCar.getStatusJson();
                 webSocket.sendTXT(num, statusPayload);
             }
            break;
        }

        case WStype_BIN:
            Serial.printf("[%u] Got binary length: %u\n", num, length);
            break;

        default:
            Serial.printf("Unhandled WebSocket event type: %d\n", type);
            break;
    }
}

// --- Setup ---
// @brief Initializes the ESP8266, peripherals, network, and servers.
void setup() {
    Serial.begin(115200); // Initialize serial communication for debugging.
    Serial.println("\nCar Controller Starting...");

    // --- Initialize Filesystem ---
    if (!LittleFS.begin()) {
        Serial.println("Failed to mount LittleFS");
        // Halt program execution if filesystem fails.
        while (true) { delay(1000); }
    }
    Serial.println("LittleFS mounted.");
     if (!LittleFS.exists("/index.html")) {
         Serial.println("ERROR: index.html not found in LittleFS!");
         // Consider adding code to serve a basic error page or halt if index.html is critical.
     }

    // --- Initialize Pins ---
    pinMode(PIN_MOTOR_A1, OUTPUT);
    pinMode(PIN_MOTOR_A2, OUTPUT);
    pinMode(PIN_MOTOR_A_ENABLE, OUTPUT);
    pinMode(PIN_FRONT_LIGHTS, OUTPUT);
    pinMode(PIN_BACK_LIGHTS, OUTPUT);
    pinMode(PIN_TRIG, OUTPUT);         // Shared trigger pin for both ultrasonic sensors.
    pinMode(PIN_ECHO_FRONT, INPUT);    // Front ultrasonic sensor echo pin.
    pinMode(PIN_ECHO_BACK, INPUT);     // Rear ultrasonic sensor echo pin (was IR_SENSE).
    pinMode(PIN_BATTERY, INPUT);       // Battery sensor pin is analog input.
    steeringServo.attach(PIN_SERVO);     // Attach the Servo object to the servo pin.

    // --- Initial State ---
    stopAllMotors(); // Set motors off and servo to center initially.
    setLights(LightMode::OFF); // Start with lights off.

    // --- Wi-Fi Access Point Setup ---
    Serial.printf("Setting up AP: %s\n", WIFI_SSID);
    WiFi.mode(WIFI_AP); // Configure ESP8266 as an Access Point.
    WiFi.softAP(WIFI_SSID, WIFI_PASS); // Start the Access Point.
    IPAddress IP = WiFi.softAPIP();    // Get the IP address assigned to the AP interface.
    Serial.print("AP IP address: ");
    Serial.println(IP);

    // --- Web Server Setup ---
    // Serve files from the root of LittleFS ("/") when a client requests files from the root ("/").
    // Set "index.html" as the default file for directory requests.
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    // Handle requests for files not found.
    server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "Not found");
    });
    server.begin(); // Start the HTTP web server.
    Serial.println("HTTP server started.");

    // --- WebSocket Server Setup ---
    webSocket.begin();          // Start the WebSocket server.
    webSocket.onEvent(webSocketEvent); // Register the event handler function.
    Serial.println("WebSocket server started.");

    // --- Startup Indicator (Simple Blocking Blink) ---
    Serial.println("Setup complete. Running startup blink...");
    for (int i = 0; i < 3; ++i) { // Blink front and back lights 3 times
        digitalWrite(PIN_FRONT_LIGHTS, HIGH);
        digitalWrite(PIN_BACK_LIGHTS, HIGH);
        delay(150);
        digitalWrite(PIN_FRONT_LIGHTS, LOW);
        digitalWrite(PIN_BACK_LIGHTS, LOW);
        delay(150);
    }
     Serial.println("Ready.");
}

// --- Main Loop ---
// @brief The main program loop, continuously handling WebSocket traffic and periodic tasks.
void loop() {
    // Handle WebSocket connections and messages. Must be called continuously.
    webSocket.loop();

    // Non-blocking sensor reads and status broadcast timer.
    unsigned long currentMillis = millis();

    // Check if enough time has passed since the last sensor read and broadcast.
    if (currentMillis - lastSensorReadMillis >= SENSOR_READ_INTERVAL_MS) {
        lastSensorReadMillis = currentMillis; // Update the last read time.

        // --- Read Ultrasonic Sensors (Shared Trigger) ---
        // Trigger both sensors simultaneously.
        digitalWrite(PIN_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_TRIG, LOW);

        // Read echo from the front sensor.
        long durationFront = pulseIn(PIN_ECHO_FRONT, HIGH, 30000); // 30ms timeout
        if (durationFront == 0) {
            myCar.frontObstacleDistanceCm = -1; // Indicate timeout
        } else {
            myCar.frontObstacleDistanceCm = durationFront * 0.034 / 2; // Convert to cm
        }

        // Read echo from the back sensor.
        long durationBack = pulseIn(PIN_ECHO_BACK, HIGH, 30000); // 30ms timeout
        if (durationBack == 0) {
             myCar.backObstacleDistanceCm = -1; // Indicate timeout
        } else {
            myCar.backObstacleDistanceCm = durationBack * 0.034 / 2; // Convert to cm
        }

        // --- Read Battery Voltage ---
        float voltage = readBatteryVoltage();
        myCar.updateBatteryLevel(voltage); // Update battery percentage state.

        // --- Safety Checks based on sensor data ---
        // Stop forward motion if a valid front obstacle reading is below the threshold.
         if (myCar.currentMotorDirection == MotorDirection::FORWARD &&
             myCar.frontObstacleDistanceCm >= 0 &&
             myCar.frontObstacleDistanceCm < OBSTACLE_STOP_THRESHOLD_CM) {
             Serial.println("LOOP Safety Stop: Obstacle detected ahead!");
             setBackMotor(MotorDirection::HARD_STOP);
         }
        // Stop backward motion if a valid back obstacle reading is below the threshold.
         if (myCar.currentMotorDirection == MotorDirection::REVERSE &&
             myCar.backObstacleDistanceCm >= 0 &&
             myCar.backObstacleDistanceCm < OBSTACLE_STOP_THRESHOLD_CM) {
             Serial.println("LOOP Safety Stop: Obstacle detected behind!");
              setBackMotor(MotorDirection::HARD_STOP);
         }

        // Broadcast the updated status to all connected clients.
        String statusPayload = myCar.getStatusJson();
        webSocket.broadcastTXT(statusPayload);

         // Optional: Print status to Serial for monitoring.
        Serial.printf(" Batt V: %.2f, Front Dist: %ld cm, Back Dist: %ld cm\n", voltage, myCar.frontObstacleDistanceCm, myCar.backObstacleDistanceCm);
    }

    // Other non-blocking tasks can go here.
}
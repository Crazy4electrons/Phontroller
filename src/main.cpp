#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <LittleFS.h>
#include <Servo.h>

Servo myServo;
const char* ssid = "Car_AP";
const char* password = "knjokies"; // Minimum 8 characters
AsyncWebServer server(80);
WebSocketsServer webSocket(81);
// Motor control pins
#define BatteryPin A0
#define Servo D2
#define MotorAEnable D3
#define EchPin D4
#define FrontLights D5
#define MotorA1 D6
#define MotorA2 D7
#define IRSense D8
#define BackLights D9
#define TrigPin D10
unsigned long previousMillis = 0;  // Stores the last time the action was performed
unsigned long previousMillis2 = 0;  // Stores the last time the action was performed
const long interval = 500;        // Interval at which to perform the action (in milliseconds)
int desiredPosition = 90;
int backMotorSpeed = 255;
int stopIr = 0;

// Define the resistor values
const float R1 = 10000.0; // 10k ohms
const float R2 = 15000.0; // 15k ohms


float readBatteryLevel() {
    int sensorValue = analogRead(A0);
    float voltage = sensorValue * (3.3 / 1023.0); // Convert ADC value to voltage
    float batteryVoltage = voltage * ((R1 + R2) / R2); // Calculate actual battery voltage

    return batteryVoltage;
}

bool motorLights = false;
int previourBackMotorDirection ;
void BackMotor(int direction = 2) {
    previourBackMotorDirection = direction;
    switch(direction) {
    case 0://forwards
        if(motorLights) {
            digitalWrite(FrontLights, HIGH);
            digitalWrite(BackLights, LOW);
        }
        digitalWrite(MotorA1, HIGH);
        digitalWrite(MotorA2, LOW);
        (stopIr) ? stopIr = 0 : stopIr = 0;
        break;
    case 1://reverse
        if(motorLights) {
            digitalWrite(FrontLights, LOW);
            digitalWrite(BackLights, HIGH);
        }
        if(!stopIr) {
            digitalWrite(MotorA1, LOW);
            digitalWrite(MotorA2, HIGH);
        } 
        break;
    case 2://motors off
        digitalWrite(MotorA1, LOW);
        digitalWrite(MotorA2, LOW);
        if(motorLights) {
            digitalWrite(FrontLights, LOW);
            digitalWrite(BackLights, LOW);
        }
        break;
    case 3://Hard stop
        if(previourBackMotorDirection == 0) {
            digitalWrite(MotorA1, LOW);
            digitalWrite(MotorA2, HIGH);
            delay(200);
            digitalWrite(MotorA1, LOW);
        } else if(previourBackMotorDirection == 1) {

            digitalWrite(MotorA1, HIGH);
            digitalWrite(MotorA2, LOW);
            delay(200);
            digitalWrite(MotorA2, LOW);
        } else {
            digitalWrite(MotorA1, HIGH);
            digitalWrite(MotorA2, HIGH);
        }
        if(motorLights) {
            digitalWrite(FrontLights, LOW);
        }
        digitalWrite(BackLights, HIGH);
        delay(200);
        digitalWrite(MotorA1, LOW);
        digitalWrite(MotorA2, LOW);

        break;
    }
}



//the following functions is used to turn a the car
void frontMotor(int direction = 90) {
    myServo.write(direction);
}


//stop all motors
void stopMotors() {
    BackMotor(2);
    frontMotor(90);
}

void turnLightsOn(int mode) {
    switch (mode) {
    case 1:
        digitalWrite(FrontLights, HIGH);
        motorLights  = false;
        break;
    case 2:
        digitalWrite(FrontLights, HIGH);
        digitalWrite(BackLights, HIGH);
        motorLights  = false;
        break;
    case 3:
        motorLights  = true;
        break;
    default:
        digitalWrite(FrontLights, LOW);
        digitalWrite(BackLights, LOW);
        motorLights  = false;
        break;
    }
}

// Motor control functions
int engineStart = 0;
void EngineStart() {
    engineStart = !engineStart;
}

class CarDetail {
  public:
    int LightsMode;
    int engineStarted;
    int BatteryStatus;
    int ObstacleDetected;

    // Method to display car details
    String getDetails() {
        ObstacleDetected = stopIr;
        BatteryStatus = readBatteryLevel();
        engineStarted = engineStart;

        String message = "{";
        message += "\"LightsStatus\":" + String(LightsMode) + ",";
        message += "\"EngineStatus\":" + String(engineStarted) + ",";
        message += "\"ObstacleDetected\":" + String(ObstacleDetected) + ",";
        message += "\"BatteryStatus\":" + String(BatteryStatus);
        message += "}";
        return message;

    }
};
CarDetail MyCar;

int clientId;
bool frontMotorTurned = false;
// WebSocket event handler

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    clientId = num;
    String command = String((char*)payload);
    Serial.println(command);
    if(type == WStype_DISCONNECTED) {
        Serial.printf("[%u] Disconnected!\n", num);
    } else if(type == WStype_CONNECTED) {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
        // Send a welcome message to the newly connected client
    } else if(type ==  WStype_TEXT) {


        Serial.print("command received:");
        Serial.println(command);

        if(command == "enginestart") {//engine start
            EngineStart();
            Serial.println("engine Start");
        } else if(command.charAt(0) == 'F') {
            (engineStart) ? frontMotor(command.substring(1).toInt()) : frontMotor(90);

        } else if(command == "center") {
            frontMotor(90);
        } else if(command == "left") {
            (engineStart) ? frontMotor(0): frontMotor(90);
        } else if(command == "right") {
            (engineStart) ? frontMotor(180): frontMotor(90);
        } else if(command.charAt(0) == 'B') {
            if(command.substring(1).toInt() >= 0 && command.substring(1).toInt() <= 50) {
                BackMotor(1);
                backMotorSpeed = map(command.substring(1).toInt(), 0, 50, 255, 0);
            } else if(command.substring(1).toInt() >= 51 && command.substring(1).toInt() <= 100) {
                BackMotor(0);
                backMotorSpeed = map(command.substring(1).toInt(), 51, 100, 255, 0);

            };
            analogWrite(MotorAEnable, backMotorSpeed);
        } else if(command == "stopAcc") {
            BackMotor(2);
        } else if (command == "forward") {//forward
            if(engineStart) {
                BackMotor(0);
                analogWrite(MotorAEnable, 0);
            }
        } else if (command == "backward") {//backward
            if(engineStart) {
                BackMotor(1);
                analogWrite(MotorAEnable, 0);
            }

        } else if(command == "L0") {//lights off
            turnLightsOn(0);
            MyCar.LightsMode = 0;
        } else if(command == "L1") {//lights on
            turnLightsOn(1);
            MyCar.LightsMode = 1;
        } else if(command == "L2") {//lights on
            turnLightsOn(2);
            MyCar.LightsMode = 2;
        } else if(command == "L3") {//lights on
            turnLightsOn(3);
            MyCar.LightsMode = 3;
        } else if(command == "stop") {//stop
            stopMotors();

        } else {
            webSocket.sendTXT(clientId, MyCar.getDetails().c_str());
        }


    } else if(type == WStype_BIN) {
        Serial.printf("[%u] Received binary data\n", num);
    }
    if(!engineStart) {//if engine is off stop all motors
        stopMotors();
    }
    webSocket.sendTXT(clientId, MyCar.getDetails().c_str());
}


void setup() {
    Serial.begin(9600);

    // Set the Wi-Fi mode to AP
    WiFi.mode(WIFI_AP);
    // Start the Access Point
    WiFi.softAP(ssid, password);

    // Get the IP address of the AP
    IPAddress IP = WiFi.softAPIP();
    Serial.print("Access Point started. IP address: ");
    Serial.println(IP);

    if (!LittleFS.begin()) {
        Serial.println("Failed to mount LittleFS");
        return;
    }
    // Check if index.html exists
    if (!LittleFS.exists("/index.html")) {
        Serial.println("index.html not found in LittleFS");
        return;
    }

    // Start web server
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    server.onNotFound([](AsyncWebServerRequest * request) {
        request->send(404, "text/plain", "Not found");
    });
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
        request->send(LittleFS, "/index.html", "text/html");
    });

    server.begin();
    Serial.println("HTTP server started");

    // Start WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("WebSocket server started");
    // Initialize motor pins
    pinMode(MotorA1, OUTPUT);
    pinMode(MotorA2, OUTPUT);
    myServo.attach(Servo);
    pinMode(MotorAEnable, OUTPUT);
    // Stop motors initially
    stopMotors();

    //lights setup
    pinMode(FrontLights, OUTPUT);
    pinMode(BackLights, OUTPUT);
    // for(int i = 0; i <= 10; i++) {
    //     turnLightsOn(2);
    //     delay(200);
    //     turnLightsOn(0);
    //     delay(100);
    //     turnLightsOn(2);
    //     delay(200);
    //     turnLightsOn(0);
    // }
    
    //irSensor setup
    pinMode(IRSense, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IRSense), [] {
        stopIr = 1;
        BackMotor(3);
        Serial.println("Obstacle detected");
    }, FALLING);

    delay(500);
    Serial.println("done initializing");

}
void loop() {
    // Handle WebSocket events
    webSocket.loop();

    // Get the current time
    unsigned long currentMillis = millis();

    // Check if the interval has passed
    if (currentMillis - previousMillis >= interval) {
        // Save the last time data was sent
        previousMillis = currentMillis;

        // Create a JSON string with car details
        String message = MyCar.getDetails();

        // Send the message to all connected clients
        webSocket.broadcastTXT(message);
        Serial.println("Data sent: " + message);
        Serial.printf("Battery level: %u", analogRead(BatteryPin));
        Serial.printf("Desired Position: %u", desiredPosition);
    }
}
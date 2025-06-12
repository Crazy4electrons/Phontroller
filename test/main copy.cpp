#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <LittleFS.h>


// Motor control pins
#define MotorA1 D6
#define MotorA2 D7
#define MotorB1 D4
#define MotorB2 D5
#define IRSense D2
#define PotSense A0
#define FrontLights D3
#define BackLights D8



// Access Point credentials
const char* ssid = "Car_AP";
const char* password = "knjokies"; // Minimum 8 characters



class CarDetail {
  public:
    int LightsMode;
    bool engineStart;

    // Method to display car details
    String getDetails() {
        String message = "{";
        message += "\"LightsMode\":" + String(LightsMode) + ",";
        message += "\"engineStart\":" + String(engineStart);
        message += "}";
        return message;

    }
};
CarDetail MyCar;

// Web server and WebSocket
AsyncWebServer server(80);
WebSocketsServer webSocket(81);

// filepath: /c:/Users/eswartz/Documents/PlatformIO/Projects/Phonetroller Car/src/main.cpp
const int batteryPin = D8; // Analog pin to read battery voltage
// const float voltageDividerRatio = 10/15; // Ratio of the voltage divider
// const float referenceVoltage = 3.3; // Reference voltage of the ADC (e.g., 3.3V for ESP8266)// filepath: /c:/Users/eswartz/Documents/PlatformIO/Projects/Phonetroller Car/src/main.cpp
float readBatteryLevel() {
    int batteryVoltage = digitalRead(batteryPin);//analogRead(batteryPin);
    // float batteryVoltage = //(analogValue / 1023.0) * referenceVoltage * voltageDividerRatio;
    return batteryVoltage;
}
// Motor control functions
bool engineStart = false;
bool motorLights = false;
int previourBackMotorDirection ;
void BackMotor(int direction = 0) {
    previourBackMotorDirection = direction;
    switch(direction) {
    case 0://forwards
        if(motorLights) {
            digitalWrite(FrontLights, HIGH);
            digitalWrite(BackLights, LOW);
        }
        digitalWrite(MotorA1, LOW);
        digitalWrite(MotorA2, HIGH);
        break;
    case 1://reverse
        if(motorLights) {
            digitalWrite(FrontLights, LOW);
            digitalWrite(BackLights, HIGH);
        }
        digitalWrite(MotorA1, HIGH);
        digitalWrite(MotorA2, LOW);
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
            digitalWrite(MotorA1, HIGH);
            digitalWrite(MotorA2, LOW);
            delay(200);
            digitalWrite(MotorA1, LOW);
        } else if(previourBackMotorDirection == 1) {
            digitalWrite(MotorA1, LOW);
            digitalWrite(MotorA2, HIGH);
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
//the following function is used to turn a the car
int potHigh, potLow = 0;
void frontMotor(int direction = 2, bool inverseDirection = false) {
    int potRead = analogRead(PotSense);
    if(inverseDirection) {
        if(direction == 0) {
            direction = 1;
        }
        if(direction == 1) {
            direction = 0;
        }

        switch(direction) {
        case 0://left
            if(potRead >= potLow) {
                digitalWrite(MotorB1, LOW);
                digitalWrite(MotorB2, HIGH);
            } else {
                digitalWrite(MotorB2, LOW);
            }
            break;
        case 1://Right
            if(potRead <= potHigh) {
                digitalWrite(MotorB1, HIGH);
                digitalWrite(MotorB2, LOW);
            } else {
                digitalWrite(MotorB1, LOW);
            }
            break;
        case 2://motors off
            digitalWrite(MotorB1, LOW);
            digitalWrite(MotorB2, LOW);
            break;
        case 3://Center
            int center = ((potHigh - potLow) + potLow) / 2;
            while(potRead > center + 2 && potRead < center - 2) {
                if(potRead > center + 2) {
                    digitalWrite(MotorB1, LOW);
                    digitalWrite(MotorB2, HIGH);
                    delay(1);
                    potRead = analogRead(PotSense);
                } else if(potRead < center - 2) {
                    digitalWrite(MotorB1, HIGH);
                    digitalWrite(MotorB2, LOW);
                    delay(1);
                    potRead = analogRead(PotSense);
                }
            }
            digitalWrite(MotorB1, LOW);
            digitalWrite(MotorB2, LOW);
        }
    }

}
//this initializes the front motor potread values
void initializeFrontMotor() {
    frontMotor(0, false);
    delay(1000);
    int firstValue = analogRead(PotSense);
    delay(1000);
    frontMotor(1, false);
    delay(1000);
    int secondValue = analogRead(PotSense);
    delay(1000);
    frontMotor(2, false);
    if(firstValue > secondValue) {
        potHigh = firstValue;
        potLow = secondValue;
    } else {
        potHigh = secondValue;
        potLow = firstValue;
    }
}
//stop all motors
void stopMotors() {
    BackMotor(2);
    frontMotor(2, false);
}

//stop the motors to move forward
int irValue;
void IrSense() {
    irValue = !digitalRead(IRSense);
    if(irValue == 1) {
        stopMotors();
    }
}

void turnLightsOn(int mode) {
    switch (mode) {
    case 1:
        digitalWrite(FrontLights, HIGH);
        break;
    case 2:
        digitalWrite(FrontLights, HIGH);
        digitalWrite(BackLights, HIGH);
        break;
    case 3:
        motorLights  = !motorLights;
        break;
    default:
        digitalWrite(FrontLights, LOW);
        digitalWrite(BackLights, LOW);
        break;
    }
    digitalWrite(BackLights, HIGH);
}


void sendDataToClient(uint8_t clientNum, const char* message) {
    webSocket.sendTXT(clientNum, message);
}

void sendDataToAllClients(const char* message) {
    webSocket.broadcastTXT(message);
}

// WebSocket event handler
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    String command = String((char*)payload);
    if(type == WStype_DISCONNECTED) {

        Serial.printf("[%u] Disconnected!\n", num);
    } else if(type == WStype_CONNECTED) {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
        // Send a welcome message to the newly connected client
    } else if(type ==  WStype_TEXT) {
        // Serial.printf("[%u] Received text: %s\n", num, payload);
        // // Echo the message back
        // webSocket.sendTXT(num, payload, length);

        Serial.printf("command received: %s \n", command);
        if (command == "forward") {//forward
            BackMotor(0);
        } else if (command == "backward") {//backward
            if(engineStart) {
                BackMotor(1);
            }
        } else if (command == "stopAcc") {//stop
            if(engineStart) {

                BackMotor(3);
            }
        } else if (command == "left") {//left
            if(engineStart) {

                frontMotor(0);
            }
        } else if (command == "right") {//right
            if(engineStart) {

                frontMotor(1);
            }
        } else if(command == "center") {//center
            if(engineStart) {

                frontMotor(3);
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
        } else if(command == "engineStart") {//engine start
            engineStart = !engineStart;
            MyCar.engineStart = engineStart;
        }
        if(!engineStart) {//if engine is off stop all motors
            stopMotors();
        }
    } else if(type == WStype_BIN) {
        Serial.printf("[%u] Received binary data\n", num);
    }
    sendDataToClient(num, MyCar.getDetails().c_str());

}


void setup() {
    Serial.begin(115200);
    delay(10);
   
    // Set up the Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    Serial.println("Access Point started");
    Serial.print("IP Address: ");
    // Serial.println(WiFi.broadcastIP());
    Serial.println(WiFi.softAPIP()); // Typically 192.168.4.1


    // Start WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    if (!LittleFS.begin()) {
        Serial.println("Failed to mount LittleFS");
        return;
    }

    // Start web server
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    server.begin();
     // Initialize motor pins
    pinMode(MotorA1, OUTPUT);
    pinMode(MotorA2, OUTPUT);
    pinMode(MotorB1, OUTPUT);
    pinMode(MotorB2, OUTPUT);

    // Stop motors initially
    // initializeFrontMotor();
    stopMotors();


    Serial.println("done initializing");
}

void loop() {
    webSocket.loop();
    IrSense();
}





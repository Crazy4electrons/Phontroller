# Phontroller: RC Car Controller

Phontroller is a web-based remote control system for an RC car, powered by an ESP8266 microcontroller. The project allows users to control the car's movement, steering, lights, and other features through a responsive web interface. The system communicates in real-time using WebSockets, providing a seamless and interactive experience.

## Features

### 1. **Web-Based Control Interface**
- A responsive web interface accessible via a browser.
- Control buttons for forward, backward, left, and right movement.
- Toggle buttons for switching between button-based and slider-based controls for speed and steering.
- Real-time status updates for battery level, connection status, and obstacle detection.

### 2. **Real-Time Communication**
- Uses WebSockets for low-latency communication between the web interface and the ESP8266.
- Sends commands and receives status updates in JSON format.

### 3. **Obstacle Detection**
- Ultrasonic sensor for detecting obstacles in front of the car.
- IR sensor for detecting obstacles behind the car.
- Automatic safety stop when obstacles are detected.

### 4. **Lighting Control**
- Multiple light modes:
  - Off
  - Front lights only
  - Front and back lights
  - Automatic mode (lights adjust based on movement direction).

### 5. **Steering Control**
- Steering wheel UI for precise control.
- Button-based steering for simpler control.
- Servo motor for steering adjustments.

### 6. **Speed Control**
- Slider-based speed control for fine adjustments.
- Button-based forward and backward movement.

### 7. **Battery Monitoring**
- Real-time battery percentage display.
- Alerts for low battery levels.

### 8. **Engine Toggle**
- Start and stop the engine with a single button.

## Hardware Requirements
- **ESP8266** microcontroller.
- **Servo motor** for steering.
- **DC motor** with an H-bridge motor driver for movement.
- **Ultrasonic sensor** for front obstacle detection.
- **IR sensor** for back obstacle detection.
- **LEDs** for front and back lights.
- **Voltage divider** for battery monitoring.

## Software Components
### 1. **ESP8266 Firmware**
- Written in C++ using the Arduino framework.
- Handles WebSocket communication, sensor readings, and motor control.
- Serves the web interface files using LittleFS.

### 2. **Web Interface**
- **HTML**: Provides the structure of the control interface.
- **CSS**: Styles the interface for a responsive and user-friendly design.
- **JavaScript**: Implements the control logic, WebSocket communication, and UI updates.

## How It Works
1. The ESP8266 creates a Wi-Fi Access Point (AP) that users can connect to.
2. The web interface is served from the ESP8266's filesystem (LittleFS).
3. Users interact with the web interface to send commands (e.g., move forward, turn left).
4. Commands are sent to the ESP8266 via WebSockets.
5. The ESP8266 processes the commands and controls the motors, lights, and sensors accordingly.
6. Sensor data (e.g., battery level, obstacle detection) is sent back to the web interface in real-time.

## Setup Instructions
1. **Hardware Setup**:
   - Connect the motors, sensors, and LEDs to the ESP8266 as per the pin configuration in `main.cpp`.
   - Ensure the battery voltage divider is correctly configured for accurate readings.

2. **Software Setup**:
   - Install [PlatformIO](https://platformio.org/) in your IDE (e.g., VS Code).
   - Clone this repository and open the project in PlatformIO.
   - Upload the firmware to the ESP8266.
   - Upload the web interface files (`index.html`, `style.css`, `main.js`) to the ESP8266's LittleFS.

3. **Usage**:
   - Connect to the ESP8266's Wi-Fi AP (`Car_AP` with password `knjokies`).
   - Open a browser and navigate to the ESP8266's IP address (default: `192.168.4.1`).
   - Use the web interface to control the car.

## File Structure
- **`src/main.cpp`**: ESP8266 firmware for handling motor control, WebSocket communication, and sensor readings.
- **`data/index.html`**: Web interface for controlling the car.
- **`data/style.css`**: Styles for the web interface.
- **`data/main.js`**: JavaScript logic for the web interface.
- **`readme.md`**: Project documentation.

## Future Improvements
- Add support for additional sensors (e.g., GPS, accelerometer).
- Implement a mobile app for control.
- Enhance obstacle detection with more advanced algorithms.
- Add support for multiple clients controlling the car simultaneously.

## License
This project is open-source and available under the MIT License.
/*
 * JavaScript file for the RC Car Control web interface.
 * Handles user interactions, WebSocket communication, and UI updates.
 */

(function () { // Immediately Invoked Function Expression to create a private scope
    'use strict'; // Enforce stricter parsing and error handling

    // --- Constants ---
    // Interval in milliseconds to throttle sending commands from continuous controls (steering, slider)
    const WEBSOCKET_THROTTLE_MS = 50;
    // Short delay in milliseconds before removing the 'active' class from a button after release
    const BUTTON_ACTIVE_DELAY_MS = 100;
    // Distance threshold in cm for triggering the obstacle warning in the UI.
    // Note: This is separate from the safety stop threshold on the ESP8266.
    const OBSTACLE_DISTANCE_THRESHOLD_CM = 20;
    // Maximum rotation angle in degrees for the steering wheel UI element.
    const STEERING_LIMIT_DEG = 90;
    // The value of the speed slider that corresponds to the stopped state.
    const SLIDER_CENTER_VALUE = 50;

    // Mapping of logical actions to the specific WebSocket command strings expected by the ESP8266.
    const Commands = {
        STEER_PREFIX: 'F', // Command prefix for steering angle, e.g., 'F90'
        SPEED_PREFIX: 'B', // Command prefix for speed/direction value (0-100), e.g., 'B75'
        LEFT: 'left', // Command for discrete left turn
        RIGHT: 'right', // Command for discrete right turn
        CENTER: 'center', // Command for centering steering
        FORWARD: 'forward', // Command for discrete forward motion
        BACKWARD: 'backward', // Command for discrete backward motion
        STOP_ACC: 'stopAcc', // Command for braking/hard stop (matches command used for HARD_STOP)
        ENGINE_TOGGLE: 'enginestart', // Command to toggle engine state
        LIGHT_PREFIX: 'L', // Command prefix for light mode, e.g., 'L1'
    };

    // --- DOM Element Cache ---
    // Stores references to frequently used HTML elements for efficient access.
    const DOMElements = {
        body: document.body,
        connectionStatus: document.getElementById('connection-status'),
        obstacleDetected: document.getElementById('obstacle-detected'),
        batteryStatus: document.getElementById('battery-status'),
        leftButton: document.getElementById('left-button'),
        rightButton: document.getElementById('right-button'),
        steeringWheelContainer: document.getElementById('steering-wheel-container'),
        steeringWheel: document.getElementById('steering-wheel'), // Draggable wheel element
        forwardButton: document.getElementById('forward-button'),
        backwardButton: document.getElementById('backward-button'),
        speedSliderContainer: document.getElementById('speed-slider-container'),
        speedSlider: document.getElementById('speed-slider'),
        brakeButton: document.getElementById('brake-button'),
        lightsButton: document.getElementById('lights-button'),
        toggleSteeringBtn: document.getElementById('toggle-steering'),
        toggleDriveBtn: document.getElementById('toggle-drive'),
        engineButton: document.getElementById('engine-button'),
        // References to the toggle button span elements that display the current mode text/icon.
        toggleSteeringOn: document.querySelector('#toggle-steering .state-on'),
        toggleSteeringOff: document.querySelector('#toggle-steering .state-off'),
        toggleDriveOn: document.querySelector('#toggle-drive .state-on'),
        toggleDriveOff: document.querySelector('#toggle-drive .state-off'),
    };

    // --- State Variables ---
    let webSocket = null; // Holds the active WebSocket connection instance.
    let isSteeringDragging = false; // Flag to track if the steering wheel UI is currently being dragged.
    // let lastSteeringAngleRad = 0; // Variable commented out as it might not be needed with absolute angle calculation.
    let currentLightMode = 0; // Stores the current light mode state (0-3), updated from ESP8266 status.
    let obstacleAlertPlaying = false; // Flag to prevent the obstacle alert sound from playing repeatedly.

    // --- Audio ---
    // Audio object for playing an alert sound when an obstacle is detected.
    const alertSound = new Audio('alert.mp3');
    alertSound.preload = 'auto'; // Start loading the audio file as soon as the page is loaded.
    alertSound.onended = () => { obstacleAlertPlaying = false; }; // Reset the flag when the sound finishes.

    // --- Utility Functions ---

    /*
     * @brief Limits how often a function can be called.
     * @param func The function to throttle.
     * @param limit The minimum time in milliseconds between consecutive calls.
     * @return A new function that, when called, will execute the original function
     * at most once per `limit` milliseconds.
     */
    function throttle(func, limit) {
        let lastFunc;
        let lastRan;
        return function (...args) {
            const context = this;
            if (!lastRan) {
                func.apply(context, args);
                lastRan = Date.now();
            } else {
                clearTimeout(lastFunc);
                lastFunc = setTimeout(function () {
                    if ((Date.now() - lastRan) >= limit) {
                        func.apply(context, args);
                        lastRan = Date.now();
                    }
                }, limit - (Date.now() - lastRan));
            }
        }
    }

    /*
     * @brief Sends data over the WebSocket connection if it is open.
     * @param data The data (string or other serializable format) to send.
     */
    function sendWebSocketData(data) {
        if (webSocket && webSocket.readyState === WebSocket.OPEN) {
            // console.log('Sending:', data); // Debugging line commented out
            webSocket.send(data);
        } else {
            console.warn('WebSocket not open. Cannot send:', data);
        }
    }
    // Throttled version of sendWebSocketData for controls sending frequent updates.
    const sendWebSocketDataThrottled = throttle(sendWebSocketData, WEBSOCKET_THROTTLE_MS);

    /*
     * @brief Shows or hides an HTML element by toggling the 'noDisplay' class.
     * @param element The HTML element to update.
     * @param show Boolean indicating whether to show (true) or hide (false) the element.
     */
    function updateElementVisibility(element, show) {
        if (element) {
            element.classList.toggle('noDisplay', !show);
        }
    }

    /*
     * @brief Adds or removes the 'active' class to a button for visual feedback.
     * @param button The button element to update.
     * @param isActive Boolean indicating whether the button should be active (true) or not (false).
     */
    function setButtonActive(button, isActive) {
        if (button) {
            button.classList.toggle('active', isActive);
        }
    }

    /*
     * @brief Removes the 'active' class from a button after a short delay.
     * @param button The button element to update.
     */
    function handleButtonRelease(button) {
        setTimeout(() => setButtonActive(button, false), BUTTON_ACTIVE_DELAY_MS);
    }

    /*
     * @brief Sets up event listeners for button pointer interactions (down, up, leave).
     * @param button The button element.
     * @param commandPress The command string to send on 'touchdown'.
     * @param commandRelease The command string to send on 'touchup' or 'touchleave' (optional).
     * @param useThrottle Boolean indicating whether to use the throttled send function.
     */
    function handleButtonInteraction(button, commandPress, commandRelease = null, useThrottle = false) {
        if (!button) return;
        const sendFunc = useThrottle ? sendWebSocketDataThrottled : sendWebSocketData;

      button.addEventListener('pointerdown', (e) => {
            e.preventDefault(); // Prevent default browser actions
            setButtonActive(button, true);
            sendFunc(commandPress);
            // Capture the pointer for reliable pointerup/leave events, even if pointer moves off the element.
          if(commandRelease || typeof button.setPointerCapture === 'function') {
              try { button.setPointerCapture(e.pointerId); } catch(err) { /* console.warn("Failed to capture pointer:", err); */ } // Error log commented out as it's usually non-critical browser support issue
            }
        });

        const releaseAction = (e) => {
            handleButtonRelease(button);
            if (commandRelease) {
                sendFunc(commandRelease);
            }
            // Release pointer capture.
            if(commandRelease || typeof button.releasePointerCapture === 'function') {
               try { button.releasePointerCapture(e.pointerId); } catch(err){}
            }
        };

      button.addEventListener('pointerup', releaseAction);
      button.addEventListener('pointerleave', (e) => {
            // Only trigger release if the pointer left while the button was active (pointer was down).
            if (button.classList.contains('active')) {
                releaseAction(e);
            }
        });
    }

    /*
     * @brief Updates the UI elements based on status data received from the car.
     * @param data The JSON object received from the WebSocket.
     */
    function updateUI(data) {
        console.log("Updating UI with data:", data); // Keep this log for debugging
        // Update Battery Status display.
        if (data.BatteryStatus !== undefined && DOMElements.batteryStatus) {
            const batteryPercent = parseInt(data.BatteryStatus, 10);
            const isLow = batteryPercent < 20; // Example threshold for low battery warning UI.
            DOMElements.batteryStatus.textContent = `Battery: ${batteryPercent}%`;
            // Update CSS classes based on battery level for styling.
            DOMElements.batteryStatus.classList.toggle('low', isLow);
            DOMElements.batteryStatus.classList.toggle('ok', !isLow);
        }

        // Update Engine Status (affects engine button styling).
        if (data.EngineStatus !== undefined && DOMElements.engineButton) {
            const engineOn = data.EngineStatus === true || data.EngineStatus === 1;
            // Set data attribute which is used by CSS for styling.
            DOMElements.engineButton.dataset.engineState = engineOn ? 'on' : 'off';
        }

        // Update Lights Status (affects lights button styling and internal state).
        if (data.LightsStatus !== undefined && DOMElements.lightsButton) {
            currentLightMode = parseInt(data.LightsStatus, 10); // Update local state.
            // Set data attribute used by CSS for styling.
            DOMElements.lightsButton.dataset.lightMode = currentLightMode;
        }

        // Update Obstacle Detection warning and sound alert.
        if ((data.BackObstacleDistance !== undefined || data.FrontObstacleDistance !== undefined) && DOMElements.obstacleDetected) {
            const BackDist = parseFloat(data.BackObstacleDistance);
            const backObstacle = !isNaN(BackDist) && BackDist >= 0 && BackDist < OBSTACLE_DISTANCE_THRESHOLD_CM;
            console.log("Back Distance:", BackDist, "Obstacle Present:", backObstacle); // Debugging line
            
            const frontDist = parseFloat(data.FrontDistance);
            // Check if front distance is valid and below the UI warning threshold.
            const frontObstacle = !isNaN(frontDist) && frontDist >= 0 && frontDist < OBSTACLE_DISTANCE_THRESHOLD_CM;
            const obstaclePresent = backObstacle

            // Show/hide the obstacle warning element.
            updateElementVisibility(DOMElements.obstacleDetected, obstaclePresent);

            // Play alert sound if an obstacle is present and the sound is not already playing.
            if (obstaclePresent && !obstacleAlertPlaying) {
                obstacleAlertPlaying = true;
                alertSound.play().catch(error => {
                    console.warn('Alert sound playback failed:', error); // Keep audio playback errors
                    obstacleAlertPlaying = false; // Reset flag on error
                });
            }
        }

        // Note: UI does not update steering wheel/slider position based on received state.
        // This assumes controls are primarily driven by the UI, not physical overrides or complex ESP logic.
    }

    // --- WebSocket Setup ---
    /*
     * @brief Establishes and manages the WebSocket connection to the ESP8266 server.
     */
    function connectWebSocket() {
        // Determine the WebSocket protocol (ws:// or wss://) based on the current page's protocol.
        const wsProtocol = window.location.protocol === 'https:' ? 'wss://' : 'ws://';
        // Construct the WebSocket URL using the current hostname and port 81.
        const wsUrl = `${wsProtocol}${window.location.hostname}:81/`;
        console.log(`Attempting to connect to WebSocket: ${wsUrl}`); // Keep connection attempt log.

        // Prevent creating multiple WebSocket instances if one is already connecting or open.
        if (webSocket && (webSocket.readyState === WebSocket.CONNECTING || webSocket.readyState === WebSocket.OPEN)) {
            console.log("WebSocket already connecting or open."); // Keep this log for clarity.
            return;
        }

        // Create a new WebSocket instance.
        webSocket = new WebSocket(wsUrl);

        // --- WebSocket Event Handlers ---
        // Called when the WebSocket connection is successfully opened.
        webSocket.onopen = (event) => {
            console.log("WebSocket connected"); // Keep connection status log.
            if (DOMElements.connectionStatus) {
                DOMElements.connectionStatus.textContent = "Connected";
                DOMElements.connectionStatus.className = 'display-info connected'; // Update UI class.
            }
        };

        // Called when a WebSocket error occurs.
        webSocket.onerror = (event) => {
            console.error("WebSocket error:", event); // Keep error log.
            if (DOMElements.connectionStatus) {
                DOMElements.connectionStatus.textContent = "Error";
                DOMElements.connectionStatus.className = 'display-info error'; // Update UI class.
            }
        };

        // Called when the WebSocket connection is closed.
        webSocket.onclose = (event) => {
            console.log(`WebSocket closed. Code: ${event.code}, Reason: '${event.reason}'`); // Keep close status log.
            if (DOMElements.connectionStatus) {
                DOMElements.connectionStatus.textContent = "Disconnected";
                DOMElements.connectionStatus.className = 'display-info'; // Revert UI class.
            }
            webSocket = null; // Clear the WebSocket object reference.
            // Attempt to reconnect after a delay.
            setTimeout(connectWebSocket, 5000);
        };

        // Called when a message is received from the WebSocket server.
        webSocket.onmessage = (event) => {
            // console.log("WebSocket message:", event.data); // Debugging line commented out
            try {
                // Parse the incoming data as a JSON object.
                const data = JSON.parse(event.data);
                // Update the UI based on the received status data.
                updateUI(data);
            } catch (e) {
                console.error("Failed to parse WebSocket message:", e); // Keep parsing error log.
                // console.error("Received data:", event.data); // Debugging line commented out
            }
        };
    }

    // --- Event Listeners Setup ---
    /*
     * @brief Sets up all event listeners for user interactions on the web page.
     */
    function setupEventListeners() {
        // Setup pointer event listeners for basic buttons (press/release).
        handleButtonInteraction(DOMElements.leftButton, Commands.LEFT, Commands.CENTER);
        handleButtonInteraction(DOMElements.rightButton, Commands.RIGHT, Commands.CENTER);
        handleButtonInteraction(DOMElements.forwardButton, Commands.FORWARD, Commands.STOP_ACC);
        handleButtonInteraction(DOMElements.backwardButton, Commands.BACKWARD, Commands.STOP_ACC);
        handleButtonInteraction(DOMElements.brakeButton, Commands.STOP_ACC, null); // Brake button has no release command.

        // Engine Button: Toggles the engine state.
        if (DOMElements.engineButton) {
          DOMElements.engineButton.addEventListener('pointerdown', (e) => {
                e.preventDefault();
                setButtonActive(DOMElements.engineButton, true);
                sendWebSocketData(Commands.ENGINE_TOGGLE);
              try { DOMElements.engineButton.setPointerCapture(e.pointerId); } catch(err){}
            });
            DOMElements.engineButton.addEventListener('pointerup', (e) => {
                handleButtonRelease(DOMElements.engineButton);
                 try { DOMElements.engineButton.releasePointerCapture(e.pointerId); } catch(err){}
            });
          DOMElements.engineButton.addEventListener('pointerleave', (e) => {
                if (DOMElements.engineButton.classList.contains('active')) {
                    handleButtonRelease(DOMElements.engineButton);
                   try { DOMElements.engineButton.releasePointerCapture(e.pointerId); } catch(err){}
                }
            });
        }

        // Lights Button: Cycles through the defined light modes.
        if (DOMElements.lightsButton) {
            DOMElements.lightsButton.addEventListener('pointerdown', (e) => {
                e.preventDefault();
                setButtonActive(DOMElements.lightsButton, true);
                currentLightMode = (currentLightMode + 1) % 4; // Cycle mode from 0 to 3.
                sendWebSocketData(`${Commands.LIGHT_PREFIX}${currentLightMode}`); // Send command like "L0", "L1", etc.
                // Update the data attribute immediately to trigger CSS styling changes.
                DOMElements.lightsButton.dataset.lightMode = currentLightMode;
                 try { DOMEElements.lightsButton.setPointerCapture(e.pointerId); } catch(err){}
            });
            DOMElements.lightsButton.addEventListener('pointerup', (e) => {
                handleButtonRelease(DOMElements.lightsButton);
                 try { DOMElements.lightsButton.releasePointerCapture(e.pointerId); } catch(err){}
            });
             DOMElements.lightsButton.addEventListener('pointerleave', (e) => {
                if (DOMElements.lightsButton.classList.contains('active')) {
                    handleButtonRelease(DOMElements.lightsButton);
                   try { DOMElements.lightsButton.releasePointerCapture(e.pointerId); } catch(err){}
                }
            });
        }

        // Toggle Steering Mode: Switches between button-based steering and steering wheel control.
        if (DOMElements.toggleSteeringBtn && DOMElements.leftButton && DOMElements.rightButton && DOMElements.steeringWheelContainer && DOMElements.toggleSteeringOn && DOMElements.toggleSteeringOff) {
            DOMElements.toggleSteeringBtn.addEventListener('click', () => {
                const isPressed = DOMElements.toggleSteeringBtn.getAttribute('aria-pressed') === 'true';
                const newPressedState = !isPressed; // Toggle the state.
                DOMElements.toggleSteeringBtn.setAttribute('aria-pressed', newPressedState); // Update accessibility attribute.

                const showWheel = newPressedState; // If toggled 'on', show the wheel.
                updateElementVisibility(DOMElements.steeringWheelContainer, showWheel); // Show/hide steering wheel container.
                updateElementVisibility(DOMElements.leftButton, !showWheel); // Show/hide left button.
                updateElementVisibility(DOMElements.rightButton, !showWheel); // Show/hide right button.
                updateElementVisibility(DOMElements.toggleSteeringOn, showWheel); // Show the 'Wheel' text/icon when wheel is shown.
                updateElementVisibility(DOMElements.toggleSteeringOff, !showWheel); // Show the 'Btns' text/icon when buttons are shown.

                // When switching *to* buttons mode (showWheel is false).
                if (!showWheel) {
                    sendWebSocketData(Commands.CENTER); // Send a center command.
                    // Reset steering wheel appearance to centered.
                    if (DOMElements.steeringWheel) {
                        DOMElements.steeringWheel.style.setProperty('--rotate-angle', '0deg');
                        // lastSteeringAngleRad = 0; // Variable commented out
                    }
                } else {
                    // When switching *to* wheel mode (showWheel is true), ensure it starts centered.
                    if (DOMElements.steeringWheel) {
                        DOMElements.steeringWheel.style.setProperty('--rotate-angle', '0deg');
                        // lastSteeringAngleRad = 0; // Variable commented out
                    }
                    sendWebSocketData(Commands.CENTER); // Send center command just in case.
                }
            });
        }

        // Toggle Drive Mode: Switches between button-based drive and speed slider control.
        if (DOMElements.toggleDriveBtn && DOMElements.forwardButton && DOMElements.backwardButton && DOMElements.speedSliderContainer && DOMElements.toggleDriveOn && DOMElements.toggleDriveOff) {
            DOMElements.toggleDriveBtn.addEventListener('click', () => {
                const isPressed = DOMElements.toggleDriveBtn.getAttribute('aria-pressed') === 'true';
                const newPressedState = !isPressed; // Toggle the state.
                DOMElements.toggleDriveBtn.setAttribute('aria-pressed', newPressedState); // Update accessibility attribute.

                const showSlider = newPressedState; // If toggled 'on', show the slider.
                updateElementVisibility(DOMElements.speedSliderContainer, showSlider); // Show/hide slider container.
                updateElementVisibility(DOMElements.forwardButton, !showSlider); // Show/hide forward button.
                updateElementVisibility(DOMElements.backwardButton, !showSlider); // Show/hide backward button.
                updateElementVisibility(DOMElements.toggleDriveOn, showSlider); // Show the 'Slider' text/icon when slider is shown.
                updateElementVisibility(DOMElements.toggleDriveOff, !showSlider); // Show the 'Btns' text/icon when buttons are shown.

                // When switching *to* buttons mode (showSlider is false).
                if (!showSlider) {
                    sendWebSocketData(Commands.STOP_ACC); // Send a stop command.
                    // Reset slider value to the center (stop) position.
                    if (DOMElements.speedSlider) {
                        DOMElements.speedSlider.value = SLIDER_CENTER_VALUE;
                    }
                } else {
                    // When switching *to* slider mode (showSlider is true), ensure it starts centered and stopped.
                    if (DOMElements.speedSlider) {
                        DOMElements.speedSlider.value = SLIDER_CENTER_VALUE;
                    }
                    sendWebSocketData(`${Commands.SPEED_PREFIX}${SLIDER_CENTER_VALUE}`); // Send initial centered value.
                }
            });
        }


      // --- Steering Wheel Drag Logic ---
// Implements dragging functionality for the steering wheel UI element.
if (DOMElements.steeringWheelContainer && DOMElements.steeringWheel) {
    const sw = DOMElements.steeringWheel;
    const swContainer = DOMElements.steeringWheelContainer;
    let isSteeringDragging = false;
    let initialTouchX = 0; // Stores the x-coordinate of the initial touch point.
    let currentAngle = 0; // Stores the current rotation angle of the wheel.
    const MAX_ROTATION_DEGREES = 90; // The maximum rotation limit in degrees (-90 to +90).
    const ROTATION_SENSITIVITY = 0.5; // Adjust this value to change how fast the wheel turns.

    // Handle the start of a drag gesture on the steering wheel.
    sw.addEventListener('touchstart', (e) => {
        e.preventDefault();
        isSteeringDragging = true; // Set dragging flag.
        // Get the x-coordinate of the first touch
        initialTouchX = e.touches[0].clientX; 
        // console.log("Steering wheel drag start.");
    });

    // Handle the movement during a drag gesture.
    sw.addEventListener('touchmove', (e) => {
        if (!isSteeringDragging) return; // Only process if dragging is active.

        // Get the x-coordinate of the current touch
        const currentTouchX = e.touches[0].clientX;

        // Calculate the change in horizontal position since the last move.
        const deltaX = currentTouchX - initialTouchX;

        // Convert the change in position to an angle.
        const rotationChange = deltaX * ROTATION_SENSITIVITY;
        
        // Add the rotation change to the current angle.
        let newAngle = currentAngle + rotationChange;

        // Clamp the new angle to the desired range of -90 to +90 degrees.
        newAngle = Math.max(-MAX_ROTATION_DEGREES, Math.min(MAX_ROTATION_DEGREES, newAngle));
        
        // Update the current angle for the next movement event.
        currentAngle = newAngle;

        // Update the initialTouchX to create relative movement from the new position.
        initialTouchX = currentTouchX;

        // Apply the new angle to the CSS variable to visually rotate the wheel.
        sw.style.setProperty('--rotate-angle', `${currentAngle}deg`);

        // --- Your existing logic for sending data, slightly adjusted ---
        // Map the UI angle (-90 to +90) to the servo angle (0 to 180).
        const servoAngle = Math.round(
            (currentAngle + MAX_ROTATION_DEGREES) / (2 * MAX_ROTATION_DEGREES) * 180
        );

        // Send the steering command.
        sendWebSocketDataThrottled(`${Commands.STEER_PREFIX}${servoAngle}`);
    });

    // Handles the end of a drag gesture (touch up or cancel).
    const dragEnd = (e) => {
        if (!isSteeringDragging) return;
        isSteeringDragging = false; // Clear dragging flag.

        // Reset the steering wheel UI appearance to centered.
        sw.style.setProperty('--rotate-angle', '0deg');
        currentAngle = 0; // Reset the current angle to 0.

        // Send a center steering command.
        sendWebSocketData(Commands.CENTER);
        // console.log("Steering wheel drag end.");
    };

    // Add event listeners for the end of the drag.
    sw.addEventListener('touchend', dragEnd);
    sw.addEventListener('touchcancel', dragEnd); // Handle interruptions.
}
        // --- Speed Slider Logic ---
        // Implements functionality for the speed/direction slider.
        if (DOMElements.speedSlider) {
            const slider = DOMElements.speedSlider;

            // Handle continuous slider movement (fires while dragging).
            slider.addEventListener('input', () => {
                const value = slider.value; // Get the slider's current value (0-100).
                // Send the speed/direction command (e.g., "B75") using the throttled function.
                sendWebSocketDataThrottled(`${Commands.SPEED_PREFIX}${value}`);
                console.log("Slider input value:", value); // Debugging line commented out
            });

            // Handle the end of slider movement (fires when dragging finishes).
            slider.addEventListener('change', () => {
                const value = slider.value; // Get the final slider value.
                // Send the speed/direction command (e.g., "B75") using the regular send function to ensure the last value is sent.
                sendWebSocketData(`${Commands.SPEED_PREFIX}${value}`);
                console.log("Slider change value:", value); // Debugging line commented out
            });
        }

        // Optional: Prevent default touch behavior on the body to avoid issues like pull-to-refresh.
        DOMElements.body.addEventListener('touchdown', (e) => { e.preventDefault(); }); // Commented out, uncomment if needed.
    }

    // --- Initialization ---
    // Code that runs after the entire HTML document is loaded and parsed.
    document.addEventListener('DOMContentLoaded', () => {
        console.log("DOM fully loaded. Setting up..."); // Keep setup log.
        setupEventListeners(); // Set up all event handlers for UI interactions.
        connectWebSocket(); // Attempt to connect to the WebSocket server on the ESP8266.
    });

})(); // End of IIFE.
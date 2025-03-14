const leftButton = document.getElementById('leftButton');
const rightButton = document.getElementById('rightButton');
const steeringWheel = document.querySelector('#steering-wheel');
// const middleAlign = document.getElementById('middleAlign');

const backMotorSpeedSet = document.getElementById('backMotorSpeedSet');
const backMotorSpeedSetContainer = document.getElementById('backMotorSpeedSetContainer');

const forwardButton = document.getElementById('forwardButton');
const backwardButton = document.getElementById('backwardButton');

const BatteryStatus = document.getElementById('BatteryStatus');

// const EngineStatus = document.getElementById('EngineStatus');
const engineButton = document.getElementById('engineButton');

// const LightsStatus = document.getElementById('LightsStatus');
const lightsButton = document.getElementById('lightsButton');

const connectionStatus = document.getElementById('connectionStatus');
const ObstacleDetected = document.getElementById('ObstacleDetected');

const ToggleDrive = document.getElementById('ToggleDrive');
const ToggleSteering = document.getElementById('ToggleSteering');

function setBodySize() {
  document.body.style.setProperty('--displayHeight', `${window.innerHeight}px`);
  document.body.style.setProperty('--displayWidth', `${window.innerWidth}px`);
}
setBodySize();
window.addEventListener('resize', setBodySize);


ToggleSteering.addEventListener('touchstart', (event) => {
  // event.preventDefault();
  if (document.getElementById('ToggleSteeringOn').classList.contains('noDisplay')) {
    document.getElementById('ToggleSteeringOn').classList.remove('noDisplay')
    document.getElementById('ToggleSteeringOff').classList.add('noDisplay')
    leftButton.classList.add('noDisplay');
    rightButton.classList.add('noDisplay');
    steeringWheel.classList.remove('noDisplay');
    ToggleSteering.style.setProperty('background', 'linear-gradient(132deg,rgb(255 255 255 / 62%), blue)')

  } else {
    document.getElementById('ToggleSteeringOn').classList.add('noDisplay')
    document.getElementById('ToggleSteeringOff').classList.remove('noDisplay')
    leftButton.classList.remove('noDisplay');
    rightButton.classList.remove('noDisplay');
    steeringWheel.classList.add('noDisplay');
    ToggleSteering.style.background= '';

  };
});

ToggleDrive.addEventListener('touchstart', (event) => {
  // event.preventDefault();
  if (document.getElementById('ToggleDriveOn').classList.contains('noDisplay')) {
    document.getElementById('ToggleDriveOn').classList.remove('noDisplay');
    document.getElementById('ToggleDriveOff').classList.add('noDisplay');
    backwardButton.classList.add('noDisplay');
    forwardButton.classList.add('noDisplay');
    backMotorSpeedSetContainer.classList.remove('noDisplay');
    ToggleDrive.style.setProperty('background', 'linear-gradient(132deg,rgb(255 255 255 / 62%), blue)');
  } else {
    document.getElementById('ToggleDriveOn').classList.add('noDisplay');
    document.getElementById('ToggleDriveOff').classList.remove('noDisplay');
    backwardButton.classList.remove('noDisplay');
    forwardButton.classList.remove('noDisplay');
    backMotorSpeedSetContainer.classList.add('noDisplay');
    ToggleDrive.style.background= '';

  };
});


ObstacleDetected.style.display = "none";

let lastSend = 0;
let Socket = new WebSocket('ws://' + window.location.hostname + ':81/');
Socket.onopen = function (event) {
  console.log("web socket opened");
  connectionStatus.innerHTML = "Connected";
  connectionStatus.style.backgroundColor = "#7c37c9";
};

Socket.onerror = function (event) {
  // console.log("web socket error");
  connectionStatus.innerHTML = "error!!!";
  connectionStatus.style.backgroundColor = "red";
};

Socket.onclose = function (event) {
  console.log("web socket closed");
  connectionStatus.innerHTML = "disconnected";
  connectionStatus.style.backgroundColor = "#c93737";
};
// // receving data from server
Socket.onmessage = function (event) {
  console.log("web socket connect. It transfer below message");
  console.log(event);
  if (event.data) {
    let data = JSON.parse(event.data);
    console.log(data);
    console.log("battery status:" + data["BatteryStatus"]);
    console.log("engine status:" + data["EngineStatus"]);
    console.log("lights status:" + data["LightsStatus"]);
    console.log("Obstacle detected:" + data["ObstacleDetected"]);
    BatteryStatus.innerHTML = (data["BatteryStatus"]) ? `Battery at: ${data["BatteryStatus"]}V` : "Battery is Low";
    BatteryStatus.style.backgroundColor = (data["BatteryStatus"]) ? "#7c37c9" : "#c93737";
    // engineButton.innerHTML = (data["EngineStatus"]) ? "Engine Started" : "Engine Stopped";
    if (data["EngineStatus"]) {
      engineButton.style.setProperty('background', 'linear-gradient(132deg, transparent, blue)')
      engineButton.style.setProperty('box-shadow', 'inset 0px 1px 13px 2px white');
    } else {
      engineButton.style.setProperty('background', 'linear-gradient(blue,green)')
      engineButton.style.setProperty('box-shadow', 'unset');
    }
    switch (data["LightsStatus"]) {
      case 1:
        lightsButton.style.setProperty('background', 'linear-gradient(blue, transparent)')
        break;
      case 2:
        lightsButton.style.setProperty('background', 'linear-gradient(blue,green)')
        break;
      case 3:
        lightsButton.style.setProperty('background', 'linear-gradient(blue, transparent,green)')
        break;
      default:
        lightsButton.style.backgroundColor = "grey";
        break;
    }
    ObstacleDetected.style.display = (data["ObstacleDetected"]) ? "block" : "none";

  }
};



//turning
//comannds to send to server


// console.log(steeringWheel.value);
let isDragging = false;
let lastAngle = 0;

steeringWheel.addEventListener('touchstart', (event) => {
  event.preventDefault();
  isDragging = true;
  lastAngle = getRotation(event.targetTouches[0].clientX, event.targetTouches[0].clientY);
});

steeringWheel.addEventListener('touchmove', (event) => {
  event.preventDefault();
  let now = (new Date).getTime();
  if (lastSend > now - 20) return;
  lastSend = now;

  if (!isDragging) return;

  // Get the current touch position and calculate the angle
  const currentAngle = getRotation(event.targetTouches[0].clientX, event.targetTouches[0].clientY);

  // Calculate the delta angle by subtracting the last angle
  let deltaAngle = currentAngle - lastAngle;


  // Get the current rotation of the steering wheel element
  const currentRotation = parseFloat(getComputedStyle(steeringWheel).getPropertyValue('--rotateAngle') || 0);
  // Calculate the new rotation angle
  let newRotation = currentRotation + deltaAngle;

  console.log(`lastAngle: ${lastAngle}\ncurrentAngle: ${currentAngle}\ndeltaAngle:${deltaAngle}\ncurrentRotation: ${currentRotation}\nnewRotation: ${newRotation}`);

  // Prevent rotation beyond the limits (e.g., -90 to 90 degrees)
  if (newRotation < -90 || newRotation > 180 && newRotation < 270) {
    newRotation = -90;
  } else if (newRotation > 90 && newRotation < 180) {
    newRotation = 90;
  };

  // Update the steering wheel's rotation based on the calculated angle
  steeringWheel.style.setProperty('--rotateAngle', `${newRotation}deg`);

  // Send the updated rotation angle to the server via WebSocket
  Socket.send(`F${Number(steeringWheel.style.getPropertyValue('--rotateAngle').replace(/deg/g, "")) + 90}`);

  // Update the last angle for the next move event
  lastAngle = currentAngle;
});

steeringWheel.addEventListener('touchend', () => {
  isDragging = false;
  const currentRotation = parseFloat(getComputedStyle(steeringWheel).getPropertyValue('--rotateAngle') || 0);
  if (currentRotation <= -80) {
    steeringWheel.style.setProperty('--rotateAngle', `-80deg`);
  } else if (currentRotation >= 80) {
    steeringWheel.style.setProperty('--rotateAngle', `80deg`);
  }
});

function getRotation(x, y) {
  const rect = steeringWheel.getBoundingClientRect();
  const centerX = rect.left + rect.width / 2;
  const centerY = rect.top + rect.height / 2;

  return Math.atan2(y - centerY, x - centerX) * (180 / Math.PI);
}

//back motor
lastSend = 0;
leftButton.addEventListener('touchstart', () => {
  Socket.send(`left`);
  leftButton.classList.add('active');
});
rightButton.addEventListener('touchstart', () => {
  Socket.send(`right`);
  rightButton.classList.add('active');
});
leftButton.addEventListener('touchend', () => {
  Socket.send(`center`);
  setTimeout(() => {
    leftButton.classList.remove('active');
  }, 200);
});
rightButton.addEventListener('touchend', () => {
  Socket.send(`center`);
  setTimeout(() => {
    rightButton.classList.remove('active');
  }, 200);
});
forwardButton.addEventListener('touchstart', () => {
  // Socket.send(`forward`);
  document.body.style.setProperty('background','linear-gradient(blue, transparent),radial-gradient(violet,aqua)');

  forwardButton.classList.add('active');
})
backwardButton.addEventListener('touchstart', () => {
  // Socket.send(`backward`);
  document.body.style.setProperty('background','linear-gradient(transparent,red),radial-gradient(violet,aqua)');
  backwardButton.classList.add('active');
})


stopButton.addEventListener('touchstart', () => {
  Socket.send(`stopAcc`);
  forwardButton.classList.add('active');
})
stopButton.addEventListener('touchend', () => {
  forwardButton.classList.remove('active');
})


forwardButton.addEventListener('touchend', () => {
  setTimeout(() => {
    Socket.send(`stopAcc`);
    document.body.style.background='';
    forwardButton.classList.remove('active');
  }, 200);
});
backwardButton.addEventListener('touchend', () => {
  Socket.send(`stopAcc`);
  setTimeout(() => {
    document.body.style.background='';
    backwardButton.classList.remove('active');
  }, 200);
});



backMotorSpeedSet.addEventListener('input', (event) => {
  event.preventDefault();
  Socket.send(`B${backMotorSpeedSet.value}`);

});

engineButton.addEventListener('touchstart', () => {
  engineButton.classList.add('active');
  let now = (new Date).getTime();
  if (lastSend > now - 20) return;
  lastSend = now;
  console.log("enginestart");
  Socket.send(`enginestart`);
});


engineButton.addEventListener('touchend', () => {
  setTimeout(() => {
    engineButton.classList.remove('active');
  }, 200);
})

let lightSettings = 0;
lightsButton.addEventListener('touchstart', () => {
  lightsButton.classList.add('active');
  lightSettings++
  if (lightSettings > 3) {
    lightSettings = 0;
  }
  let now = (new Date).getTime();


  if (lastSend > now - 20) return;
  lastSend = now;
  Socket.send(`L${lightSettings}`);

});
lightsButton.addEventListener('touchend', () => {
  setTimeout(() => {
    lightsButton.classList.remove('active');
  }, 200);
})
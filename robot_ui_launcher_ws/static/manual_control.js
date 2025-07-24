//manual_control.js
const statusSpan = document.getElementById('status');
const velocityDisplay = document.getElementById('velocity');

const ros = new ROSLIB.Ros({
  url: 'ws://' + location.hostname + ':9090'
});

ros.on('connection', () => {
  console.log('Connected to rosbridge websocket.');
  statusSpan.textContent = 'Connected to rosbridge.';
  statusSpan.style.color = 'green';
  reconnectTimeout = 1000; // reset reconnect timeout on success
});

ros.on('error', (error) => {
  console.error('Error connecting to rosbridge websocket:', error);
  statusSpan.textContent = 'Error connecting to rosbridge.';
  statusSpan.style.color = 'red';
  attemptReconnect();
});

ros.on('close', () => {
  console.warn('Connection to rosbridge websocket closed.');
  statusSpan.textContent = 'Connection closed. Reconnecting...';
  statusSpan.style.color = 'orange';
  attemptReconnect();
});

// Auto reconnect with exponential backoff
let reconnectTimeout = 1000;
function attemptReconnect() {
  setTimeout(() => {
    ros.connect('ws://' + location.hostname + ':9090');
    reconnectTimeout = Math.min(30000, reconnectTimeout * 2);
  }, reconnectTimeout);
}

// ROS publisher
const cmdVelTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/cmd_vel',
  messageType: 'geometry_msgs/msg/Twist',
});

// Initial speed parameters
let linearSpeed = 0.5;
let angularSpeed = 1.0;

let currentLinear = 0;
let currentAngular = 0;

const maxLinearSpeed = 2.0;
const maxAngularSpeed = 3.0;
const rampStep = 0.05; // Acceleration smoothness

// Track keys pressed
const keysPressed = new Set();

// Emergency stop key
const EMERGENCY_STOP_KEY = 'x';

// Keep track of last published velocities to prevent flooding
let lastPublishedLinear = null;
let lastPublishedAngular = null;

function updateVelocityDisplay() {
  if (velocityDisplay) {
    velocityDisplay.textContent = `Linear: ${linearSpeed.toFixed(2)}, Angular: ${angularSpeed.toFixed(2)}`;
  }
}

// Smooth velocity ramping
function rampVelocity(targetLinear, targetAngular) {
  if (Math.abs(currentLinear - targetLinear) > rampStep) {
    currentLinear += (targetLinear > currentLinear) ? rampStep : -rampStep;
  } else {
    currentLinear = targetLinear;
  }
  if (Math.abs(currentAngular - targetAngular) > rampStep) {
    currentAngular += (targetAngular > currentAngular) ? rampStep : -rampStep;
  } else {
    currentAngular = targetAngular;
  }
}

// Publish velocity only if changed
function publishTwist(linearX, angularZ) {
  if (linearX === lastPublishedLinear && angularZ === lastPublishedAngular) {
    return; // skip duplicate publishes
  }

  const twist = new ROSLIB.Message({
    linear: { x: linearX, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angularZ },
  });
  cmdVelTopic.publish(twist);

  lastPublishedLinear = linearX;
  lastPublishedAngular = angularZ;
}

// Calculate velocity based on pressed keys
function calculateTargetVelocity() {
  let targetLinear = 0;
  let targetAngular = 0;

  if (keysPressed.has('w')) targetLinear += linearSpeed;
  if (keysPressed.has('s')) targetLinear -= linearSpeed;
  if (keysPressed.has('a')) targetAngular += angularSpeed;
  if (keysPressed.has('d')) targetAngular -= angularSpeed;

  return { targetLinear, targetAngular };
}

// Watchdog timer to stop robot if no command received recently
let watchdogTimer = null;
const WATCHDOG_TIMEOUT = 500; // ms

function resetWatchdog() {
  if (watchdogTimer) clearTimeout(watchdogTimer);
  watchdogTimer = setTimeout(() => {
    currentLinear = 0;
    currentAngular = 0;
    publishTwist(0, 0);
    console.warn('Watchdog timeout - stopping robot!');
  }, WATCHDOG_TIMEOUT);
}

// Main loop to ramp velocities and publish at ~30Hz
function mainLoop() {
  const { targetLinear, targetAngular } = calculateTargetVelocity();

  rampVelocity(targetLinear, targetAngular);

  publishTwist(currentLinear, currentAngular);

  resetWatchdog();

  requestAnimationFrame(mainLoop);
}

document.addEventListener('keydown', (event) => {
  const key = event.key.toLowerCase();

  if (event.repeat) return;

  // Emergency stop
  if (key === EMERGENCY_STOP_KEY) {
    keysPressed.clear();
    currentLinear = 0;
    currentAngular = 0;

    // Reset last published to force publish zero velocity again
    lastPublishedLinear = null;
    lastPublishedAngular = null;

    publishTwist(0, 0);
    statusSpan.textContent = 'EMERGENCY STOP!';
    statusSpan.style.color = 'red';
    return;
  }

  // Speed increase
  if (key === 'q') {
    linearSpeed = Math.min(linearSpeed + 0.1, maxLinearSpeed);
    angularSpeed = Math.min(angularSpeed + 0.1, maxAngularSpeed);
    updateVelocityDisplay();
    return;
  }

  // Speed decrease
  if (key === 'e') {
    linearSpeed = Math.max(linearSpeed - 0.1, 0.1);
    angularSpeed = Math.max(angularSpeed - 0.1, 0.1);
    updateVelocityDisplay();
    return;
  }

  // Movement keys
  if (['w', 'a', 's', 'd'].includes(key)) {
    keysPressed.add(key);
  }
});

document.addEventListener('keyup', (event) => {
  const key = event.key.toLowerCase();
  if (['w', 'a', 's', 'd'].includes(key)) {
    keysPressed.delete(key);
  }
});

// Initialize display and start main loop
updateVelocityDisplay();
mainLoop();


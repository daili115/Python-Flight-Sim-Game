// Control inputs
const controls = {
    aileron: 0.0, // Roll
    elevator: 0.0, // Pitch
    rudder: 0.0, // Yaw
    throttle: 0.5, // Thrust
};

// Event listeners for keyboard input
document.addEventListener('keydown', (event) => {
    switch (event.key) {
        case 'a':
            controls.aileron = 1;
            break;
        case 'd':
            controls.aileron = -1;
            break;
        case 'w':
            controls.elevator = 1;
            break;
        case 's':
            controls.elevator = -1;
            break;
        case 'q':
            controls.rudder = -1;
            break;
        case 'e':
            controls.rudder = 1;
            break;
        case 'Shift':
            controls.throttle = Math.min(1.0, controls.throttle + 0.1);
            break;
        case 'Control':
            controls.throttle = Math.max(0.0, controls.throttle - 0.1);
            break;
    }
});

document.addEventListener('keyup', (event) => {
    switch (event.key) {
        case 'a':
        case 'd':
            controls.aileron = 0;
            break;
        case 'w':
        case 's':
            controls.elevator = 0;
            break;
        case 'q':
        case 'e':
            controls.rudder = 0;
            break;
    }
});
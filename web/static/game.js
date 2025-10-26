// Game variables
let score = 0;
let currentCheckpointIndex = 0;
const checkpointRadius = 50.0;
const checkpoints = [
    new THREE.Vector3(0, 500, 100),
    new THREE.Vector3(500, 1000, 150),
    new THREE.Vector3(1000, 1500, 200),
    new THREE.Vector3(500, 2000, 150),
    new THREE.Vector3(0, 2500, 100),
];

// Create a visual representation of the current checkpoint
const checkpointGeometry = new THREE.SphereGeometry(checkpointRadius / 2, 32, 32);
const checkpointMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000, transparent: true, opacity: 0.5 });
const checkpoint = new THREE.Mesh(checkpointGeometry, checkpointMaterial);
scene.add(checkpoint);

function setNextCheckpoint() {
    if (currentCheckpointIndex < checkpoints.length) {
        checkpoint.position.copy(checkpoints[currentCheckpointIndex]);
        checkpoint.visible = true;
    } else {
        // Mission complete
        checkpoint.visible = false;
        console.log("Mission Complete! Final Score:", score);
    }
}

function checkCollision() {
    if (currentCheckpointIndex >= checkpoints.length) {
        return;
    }

    if (aircraft) {
        const distance = aircraft.position.distanceTo(checkpoints[currentCheckpointIndex]);
        if (distance < checkpointRadius) {
            score += 100;
            console.log("Checkpoint passed! Score:", score);
            currentCheckpointIndex++;
            setNextCheckpoint();
        }
    }
}

setNextCheckpoint();
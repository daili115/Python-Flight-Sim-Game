// Get the canvas element
const canvas = document.getElementById('three-canvas');

// Create a scene
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x87CEEB); // Sky blue background

// Create a camera
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.z = 20;
camera.position.y = 5;

// Create a renderer
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);

// Add lighting
const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(200, 500, 300);
scene.add(directionalLight);

// Load the aircraft model
const loader = new THREE.OBJLoader();
let aircraft;
loader.load(
    'models/aircraft/aircraft_med.obj',
    function (object) {
        aircraft = object;
        aircraft.scale.set(0.1, 0.1, 0.1);
        scene.add(aircraft);
    },
    function (xhr) {
        console.log((xhr.loaded / xhr.total * 100) + '% loaded');
    },
    function (error) {
        console.log('An error happened', error);
    }
);

// Create a ground plane
const groundGeometry = new THREE.PlaneGeometry(2000, 2000);
const groundMaterial = new THREE.MeshBasicMaterial({ color: 0x3A5F0B, side: THREE.DoubleSide });
const ground = new THREE.Mesh(groundGeometry, groundMaterial);
ground.rotation.x = -Math.PI / 2;
scene.add(ground);

// Animation loop
function animate() {
    requestAnimationFrame(animate);

    // Update physics
    if (physicsWorld) {
        applyFlightForces();
        physicsWorld.stepSimulation(1 / 60, 10);

        // Update aircraft position and rotation
        if (aircraft && rigidBody) {
            let ms = rigidBody.getMotionState();
            if (ms) {
                let transform = new Ammo.btTransform();
                ms.getWorldTransform(transform);
                let pos = transform.getOrigin();
                let quat = transform.getRotation();
                aircraft.position.set(pos.x(), pos.y(), pos.z());
                aircraft.quaternion.set(quat.x(), quat.y(), quat.z(), quat.w());
            }
        }
    }

    // Update game logic
    checkCollision();

    // Render the scene
    renderer.render(scene, camera);
}

// Start the animation loop
animate();
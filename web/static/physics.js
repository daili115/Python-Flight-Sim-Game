// Physics variables
let physicsWorld, rigidBody;

function applyFlightForces() {
    if (!rigidBody) return;

    const transform = new Ammo.btTransform();
    rigidBody.getMotionState().getWorldTransform(transform);
    const quat = transform.getRotation();
    const position = transform.getOrigin();

    const forward = new Ammo.btVector3(0, 0, 1);
    const up = new Ammo.btVector3(0, 1, 0);
    const right = new Ammo.btVector3(1, 0, 0);

    const q = new THREE.Quaternion(quat.x(), quat.y(), quat.z(), quat.w());

    const worldForward = new THREE.Vector3(0, 0, 1).applyQuaternion(q);
    const worldUp = new THREE.Vector3(0, 1, 0).applyQuaternion(q);
    const worldRight = new THREE.Vector3(1, 0, 0).applyQuaternion(q);

    const thrustForce = 10000;
    const thrust = new Ammo.btVector3(worldForward.x, worldForward.y, worldForward.z);
    thrust.op_mul(controls.throttle * thrustForce);

    const liftForce = 10000;
    const lift = new Ammo.btVector3(worldUp.x, worldUp.y, worldUp.z);
    lift.op_mul(controls.elevator * liftForce);

    const rollTorque = 10000;
    const roll = new Ammo.btVector3(worldForward.x, worldForward.y, worldForward.z);
    roll.op_mul(controls.aileron * rollTorque);

    const pitchTorque = 10000;
    const pitch = new Ammo.btVector3(worldRight.x, worldRight.y, worldRight.z);
    pitch.op_mul(controls.elevator * pitchTorque);

    const yawTorque = 10000;
    const yaw = new Ammo.btVector3(worldUp.x, worldUp.y, worldUp.z);
    yaw.op_mul(controls.rudder * yawTorque);

    rigidBody.applyCentralForce(thrust);
    rigidBody.applyCentralForce(lift);
    rigidBody.applyTorque(roll);
    rigidBody.applyTorque(pitch);
    rigidBody.applyTorque(yaw);
}

Ammo().then(function (Ammo) {
    // Initialize the physics world
    let collisionConfiguration = new Ammo.btDefaultCollisionConfiguration();
    let dispatcher = new Ammo.btCollisionDispatcher(collisionConfiguration);
    let overlappingPairCache = new Ammo.btDbvtBroadphase();
    let solver = new Ammo.btSequentialImpulseConstraintSolver();
    physicsWorld = new Ammo.btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
    physicsWorld.setGravity(new Ammo.btVector3(0, -9.81, 0));

    // Create a rigid body for the aircraft
    let colShape = new Ammo.btBoxShape(new Ammo.btVector3(2, 0.5, 2));
    let startTransform = new Ammo.btTransform();
    startTransform.setIdentity();
    let mass = 1000;
    let localInertia = new Ammo.btVector3(0, 0, 0);
    colShape.calculateLocalInertia(mass, localInertia);
    let motionState = new Ammo.btDefaultMotionState(startTransform);
    let rbInfo = new Ammo.btRigidBodyConstructionInfo(mass, motionState, colShape, localInertia);
    rigidBody = new Ammo.btRigidBody(rbInfo);
    physicsWorld.addRigidBody(rigidBody);
});
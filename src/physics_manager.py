import pybullet as p
import pybullet_data
from panda3d.core import LVector3, LPoint3, LQuaternion, NodePath

class PhysicsManager:
    def __init__(self, base):
        self.base = base
        # Setup PyBullet physics world
        self.physicsClient = p.connect(p.DIRECT) # p.GUI for visual debugging, p.DIRECT for simulation only
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81) # Standard gravity in Z-up

        self.aircraft_id = None
        self.aircraft_np = None # Panda3D NodePath for the aircraft

        # Constants for air density (at sea level, 15 deg C)
        self.RHO = 1.225 # kg/m^3

        # Aircraft parameters (Placeholder for a small propeller plane like Cessna 172)
        self.mass = 1000.0 # kg
        self.wing_area = 16.2 # m^2 (approx for Cessna 172)
        self.cl_max = 1.6 # Max lift coefficient
        self.cd_min = 0.03 # Min drag coefficient
        self.engine_power = 10000.0 # Placeholder for thrust force (Newtons)

        # Current state
        self.velocity = LVector3(0, 0, 0)
        
        # Import math functions from pybullet
        import math as p
        self.angular_velocity = LVector3(0, 0, 0)
        self.thrust = 0.0
        self.control_inputs = {'aileron': 0.0, 'elevator': 0.0, 'rudder': 0.0, 'throttle': 0.0}

    def load_aircraft(self, model_path: str, initial_pos: LPoint3, initial_hpr: LVector3):
        # Load the visual model into Panda3D
        self.aircraft_np = self.base.loader.loadModel(model_path)
        self.aircraft_np.reparentTo(self.base.render)
        self.aircraft_np.setPos(initial_pos)
        self.aircraft_np.setHpr(initial_hpr)

        # Create a simple collision shape in PyBullet for the aircraft
        # In a real scenario, we would load a more accurate collision mesh
        col_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[2, 2, 0.5])
        
        # Create a visual shape (optional, since Panda3D handles visuals)
        vis_shape_id = -1 # No visual shape needed if using Panda3D

        # Initial position/orientation for PyBullet
        pos = [initial_pos.x, initial_pos.y, initial_pos.z]
        quat = self.aircraft_np.getQuat().getQuat()
        orn = [quat.getI(), quat.getJ(), quat.getK(), quat.getR()]

        # Create the rigid body in PyBullet
        self.aircraft_id = p.createMultiBody(
            baseMass=self.mass,
            baseCollisionShapeIndex=col_shape_id,
            baseVisualShapeIndex=vis_shape_id,
            basePosition=pos,
            baseOrientation=orn
        )
        
        # Disable PyBullet's default gravity and dynamics for the aircraft
        # We will apply forces manually to implement custom flight physics
        p.changeDynamics(self.aircraft_id, -1, mass=self.mass, linearDamping=0, angularDamping=0)
        p.setBodyActivationState(self.aircraft_id, p.STATE_ISLAND_SLEEPING, 0) # Keep it active
        
        # Set initial velocity (e.g., 50 m/s forward along Y-axis)
        p.resetBaseVelocity(self.aircraft_id, linearVelocity=[0, 50, 0], angularVelocity=[0, 0, 0])
        
        # Get the initial velocity from PyBullet
        linear_vel, _ = p.getBaseVelocity(self.aircraft_id)
        self.velocity = LVector3(linear_vel[0], linear_vel[1], linear_vel[2])

        return self.aircraft_np

    def apply_flight_forces(self, dt):
        """
        Calculates and applies custom flight physics forces (Lift, Drag, Thrust, Gravity).
        """
        if not self.aircraft_id:
            return

        # 1. Get current state from PyBullet
        pos_pb, quat_pb = p.getBasePositionAndOrientation(self.aircraft_id)
        linear_vel_pb, angular_vel_pb = p.getBaseVelocity(self.aircraft_id)
        
        # Convert PyBullet state to Panda3D types
        pos = LPoint3(pos_pb[0], pos_pb[1], pos_pb[2])
        quat = LQuaternion(quat_pb[3], quat_pb[0], quat_pb[1], quat_pb[2]) # (w, i, j, k) -> (r, i, j, k)
        self.velocity = LVector3(linear_vel_pb[0], linear_vel_pb[1], linear_vel_pb[2])
        
        # Update Panda3D visual model
        self.aircraft_np.setPos(pos)
        self.aircraft_np.setQuat(quat)

        # Get the aircraft's forward (Y-axis) and up (Z-axis) vectors in world space
        forward_vec = quat.xform(LVector3(0, 1, 0))
        up_vec = quat.xform(LVector3(0, 0, 1))

        # Airspeed (magnitude of velocity)
        airspeed = self.velocity.length()
        
        # Angle of Attack (AoA) calculation (simplified: angle between velocity vector and forward vector)
        if airspeed > 1.0:
            vel_norm = self.velocity.normalized()
            # 2. Calculate Angle of Attack (AoA)
            # AoA is the angle between the velocity vector and the aircraft's longitudinal axis (body Y-axis)
            # Find velocity vector in the aircraft's body frame
            vel_body = quat.conjugate().xform(self.velocity)
            
            # Calculate AoA (angle between vel_body's projection on YZ plane and Y-axis)
            # atan2(Z component, Y component)
            aoa_rad = -p.atan2(vel_body.z, vel_body.y) # Negative sign because positive Z is down in PyBullet's default setup, but Panda3D's Z is up. Let's stick to Panda3D's Z-up convention.
            # Wait, PyBullet is Z-up by default. Let's ensure consistency.
            # Panda3D: Y is forward, Z is up, X is right.
            # PyBullet: Z is up, X is forward, Y is left (by default, but we are using Panda3D's convention of Y forward).
            # If we assume Panda3D's coordinate system (Y-forward, Z-up, X-right) is used for the model:
            
            # AoA is the angle between the velocity vector and the body Y-axis (forward)
            aoa_rad = p.atan2(vel_body.z, vel_body.y) # Angle in YZ plane
            
            # Convert to degrees for easier parameterization
            aoa_deg = p.degrees(aoa_rad)
            
            # 3. Lift and Drag Coefficient Calculation (Simplified for demonstration)
            
            # Lift Coefficient (CL) - Simplified stall model
            # Max CL at 15 degrees (stall angle)
            stall_angle = 15.0
            
            if abs(aoa_deg) < stall_angle:
                # Linear lift before stall
                CL = self.cl_max * (aoa_deg / stall_angle)
            else:
                # Post-stall: lift drops off sharply
                sign = 1 if aoa_deg > 0 else -1
                CL = sign * (self.cl_max * 0.8 * p.cos(aoa_rad)) # Simple drop-off
            
            # Drag Coefficient (CD) - Induced drag + parasite drag
            CD = self.cd_min + 0.05 * CL**2 # Induced drag component (K*CL^2)

            # Add control surface deflection effect to CL
            # Elevator (Pitch control) primarily affects AoA/CL
            CL += self.control_inputs['elevator'] * 0.5 # 0.5 is a control effectiveness factor

            # Dynamic Pressure (Q)
            Q = 0.5 * self.RHO * airspeed**2

            # 4. Lift Force
            lift_magnitude = Q * self.wing_area * CL
            
            # Lift Direction: Perpendicular to the velocity vector, in the plane of the wings.
            # In the body frame, lift is along the Z-axis (up). In the wind frame, it's perpendicular to velocity.
            # We apply force in the **wind frame**: Lift is perpendicular to velocity, Drag is parallel to velocity.
            
            # Calculate the lift direction vector (perpendicular to velocity, in the plane of symmetry)
            # Cross product of velocity and the wing span vector (body X-axis)
            wing_span_vec = quat.xform(LVector3(1, 0, 0)) # Body X-axis in world space
            lift_direction = (self.velocity.cross(wing_span_vec)).normalized()
            lift_force = lift_direction * lift_magnitude
            
            # 5. Drag Force (opposite to velocity vector)
            drag_magnitude = Q * self.wing_area * CD
            drag_force = -vel_norm * drag_magnitude

        # 3. Thrust Force (along the forward vector)
        self.thrust = self.control_inputs['throttle'] * self.engine_power # Throttle is 0 to 1
        thrust_force = forward_vec * self.thrust

        # 4. Gravity Force (constant downwards force)
        gravity_force = LVector3(0, 0, -9.81 * self.mass)

        # 5. Total Force
        total_force = gravity_force + thrust_force + lift_force + drag_force
        
        # 6. Apply force and torque to PyBullet
        # Apply force at the center of mass (COM)
        p.applyExternalForce(
            self.aircraft_id, 
            -1, # Link index (-1 for base)
            [total_force.x, total_force.y, total_force.z], 
            [0, 0, 0], # Position in link frame (0,0,0 is COM)
            p.WORLD_FRAME
        )

        # 7. Apply Control Torques (Aerodynamic Moments)
        # Torques are generated by control surfaces (aileron, elevator, rudder)
        # They are proportional to dynamic pressure (Q) and control input.
        
        # Dynamic Pressure (Q) - calculated earlier
        Q = 0.5 * self.RHO * airspeed**2
        
        # Simplified Moment Coefficients (Cm)
        # Assuming a reference area (wing_area) and a reference length (e.g., wing chord, fuselage length)
        ref_length = 5.0 # meters
        
        # Pitch Moment (around body X-axis) - primarily controlled by Elevator
        # Cm_pitch = Cm_0 + Cm_alpha * AoA + Cm_elevator * Elevator
        # For simplicity, we only use the control input for the moment
        Cm_pitch = self.control_inputs['elevator'] * 0.1 # 0.1 is a moment coefficient factor
        
        # Roll Moment (around body Y-axis) - primarily controlled by Aileron
        Cm_roll = self.control_inputs['aileron'] * 0.05
        
        # Yaw Moment (around body Z-axis) - primarily controlled by Rudder
        Cm_yaw = self.control_inputs['rudder'] * 0.03
        
        # Torque Magnitude = Q * WingArea * RefLength * Cm
        pitch_torque_mag = Q * self.wing_area * ref_length * Cm_pitch
        roll_torque_mag = Q * self.wing_area * ref_length * Cm_roll
        yaw_torque_mag = Q * self.wing_area * ref_length * Cm_yaw
        
        # Torques are applied in the *body frame* (relative to the aircraft's orientation)
        # Panda3D Body Frame: X-Right, Y-Forward, Z-Up
        body_torque = LVector3(
            roll_torque_mag,    # Roll around Y (Pitch) - WRONG. Roll is around Y in Panda3D's default frame, but around X in aircraft body frame
            -pitch_torque_mag,  # Pitch around X (Roll) - WRONG. Pitch is around X in Panda3D's default frame, but around Y in aircraft body frame
            yaw_torque_mag      # Yaw around Z
        )
        
        # Correct Body Frame (X-Right, Y-Forward, Z-Up)
        # Roll: around Y (Panda3D) / around X (Aircraft) -> Let's use Aircraft convention: X-Roll, Y-Pitch, Z-Yaw
        # Torque Vector: (Roll_X, Pitch_Y, Yaw_Z)
        
        body_torque = LVector3(
            roll_torque_mag,
            -pitch_torque_mag, # Negative for nose up (positive elevator input)
            yaw_torque_mag
        )
        
        # Convert body torque to world torque
        world_torque = quat.xform(body_torque)

        p.applyExternalTorque(
            self.aircraft_id,
            -1,
            [world_torque.x, world_torque.y, world_torque.z],
            p.WORLD_FRAME
        )

    def update_controls(self, inputs: dict):
        """
        Updates the control inputs from the user.
        """
        self.control_inputs.update(inputs)

    def update(self, dt):
        """
        Main physics update loop.
        """
        # Apply custom flight forces and torques
        self.apply_flight_forces(dt)
        
        # Step the PyBullet simulation
        p.stepSimulation()

    def get_aircraft_state(self):
        """
        Returns the current position, orientation, and velocity of the aircraft.
        """
        if not self.aircraft_id:
            return None, None, None
        
        pos_pb, quat_pb = p.getBasePositionAndOrientation(self.aircraft_id)
        linear_vel_pb, angular_vel_pb = p.getBaseVelocity(self.aircraft_id)
        
        pos = LPoint3(pos_pb[0], pos_pb[1], pos_pb[2])
        quat = LQuaternion(quat_pb[3], quat_pb[0], quat_pb[1], quat_pb[2])
        linear_vel = LVector3(linear_vel_pb[0], linear_vel_pb[1], linear_vel_pb[2])
        
        return pos, quat, linear_vel

    def disconnect(self):
        p.disconnect()

# Example usage in main.py:
# from physics_manager import PhysicsManager
# ...
# self.physics_mgr = PhysicsManager(self)
# self.aircraft = self.physics_mgr.load_aircraft("path/to/model.egg", LPoint3(0, 0, 10), LVector3(0, 0, 0))
# ...
# self.physics_mgr.update_controls({'throttle': 0.8, 'elevator': 0.1})
# ...
# In run_simulation: self.physics_mgr.update(dt)

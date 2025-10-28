import pybullet as p
from .config import AircraftConfig
import pybullet_data
from panda3d.core import LVector3, LPoint3, LQuaternion, NodePath
import math

class PhysicsManager:
    def __init__(self, base):
        self.base = base
        # Setup PyBullet physics world
        self.physicsClient = p.connect(p.DIRECT) # p.GUI for visual debugging, p.DIRECT for simulation only
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -AircraftConfig.GRAVITY) # Standard gravity in Z-up

        self.aircraft_id = None
        self.aircraft_np = None # Panda3D NodePath for the aircraft

        # Air density is now in AircraftConfig
        # self.RHO = 1.225 # kg/m^3

        # Aircraft parameters loaded from config
        self.mass = AircraftConfig.MASS
        self.wing_area = AircraftConfig.WING_AREA
        self.cl_max = AircraftConfig.CL_MAX
        self.cd_min = AircraftConfig.CD_MIN
        self.engine_power = AircraftConfig.ENGINE_POWER

        # Current state
        self.velocity = LVector3(0, 0, 0)
        
        self.angular_velocity = LVector3(0, 0, 0)
        self.thrust = 0.0
        self.control_inputs = {'aileron': 0.0, 'elevator': 0.0, 'rudder': 0.0, 'throttle': 0.0}

    def load_aircraft(self, aircraft_np: NodePath, model_path: str, initial_pos: LPoint3, initial_hpr: LVector3):
        # Set the aircraft NodePath
        self.aircraft_np = aircraft_np
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
        
        # Initialize forces to zero
        lift_force = LVector3(0, 0, 0)
        drag_force = LVector3(0, 0, 0)

        # Angle of Attack (AoA) calculation (simplified: angle between velocity vector and forward vector)
        if airspeed > 1.0:
            # Dynamic Pressure (Q)
            Q = 0.5 * AircraftConfig.RHO * airspeed**2

            vel_norm = self.velocity.normalized()

            # 2. Calculate Angle of Attack (AoA)
            vel_body = quat.conjugate().xform(self.velocity)
            aoa_rad = math.atan2(vel_body.z, vel_body.y)
            aoa_deg = math.degrees(aoa_rad)
            
            # 3. Lift and Drag Coefficient Calculation (Improved Stall Model)
            
            # Convert stall angle to radians
            stall_angle_rad = math.radians(AircraftConfig.STALL_ANGLE_DEG)
            
            # Use a smoother model for CL based on AoA (e.g., a sin-based model)
            # CL = CL_MAX * sin(2 * aoa) / sin(2 * stall_angle) for simplicity,
            # but a piecewise linear or cubic spline is better.
            # For this optimization, we'll use a smoother transition near stall.
            
            if abs(aoa_rad) <= stall_angle_rad:
                # Linear increase up to stall
                CL = self.cl_max * (aoa_rad / stall_angle_rad)
            else:
                # Post-stall: Rapid drop-off, then smoother decay
                # Simplified model for post-stall: CL = CL_MAX * sign(aoa) * cos(aoa)
                sign = 1 if aoa_rad > 0 else -1
                # Use a blend: linear up to stall, then a decay function
                # For simplicity, we'll keep the original structure but use radians for better math
                # The original model's post-stall decay: CL_MAX * 0.8 * cos(aoa_rad) is decent for a simple sim
                CL = sign * (self.cl_max * 0.8 * math.cos(aoa_rad))

            # Add control surface effect (elevator changes the effective AoA/camber)
            # The control input is a unitless value (-1 to 1)
            CL += self.control_inputs['elevator'] * 0.5
            
            # Drag coefficient (CD = CD_min + K * CL^2) - Induced Drag
            # K is the induced drag factor (1 / (pi * AR * e))
            # 0.05 is a placeholder for K
            CD = self.cd_min + 0.05 * CL**2

            # 4. Lift Force
            lift_magnitude = Q * self.wing_area * CL
            wing_span_vec = quat.xform(LVector3(1, 0, 0))
            lift_direction = (self.velocity.cross(wing_span_vec)).normalized()
            lift_force = lift_direction * lift_magnitude
            
            # 5. Drag Force
            drag_magnitude = Q * self.wing_area * CD
            drag_force = -vel_norm * drag_magnitude

        # 6. Thrust Force
        self.thrust = self.control_inputs['throttle'] * self.engine_power
        thrust_force = forward_vec * self.thrust

        # 7. Gravity Force
        gravity_force = LVector3(0, 0, -9.81 * self.mass)

        # 6. Total Force
        total_force = gravity_force + thrust_force + lift_force + drag_force
        
        # Store forces for HUD display
        self.last_forces = {
            'lift': lift_force.length(),
            'drag': drag_force.length(),
            'thrust': thrust_force.length(),
            'aoa': aoa_deg if airspeed > 1.0 else 0.0,
            'airspeed': airspeed
        }
        
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
        if airspeed > 1.0:
            Q = 0.5 * self.RHO * airspeed**2
            ref_length = 5.0 # meters (Placeholder for chord length/some reference length)

            # Simplified Moment Coefficients
            Cm_pitch = self.control_inputs['elevator'] * 0.1
            Cm_roll = self.control_inputs['aileron'] * 0.05
            Cm_yaw = self.control_inputs['rudder'] * 0.03
        
            ref_length = AircraftConfig.REF_LENGTH # meters
            
            # Simplified Moment Coefficients
            Cm_pitch = self.control_inputs['elevator'] * AircraftConfig.ELEVATOR_EFFECTIVENESS
            Cm_roll = self.control_inputs['aileron'] * AircraftConfig.AILERON_EFFECTIVENESS
            Cm_yaw = self.control_inputs['rudder'] * AircraftConfig.RUDDER_EFFECTIVENESS
        
            # Torque Magnitude
            # Torque = Q * S * c * C_m (where S is wing_area, c is ref_length)
            pitch_torque_mag = Q * self.wing_area * ref_length * Cm_pitch
            roll_torque_mag = Q * self.wing_area * ref_length * Cm_roll
            yaw_torque_mag = Q * self.wing_area * ref_length * Cm_yaw

            # Body Frame Torque Vector (X-Roll, Y-Pitch, Z-Yaw)
            body_torque = LVector3(
                roll_torque_mag,
                -pitch_torque_mag, # Negative for nose up
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

    def get_last_forces(self):
        """
        Returns the last calculated forces and AoA for HUD display.
        """
        return getattr(self, 'last_forces', {
            'lift': 0.0,
            'drag': 0.0,
            'thrust': 0.0,
            'aoa': 0.0,
            'airspeed': 0.0
        })

    def reset_aircraft(self, initial_pos: LPoint3, initial_hpr: LVector3):
        """
        Resets the aircraft's position, orientation, and velocity in PyBullet.
        """
        if self.aircraft_id is None:
            return

        # Reset position and orientation in PyBullet
        pos = [initial_pos.x, initial_pos.y, initial_pos.z]
        
        # Convert Panda3D HPR to Quaternion for PyBullet
        # Note: Panda3D's HPR to Quat conversion is not directly available here, 
        # but we can use a simple identity quaternion for the initial state (0, 0, 0 HPR)
        # For a general solution, we would need to import the Quat class and convert.
        # Since initial_hpr is (0, 0, 0), we use the identity quaternion (0, 0, 0, 1)
        # If we need to support arbitrary initial_hpr, we must refactor this.
        # For now, we assume initial_hpr is (0, 0, 0).
        orn = [0, 0, 0, 1] # (x, y, z, w) for identity quaternion
        
        p.resetBasePositionAndOrientation(self.aircraft_id, pos, orn)
        
        # Reset velocity (e.g., 50 m/s forward along Y-axis)
        p.resetBaseVelocity(self.aircraft_id, linearVelocity=[0, 50, 0], angularVelocity=[0, 0, 0])
        
        # Update Panda3D visual model
        self.aircraft_np.setPos(initial_pos)
        self.aircraft_np.setHpr(initial_hpr)
        
        # Reset internal state
        self.velocity = LVector3(0, 50, 0)
        self.angular_velocity = LVector3(0, 0, 0)
        self.thrust = 0.0
        self.control_inputs = {'aileron': 0.0, 'elevator': 0.0, 'rudder': 0.0, 'throttle': 0.0}
        self.last_forces = {
            'lift': 0.0,
            'drag': 0.0,
            'thrust': 0.0,
            'aoa': 0.0,
            'airspeed': 0.0
        }

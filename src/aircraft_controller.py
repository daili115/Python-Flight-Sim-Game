from panda3d.core import LVector3, LPoint3, Quat
from direct.task import Task
from physics_manager import PhysicsManager

class AircraftController:
    def __init__(self, base, physics_mgr: PhysicsManager, aircraft_np):
        self.base = base
        self.physics_mgr = physics_mgr
        self.aircraft_np = aircraft_np
        
        # Control inputs (range -1.0 to 1.0, except throttle 0.0 to 1.0)
        self.aileron = 0.0 # Roll
        self.elevator = 0.0 # Pitch
        self.rudder = 0.0 # Yaw
        self.throttle = 0.5 # Thrust

        # Control sensitivity
        self.control_rate = 0.5 # Rate of change per second for aileron, elevator, rudder
        self.throttle_rate = 0.2 # Rate of change per second for throttle

        # Camera control
        self.view_mode = 1 # 1: Third-person, 2: First-person, 3: Free-look
        self.camera_distance = 50 # Third-person distance
        self.camera_pitch = -10 # Third-person pitch angle
        
        # Free-look camera state
        self.free_look_h = 0
        self.free_look_p = 0
        self.free_look_r = 0
        self.free_look_speed = 50.0
        self.free_look_active = False

        # Setup input handling
        self.setup_controls()

        # Start the control update task
        self.base.taskMgr.add(self.update_controls, "AircraftControlTask")
        self.base.taskMgr.add(self.update_camera, "CameraUpdateTask")

    def setup_controls(self):
        # Aileron (Roll)
        self.base.accept("a", self.set_control_if_not_free_look, ["aileron", 1])
        self.base.accept("a-up", self.set_control_if_not_free_look, ["aileron", 0])
        self.base.accept("d", self.set_control_if_not_free_look, ["aileron", -1])
        self.base.accept("d-up", self.set_control_if_not_free_look, ["aileron", 0])

        # Elevator (Pitch)
        self.base.accept("w", self.set_control_if_not_free_look, ["elevator", 1]) # Nose down
        self.base.accept("w-up", self.set_control_if_not_free_look, ["elevator", 0])
        self.base.accept("s", self.set_control_if_not_free_look, ["elevator", -1]) # Nose up
        self.base.accept("s-up", self.set_control_if_not_free_look, ["elevator", 0])
        
        # Rudder (Yaw)
        self.base.accept("q", self.set_control, ["rudder", -1])
        self.base.accept("q-up", self.set_control, ["rudder", 0])
        self.base.accept("e", self.set_control, ["rudder", 1])
        self.base.accept("e-up", self.set_control, ["rudder", 0])

        # Throttle
        self.base.accept("shift", self.set_throttle_input, [1]) # Increase
        self.base.accept("shift-up", self.set_throttle_input, [0])
        self.base.accept("control", self.set_throttle_input, [-1]) # Decrease
        self.base.accept("control-up", self.set_throttle_input, [0])

        # View Toggle
        self.base.accept("v", self.toggle_view)
        
        # Free-look controls (using mouse for rotation, WASD for movement)
        self.base.accept("mouse1", self.toggle_free_look, [True])
        self.base.accept("mouse1-up", self.toggle_free_look, [False])
        
        # Free-look movement controls (W, S, A, D, Space, Z)
        self.base.accept("space", self.set_control, ["free_up", 1])
        self.base.accept("space-up", self.set_control, ["free_up", 0])
        self.base.accept("z", self.set_control, ["free_down", 1])
        self.base.accept("z-up", self.set_control, ["free_down", 0])
        self.base.accept("w-repeat", self.set_control, ["free_forward", 1])
        self.base.accept("w-up", self.set_control, ["free_forward", 0])
        self.base.accept("s-repeat", self.set_control, ["free_backward", 1])
        self.base.accept("s-up", self.set_control, ["free_backward", 0])
        self.base.accept("a-repeat", self.set_control, ["free_left", 1])
        self.base.accept("a-up", self.set_control, ["free_left", 0])
        self.base.accept("d-repeat", self.set_control, ["free_right", 1])
        self.base.accept("d-up", self.set_control, ["free_right", 0])

        # State flags for continuous input
        self.input_states = {
            'aileron': 0, 'elevator': 0, 'rudder': 0,
            'throttle_input': 0, # 1 for up, -1 for down, 0 for none
            'free_up': 0, 'free_down': 0, 'free_forward': 0, 'free_backward': 0, 'free_left': 0, 'free_right': 0
        }

    def set_control(self, control_name, value):
        """Sets the flag for continuous control input."""
        self.input_states[control_name] = value

    def set_control_if_not_free_look(self, control_name, value):
        """Sets the flag for flight control input only if not in free-look mode."""
        if self.view_mode != 3:
            self.input_states[control_name] = value

    def set_throttle_input(self, value):
        """Sets the flag for continuous throttle input."""
        self.input_states['throttle_input'] = value

    def update_controls(self, task):
        dt = self.base.globalClock.getDt()

        # Smoothly adjust control surfaces towards the input value
        self.aileron += self.input_states['aileron'] * self.control_rate * dt
        self.elevator += self.input_states['elevator'] * self.control_rate * dt
        self.rudder += self.input_states['rudder'] * self.control_rate * dt
        
        # Clamp controls to [-1.0, 1.0]
        self.aileron = max(-1.0, min(1.0, self.aileron))
        self.elevator = max(-1.0, min(1.0, self.elevator))
        self.rudder = max(-1.0, min(1.0, self.rudder))

        # Smoothly adjust throttle
        self.throttle += self.input_states['throttle_input'] * self.throttle_rate * dt
        # Clamp throttle to [0.0, 1.0]
        self.throttle = max(0.0, min(1.0, self.throttle))

        # Send updated controls to the physics manager
        self.physics_mgr.update_controls({
            'aileron': self.aileron,
            'elevator': self.elevator,
            'rudder': self.rudder,
            'throttle': self.throttle
        })

        return Task.cont

    def toggle_view(self):
        self.view_mode = (self.view_mode % 3) + 1 # Toggles between 1, 2, 3
        
        if self.view_mode == 3:
            # Entering free-look
            pos, quat, _ = self.physics_mgr.get_aircraft_state()
            if pos is not None:
                # Set camera to a position behind the aircraft, but not directly following
                self.base.camera.setPos(pos + quat.xform(LVector3(0, -50, 10))) 
                self.base.camera.lookAt(self.aircraft_np)
                self.free_look_h, self.free_look_p, self.free_look_r = self.base.camera.getHpr()
                self.base.disableMouse() # Ensure mouse is disabled for Panda3D's default camera control
        else:
            self.base.disableMouse() # Ensure mouse is disabled for Panda3D's default camera control
        
        print(f"View mode toggled to: {['Third-person', 'First-person', 'Free-look'][self.view_mode - 1]}")

    def toggle_free_look(self, active):
        self.free_look_active = active
        self.base.mouseWatcherNode.setCursorHidden(not active)
        if active:
            # Center mouse when starting free-look
            self.base.win.movePointer(0, int(self.base.win.getXSize() / 2), int(self.base.win.getYSize() / 2))

    def update_camera(self, task):
        pos, quat, _ = self.physics_mgr.get_aircraft_state()
        
        if pos is None:
            return Task.cont

        if self.view_mode == 1:
            # Third-person view (Follow cam)
            self.base.mouseWatcherNode.setCursorHidden(True)
            
            # Calculate offset vector: back and slightly up
            offset = LVector3(0, -self.camera_distance, 10)
            
            # Rotate offset by aircraft's orientation
            offset_rotated = quat.xform(offset)
            
            # Set camera position
            cam_pos = pos + offset_rotated
            self.base.camera.setPos(cam_pos)
            
            # Look at the aircraft
            self.base.camera.lookAt(self.aircraft_np)

        elif self.view_mode == 2:
            # First-person view (Cockpit cam)
            self.base.mouseWatcherNode.setCursorHidden(True)
            
            # Set camera position to a point slightly in front of the aircraft's COM
            cockpit_offset = LVector3(0, 5, 1) # 5 units forward, 1 unit up
            cockpit_pos = pos + quat.xform(cockpit_offset)
            
            # Set camera orientation to match the aircraft's orientation
            self.base.camera.setPos(cockpit_pos)
            # Quat.getQuat() returns (r, i, j, k) where r is the real part.
            # Panda3D's Quat constructor takes (r, i, j, k)
            self.base.camera.setQuat(quat)

            # A small, subtle camera shake could enhance the feeling of speed/vibration
            # Not implemented for simplicity, but a good future enhancement.

        elif self.view_mode == 3:
            # Free-look view
            self.base.mouseWatcherNode.setCursorHidden(False)
            
            # Handle mouse input for rotation
            if self.base.mouseWatcherNode.hasMouse() and self.free_look_active:
                x = self.base.mouseWatcherNode.getMouseX()
                y = self.base.mouseWatcherNode.getMouseY()
                
                # Center the mouse
                self.base.win.movePointer(0, int(self.base.win.getXSize() / 2), int(self.base.win.getYSize() / 2))
                
                # Calculate change
                dx = x * 100 # Sensitivity factor
                dy = y * 100
                
                self.free_look_h -= dx
                self.free_look_p += dy
                
                # Clamp pitch
                self.free_look_p = max(-89, min(89, self.free_look_p))
                
                self.base.camera.setHpr(self.free_look_h, self.free_look_p, self.free_look_r)

            # Handle keyboard input for movement
            dt = self.base.globalClock.getDt()
            
            forward_vec = self.base.camera.getQuat().xform(LVector3(0, 1, 0))
            right_vec = self.base.camera.getQuat().xform(LVector3(1, 0, 0))
            up_vec = self.base.camera.getQuat().xform(LVector3(0, 0, 1))
            
            movement = LVector3(0, 0, 0)
            
            if self.input_states['free_forward']:
                movement += forward_vec
            if self.input_states['free_backward']:
                movement -= forward_vec
            if self.input_states['free_right']:
                movement += right_vec
            if self.input_states['free_left']:
                movement -= right_vec
            if self.input_states['free_up']:
                movement += up_vec
            if self.input_states['free_down']:
                movement -= up_vec
                
            if movement.length() > 0:
                movement.normalize()
                self.base.camera.setPos(self.base.camera.getPos() + movement * self.free_look_speed * dt)
                
            # If entering free-look, set camera position to current aircraft position
            if task.time == 0: # First time running this view mode
                # The initial position logic is now in toggle_view, so this is not strictly needed.
                pass

        return Task.cont

# Example usage in main.py:
# from aircraft_controller import AircraftController
# ...
# self.controller = AircraftController(self, self.physics_mgr, self.aircraft)
# ...
# No need to manually call update_controls or update_camera, they are added as tasks.

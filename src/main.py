from direct.showbase.ShowBase import ShowBase
from panda3d.core import LPoint3, LVector3, CullFaceAttrib, TextNode
from direct.gui.OnscreenText import OnscreenText
from physics_manager import PhysicsManager
from aircraft_controller import AircraftController
from game_manager import GameManager

class FlightSim(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        
        # Performance Optimization Settings
        self.render.setTwoSided(False) # Optimize rendering by disabling two-sided rendering by default
        self.render.setShaderAuto() # Enable automatic shader generation for better performance
        self.setFrameRateMeter(True) # Show FPS meter for performance monitoring
        
        self.set_background_color(0.53, 0.81, 0.98, 1) # Light blue sky
        self.disable_mouse() # Disable default mouse control

        # Setup lighting
        self.setup_lighting()

        # Load environment/skybox
        self.load_environment()
        
        # Initialize Physics Manager (MUST be before loading aircraft)
        self.physics_mgr = PhysicsManager(self)
        
        # Initialize Game Manager
        self.game_mgr = GameManager(self, self.physics_mgr)
        
        # Load the placeholder aircraft model with LOD
        from panda3d.core import LODNode
        
        lod_node = LODNode('aircraft_lod')
        lod_np = self.render.attachNewNode(lod_node)

        # Define distances for LOD switching
        lod_node.addSwitch(100, 0)    # High detail up to 100 units
        lod_node.addSwitch(500, 100)   # Medium detail from 100 to 500 units
        lod_node.addSwitch(2000, 500)  # Low detail from 500 to 2000 units

        # Load models for each LOD level
        high_model = self.loader.loadModel("models/aircraft/aircraft_high.obj")
        med_model = self.loader.loadModel("models/aircraft/aircraft_med.obj")
        low_model = self.loader.loadModel("models/aircraft/aircraft_low.obj")

        # Attach models to the LOD node path
        high_model.reparentTo(lod_np)
        med_model.reparentTo(lod_np)
        low_model.reparentTo(lod_np)

        # The physics body will use the medium-detail model's path for simplicity
        aircraft_model_path = "models/aircraft/aircraft_med.obj"
        
        initial_pos = LPoint3(0, 0, 100) # Start high in the air
        initial_hpr = LVector3(0, 0, 0) # Heading, Pitch, Roll
        
        # We pass the LOD node path to the physics manager
        self.aircraft = self.physics_mgr.load_aircraft(lod_np, aircraft_model_path, initial_pos, initial_hpr)

        # Initialize Aircraft Controller
        self.controller = AircraftController(self, self.physics_mgr, self.aircraft)

        # Placeholder for controls
        self.accept("escape", self.userExit)
        
        # Setup Sound System
        self.engine_sound = self.loader.loadSfx("audio/engine.wav") # Placeholder path
        if self.engine_sound:
            self.engine_sound.setLoop(True)
            self.engine_sound.play()
            
        # Setup HUD
        self.hud_text = OnscreenText(text="HUD Initializing...", pos=(-1.3, 0.9), scale=0.07, 
                                     align=TextNode.ALeft, fg=(1, 1, 1, 1), shadow=(0, 0, 0, 1))

        # Add HUD update task
        self.taskMgr.add(self.update_hud, "UpdateHUDTask")

    def setup_lighting(self):
        from panda3d.core import DirectionalLight, AmbientLight, VBase4
        
        # Ambient Light
        ambientLight = AmbientLight('ambientLight')
        ambientLight.setColor(VBase4(0.6, 0.6, 0.6, 1))
        self.ambientLightNode = self.render.attachNewNode(ambientLight)
        self.render.setLight(self.ambientLightNode)

        # Directional Light (Sun)
        mainLight = DirectionalLight('mainLight')
        mainLight.setColor(VBase4(0.8, 0.8, 0.8, 1))
        mainLight.setDirection(0, 0, -1) # Shining straight down
        self.mainLightNode = self.render.attachNewNode(mainLight)
        self.mainLightNode.setHpr(45, -45, 0)
        self.render.setLight(self.mainLightNode)

    def load_environment(self):
        # Load Skybox
        # A skybox is typically a large cube or sphere. We will use a simple sphere model 
        # and apply a sky texture (or just the background color for now).
        self.skybox = self.loader.loadModel("models/sphere")
        self.skybox.reparentTo(self.render)
        self.skybox.setScale(5000) # Make it very large
        self.skybox.setAttrib(CullFaceAttrib.make(CullFaceAttrib.MCullClockwise)) # Invert culling for inside-out viewing
        self.skybox.setLightOff() # Skybox should not be affected by scene lighting
        self.skybox.setBin('background', 1)
        self.skybox.setDepthWrite(0)
        
        # Load Terrain Mesh (Placeholder for now)
        # We will use a simple plane as a placeholder for the terrain mesh
        self.terrain = self.loader.loadModel("models/plane")
        self.terrain.reparentTo(self.render)
        self.terrain.setScale(2000, 2000, 1)
        self.terrain.setPos(0, 0, 0)
        self.terrain.setColor(0.3, 0.5, 0.3, 1) # Greenish color
        self.terrain.setTwoSided(True) # Ensure it's visible from both sides

        # The background color is set in __init__ for the sky (no texture needed for now)

    def run_simulation(self, task):
        # This function will be the main game loop for physics and updates
        dt = self.globalClock.getDt()
        
        # Physics update
        if self.physics_mgr:
            self.physics_mgr.update(dt)
            
        # Sound update
        if self.engine_sound:
            # Adjust pitch and volume based on throttle (or speed for more realism)
            throttle_norm = self.controller.throttle
            self.engine_sound.setVolume(0.3 + throttle_norm * 0.7) # Volume 0.3 to 1.0
            self.engine_sound.setPlayRate(1.0 + throttle_norm * 0.5) # Pitch 1.0 to 1.5
        
        return task.cont

    def update_hud(self, task):
        if self.physics_mgr and self.controller:
            pos, quat, linear_vel = self.physics_mgr.get_aircraft_state()
            
            if pos is not None:
                # Calculate speed in km/h
                speed_mps = linear_vel.length()
                speed_kph = speed_mps * 3.6
                
                # Altitude (Z-coordinate)
                altitude = pos.z
                
                # Throttle
                throttle = self.controller.throttle * 100
                
                # Angle of Attack (AoA) - Need to get from physics_manager (for now, use a placeholder)
                # In a real scenario, we would expose AoA from physics_manager
                
                game_status = self.game_mgr.get_game_status()
                
                hud_text = f"Speed: {speed_kph:.1f} km/h\n"
                hud_text += f"Altitude: {altitude:.1f} m\n"
                hud_text += f"Throttle: {throttle:.0f}%\n"
                hud_text += f"Score: {game_status['score']}\n"
                hud_text += f"Mission: {game_status['current_checkpoint']}/{game_status['total_checkpoints']} ({game_status['mission_status']})\n"
                hud_text += f"View: {['Third-person', 'First-person', 'Free-look'][self.controller.view_mode - 1]}"
                
                self.hud_text.setText(hud_text)

        return task.cont

if __name__ == '__main__':
    app = FlightSim()
    # The physics update is critical and should run every frame
    app.taskMgr.add(app.run_simulation, "PhysicsSimulationTask", priority=0) 
    app.run()

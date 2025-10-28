from panda3d.core import LPoint3, LVector3
from direct.task import Task
import random

class GameManager:
    def __init__(self, base, physics_mgr):
        self.base = base
        self.physics_mgr = physics_mgr
        
        # Game State
        self.score = 0
        self.current_checkpoint_index = 0
        self.checkpoint_radius = 50.0 # Radius for checkpoint detection
        
        # Checkpoint list (World coordinates: X, Y, Z)
        self.checkpoints = [
            LPoint3(0, 500, 100),
            LPoint3(500, 1000, 150),
            LPoint3(1000, 1500, 200),
            LPoint3(500, 2000, 150),
            LPoint3(0, 2500, 100)
        ]
        
        # Visual representation of the current checkpoint
        self.checkpoint_model = self.base.loader.loadModel("models/sphere")
        self.checkpoint_model.reparentTo(self.base.render)
        self.checkpoint_model.setScale(self.checkpoint_radius / 2)
        self.checkpoint_model.setColor(1, 0, 0, 0.5) # Red and transparent

        # Start game loop
        self.base.taskMgr.add(self.game_loop, "GameLoopTask")
        
        # Initialize first checkpoint
        self.set_next_checkpoint()

    def set_next_checkpoint(self):
        if self.current_checkpoint_index < len(self.checkpoints):
            pos = self.checkpoints[self.current_checkpoint_index]
            self.checkpoint_model.setPos(pos)
            self.checkpoint_model.show()
        else:
            # Mission complete
            self.checkpoint_model.hide()
            print("Mission Complete! Final Score:", self.score)

    def check_collision(self):
        # In a real game, this would be handled by the physics engine's collision callback.
        # Here, we use simple distance check for the checkpoint.
        
        if self.current_checkpoint_index >= len(self.checkpoints):
            return

        aircraft_pos, _, _ = self.physics_mgr.get_aircraft_state()
        if aircraft_pos is None:
            return

        current_checkpoint_pos = self.checkpoints[self.current_checkpoint_index]
        
        distance = (aircraft_pos - current_checkpoint_pos).length()
        
        if distance < self.checkpoint_radius:
            self.score += 100
            print(f"Checkpoint passed! Score: {self.score}")
            self.current_checkpoint_index += 1
            self.set_next_checkpoint()

    def game_loop(self, task):
        self.check_collision()
        return Task.cont

    def get_game_status(self):
        """Returns the current score and mission status for HUD."""
        status = {
            'score': self.score,
            'current_checkpoint': self.current_checkpoint_index + 1,
            'total_checkpoints': len(self.checkpoints),
            'mission_status': "In Progress" if self.current_checkpoint_index < len(self.checkpoints) else "Complete"
        }
        return status


    def check_for_crash(self, aircraft_pos: LPoint3):
        """
        Checks for a simple ground crash (altitude < 0) or a stall/low-speed crash.
        """
        if aircraft_pos is None:
            return

        # Simple Ground Collision (Z < 0)
        if aircraft_pos.z < 0.5: # 0.5m buffer for the aircraft's size
            print("CRASH! Ground collision detected.")
            self.reset_game_state()
            return

        # Optional: Check for stall/low-speed crash (needs velocity/AoA from physics_mgr)
        # For now, we only implement the ground collision.

    def reset_game_state(self):
        """
        Resets the aircraft to the initial position and resets the game state.
        """
        print("Game state reset.")
        # Stop the game loop temporarily
        self.base.taskMgr.remove("GameLoopTask")

        # Reset aircraft position and velocity (Assuming initial pos is 0, 0, 100)
        initial_pos = LPoint3(0, 0, 100)
        initial_hpr = LVector3(0, 0, 0)
        self.physics_mgr.reset_aircraft(initial_pos, initial_hpr)
        
        # Reset game variables
        self.score = 0
        self.current_checkpoint_index = 0
        self.set_next_checkpoint()
        
        # Restart the game loop
        self.base.taskMgr.add(self.game_loop, "GameLoopTask")

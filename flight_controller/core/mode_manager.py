from utils.logger import Logger

class ModeManager:
    def __init__(self):
        self.current_mode = None
        self.mode_name = None

    def switch_mode(self, new_mode):
        new_mode_name = new_mode.__class__.__name__
        Logger.info(f"Switching to {new_mode_name}")
        
        # Activate the mode if it has an activate() method (used by GPSHoldMode)
        if hasattr(new_mode, "activate") and callable(new_mode.activate):
            Logger.debug(f"[ModeManager] Activating {new_mode_name} mode...")
            new_mode.activate()
        
        self.current_mode = new_mode
        self.mode_name = new_mode_name

    def update(self, pilot_input, dt):
        if self.current_mode:
            Logger.debug(f"[ModeManager] Updating {self.mode_name} mode...")
            self.current_mode.update(pilot_input, dt)


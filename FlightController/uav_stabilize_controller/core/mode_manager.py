from utils.logger import Logger

class ModeManager:
    def __init__(self):
        self.current_mode = None

    def switch_mode(self, new_mode):
        Logger.info(f"Switching to {new_mode.__class__.__name__}")
        self.current_mode = new_mode

    def update(self, pilot_input, dt):
        if self.current_mode:
            self.current_mode.update(pilot_input, dt)

from core.flight_mode import FlightMode
from utils.logger import Logger


class StabilizeMode:
    def __init__(self, angle_pid_roll, rate_pid_roll, angle_pid_pitch, rate_pid_pitch, rate_pid_yaw, mixer, sensors, esc):
        self.angle_pid_roll = angle_pid_roll
        self.rate_pid_roll = rate_pid_roll
        
        self.angle_pid_pitch = angle_pid_pitch
        self.rate_pid_pitch = rate_pid_pitch

        self.rate_pid_yaw = rate_pid_yaw
        
        self.mixer = mixer
        self.sensors = sensors
        self.esc = esc

    def update(self, pilot_input, dt):
    
        #Roll signal
        desired_roll = pilot_input.get_desired_roll_angle()
        actual_roll = self.sensors.read_roll()
        desired_rate = self.angle_pid_roll.compute(desired_roll, actual_roll, dt)

        actual_rate = self.sensors.read_roll_rate()
        torque_roll_command = self.rate_pid_roll.compute(desired_rate, actual_rate, dt)


        #Pitch signal
        desired_pitch = pilot_input.get_desired_pitch_angle()
        actual_pitch = self.sensors.read_pitch()
        desired_rate = self.angle_pid_pitch.compute(desired_pitch, actual_pitch, dt)

        actual_rate = self.sensors.read_pitch_rate()
        torque_pitch_command = self.rate_pid_pitch.compute(desired_rate, actual_rate, dt)


        #Yaw signal
        desired_rate= pilot_input.get_desired_yaw_rate()
        actual_rate = self.sensors.read_yaw_rate()
        torque_yaw_command = self.rate_pid_yaw.compute(desired_rate, actual_rate, dt)

        
        esc_pwm_outputs = self.mixer.mix(pilot_input.get_throttle_pwm(), torque_pitch_command, torque_roll_command, torque_yaw_command)
        self.esc.send_pwm(self.sensors,esc_pwm_outputs)

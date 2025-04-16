class StabilizeMode:
    def __init__(self, angle_pid, rate_pid, mixer, sensors, esc):
        self.angle_pid = angle_pid
        self.rate_pid = rate_pid
        self.mixer = mixer
        self.sensors = sensors
        self.esc = esc

    def update(self, pilot_input, dt):
    
        #Roll signal
        desired_roll = pilot_input.get_desired_roll_angle()
        actual_roll = self.sensors.read_roll()
        desired_rate = self.angle_pid.compute(desired_roll, actual_roll, dt)

        actual_rate = self.sensors.read_roll_rate()
        torque_roll_command = self.rate_pid.compute(desired_rate, actual_rate, dt)


        #Pitch signal
        desired_pitch = pilot_input.get_desired_pitch_angle()
        actual_pitch = self.sensors.read_pitch()
        desired_rate = self.angle_pid.compute(desired_pitch, actual_pitch, dt)

        actual_rate = self.sensors.read_pitch_rate()
        torque_pitch_command = self.rate_pid.compute(desired_rate, actual_rate, dt)


        #Yaw signal
        desired_rate= pilot_input.get_desired_yaw_rate()
        actual_rate = self.sensors.read_yaw_rate()
        torque_yaw_command = self.rate_pid.compute(desired_rate, actual_rate, dt)

        
        esc_pwm_outputs = self.mixer.mix(pilot_input.get_throttle_pwm(), torque_pitch_command, torque_roll_command, torque_yaw_command)
        self.esc.send_pwm(self.sensors,esc_pwm_outputs)

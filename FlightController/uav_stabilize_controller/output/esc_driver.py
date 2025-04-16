from utils.logger import Logger
class ESCDriver:
    def send_pwm(self, sensors, esc_pwm_outputs):

        #Logger.debug(f"PWM Outputs: {pwm_outputs}")'''
        motor1 = esc_pwm_outputs[0]
        motor2 = esc_pwm_outputs[1]
        motor3 = esc_pwm_outputs[2]          
        motor4 = esc_pwm_outputs[3]

        # Send PWM signals to ESC via MAVLink
        sensors.send_mavlink_pwm(motor1, motor2, motor3, motor4)

         



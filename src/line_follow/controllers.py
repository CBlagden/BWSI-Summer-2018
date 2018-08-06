import time
import math


class PIDController:

    def __init__(self, kP=0, kI=0, kD=0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.integral = 0
        self.past_error = 0
        self.prev_time = 0

    def output(self, current, setpoint):
        pres_time = time.time()
        dt = pres_time - self.prev_time

        error = setpoint - current

        proportional = self.kP * error
        self.integral += self.kI * (error + self.past_error) * dt
        derivative = self.kD * (error - self.past_error) / dt
        output = proportional + self.integral + derivative

        self.past_error = error
        self.prev_time = pres_time
        return output

    def output_error(self, error):
        return self.output(0, error)

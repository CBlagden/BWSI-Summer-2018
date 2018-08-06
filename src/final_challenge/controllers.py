import time
import math
import numpy as np


class PIDController:

    def __init__(self, kP=0.0, kI=0.0, kD=0.0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.integral = 0.0
        self.past_error = 0.0
        self.prev_time = 0.0

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


class PotentialFieldController:

    def __init__(self, steer_gain=1.0, speed_gain=1.0, alpha=0.92, mu=0.06):
        self.steer_gain = steer_gain
        self.speed_gain = speed_gain
        self.alpha = alpha
        self.mu = mu

        # Linspace creates an array of evenly spaced values
        self.angles = np.linspace(math.radians(-135), math.radians(135), num=1081)
        self.sines = np.sin(self.angles)
        self.cosines = np.cos(self.angles)
        self.prev_speed = 0.0

    def output(self, scan_data):
        scd = np.asarray(scan_data)

        # Get inverse squares of distances, and combine them to form a new vector
        x_sum = np.sum(np.multiply(self.cosines, np.power(scd, -2)))
        y_sum = np.sum(np.multiply(self.sines, np.power(scd, -2)))
        x_sum += 3309

        # Calculate the angle using atan
        angle = math.atan(y_sum / x_sum)  * self.steer_gain
        speed = self.speed_gain * math.hypot(x_sum, y_sum) * np.sign(y_sum)

        speed = self.alpha * self.prev_speed + self.mu * speed

        self.prev_speed = speed
        return angle, speed

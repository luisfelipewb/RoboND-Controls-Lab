# Copyright (C) 2017 Udacity Inc.
# All Rights Reserved.

# Author: Brandon Kinman


class PIDController:
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, max_windup = 10):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.max_windup = float(max_windup)

        self.measured_value = 0.0
        self.last_timestamp = 0.0
        self.last_error = 0.0
        self.error_sum = 0.0
        self.target = 0.0

    def reset(self):
        self.measured_value = 0
        self.last_timestamp = 0.0
        self.last_error = 0.0
        self.error_sum = 0.0

    def setTarget(self, target):
        self.target = float(target)

    def setKP(self, kp):
        self.kp = float(kp)

    def setKI(self, ki):
        self.ki = float(ki)

    def setKD(self, kd):
        self.kd = float(kd)

    def setMaxWindup(self, max_windup):
        self.max_windup = max_windup

    def update(self, measured_value, timestamp):

        delta_time = self.last_timestamp - timestamp
        if delta_time == 0:
            pass

        error = self.target - measured_value

        self.last_timestamp = timestamp

        self.error_sum += error * delta_time

        if self.error_sum > self.max_windup:
            self.error_sum = self.max_windup
        if self.error_sum <  -self.max_windup:
            self.error_sum = -self.max_windup

        delta_error = error - self.last_error

        self.last_error = error

        p = self.kp * error
        i = self.ki * self.error_sum
        d = self.kd * (delta_error / delta_time)

        u = p + i + d

        return u

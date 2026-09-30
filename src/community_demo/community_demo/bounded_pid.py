"""Small bounded PID adapted from PolarBear's RoboMaster controller.

Original: SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller,
src/pid.cpp (Lihan Chen, 2025), Apache-2.0. This Python version only keeps the
bounded error controller required by a slow differential patrol robot.
"""


class BoundedPID:
    def __init__(self, kp, ki=0.0, kd=0.0, dt=0.1, lower=-1.0, upper=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.lower, self.upper = lower, upper
        self.integral = 0.0
        self.previous = 0.0

    def calculate(self, error):
        self.integral = max(-1.0, min(1.0, self.integral + error * self.dt))
        derivative = (error - self.previous) / self.dt
        self.previous = error
        command = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(self.lower, min(self.upper, command))

import numpy as np
import time
from collections import deque
from raven import Raven

class WheelOdometry:
    def __init__(
        self,
        maslab,
        wheel_diameter: float,
        track_width: float,
        cpr: int,
        left_encoder_init: int = 0,
        right_encoder_init: int = 0,
        buffer_size: int = 200,
    ):
        self.reset()

        # Robot properties
        self.__WHEEL_DIAMETER = wheel_diameter
        self.__TRACK_WIDTH = track_width
        self.__WHEEL_MOTOR_MPC = self.__WHEEL_DIAMETER * np.pi / cpr
        self.__left_encoder = left_encoder_init
        self.__right_encoder = right_encoder_init
        self.maslab = maslab

        # Pose history buffer: (timestamp, x, y, theta)
        self.pose_buffer = deque(maxlen=buffer_size)

        # Store initial pose
        self._push_pose()

    def reset(self):
        self.__x = 0.0
        self.__y = 0.0
        self.__theta = 0.0

    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y

    @property
    def theta(self):
        return self.__theta

    def __repr__(self):
        return f"x: {self.x}\ny: {self.y}\nheading: {self.theta * 180/np.pi} degree"

    def _push_pose(self):
        """
        Save current pose with timestamp
        """
        t = time.time()
        self.pose_buffer.append((t, self.__x, self.__y, self.__theta))

    def get_pose_at_time(self, t_query: float) -> tuple:
        """
        Returns interpolated (x, y, theta) at time t_query.
        If outside buffer, returns closest pose.

        :param t_query: target time
        """
        if not self.pose_buffer:
            return self.__x, self.__y, self.__theta

        # If too early or too late, clamp
        if t_query <= self.pose_buffer[0][0]:
            _, x, y, th = self.pose_buffer[0]
            return x, y, th
        if t_query >= self.pose_buffer[-1][0]:
            _, x, y, th = self.pose_buffer[-1]
            return x, y, th

        # Find surrounding poses
        for i in range(len(self.pose_buffer) - 1):
            t0, x0, y0, th0 = self.pose_buffer[i]
            t1, x1, y1, th1 = self.pose_buffer[i + 1]

            if t0 <= t_query <= t1:
                alpha = (t_query - t0) / (t1 - t0)

                x = x0 + alpha * (x1 - x0)
                y = y0 + alpha * (y1 - y0)

                # angle interpolation (wrap-safe)
                dth = (th1 - th0 + np.pi) % (2*np.pi) - np.pi
                th = th0 + alpha * dth

                return x, y, th

        # fallback (should not happen)
        _, x, y, th = self.pose_buffer[-1]
        return x, y, th

    def update(self):
        """
        Updates odometry information with latest motor encoder data.
        """
        left_encoder = self.maslab.raven.get_motor_encoder(Raven.MotorChannel.CH1)
        right_encoder = self.maslab.raven.get_motor_encoder(Raven.MotorChannel.CH2)

        d_left_enc = left_encoder - self.__left_encoder
        d_right_enc = right_encoder - self.__right_encoder

        self.__left_encoder = left_encoder
        self.__right_encoder = right_encoder

        d_left = d_left_enc * self.__WHEEL_MOTOR_MPC
        d_right = -d_right_enc * self.__WHEEL_MOTOR_MPC

        d_theta = (d_right - d_left) / self.__TRACK_WIDTH
        d_center = (d_left + d_right) / 2.0
        theta_mid = self.__theta + d_theta / 2.0

        self.__x += d_center * np.sin(theta_mid)
        self.__y += d_center * np.cos(theta_mid)
        self.__theta = (self.__theta + d_theta + np.pi) % (2*np.pi) - np.pi

        # store pose
        self._push_pose()

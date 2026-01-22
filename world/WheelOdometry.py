import numpy as np
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
    ):
        self.reset()
        # Robot properties
        self.__WHEEL_DIAMETER = wheel_diameter
        self.__TRACK_WIDTH = track_width
        self.__CPR = cpr
        self.__WHEEL_MOTOR_MPC = self.__WHEEL_DIAMETER * np.pi / cpr  # Distance per count
        self.__left_encoder = left_encoder_init
        self.__right_encoder = right_encoder_init
        self.maslab = maslab

    def reset(self):
        self.__x = 0.0
        self.__y = 0.0
        self.__theta = 0.0

    @property
    def x(self) -> float:
        return self.__x

    @property
    def y(self) -> float:
        return self.__y

    @property
    def theta(self) -> float:
        return self.__theta

    def __repr__(self):
        return f"x: {self.x}\ny: {self.y}\nheading: {self.theta * 180/np.pi} degree"

    def get_theta_encoders(self, theta):
        meters_per_count = self.__WHEEL_DIAMETER * np.pi / self.__CPR

        # Distance each wheel must travel
        d_left = -theta * self.__TRACK_WIDTH / 2
        d_right = theta * self.__TRACK_WIDTH / 2

        # Convert to encoder counts
        d_left_counts = int(round(d_left / meters_per_count))
        d_right_counts = int(round(d_right / meters_per_count))

        # Target encoder values
        target_left = d_left_counts
        target_right = d_right_counts

        return target_right, target_left


    def get_xy_encoders(self, target_x, target_y):

        right_encoder = self.maslab.raven.get_motor_encoder(Raven.MotorChannel.CH1)
        left_encoder = self.maslab.raven.get_motor_encoder(Raven.MotorChannel.CH2)

        meters_per_count = self.__WHEEL_DIAMETER * np.pi / self.__CPR

        # --- Step 1: vector to target ---
        dx = target_x - self.x
        dy = target_y - self.y
        distance = np.hypot(dx, dy)

        # --- Step 2: required heading ---
        target_theta = np.arctan2(dy, dx)
        d_theta = (target_theta - self.__theta + np.pi) % (2*np.pi) - np.pi

        # --- Step 3: encoder deltas for turn ---
        d_left_turn = -d_theta * self.__TRACK_WIDTH / 2
        d_right_turn = d_theta * self.__TRACK_WIDTH / 2

        # --- Step 4: encoder deltas for straight drive ---
        d_left_drive = distance
        d_right_drive = distance

        # --- Step 5: total wheel distances ---
        d_left_total = d_left_turn + d_left_drive
        d_right_total = d_right_turn + d_right_drive

        # --- Step 6: convert to counts ---
        left_counts = int(round(d_left_total / meters_per_count))
        right_counts = int(round(d_right_total / meters_per_count))

        # --- Step 7: absolute encoder targets ---
        target_left = left_encoder + left_counts
        target_right = right_encoder + right_counts

        return target_left, target_right


    def update(self):
        left_encoder = self.maslab.raven.get_motor_encoder(Raven.MotorChannel.CH1)
        right_encoder = self.maslab.raven.get_motor_encoder(Raven.MotorChannel.CH2)
        # Get encoder change
        d_left_encoder = left_encoder - self.__left_encoder
        d_right_encoder = right_encoder - self.__right_encoder

        # Update encoder values
        self.__left_encoder = left_encoder
        self.__right_encoder = right_encoder

        # Get distance change
        d_left_distance = d_left_encoder * self.__WHEEL_MOTOR_MPC
        d_right_distance = -d_right_encoder * self.__WHEEL_MOTOR_MPC

        ###### TODO: Calculate changes ###### 
        d_theta = (d_right_distance - d_left_distance) / self.__TRACK_WIDTH
        d_center = (d_left_distance + d_right_distance) / 2.0
        theta_mid = self.__theta + d_theta / 2.0
        d_x = d_center * np.sin(theta_mid)
        d_y = d_center * np.cos(theta_mid)
        ###### End of TODO ######

        # Update reading
        self.__theta = (self.__theta + d_theta + np.pi) % (
            2 * np.pi
        ) - np.pi  # Wrapping to 2*pi
        self.__x += d_x
        self.__y += d_y
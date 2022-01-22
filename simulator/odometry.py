class Odometry:
    """Create an odometry object."""

    def __init__(self, speed, angular_velocity):
        self.speed = speed
        self.angular_velocity = angular_velocity

    def __str__(self):
        return f"{self.speed} {self.angular_velocity}"

    def get_speed(self):
        return self.speed

    def get_angular_velocity(self):
        return self.angular_velocity

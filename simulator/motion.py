class Motion:
    """Create an motion object."""

    def __init__(self, speed: float, angular_velocity: float):
        self.speed = speed
        self.angular_velocity = angular_velocity

    def __str__(self) -> str:
        return f"{self.speed} {self.angular_velocity}"

    def get_speed(self) -> float:
        return self.speed

    def get_angular_velocity(self) -> float:
        return self.angular_velocity

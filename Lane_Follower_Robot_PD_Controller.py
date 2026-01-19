#PD Controller with error smoothing for lane-following robot

class PDController:
    
    def __init__(self, Kp: float, Kd: float, alpha: float):
        self.Kp = Kp
        self.Kd = Kd
        self.alpha = alpha
        self.prev_error = 0.0
    
    # Compute PD control output for the given error and timestep.

    def update(self, error: float, dt: float) -> float:

        # Smooth the error
        smoothed_error = self.alpha * error + (1 - self.alpha) * self.prev_error

        # Derivative term
        derivative = (smoothed_error - self.prev_error) / dt

        # PD output
        omega = -(self.Kp * smoothed_error + self.Kd * derivative)

        # Update previous error
        self.prev_error = smoothed_error

        return omega

    # Reset the controller state

    def reset(self):
        self.prev_error = 0.0
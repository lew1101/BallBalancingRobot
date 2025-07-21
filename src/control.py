from .constants import ALPHA



class PIDController():
    def __init__(self, kp, ki, kd, setpoint=0, alpha=ALPHA):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.alpha = alpha
        
        self.prevInput = None
        self.prevError = 0
        self.integral = 0
        self.setpoint = 0
        
    def __call__(self, rawInput: float, dt: float) -> float:
        if self.prevInput is None:
            filteredInput = rawInput
        else: # apply low-pass filter (EMA)
            filteredInput = self.alpha * rawInput + (1 - self.alpha) * self.prevInput
        self.prevInput = filteredInput
        
        error = self.setpoint - rawInput
        self.integral += error * dt
        derivative = (error - self.prevError) / dt if dt > 0 else 0
        
        output = (self.kp * error + self.ki * self.integral + self.kd * derivative)
        
        self.prevError = error
        
        return output, error
    
    def updateSetpoint(self, setpoint: float) -> None:
        self.setpoint = setpoint
        self.prev_input = None  # reset previous input for filtering
        self.integral = 0  # reset integral when setpoint changes
        self.prevError = 0  # reset previous error
        
        
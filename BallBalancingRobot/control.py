class PIDController:
    
    def __init__(self, kp: float, ki: float, kd: float, setpoint=0.0, alpha=1.0, maxIntegral=float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.alpha = alpha
        
        self.prevInput = 0.0
        self.prevError = 0.0
        self.integral = 0.0
        self.setpoint = setpoint
        self.maxIntegral = maxIntegral
        
    def __call__(self, rawInput: float, dt: float) -> tuple[float, float]:
        if self.prevInput is None:
            filteredInput = rawInput
        else: # apply low-pass filter (EMA)
            filteredInput = self.alpha * rawInput + (1 - self.alpha) * self.prevInput
        self.prevInput = filteredInput
        
        error = self.setpoint - rawInput # P
        
        self.integral += error * dt
        # clamp integral to prevent windup
        self.integral = max(min(self.integral, self.maxIntegral), -self.maxIntegral) # I
        
        derivative = (error - self.prevError) / dt if dt > 0 else 0 # D
        
        output = (self.kp * error + self.ki * self.integral + self.kd * derivative)
        
        self.prevError = error
        
        return output, error
    
    def updateSetpoint(self, setpoint: float) -> None:
        if setpoint == self.setpoint: # prevent jittering near setpoint
            return
        
        self.setpoint = setpoint
        self.prev_input = None  # reset previous input for filtering
        self.integral = 0  # reset integral when setpoint changes
        self.prevError = 0  # reset previous error
        
        
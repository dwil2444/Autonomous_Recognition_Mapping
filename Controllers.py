

class PIDController:
    def __init__(self, Kp, Ki, Kd, dt):
        self.dt = dt
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error=0
        self.integral=0
        self.output=0
    
    def getCtrlEffort(self, setpoint, measurement):
        error = setpoint - measurement
        self.integral = self.integral + error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.output = self.output + self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return self.output

    def resetValues(self):
        self.prev_error = 0
        self.integral = 0
        self.output = 0

class BangBangController:
	def __init__(self,umargin=0,lmargin=0):
		self.umargin=umargin
		self.lmargin=lmargin
		self.prevEffort=0

	def getCtrlEffort(self, setpoint, measurement):
		if self.prevEffort==1:
			if measurement < setpoint-self.lmargin:
				self.prevEffort= -1
		elif self.prevEffort==-1:
			if measurement >setpoint+self.umargin:
				self.prevEffort=1
		elif self.prevEffort==0:
			if measurement < setpoint-self.lmargin:
				self.prevEffort= -1
			elif measurement > setpoint+self.umargin:
				self.prevEffort= 1
		return self.prevEffort


import numpy as np 
from simple_pid import PID

class PidController(object):
    def __init__(self, Kpc, Kdc, Kic, Kpp, Kdp, Kip):
        self.pid_cart = PID(Kpc,Kdc,Kic, setpoint=0) # cart is need to fix at 0 around in  x-axis
        self.pid_pole = PID(Kpp,Kdp,Kip, setpoint=0) # pole is need to fix at 0 radian in  y-axis
    
    def compute(self, x, y):
        u = self.pid_cart(x)+self.pid_pole(y)
        return u


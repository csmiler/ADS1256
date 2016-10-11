# python module for motor control
from dSPIN_CONST import *
from Motor_dSPIN import Motor_dSPIN

class Motor(Motor_dSPIN):

    def __init__(self, _is_cw=True, _speed=100):
        Motor_dSPIN.__init__()
        self.is_cw = _direction
        self.speed = _speed
    
    def motor_init(self):
        '''config the motor'''
        # First, let's set the step mode register:
        #   - dSPIN_SYNC_EN controls whether the BUSY/SYNC pin reflects the step
        #     frequency or the BUSY status of the chip. We want it to be the BUSY
        #     status.
        #   - dSPIN_STEP_SEL_x is the microstepping rate- we'll go full step.
        #   - dSPIN_SYNC_SEL_x is the ratio of (micro)steps to toggles on the
        #     BUSY/SYNC pin (when that pin is used for SYNC). Make it 1:1, despite
        #     not using that pin.
        self.dSPIN_SetParam(dSPIN_STEP_MODE, !dSPIN_SYNC_EN | dSPIN_STEP_SEL_1_128 | dSPIN_SYNC_SEL_1);
        # Configure the MAX_SPEED register- this is the maximum number of (micro)steps per
        #  second allowed. You'll want to mess around with your desired application to see
        #  how far you can push it before the motor starts to slip. The ACTUAL parameter
        #  passed to this function is in steps/tick; MaxSpdCalc() will convert a number of
        #  steps/s into an appropriate value for this function. Note that for any move or
        #  goto type function where no speed is specified, this value will be used.
        self.dSPIN_SetParam(dSPIN_MAX_SPEED, self.MaxSpdCalc(400));
        # Configure the FS_SPD register- this is the speed at which the driver ceases
        #  microstepping and goes to full stepping. FSCalc() converts a value in steps/s
        #  to a value suitable for this register; to disable full-step switching, you
        #  can pass 0x3FF to this register.
        self.dSPIN_SetParam(dSPIN_FS_SPD, self.FSCalc(400));
        # Configure the acceleration rate, in steps/tick/tick. There is also a DEC register;
        #  both of them have a function (AccCalc() and DecCalc() respectively) that convert
        #  from steps/s/s into the appropriate value for the register. Writing ACC to 0xfff
        #  sets the acceleration and deceleration to 'infinite' (or as near as the driver can
        #  manage). If ACC is set to 0xfff, DEC is ignored. To get infinite deceleration
        #  without infinite acceleration, only hard stop will work.
        self.dSPIN_SetParam(dSPIN_ACC, 0xfff);
        # Configure the overcurrent detection threshold. The constants for this are defined
        #  in the dSPIN.h file.
        # 3000mA is somewhere a bit above the rated capacity w/o heatsinking.
        self.dSPIN_SetParam(dSPIN_OCD_TH, dSPIN_OCD_TH_3000mA);
        # Set up the CONFIG register as follows:
        #  PWM frequency divisor = 1
        #  PWM frequency multiplier = 2 (62.5kHz PWM frequency)
        #  Slew rate is 530V/us
        #  Do NOT shut down bridges on overcurrent
        #  Disable motor voltage compensation
        #  Hard stop on switch low
        #  16MHz internal oscillator, nothing on output
        self.dSPIN_SetParam(dSPIN_CONFIG, 
                       dSPIN_CONFIG_PWM_DIV_1 | dSPIN_CONFIG_PWM_MUL_2 | dSPIN_CONFIG_SR_290V_us
                     | dSPIN_CONFIG_OC_SD_DISABLE | dSPIN_CONFIG_VS_COMP_ENABLE 
                     | dSPIN_CONFIG_SW_HARD_STOP | dSPIN_CONFIG_INT_16MHZ);
        # Configure the RUN KVAL. This defines the duty cycle of the PWM of the bridges
        #  during running. 0xFF means that they are essentially NOT PWMed during run; this
        #  MAY result in more power being dissipated than you actually need for the task.
        #  Setting this value too low may result in failure to turn.
        #  There are ACC, DEC, and HOLD KVAL registers as well; you may need to play with
        #  those values to get acceptable performance for a given application.
        self.dSPIN_SetParam(dSPIN_KVAL_RUN, 0x10);
        # Calling GetStatus() clears the UVLO bit in the status register, which is set by
        #  default on power-up. The driver may not run without that bit cleared by this
        #  read operation.
        
    def run(self):
        '''run the motor'''
        if (_is_cw):
            self.dSPIN_Run(FWD, SpdCalc(_speed))
        else:
            self.dSPIN_Run(REV, SpdCalc(_speed))
        
    def stop(self, hard_stop=True):
        '''stop the motor'''
        if (hard_stop):
            self.dSPIN_HardStop()
        else:
            self.dSPIN_SoftStop()
            
    def get_pos(self):
        '''get the current position of the motor'''
        pos = self.dSPIN_GetParam(dSPIN_EL_POS);
        # Since ABS_POS is a 22-bit 2's comp value, we need to check bit 21 and, if
        #  it's set, set all the bits ABOVE 21 in order for the value to maintain
        #  its appropriate sign.
        if (pos & 0x00200000):
            pos |= 0xffC00000;
        return pos
    
    def reset_pos(self):
        '''reset position of the motor'''
        self.dSPIN_ResetPos()
        
    def move(self, steps):
        '''move the motor by steps'''
        steps = int(steps)
        if (steps > 0):
            self.dSPIN_Move(FWD, steps)
        else:
            self.dSPIN_Move(REV, -steps)
            
    def get_direction(self):
        '''get the current direction of the motor'''
        if (self.is_cw):
            return FWD
        return REV
    
    def get_speed(self):
        '''get the speed of the motor'''
        return self.speed
    
    def set_direction(self, _str):
        '''set the direction of the motor'''
        if (_str == "CW"):
            self.is_cw = True
        elif (_str == "CCW"):
            self.is_cw = False
            
        self.run()
        
    def set_speed(self, _speed):
        '''set the speed of the motor'''
        max_speed = 400
        if (isNumber(_speed)):
            _speed = abs(int(_speed))
            if (_speed <= max_speed):
                self.speed = _speed
                self.run()
                return            
        
        print "WRONG INPUT SPEED!"
            
            
    
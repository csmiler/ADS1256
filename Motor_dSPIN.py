#!/usr/bin/env python

# python module for STMicro L6470 dSPIN stepper motor
from libbcm2835._bcm2835 import *
from dSPIN_CONST import *
import numpy as np


class Motor_dSPIN:

    def __init__(self, _dSPIN_BUSYN=24, _dSPIN_RESET=16, _dSPIN_CS=25):
        '''set pins and initiate'''
        # dSPIN_RESET = 16 #RPI_V2_GPIO_P1_36    # Wire this to the STBY line
        # dSPIN_BUSYN = 24 #RPI_V2_GPIO_P1_18   # Wire this to the BSYN line
        # dSPIN_CS = 25 #RPI_V2_GPIO_P1_22	# Wire this to the CSN line
        self.dSPIN_BUSYN = _dSPIN_BUSYN
        self.dSPIN_RESET = _dSPIN_RESET
        self.dSPIN_CS = _dSPIN_CS

        self.dSPIN_init()
        
    
    # This is the generic initialization function to set up the Arduino to
    #  communicate with the dSPIN chip. 
    def dSPIN_init(self):
        # initialize SPI for the dSPIN chip's needs:
        #  most significant bit first,
        #  SPI clock not to exceed 5MHz,
        # SPI_MODE3 (clock idle high, latch data on rising edge of clock) 
        if (not bcm2835_init()):
            return 1;
        bcm2835_spi_begin();
        bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      # Edited follow Arduino command
        bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);                   # Edited follow Arduino Command
        bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_4096);    # Edited follow Arduino Command
        #int err = 0;
        #err = wiringPiSetupGpio();
        # set up the input/output pins for the application.
        bcm2835_gpio_fsel(self.dSPIN_BUSYN, BCM2835_GPIO_FSEL_INPT);
        bcm2835_gpio_fsel(self.dSPIN_RESET, BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_fsel(self.dSPIN_CS, BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_write(self.dSPIN_CS, HIGH);
#        /*pinMode(dSPIN_MOSI, OUTPUT);
#        pinMode(dSPIN_MISO, INPUT);
#        pinMode(dSPIN_CLK, OUTPUT);
#
#        if( err !=0){
#            fprintf(stderr, "wiringPi Setup failed with Error %x\n", err);
#            return dSPIN_STATUS_FATAL;
#        }*/

        from time import sleep
        bcm2835_gpio_write(self.dSPIN_RESET, HIGH);
        sleep(0.001);
        bcm2835_gpio_write(self.dSPIN_RESET, LOW);
        sleep(0.001);
        bcm2835_gpio_write(self.dSPIN_RESET, HIGH);
        sleep(0.001);

        #digitalWrite(dSPIN_CLK, HIGH);

        # reset the dSPIN chip. This could also be accomplished by
        #  calling the "dSPIN_ResetDev()" function after SPI is initialized.


        return 0;


    def dSPIN_ParamHandler(self, param, value):
        '''
        Much of the functionality between "get parameter" and "set parameter" is
        very similar, so we deal with that by putting all of it in one function
        here to save memory space and simplify the program.
        '''
        ret_val = np.uint32(0);   # This is a temp for the value to return.
        # This switch structure handles the appropriate action for each register.
        #  This is necessary since not all registers are of the same length, either
        #  bit-wise or byte-wise, so we want to make sure we mask out any spurious
        #  bits and do the right number of transfers. That is handled by the dSPIN_Param()
        #  function, in most cases, but for 1-byte or smaller transfers, we call
        #  dSPIN_Xfer() directly.
      
        # ABS_POS is the current absolute offset from home. It is a 22 bit number expressed
        #  in two's complement. At power up, this value is 0. It cannot be written when
        #  the motor is running, but at any other time, it can be updated to change the
        #  interpreted position of the motor.
        if (param == dSPIN_ABS_POS):
            ret_val = self.dSPIN_Param(value, 22);
        
        # EL_POS is the current electrical position in the step generation cycle. It can
        #  be set when the motor is not in motion. Value is 0 on power up.
        elif (param == dSPIN_EL_POS):
            ret_val = self.dSPIN_Param(value, 9);

        # MARK is a second position other than 0 that the motor can be told to go to. As
        #  with ABS_POS, it is 22-bit two's complement. Value is 0 on power up.
        elif (param == dSPIN_MARK):
            ret_val = self.dSPIN_Param(value, 22);
            
        # SPEED contains information about the current speed. It is read-only. It does 
        #  NOT provide direction information.
        elif (param == dSPIN_SPEED):
            ret_val = self.dSPIN_Param(0, 20);
            
        # ACC and DEC set the acceleration and deceleration rates. Set ACC to 0xFFF 
        #  to get infinite acceleration/decelaeration- there is no way to get infinite
        #  deceleration w/o infinite acceleration (except the HARD STOP command).
        #  Cannot be written while motor is running. Both default to 0x08A on power up.
        # AccCalc() and DecCalc() functions exist to convert steps/s/s values into
        #  12-bit values for these two registers.
        elif (param == dSPIN_ACC): 
            ret_val = self.dSPIN_Param(value, 12);
            
        elif (param == dSPIN_DEC): 
            ret_val = self.dSPIN_Param(value, 12);
            
        # MAX_SPEED is just what it says- any command which attempts to set the speed
        #  of the motor above this value will simply cause the motor to turn at this
        #  speed. Value is 0x041 on power up.
        # MaxSpdCalc() function exists to convert steps/s value into a 10-bit value
        #  for this register.
        elif (param == dSPIN_MAX_SPEED):
            ret_val = self.dSPIN_Param(value, 10);
            
        # MIN_SPEED controls two things- the activation of the low-speed optimization
        #  feature and the lowest speed the motor will be allowed to operate at. LSPD_OPT
        #  is the 13th bit, and when it is set, the minimum allowed speed is automatically
        #  set to zero. This value is 0 on startup.
        # MinSpdCalc() function exists to convert steps/s value into a 12-bit value for this
        #  register. SetLSPDOpt() function exists to enable/disable the optimization feature.
        elif (param == dSPIN_MIN_SPEED): 
            ret_val = self.dSPIN_Param(value, 12);
            
        # FS_SPD register contains a threshold value above which microstepping is disabled
        #  and the dSPIN operates in full-step mode. Defaults to 0x027 on power up.
        # FSCalc() function exists to convert steps/s value into 10-bit integer for this
        #  register.
        elif (param == dSPIN_FS_SPD):
            ret_val = self.dSPIN_Param(value, 10);
            
        # KVAL is the maximum voltage of the PWM outputs. These 8-bit values are ratiometric
        #  representations: 255 for full output voltage, 128 for half, etc. Default is 0x29.
        # The implications of different KVAL settings is too complex to dig into here, but
        #  it will usually work to max the value for RUN, ACC, and DEC. Maxing the value for
        #  HOLD may result in excessive power dissipation when the motor is not running.
        elif (param == dSPIN_KVAL_HOLD):
            ret_val = self.dSPIN_Xfer(np.uint8(value));
            
        elif (param == dSPIN_KVAL_RUN):
            ret_val = self.dSPIN_Xfer(np.uint8(value));
            
        elif (param == dSPIN_KVAL_ACC):
            ret_val = self.dSPIN_Xfer(np.uint8(value));
            
        elif (param == dSPIN_KVAL_DEC):
            ret_val = self.dSPIN_Xfer(np.uint8(value));
            
        # INT_SPD, ST_SLP, FN_SLP_ACC and FN_SLP_DEC are all related to the back EMF
        #  compensation functionality. Please see the datasheet for details of this
        #  function- it is too complex to discuss here. Default values seem to work
        #  well enough.
        elif (param == dSPIN_INT_SPD):
            ret_val = self.dSPIN_Param(value, 14);
            
        elif (param == dSPIN_ST_SLP): 
            ret_val = self.dSPIN_Xfer(np.uint8(value));
            
        elif (param == dSPIN_FN_SLP_ACC):
            ret_val = self.dSPIN_Xfer(np.uint8(value));
            
        elif (param == dSPIN_FN_SLP_DEC): 
            ret_val = self.dSPIN_Xfer(np.uint8(value));
            
        # K_THERM is motor winding thermal drift compensation. Please see the datasheet
        #  for full details on operation- the default value should be okay for most users.
        elif (param == dSPIN_K_THERM): 
            ret_val = self.dSPIN_Xfer(np.uint8(value) & 0x0F);
            
        # ADC_OUT is a read-only register containing the result of the ADC measurements.
        #  This is less useful than it sounds; see the datasheet for more information.
        elif (param == dSPIN_ADC_OUT):
            ret_val = self.dSPIN_Xfer(0);
            
        # Set the overcurrent threshold. Ranges from 375mA to 6A in steps of 375mA.
        #  A set of defined constants is provided for the user's convenience. Default
        #  value is 3.375A- 0x08. This is a 4-bit value.
        elif (param == dSPIN_OCD_TH): 
            ret_val = self.dSPIN_Xfer(np.uint8(value) & 0x0F);
            
        # Stall current threshold. Defaults to 0x40, or 2.03A. Value is from 31.25mA to
        #  4A in 31.25mA steps. This is a 7-bit value.
        elif (param == dSPIN_STALL_TH): 
            ret_val = self.dSPIN_Xfer(np.uint8(value) & 0x7F);
            
        # STEP_MODE controls the microstepping settings, as well as the generation of an
        #  output signal from the dSPIN. Bits 2:0 control the number of microsteps per
        #  step the part will generate. Bit 7 controls whether the BUSY/SYNC pin outputs
        #  a BUSY signal or a step synchronization signal. Bits 6:4 control the frequency
        #  of the output signal relative to the full-step frequency; see datasheet for
        #  that relationship as it is too complex to reproduce here.
        # Most likely, only the microsteps per step value will be needed; there is a set
        #  of constants provided for ease of use of these values.
        elif (param == dSPIN_STEP_MODE):
            ret_val = self.dSPIN_Xfer(np.uint8(value));
            
        # ALARM_EN controls which alarms will cause the FLAG pin to fall. A set of constants
        #  is provided to make this easy to interpret. By default, ALL alarms will trigger the
        #  FLAG pin.
        elif (param == dSPIN_ALARM_EN): 
            ret_val = self.dSPIN_Xfer(np.uint8(value));
            
        # CONFIG contains some assorted configuration bits and fields. A fairly comprehensive
        #  set of reasonably self-explanatory constants is provided, but users should refer
        #  to the datasheet before modifying the contents of this register to be certain they
        #  understand the implications of their modifications. Value on boot is 0x2E88; this
        #  can be a useful way to verify proper start up and operation of the dSPIN chip.
        elif (param == dSPIN_CONFIG): 
            ret_val = self.dSPIN_Param(value, 16);
            
        # STATUS contains read-only information about the current condition of the chip. A
        #  comprehensive set of constants for masking and testing this register is provided, but
        #  users should refer to the datasheet to ensure that they fully understand each one of
        #  the bits in the register.
        elif (param == dSPIN_STATUS):  # STATUS is a read-only register
            ret_val = self.dSPIN_Param(0, 16);
            
        else:   # default
            ret_val = self.dSPIN_Xfer(np.uint8(value));
        
        return ret_val;


    # Realize the "set parameter" function, to write to the various registers in
    #  the dSPIN chip.
    def dSPIN_SetParam(self, param, value):
        self.dSPIN_Xfer(dSPIN_SET_PARAM | param);
        self.dSPIN_ParamHandler(param, value);

    # Realize the "get parameter" function, to read from the various registers in
    #  the dSPIN chip.
    def dSPIN_GetParam(self, param):
        self.dSPIN_Xfer(dSPIN_GET_PARAM | param);
        return self.dSPIN_ParamHandler(param, 0);

    # Enable or disable the low-speed optimization option. If enabling,
    #  the other 12 bits of the register will be automatically zero.
    #  When disabling, the value will have to be explicitly written by
    #  the user with a SetParam() call. See the datasheet for further
    #  information about low-speed optimization.
    def SetLSPDOpt(self, enable):
        self.dSPIN_Xfer(dSPIN_SET_PARAM | dSPIN_MIN_SPEED);
        if (enable):
            self.dSPIN_Param(0x1000, 13);
        else:
            self.dSPIN_Param(0, 13);
            

    # RUN sets the motor spinning in a direction (defined by the constants
    #  FWD and REV). Maximum speed and minimum speed are defined
    #  by the MAX_SPEED and MIN_SPEED registers; exceeding the FS_SPD value
    #  will switch the device into full-step mode.
    # The SpdCalc() function is provided to convert steps/s values into
    #  appropriate integer values for this function.
    def dSPIN_Run(self, direc, spd):
        self.dSPIN_Xfer(dSPIN_RUN | direc);
        if (spd > 0xFFFFF):
            spd = 0xFFFFF;
        self.dSPIN_Xfer(np.uint8(spd >> 16));
        self.dSPIN_Xfer(np.uint8(spd >> 8));
        self.dSPIN_Xfer(np.uint8(spd));
        

    # STEP_CLOCK puts the device in external step clocking mode. When active,
    #  pin 25, STCK, becomes the step clock for the device, and steps it in
    #  the direction (set by the FWD and REV constants) imposed by the call
    #  of this function. Motion commands (RUN, MOVE, etc) will cause the device
    #  to exit step clocking mode.
    def dSPIN_Step_Clock(self, direc):
        self.dSPIN_Xfer(dSPIN_STEP_CLOCK | direc);
        

    # MOVE will send the motor n_step steps (size based on step mode) in the
    #  direction imposed by dir (FWD or REV constants may be used). The motor
    #  will accelerate according the acceleration and deceleration curves, and
    #  will run at MAX_SPEED. Stepping mode will adhere to FS_SPD value, as well.
    def dSPIN_Move(self, direc, n_step):
        self.dSPIN_Xfer(dSPIN_MOVE | direc);
        if (n_step > 0x3FFFFF):
            n_step = 0x3FFFFF;
        self.dSPIN_Xfer(np.uint8(n_step >> 16));
        self.dSPIN_Xfer(np.uint8(n_step >> 8));
        self.dSPIN_Xfer(np.uint8(n_step));

    # GOTO operates much like MOVE, except it produces absolute motion instead
    #  of relative motion. The motor will be moved to the indicated position
    #  in the shortest possible fashion.
    def dSPIN_GoTo(self, pos):
        self.dSPIN_Xfer(dSPIN_GOTO);
        if (pos > 0x3FFFFF):
            pos = 0x3FFFFF;
        self.dSPIN_Xfer(np.uint8(pos >> 16));
        self.dSPIN_Xfer(np.uint8(pos >> 8));
        self.dSPIN_Xfer(np.uint8(pos));

    # Same as GOTO, but with user constrained rotational direction.
    def dSPIN_GoTo_DIR(self, direc, pos):
        self.dSPIN_Xfer(dSPIN_GOTO_DIR);
        if (pos > 0x3FFFFF):
            pos = 0x3FFFFF;
        self.dSPIN_Xfer(np.uint8(pos >> 16));
        self.dSPIN_Xfer(np.uint8(pos >> 8));
        self.dSPIN_Xfer(np.uint8(pos));
        

    # GoUntil will set the motor running with direction dir (REV or
    #  FWD) until a falling edge is detected on the SW pin. Depending
    #  on bit SW_MODE in CONFIG, either a hard stop or a soft stop is
    #  performed at the falling edge, and depending on the value of
    #  act (either RESET or COPY) the value in the ABS_POS register is
    #  either RESET to 0 or COPY-ed into the MARK register.
    def dSPIN_GoUntil(self, act, direc, spd):
        self.dSPIN_Xfer(dSPIN_GO_UNTIL | act | direc);
        if (spd > 0x3FFFFF):
            spd = 0x3FFFFF;
        self.dSPIN_Xfer(np.uint8(spd >> 16));
        self.dSPIN_Xfer(np.uint8(spd >> 8));
        self.dSPIN_Xfer(np.uint8(spd));
        

    # Similar in nature to GoUntil, ReleaseSW produces motion at the
    #  higher of two speeds: the value in MIN_SPEED or 5 steps/s.
    #  The motor continues to run at this speed until a rising edge
    #  is detected on the switch input, then a hard stop is performed
    #  and the ABS_POS register is either COPY-ed into MARK or RESET to
    #  0, depending on whether RESET or COPY was passed to the function
    #  for act.
    def dSPIN_ReleaseSW(self, act, direc):
        self.dSPIN_Xfer(dSPIN_RELEASE_SW | act | direc);
        

    # GoHome is equivalent to GoTo(0), but requires less time to send.
    #  Note that no direction is provided; motion occurs through shortest
    #  path. If a direction is required, use GoTo_DIR().
    def dSPIN_GoHome(self):
        self.dSPIN_Xfer(dSPIN_GO_HOME);
        

    # GoMark is equivalent to GoTo(MARK), but requires less time to send.
    #  Note that no direction is provided; motion occurs through shortest
    #  path. If a direction is required, use GoTo_DIR().
    def dSPIN_GoMark(self):
        self.dSPIN_Xfer(dSPIN_GO_MARK);
        

    # Sets the ABS_POS register to 0, effectively declaring the current
    #  position to be "HOME".
    def dSPIN_ResetPos(self):
        self.dSPIN_Xfer(dSPIN_RESET_POS);
        

    # Reset device to power up conditions. Equivalent to toggling the STBY
    #  pin or cycling power.
    def dSPIN_ResetDev(self):
        self.dSPIN_Xfer(dSPIN_RESET_DEVICE);
        

    # Bring the motor to a halt using the deceleration curve.
    def dSPIN_SoftStop(self):
        self.dSPIN_Xfer(dSPIN_SOFT_STOP);
        

    # Stop the motor with infinite deceleration.
    def dSPIN_HardStop(self):
        self.dSPIN_Xfer(dSPIN_HARD_STOP);
        

    # Decelerate the motor and put the bridges in Hi-Z state.
    def dSPIN_SoftHiZ(self):
        self.dSPIN_Xfer(dSPIN_SOFT_HIZ);
        

    # Put the bridges in Hi-Z state immediately with no deceleration.
    def dSPIN_HardHiZ(self):
        self.dSPIN_Xfer(dSPIN_HARD_HIZ);
        

    # Fetch and return the 16-bit value in the STATUS register. Resets
    #  any warning flags and exits any error states. Using GetParam()
    #  to read STATUS does not clear these values.
    def dSPIN_GetStatus(self):
        temp = np.int16(0);
        self.dSPIN_Xfer(dSPIN_GET_STATUS);
        temp = self.dSPIN_Xfer(0)<<8;
        temp = temp | self.dSPIN_Xfer(0);
        return temp;
    
    def lesser(self, val, higher_limit):
        '''helper function which returns the lesser value of val and higher_limit'''
        if ( val > higher_limit ):
            return higher_limit
        else:
            return val
    

    # Contains functions used to implement the high-level commands,
    #   as well as utility functions for converting real-world units (eg, steps/s) to
    #   values usable by the dsPIN controller. Also contains the specialized configuration
    #   function for the dsPIN chip and the onboard peripherals needed to use it.



    # This simple function shifts a byte out over SPI and receives a byte over
    #  SPI. Unusually for SPI devices, the dSPIN requires a toggling of the
    #  CS (slaveSelect) pin after each byte sent. That makes this function
    #  a bit more reasonable, because we can include more functionality in it.
    #  This is SPI_MODE3 (clock idle high, latch data on rising edge of clock)
    #  MSB is first.
    def dSPIN_Xfer(self, data):
        
        bcm2835_gpio_write(self.dSPIN_CS, LOW);
#        /*
#        for(int i=0; i<8; i++){
#            digitalWrite(dSPIN_CLK, LOW);
#
#
#            if(data & 0x80){
#                digitalWrite(dSPIN_MOSI, HIGH);
#            }else{
#                digitalWrite(dSPIN_MOSI, LOW);
#            }
#            delayMicroseconds( dSPIN_SPI_CLOCK_DELAY/2 );
#
#            data <<= 1;
#
#            if(digitalRead(dSPIN_MISO))
#                data |= 1;
#
#            digitalWrite(dSPIN_CLK, HIGH);
#
#            delayMicroseconds( dSPIN_SPI_CLOCK_DELAY/2 );
#
#        }
#        */
        # bcm2835_spi_transfer() both shifts a byte out on the MOSI pin AND receives a
        #  byte in on the MISO pin.
        data_out = bcm2835_spi_transfer(data);
        bcm2835_gpio_write(self.dSPIN_CS, HIGH);
        #bcm2835_delayMicroseconds( dSPIN_SPI_CLOCK_DELAY );

        return data_out;
    
        

    # The value in the ACC register is [(steps/s/s)*(tick^2)]/(2^-40) where tick is 
    #  250ns (datasheet value)- 0x08A on boot.
    # Multiply desired steps/s/s by .137438 to get an appropriate value for this register.
    # This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
    def AccCalc(self, stepsPerSecPerSec):
        return self.lesser(np.uint32(stepsPerSecPerSec * 0.137438), 0x00000FFF)
        

    # The calculation for DEC is the same as for ACC. Value is 0x08A on boot.
    # This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
    def DecCalc(self, stepsPerSecPerSec):
        return self.lesser(np.uint32(stepsPerSecPerSec * 0.137438), 0x00000FFF)
        

    # The value in the MAX_SPD register is [(steps/s)*(tick)]/(2^-18) where tick is 
    #  250ns (datasheet value)- 0x041 on boot.
    # Multiply desired steps/s by .065536 to get an appropriate value for this register
    # This is a 10-bit value, so we need to make sure it remains at or below 0x3FF
    def MaxSpdCalc(self, stepsPerSec):
        return self.lesser(np.uint32(stepsPerSec * .065536), 0x000003FF)
        

    # The value in the MIN_SPD register is [(steps/s)*(tick)]/(2^-24) where tick is 
    #  250ns (datasheet value)- 0x000 on boot.
    # Multiply desired steps/s by 4.1943 to get an appropriate value for this register
    # This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
    def MinSpdCalc(self, stepsPerSec):
        return self.lesser(np.uint32(stepsPerSec * 4.1943), 0x00000FFF)
    

    # The value in the FS_SPD register is ([(steps/s)*(tick)]/(2^-18))-0.5 where tick is 
    #  250ns (datasheet value)- 0x027 on boot.
    # Multiply desired steps/s by .065536 and subtract .5 to get an appropriate value for this register
    # This is a 10-bit value, so we need to make sure the value is at or below 0x3FF.
    def FSCalc(self, stepsPerSec):
        return self.lesser(np.uint32((stepsPerSec * .065536)-.5), 0x000003FF)
    

    # The value in the INT_SPD register is [(steps/s)*(tick)]/(2^-24) where tick is 
    #  250ns (datasheet value)- 0x408 on boot.
    # Multiply desired steps/s by 4.1943 to get an appropriate value for this register
    # This is a 14-bit value, so we need to make sure the value is at or below 0x3FFF.
    def IntSpdCalc(self, stepsPerSec):
        return self.lesser(np.uint32(stepsPerSec * 4.1943), 0x00003FFF)
    

    # When issuing RUN command, the 20-bit speed is [(steps/s)*(tick)]/(2^-28) where tick is 
    #  250ns (datasheet value).
    # Multiply desired steps/s by 67.106 to get an appropriate value for this register
    # This is a 20-bit value, so we need to make sure the value is at or below 0xFFFFF.
    def SpdCalc(self, stepsPerSec):
        return self.lesser(np.uint32(stepsPerSec * 67.106), 0x000FFFFF)
    

    # Generalization of the subsections of the register read/write functionality.
    #  We want the end user to just write the value without worrying about length,
    #  so we pass a bit length parameter from the calling function.
    def dSPIN_Param(self, value, bit_len):
        ret_val = np.uint32(0);        # We'll return this to generalize this function
                                      #  for both read and write of registers.
        byte_len = np.uint8(bit_len/8);      # How many BYTES do we have?
        if (bit_len % 8 > 0):
            byte_len = byte_len + 1;  # Make sure not to lose any partial byte values.
        # Let's make sure our value has no spurious bits set, and if the value was too
        #  high, max it out.
        mask = 0xffffffff >> (32-bit_len);
        if (value > mask):
            value = mask;
        # The following three if statements handle the various possible byte length
        #  transfers- it'll be no less than 1 but no more than 3 bytes of data.
        # dSPIN_Xfer() sends a byte out through SPI and returns a byte received
        #  over SPI- when calling it, we typecast a shifted version of the masked
        #  value, then we shift the received value back by the same amount and
        #  store it until return time.
        if (byte_len == 3):
            ret_val |= self.dSPIN_Xfer(np.uint8(value>>16)) << 16;
            #Serial.println(ret_val, HEX);
        if (byte_len >= 2):
            ret_val |= self.dSPIN_Xfer(np.uint8(value>>8)) << 8;
            #Serial.println(ret_val, HEX);
        if (byte_len >= 1):
            ret_val |= self.dSPIN_Xfer(np.uint8(value));
            #Serial.println(ret_val, HEX);
        # Return the received values. Mask off any unnecessary bits, just for
        #  the sake of thoroughness- we don't EXPECT to see anything outside
        #  the bit length range but better to be safe than sorry.
        return (ret_val & mask);


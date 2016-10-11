# python module for scpi commands handling

class Scpi:
    
    def __init__(self, _adc=None, _motor=None, _tft=None, _pc=None):
        '''initialization'''
        # adc class
        self.adc = _adc
        # motor class
        self.motor = _motor
        # connection to the tft display -- via usb
        self.tft_pi = _tft
        # connection to the PC -- via wifi?
        self.pc = _pc
        
        self.curr_cmd = ""  # store the current command received from the PC
        
        ## adc chip initialization
        #self.adc.chipInit()
        #self.adc.ADS1256_CfgADC(ADS1256_GAIN_E['ADS1256_GAIN_1'], 'ADS1256_15SPS')
        #self.adc.ADS1256_StartScan(0, self.adc.Channel)
        #self.adc.ADS1256_Scan()
        
        
    def receive_cmd(self):
        '''receive command from the PC'''
        pass
        
    def decode_cmd(self, _cmd):
        '''decode the command, and take action accordingly'''
        # preprocess the string, and separate words by ':'
        _cmd = _cmd.upper()
        self.curr_cmd = _cmd
        cmds = _cmd.split(':')
        if (len(cmds) == 1):
            print "Empty command input!"
            return False
        else:
            cmds = cmds[1:]     # removing the first '' in the list
        
        # call different functions based on the cmd
        if (cmds[0].find('DISP') >= 0):
            # call display function
            return self._display(cmds)
        
        if (cmds[0].find('READ') >= 0):
            # call measure function
            return self._read(cmds)
        
        if (cmds[0].find('CONF') >= 0):
            # call config function
            return self._config(cmds)
        
        return self._invalid_cmd()
    
    def _atoi(self, _str):
        try:
            num = atoi(_str)
        except Exception:
            self._invalid_cmd()
            return None
        return num
        
    def _invalid_cmd(self):
        print "Invalid command!"
        return False
    
    def _display(self, cmds):
        '''display text or data'''
        if (len(cmds) < 2):
            return self._invalid_cmd()
        
        # :DISP:TEXT
        if (cmds[1].find('TEXT') >= 0):
            if (len(cmds) > 2):
                # :DISP:TEXT:DATA
                idx = cmds[2].find('DATA')
                if (idx >= 0):
                    # :DISP:TEXT:DATA?
                    if (cmds[2].find('?', idx+4) >= 0):
                        # query text message function
                        return True
                    else:
                    # :DISP:TEXT:DATA <a>
                        # *****assuming no ':' included
                        # in the message to be displayed!!!
                        self.display_msg(cmds[2][idx+4:].lstrip(' '))
                    return True
                
                # Other commands??
                return True
            
            return self._invalid_cmd()
        
        # Other commands??
        return self._invalid_cmd()
                    
    def _read(self, cmds):
        '''read the signal from ADC'''
        idx = cmds[0].find('?')
        if (idx < 0):
            return self._invalid_cmd()
        
        # extract the channel number
        substr = cmds[0][idx+1:].strip()
        if (not substr):    # no channel number provided
            return self.read_channel()
        else:               # extract channel number
            ch_num = self._atoi(substr)
            if (ch_num):
                return self.read_channel(ch_num)
        
        return self._invalid_cmd()
            
    def _config(self, cmds):
        '''configure motor settings'''
        if (len(cmds) < 2):
            return self._invalid_cmd()
        
        # :CONF:MOTO
        if (cmds[1].find('MOTO') >= 0):
            if (len(cmds) > 2):
                # :CONF:MOTO:RUN
                if (cmds[2].find('RUN') >= 0):
                    self.motor.run()
                    return True
                # :CONF:MOTO:STOP
                if (cmds[2].find('STOP') >= 0):
                    self.motor.stop()
                    return True
                # :CONF:MOTO:MOVE <n>
                if (cmds[2].find('MOVE') >= 0):
                    substr = cmds[2][cmds[2].find('MOVE')+1:].strip()
                    if (not substr):    # no steps provided
                        return self._invalid_cmd()
                    else:               # extract channel number
                        steps = self._atoi(substr)
                        if (steps):
                            self.motor.move(steps)
                            return True
                        else:
                            return self._invalid_cmd()
                # :CONF:MOTO:SET
                if (cmds[2].find('SET') >= 0):
                    if (len(cmds) > 3):
                        # :CONF:MOTO:SET:DIRE CW
                        if (cmds[3].find('DIRE') >= 0):
                            if (cmds[3].find('CCW') > cmds[3].find('DIRE')):
                                self.motor.set_direction('CCW')
                                return True
                            elif (cmds[3].find('CW') > cmds[3].find('DIRE')):
                                self.motor.set_direction('CW')
                                return True
                            else:
                                return self._invalid_cmd()
                        # :CONF:MOTO:SET:SPEE <n>
                        if (cmds[3].find('SPEE') >= 0):
                            substr = cmds[3][cmds[3].find('SPEE')+1+1:].strip()
                            if (not substr):    # no speed provided
                                return self._invalid_cmd()
                            else:               # extract speed
                                speed = self._atoi(substr)
                                if (speed):
                                    self.motor.set_speed(speed)
                                    return True
                                else:
                                    return self._invalid_cmd()
                    else:
                        return self._invalid_cmd()
                    
            else:
                return self._invalid_cmd()
        
        
        # other commands??
        return self._invalid_cmd()
        
    def display_msg(self, msg):
        '''send the message to the other pi to display it on tft'''
        if (not self.tft_pi):
            # try to establish connection 
            self.tft_pi = 1
        
        # send the message to tft
        print msg
            
        
    def read_channel(self, _ch_num=-1):
        '''read the value of one channel'''
        if (_ch_num > 7):
            return None
        self.adc.ADS1256_Scan()
        if (_ch_num == -1): # reading all channels
            readout = [self.adc.ADS1256_GetAdc(i) for i in range(8)]
        else:
            readout = self.adc.ADS1256_GetAdc(_ch_num)
        return (readout * 100) / 167 / 1000000.0
    
    def __del__(self):
        bcm2835_spi_end()
        bcm2835_close()
        
def main_scpi():
    '''Module testing helper function'''

    print "SCPI TESTING STARTS!"

    try:
        device = Scpi()
        counter = 100
        while counter:
            msg = ":DISP:TEXT:DATA Hello World!"#input()
            msg = ":READ?1"
            print device.decode_cmd(msg)
            counter = counter - 1
            device.adc.bsp_DelayUS(100000)#(100000)
    finally:
        bcm2835_spi_end()
        bcm2835_close()

    print "TESTING ENDS"
    
if __name__ == "__main__": main_scpi()

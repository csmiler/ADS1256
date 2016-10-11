from ADS1256 import *
from string import *

class Scpi:
    
    def __init__(self, ch_num=1):
        if not bcm2835_init():
            print "bcm2835 initialization failed!"
            return None
        self.adc = ADS1256(ch_num)
        self.ch_num = ch_num     # store the current channel of interest
        self.curr_cmd = ""  # store the current command received from the PC
        
        # adc chip initialization
        self.adc.chipInit()
        self.adc.ADS1256_CfgADC(ADS1256_GAIN_E['ADS1256_GAIN_1'], 'ADS1256_15SPS')
        self.adc.ADS1256_StartScan(0, self.adc.Channel)
        self.adc.ADS1256_Scan()
        
        # connection to the tft display -- via usb
        self.tft_pi = None
        
        # connection to the PC -- via wifi?
        self.pc = None
        
    def receive_cmd(self):
        '''receive command from the PC'''
        pass
        
    def decode_cmd(self, _cmd):
        '''decode the command, and take action accordingly'''
        # preprocess the string, and separating words by ':'
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
            return False
        
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
        if (len(cmds) == 1):
            return self._invalid_cmd()
        
        if (cmds[1].find('TEXT') >= 0):
            if (len(cmds) > 2):
                # DATA command
                idx = cmds[2].find('DATA')
                if (idx >= 0):
                    if (cmds[2].find('?', idx+4) >= 0):
                        # query text message function
                        return True
                    else:
                        # *****assuming no ':' included
                        # in the message to be displayed!!!
                        self.display_msg(cmds[2][idx+4:].lstrip(' '))
                    return True
                
                # Other commands??
                return True
            
            return self._invalid_cmd()
        
        # Other commands??
        return False
                    
    def _read(self, cmds):
        '''read the signal from ADC'''
        idx = cmds[0].find('?')
        if (idx < 0):
            return self._invalid_cmd()
        
        # extract the channel number
        substr = cmds[0][idx+1:].strip()
        if (not substr):    # no channel number provided
            return self.read_channel(self.ch_num)
        else:               # extract channel number
            ch_num = self._atoi(substr)
            if (ch_num):
                return self.read_channel(ch_num)
        
    def display_msg(self, msg):
        '''send the message to the other pi to display it on tft'''
        if (not self.tft_pi):
            # try to establish connection 
            self.tft_pi = 1
        
        # send the message to tft
        print msg
            
        
    def read_channel(self, _ch_num=1):
        '''read the value of one channel'''
        if (_ch_num > 7):
            return None
        self.adc.ADS1256_Scan()
        self.ch_num = _ch_num
        readout = self.adc.ADS1256_GetAdc(self.ch_num)
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

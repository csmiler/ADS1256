from ADS1256 import *

class Scpi:
    
    def __init__(self):
        if not bcm2835_init():
            print "bcm2835 initialization failed!"
            return None
        self.adc = ADS1256()
        self.adc.chipInit()
        self.ch_num = 1     # store the current channel of interest
        self.curr_cmd = ""  # store the current command received from the PC
        self.adc.ADS1256_CfgADC(ADS1256_GAIN_E['ADS1256_GAIN_1'], 'ADS1256_15SPS')
        self.adc.ADS1256_StartScan(0, self.adc.Channel)
        self.adc.ADS1256_Scan()
        
    def receive_cmd(self):
        '''receive command from the PC'''
        pass
        
    def decode_cmd(self, _cmd):
        '''decode the command, and take action accordingly'''
        pass
        
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
        
def main2():
    '''Module testing helper function'''

    print "SCPI TESTING STARTS!"

    try:
        device = Scpi()
        counter = 10
        while True:
            print " volt = ", device.read_channel()
            device.adc.bsp_DelayUS(100000)#(100000)
            counter = counter - 1
            if counter == 0:
                break    
    finally:
        bcm2835_spi_end()
        bcm2835_close()

    print "TESTING ENDS"
    
if __name__ == "__main__": main2()

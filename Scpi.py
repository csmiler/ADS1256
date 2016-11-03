#!/usr/bin/env python

from ADS1256 import *
from Motor import *
import serial
import socket

# python module for scpi commands handling

class Scpi:
    
    def __init__(self, _adc=None, _motor=None, _tft=None, _pc=None):
        '''initialization'''
        # connection to the tft display -- via usb
        if (_tft):
            print "initiating TFT..."
            self.tft_pi = self.serial_init(validate=False)
        else:
            self.tft_pi = None
            
        # adc class
        self.display_msg("initiating ADC...")
        self.adc = _adc
        # motor class
        self.display_msg("initiating motor...")
        self.motor = _motor
        
        # connection to the PC -- via wifi?
        if (_pc):
            self.display_msg("initiating PC connection...")
            self.pc = self.tcpip_init()
        else:
            self.pc = None
        
        self.curr_cmd = ""  # store the current command received from the PC

    def serial_init(self, _port="/dev/serial0", _baudrate=115200, _timeout=5.0, waiting=60, validate=True):
        '''initiate tx/rx connection to tft panel'''
        port = serial.Serial(_port, baudrate=_baudrate, timeout=_timeout)

        if ( not validate ):
            return port
        
        while ( waiting ):
            rcv = port.read(1)
            if ( rcv == 'Y' ):
                port.write('C')
                return port
            waiting -= _timeout

        return None

    def tcpip_init(self, _host='', _port=8888):
        '''initiate tcpip connection to pc'''
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((_host, _port))
        s.listen(1)
        conn, addr = s.accept()
        return conn

    def receive_cmd(self):
        '''receive command from the PC'''
        if ( not self.pc ):
            msg = raw_input("Input CMD: ")
        else:
            msg = self.pc.recv(1024)
            msg = msg[:len(msg)-1] # remove trailing \n
        return msg

    def send_result(self, _msg):
        '''send result back to pc or display'''
        if ( not self.pc ):
            print _msg
        else:
            self.pc.sendall(_msg + '\n')
        return None
        
    def decode_cmd(self, _cmd):
        '''decode the command, and take action accordingly'''
        # preprocess the string, and separate words by ':'
        _cmd = _cmd.upper()
        self.curr_cmd = _cmd
        cmds = _cmd.split(':')
        if (len(cmds) == 1):
            return self._invalid_cmd()
        else:
            cmds = cmds[1:]     # removing the first '' in the list

        #print "cmds = ", cmds
        
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
            num = int(_str)
        except Exception:
            self._invalid_cmd()
            return None
        return num
        
    def _invalid_cmd(self, _msg="Invalid command!"):
        self.display_msg(_msg)
        return False, _msg
    
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
                        pass
                    else:
                    # :DISP:TEXT:DATA <a>
                        # *****assuming no ':' included
                        # in the message to be displayed!!!
                        self.display_msg(cmds[2][idx+4:].lstrip(' '))
                        
                    return True, ''
                
                # Other commands??
                return self._invalid_cmd("Invalid TEXT command!")
            
            return self._invalid_cmd("Invalid TEXT command!")
        
        # Other commands??
        return self._invalid_cmd("Invalid DISP command!")
                    
    def _read(self, cmds):
        '''read the signal from ADC'''
        idx = cmds[0].find('?')
        if (idx < 0):
            return self._invalid_cmd()
        
        # extract the channel number
        substr = cmds[0][idx+1:].strip()
        
        if (not substr):    # no channel number provided
            return self._invalid_cmd("No channel number provided!")
        else:               # extract channel number
            ch_num = self._atoi(substr)
            #print "ch_num = ", ch_num
            if (ch_num is not None):
                return True, self.adc.ADS1256_OneShot(ch_num)
        
        return self._invalid_cmd("Wrong channel number!")
            
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
                    return True, ''
                # :CONF:MOTO:STOP
                if (cmds[2].find('STOP') >= 0):
                    self.motor.stop()
                    return True, ''
                # :CONF:MOTO:MOVE <n>
                if (cmds[2].find('MOVE') >= 0):
                    substr = cmds[2][cmds[2].find(' ')+1:].strip()
                    if (not substr):    # no steps provided
                        return self._invalid_cmd("No steps provided")
                    else:               # extract channel number
                        steps = self._atoi(substr)
                        if (steps is not None):
                            self.motor.move(steps)
                            return True, ''
                        else:
                            return self._invalid_cmd("Wrong steps number")
                # :CONF:MOTO:SET
                if (cmds[2].find('SET') >= 0):
                    if (len(cmds) > 3):
                        # :CONF:MOTO:SET:DIRE CW
                        if (cmds[3].find('DIRE') >= 0):
                            if (cmds[3].find('CCW') > cmds[3].find('DIRE')):
                                self.motor.set_direction('CCW')
                                return True, ''
                            elif (cmds[3].find('CW') > cmds[3].find('DIRE')):
                                self.motor.set_direction('CW')
                                return True, ''
                            else:
                                return self._invalid_cmd("Wrong direction (CW or CCW)")
                        # :CONF:MOTO:SET:SPEE <n>
                        if (cmds[3].find('SPEE') >= 0):
                            substr = cmds[3][cmds[3].find(' ')+1:].strip()
                            if (not substr):    # no speed provided
                                return self._invalid_cmd("No speed provided")
                            else:               # extract speed
                                speed = self._atoi(substr)
                                if (speed is not None):
                                    self.motor.set_speed(speed)
                                    return True, ''
                                else:
                                    return self._invalid_cmd("Wrong speed number")
                    else:
                        return self._invalid_cmd("Invalid SET command")
                    
            return self._invalid_cmd("Invalid MOTO command")
        
        
        # other commands??
        return self._invalid_cmd("Invalid CONF command")
        
    def display_msg(self, msg):
        '''send the message to the other pi to display it on tft'''
        if (not self.tft_pi):
            # try to establish connection? 
            # self.tft_pi = self.serial_init(waiting=10)
            print msg
            return None
        
        # send the message to tft
        self.tft_pi.write(msg+'\n')
    
    def __del__(self):
        bcm2835_spi_end()
        bcm2835_close()
        if (self.pc):
            self.pc.close()


        
def main_scpi():
    '''Module testing helper function'''

    print "SCPI TESTING STARTS!"
    from time import sleep
    
    try:
        device = Scpi(_adc=ADS1256(),_motor=Motor(),_tft=True,_pc=True)

        while True:
            msg = device.receive_cmd()
            device.display_msg(msg)
            flag, rdata = device.decode_cmd(msg)
            result = str(int(flag)) + str(rdata)
            print result
            device.send_result(result)

            
        counter = 1
        while counter:
            msg = ":DISP:TEXT:DATA Hello World!"
            
            msg = ":READ?"

            msg = ":CONF:MOTO:RUN"
            
            msg = ":CONF:MOTO:SET:DIRE CCW"

            msg = ":CONF:MOTO:SET:DIRE CW"

            msg = ":CONF:MOTO:SET:SPEE 10"
            
            msg = ":CONF:MOTO:STOP"

            msg = ":CONF:MOTO:MOVE 200"
            
            counter = counter - 1
##            device.adc.bsp_DelayUS(100000)#(100000)
            
    finally:
        #bcm2835_spi_end()
        #bcm2835_close()
        print "TESTING ENDS"
    
if __name__ == "__main__":
    main_scpi()
    

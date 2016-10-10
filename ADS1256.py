# Python module for ADS1256 
# By: Wang Zhou @IBM
# Date: 09/29/2016

from libbcm2835._bcm2835 import *
import numpy as np
import time

# define constants

# gain channel
ADS1256_GAIN_E = {'ADS1256_GAIN_1' : 0, # GAIN   1
                  'ADS1256_GAIN_2' : 1,	# GAIN   2
                  'ADS1256_GAIN_4' : 2,	# GAIN   4
                  'ADS1256_GAIN_8' : 3,	# GAIN   8
                  'ADS1256_GAIN_16' : 4,# GAIN  16
                  'ADS1256_GAIN_32' : 5,# GAIN  32
                  'ADS1256_GAIN_64' : 6,# GAIN  64
                 }

# data rate
ADS1256_DRATE_E = {'ADS1256_30000SPS' : 0xF0, # reset the default values
                   'ADS1256_15000SPS' : 0xE0,
                   'ADS1256_7500SPS' : 0xD0,
                   'ADS1256_3750SPS' : 0xC0,
                   'ADS1256_2000SPS' : 0xB0,
                   'ADS1256_1000SPS' : 0xA1,
                   'ADS1256_500SPS' : 0x92,
                   'ADS1256_100SPS' : 0x82,
                   'ADS1256_60SPS' : 0x72,
                   'ADS1256_50SPS' : 0x63,
                   'ADS1256_30SPS' : 0x53,
                   'ADS1256_25SPS' : 0x43,
                   'ADS1256_15SPS' : 0x33,
                   'ADS1256_10SPS' : 0x20,
                   'ADS1256_5SPS' : 0x13,
                   'ADS1256_2d5SPS' : 0x03
                  }

# registration definition
REG_E = {'REG_STATUS' : 0,  # x1H
         'REG_MUX' : 1,     # 01H
         'REG_ADCON' : 2,   # 20H
         'REG_DRATE' : 3,   # F0H
         'REG_IO' : 4,      # E0H
         'REG_OFC0' : 5,    # xxH
         'REG_OFC1' : 6,    # xxH
         'REG_OFC2' : 7,    # xxH
         'REG_FSC0' : 8,    # xxH
         'REG_FSC1' : 9,    # xxH
         'REG_FSC2' : 10,   # xxH
        }

# command definition
CMD = {'CMD_WAKEUP' : 0x00,     # Completes SYNC and Exits Standby Mode 0000  0000 (00h)
       'CMD_RDATA' : 0x01,      # Read Data 0000  0001 (01h)
       'CMD_RDATAC' : 0x03,     # Read Data Continuously 0000   0011 (03h)
       'CMD_SDATAC' : 0x0F,     # Stop Read Data Continuously 0000   1111 (0Fh)
       'CMD_RREG' : 0x10,       # Read from REG rrr 0001 rrrr (1xh)
       'CMD_WREG' : 0x50,       # Write to REG rrr 0101 rrrr (5xh)
       'CMD_SELFCAL' : 0xF0,    # Offset and Gain Self-Calibration 1111    0000 (F0h)
       'CMD_SELFOCAL' : 0xF1,   # Offset Self-Calibration 1111    0001 (F1h)
       'CMD_SELFGCAL' : 0xF2,   # Gain Self-Calibration 1111    0010 (F2h)
       'CMD_SYSOCAL' : 0xF3,    # System Offset Calibration 1111   0011 (F3h)
       'CMD_SYSGCAL' : 0xF4,    # System Gain Calibration 1111    0100 (F4h)
       'CMD_SYNC' : 0xFC,       # Synchronize the A/D Conversion 1111   1100 (FCh)
       'CMD_STANDBY' : 0xFD,    # Begin Standby Mode 1111   1101 (FDh)
       'CMD_RESET' : 0xFE,      # Reset to Power-Up Values 1111   1110 (FEh)
      }


# ADS1256 Python Module
class ADS1256:
    
    def __init__(self):
        '''initialization and constants registration'''
        
        # data storage
        self.Gain = ADS1256_GAIN_E['ADS1256_GAIN_1']		# GAIN  1
        self.DataRate = ADS1256_DRATE_E['ADS1256_15SPS'] # DATA output speed
        self.AdcNow = [0, 0, 0, 0, 0, 0, 0, 0]			  # ADC  Conversion value
        self.Channel = 1   # The current channel
        self.ScanMode = 0  # Scanning mode_ 0 : Single-ended input  8 channel; 1 : Differential input  4 channel
        
        
        # define constants for future usage
        self.DRDY = RPI_GPIO_P1_11  #P0
        self.RST = RPI_GPIO_P1_12    #P1
        self.SPICS = RPI_GPIO_P1_15	#P3
        
        
        
    def chipInit(self):
        '''configure the chip before data transmission'''
        bcm2835_spi_begin()
        bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_LSBFIRST );   #The default
        bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                 #The default
        bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024);#The default
        bcm2835_gpio_fsel(self.SPICS, BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_write(self.SPICS, HIGH);
        bcm2835_gpio_fsel(self.DRDY, BCM2835_GPIO_FSEL_INPT);
        bcm2835_gpio_set_pud(self.DRDY, BCM2835_GPIO_PUD_UP);
        
    def set_CS_High(self):
        '''set CS to HIGH'''
        bcm2835_gpio_write(self.SPICS,HIGH)
        
    def set_CS_Low(self):
        '''set CS to LOW'''
        bcm2835_gpio_write(self.SPICS,LOW)
        
    def dataReady_is_Low(self):
        '''check if data is ready'''
        return (bcm2835_gpio_lev(self.DRDY) == 0)
    
    def set_RST_High(self):
        '''set RST to HIGH'''
        self.bcm2835_gpio_write(self.RST,HIGH)
    
    def set_RST_Low(self):
        '''set RST to LOW'''
        self.bcm2835_gpio_write(self.RST,LOW)
        
    def bsp_DelayUS(self, micros):
        '''delay for microseconds before procede'''
        bcm2835_delayMicroseconds (micros)
        
    def bsp_InitADS1256(self):
        '''SOFT SPI?'''
        print "SOFT SPI? NOT IMPLEMENTED YET"
        
    def ADS1256_StartScan(self, _ucScanMode, _chnum):
        '''Configuration DRDY PIN for external interrupt is triggered'''
        self.ScanMode =_ucScanMode
        self.Channel = _chnum
        self.AdcNow = [0, 0, 0, 0, 0, 0, 0, 0]  # reset
        print self.AdcNow
        
    def ADS1256_Send8Bit(self, _data):
        '''SPI bus to send 8 bit data'''
        self.bsp_DelayUS(2)
        bcm2835_spi_transfer(_data)
    
    def ADS1256_CfgADC(self, _gain, _drate):
        '''The configuration parameters of ADC, gain and data rate'''
        self.Gain = _gain
        self.DataRate = _drate
        
        self.ADS1256_WaitDRDY()
        
        # Storage ads1256 register configuration parameters
        buf = [0x00, 0x00, 0x00, 0x00]; 

		### Status register define
#			Bits 7-4 ID3, ID2, ID1, ID0  Factory Programmed Identification Bits (Read Only)
#
#			Bit 3 ORDER: Data Output Bit Order
#				0 = Most Significant Bit First (default)
#				1 = Least Significant Bit First
#			Input data  is always shifted in most significant byte and bit first. Output data is always shifted out most significant
#			byte first. The ORDER bit only controls the bit order of the output data within the byte.
#
#			Bit 2 ACAL : Auto-Calibration
#				0 = Auto-Calibration Disabled (default)
#				1 = Auto-Calibration Enabled
#			When Auto-Calibration is enabled, self-calibration begins at the completion of the WREG command that changes
#			the PGA (bits 0-2 of ADCON register), DR (bits 7-0 in the DRATE register) or BUFEN (bit 1 in the STATUS register)
#			values.
#
#			Bit 1 BUFEN: Analog Input Buffer Enable
#				0 = Buffer Disabled (default)
#				1 = Buffer Enabled
#
#			Bit 0 DRDY :  Data Ready (Read Only)
#				This bit duplicates the state of the DRDY pin.
#
#			ACAL=1  enable  calibration
		###
#		//buf[0] = (0 << 3) | (1 << 2) | (1 << 1);//enable the internal buffer
        buf[0] = (0 << 3) | (1 << 2) | (0 << 1); # The internal buffer is prohibited

        # //ADS1256_WriteReg(REG_STATUS, (0 << 3) | (1 << 2) | (1 << 1));

        buf[1] = 0x08	

#		/*	ADCON: A/D Control Register (Address 02h)
#			Bit 7 Reserved, always 0 (Read Only)
#			Bits 6-5 CLK1, CLK0 : D0/CLKOUT Clock Out Rate Setting
#				00 = Clock Out OFF
#				01 = Clock Out Frequency = fCLKIN (default)
#				10 = Clock Out Frequency = fCLKIN/2
#				11 = Clock Out Frequency = fCLKIN/4
#				When not using CLKOUT, it is recommended that it be turned off. These bits can only be reset using the RESET pin.
#
#			Bits 4-3 SDCS1, SCDS0: Sensor Detect Current Sources
#				00 = Sensor Detect OFF (default)
#				01 = Sensor Detect Current = 0.5 A
#				10 = Sensor Detect Current = 2 A
#				11 = Sensor Detect Current = 10 A
#				The Sensor Detect Current Sources can be activated to verify  the integrity of an external sensor supplying a signal to the
#				ADS1255/6. A shorted sensor produces a very small signal while an open-circuit sensor produces a very large signal.
#
#			Bits 2-0 PGA2, PGA1, PGA0: Programmable Gain Amplifier Setting
#				000 = 1 (default)
#				001 = 2
#				010 = 4
#				011 = 8
#				100 = 16
#				101 = 32
#				110 = 64
#				111 = 64
#		*/
        buf[2] = (0 << 5) | (0 << 3) | (_gain << 0)
		#//ADS1256_WriteReg(REG_ADCON, (0 << 5) | (0 << 2) | (GAIN_1 << 1));	/*choose 1: gain 1 ;input 5V/
        buf[3] = ADS1256_DRATE_E[_drate]
        
        self.set_CS_Low()
        self.ADS1256_Send8Bit(CMD['CMD_WREG'] | 0) # Write command register, send the register address
        self.ADS1256_Send8Bit(0x03);  # Register number 4,Initialize the number  -1
        
        self.ADS1256_Send8Bit(buf[0]) # Set the status register
        self.ADS1256_Send8Bit(buf[1]) # Set the input channel parameters
        self.ADS1256_Send8Bit(buf[2]) # Set the ADCON control register,gain
        self.ADS1256_Send8Bit(buf[3]) # Set the output rate
        
        self.set_CS_High()    # SPI  cs = 1
        
        self.bsp_DelayUS(50)
        
        #print hex(buf[0]), hex(buf[1]), hex(buf[2]), hex(buf[3])
    
    def ADS1256_DelayDATA(self):
        '''Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands. Minimum  50   CLK = 50 * 0.13uS = 6.5uS'''
        self.bsp_DelayUS(10) # The minimum time delay 6.5us
        
    def ADS1256_Recive8Bit(self):
        '''Receive 8 bit'''
        return bcm2835_spi_transfer(0xff)
        
    def ADS1256_WriteReg(self, _RegID, _RegValue):
        '''Write the corresponding register'''
        self.set_CS_Low()
        self.ADS1256_Send8Bit(CMD['CMD_WREG'] | _RegID)     # Write command register
        self.ADS1256_Send8Bit(0x00) # Write the register number
        
        self.ADS1256_Send8Bit(_RegValue)    # send register value
        self.set_CS_High()
        
    def ADS1256_ReadReg(self, _RegID):
        '''Read  the corresponding register'''
        self.set_CS_Low()
        self.ADS1256_Send8Bit(CMD['CMD_RREG'] | _RegID)
        self.ADS1256_Send8Bit(0x00)
    
        self.ADS1256_DelayDATA()
    
        read = self.ADS1256_Recive8Bit()
        self.set_CS_High()
    
        return read
        
    def ADS1256_WriteCmd(self, _cmd):
        '''Sending a single byte order'''
        self.set_CS_Low()
        self.ADS1256_Send8Bit(_cmd)
        self.set_CS_High()
        
    def ADS1256_ReadChipID(self):
        '''Read the chip ID'''
        print "Read the chip ID: NOT IMPLEMENTED YET"
        
    def ADS1256_SetChannal(self, _ch):
        '''Configuration channel number'''
#        /*
#	Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
#		0000 = AIN0 (default)
#		0001 = AIN1
#		0010 = AIN2 (ADS1256 only)
#		0011 = AIN3 (ADS1256 only)
#		0100 = AIN4 (ADS1256 only)
#		0101 = AIN5 (ADS1256 only)
#		0110 = AIN6 (ADS1256 only)
#		0111 = AIN7 (ADS1256 only)
#		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are dont care)
#
#		NOTE: When using an ADS1255 make sure to only select the available inputs.
#
#	Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
#		0000 = AIN0
#		0001 = AIN1 (default)
#		0010 = AIN2 (ADS1256 only)
#		0011 = AIN3 (ADS1256 only)
#		0100 = AIN4 (ADS1256 only)
#		0101 = AIN5 (ADS1256 only)
#		0110 = AIN6 (ADS1256 only)
#		0111 = AIN7 (ADS1256 only)
#		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are dont care)
#	*/
        if (_ch > 7):
            return
        self.ADS1256_WriteReg(REG_E['REG_MUX'], (_ch << 4) | (1 << 3))  # Bit3 = 1, AINN connection AINCOM
        
        
    def ADS1256_SetDiffChannal(self, _ch):
        '''The configuration difference channel'''
        print "Configure difference channel: NOT IMPLEMENTED YET"
        
    def ADS1256_WaitDRDY(self):
        '''delay time  wait for automatic calibration'''
        for i in range(400000):
            if (self.dataReady_is_Low()):
                break
        if ( i >= 400000):
            print "ADS1256_WaitDRDY() Time Out ...\r\n"
        
    def ADS1256_ReadData(self):
        '''read ADC value'''
        read = 0
        buf = [0x00, 0x00, 0x00]
        
        self.set_CS_Low()
        
        self.ADS1256_Send8Bit(CMD['CMD_RDATA']);	# read ADC command
        self.ADS1256_DelayDATA(); # delay time
        
        # Read the sample results 24-bit
        buf[0] = self.ADS1256_Recive8Bit();
        buf[1] = self.ADS1256_Recive8Bit();
        buf[2] = self.ADS1256_Recive8Bit();

        read = (np.int32(buf[0] << 16)) & 0x00FF0000
        read = read | (buf[1] << 8) # Pay attention to It is wrong   read |= (buf[1] << 8) */
        read = read | buf[2]

        self.set_CS_High()

        # Extend a signed number
        if (read & 0x800000):
            read = read | 0xFF000000

        return np.int32(read)

    def ADS1256_GetAdc(self, _ch):
        '''read ADC value'''
        if (_ch > 7):
            return 0
        
        return self.AdcNow[_ch]
        
    def ADS1256_ISR(self):
        '''Collection procedures'''
        if (self.ScanMode == 0): # 0  Single-ended input  8 channel
            self.ADS1256_SetChannal(self.Channel) # Switch channel mode
            self.bsp_DelayUS(5)

            self.ADS1256_WriteCmd(CMD['CMD_SYNC'])
            self.bsp_DelayUS(5)

            self.ADS1256_WriteCmd(CMD['CMD_WAKEUP']);
            self.bsp_DelayUS(25)

            if (self.Channel == 0):
                self.AdcNow[self.Channel] = self.ADS1256_ReadData()
            else:
                self.AdcNow[self.Channel] = self.ADS1256_ReadData()

            self.Channel = self.Channel + 1
            if (self.Channel >= 8):
                self.Channel = 0
                
        else:   # 1 Differential input  4 channel
            self.ADS1256_SetDiffChannal(self.Channel) # change DiffChannal
            self.bsp_DelayUS(5)

            self.ADS1256_WriteCmd(CMD['CMD_SYNC'])
            self.bsp_DelayUS(5)

            self.ADS1256_WriteCmd(CMD['CMD_WAKEUP'])
            self.bsp_DelayUS(25)

            if (self.Channel == 0):
                self.AdcNow[3] = self.ADS1256_ReadData()
            else:
                self.AdcNow[self.Channel-1] = self.ADS1256_ReadData()
            
            self.Channel = self.Channel + 1
            if (self.Channel >= 4):
                g_tADS1256.Channel = 0
    
    def ADS1256_Scan(self):
        '''Scan function'''
        if (self.dataReady_is_Low()):   # Data is ready, collect data
            self.ADS1256_ISR()
            return 1
        return 0
    
    def Write_DAC8552(self, channel, Data):
        '''DAC send data'''
        print "DAC send data: NOT IMPLEMENTED YET"
        
    def Voltage_Convert(self, Vref, voltage):
        '''Voltage value conversion function. Vref : The reference voltage 3.3V or 5V
        voltage : output DAC value'''
        return np.int16(65536 * voltage / Vref)
    
def main_ADS1256():
    '''Module testing helper function'''

    print "TESTING STARTS!"

    if not bcm2835_init():
        return

    try:
        adc = ADS1256()
        adc.chipInit()
        ch_num = 0
        adc.ADS1256_CfgADC(ADS1256_GAIN_E['ADS1256_GAIN_1'], 'ADS1256_15SPS')
        adc.ADS1256_StartScan(0,ch_num)
        adc.ADS1256_Scan()
        
        readout_val = [0,0,0,0,0,0,0,0]#-1.0
        volt_val = [0,0,0,0,0,0,0,0] #-2.0
        while True:
            if (adc.ADS1256_Scan() == 1):
                for i in range(8):
                    readout_val[i] = adc.ADS1256_GetAdc(i)
                    volt_val[i] = (readout_val[i] * 100) / 167 / 1000000.0
                #print "readout_val = ", readout_val
                print " volt_val = ", volt_val
            else:
                print "NO scan performed!"
            adc.bsp_DelayUS(100000)#(100000)    
    finally:
        bcm2835_spi_end()
        bcm2835_close()

    print "TESTING ENDS"

if __name__ == '__main__':
    main_ADS1256()
    
    
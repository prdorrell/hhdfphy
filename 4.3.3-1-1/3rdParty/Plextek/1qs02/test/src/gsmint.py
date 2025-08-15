"""
-------------------------------------------------------------------------------
Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/gsmint.py $
$Revision: 6402 $
$Author: pdm $
$Date: 2011-07-21 14:53:32 +0100 (Thu, 21 Jul 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Interface to the FPGA GSM engine.

-------------------------------------------------------------------------------
"""

from numpy      import array
from numpy      import round, minimum, maximum
from numpy      import concatenate, fliplr, flipud
from numpy      import zeros, zeros_like
from numpy      import real, imag, conj
from numpy      import all, equal
from numpy      import sqrt
from numpy.fft  import fft
import struct

import memint
import phyint


# #############################################################################
# test correlator interface
class gsm_interface:
    def __init__( self, phy):
        # #####################################################################
        # register numbers and fields
        
        # #######################################
        # mode control register number
        self.mode_reg                = 0x02
        
        # antenna mode field values (bits 2:3)
        self.antenna_mode_ant1       = 0x004
        self.antenna_mode_ant2       = 0x008
        
        # channel filter enable (bit 8)
        self.mode_ch_filt_en         = 0x100
        
        # correlator and general reset (bit 9)
        self.corr_mode_reset         = 0x200
        
        # select integration of 375 samples per result (bit 11)
        self.gsm_1_result_per_slot   = 0x800
        
        # enable the DC offset correction
        self.gsm_enable_dc_offset    = 0x1000
        
        # #######################################
        # ADC RSSI register number
        self.adc_rssi_reg            = 0x16
        
        # #######################################
        # the GSM RSSI results register number
        self.gsm_rssi_result_reg     = 0x17
        
        # #######################################
        # the GSM RSSI status register number
        self.gsm_rssi_status_reg     = 0x18

        # the number of results in the FIFO (bits 0:9)    
        self.gsm_rssi_fifo_cnt_mask  = 0x03FF
        
        # FIFO empty (bit 10)
        self.gsm_rssi_fifo_empty     = 0x0400
        
        # FIFO full (bit 11)
        self.gsm_rssi_fifo_full      = 0x0800
        
        # FIFO underflow (bit 12)
        self.gsm_rssi_fifo_underflow = 0x1000
        
        # FIFO overflow (bit 13)
        self.gsm_rssi_fifo_overflow  = 0x2000
        
        # #######################################
        # the GSM DC offset register number
        self.gsm_dc_offset_reg       = 0x19

        # #####################################################################
        # the size of the coefficients
        self.coeff_size_bits = 8
        
        # #####################################################################
        # make local copies of the controls
        self.phy = phy
                
        # #####################################################################
        # disable the channel filter
        self.stop()
    
    def stop( self ):
        # #####################################################################
        # disable the channel filter
        self.phy.debug_reg_write( self.mode_reg, 0 )
    
    def run( self, one_result_per_slot, enable_dc_offset, antenna1 ):
        # #####################################################################
        # Let the channel filter run
        reg = self.mode_ch_filt_en
        
        if one_result_per_slot :
            reg = reg | self.gsm_1_result_per_slot
            
        if enable_dc_offset :
            reg = reg | self.gsm_enable_dc_offset
            
        if antenna1 :
            reg = reg | self.antenna_mode_ant1
            
        else :
            reg = reg | self.antenna_mode_ant2
            
        self.phy.debug_reg_write( self.mode_reg, reg )
    
    def get_adc_rssi( self ):
        # #####################################################################
        # get the RSSI results
        return self.phy.debug_reg_read(self.adc_rssi_reg)
    
    def get_chan_pwr( self ):
        # #####################################################################
        # get any results
        status_reg = self.phy.debug_reg_read( self.gsm_rssi_status_reg )
        
        if status_reg & self.gsm_rssi_fifo_overflow :
            print "FIFO overflow occurred, ",       
            
        if status_reg & self.gsm_rssi_fifo_underflow :
            print "FIFO underflow occurred, ",        
            
        if status_reg & self.gsm_rssi_fifo_full :
            print "FIFO full, ",        
            
        if status_reg & self.gsm_rssi_fifo_empty :
            print "FIFO empty, ",        
            
        result_count = status_reg & self.gsm_rssi_fifo_cnt_mask            
        print "%d results" % result_count      
            
        result_array = zeros(result_count)            
        for n in range(result_count) :
            result_array[n] = self.phy.debug_reg_read( self.gsm_rssi_result_reg )
                               
        return result_array
    
    def get_dc_offset( self ):
        # #####################################################################
        # get the DC Offset
        reg = self.phy.debug_reg_read(self.gsm_dc_offset_reg)
        
        inph_dc = reg & 0xFFFF
        if inph_dc >= 32768 :
            inph_dc -= 65536
            
        quad_dc = reg/65536
        if quad_dc >= 32768 :
            quad_dc -= 65536
            
        return (inph_dc, quad_dc)
    
    def set_dc_offset( self, inph_dc, quad_dc ):
        # #####################################################################
        # get the DC Offset
        reg = ((int(quad_dc) & 0xFFFF) << 16)+(int(inph_dc) & 0xFFFF)
        reg = self.phy.debug_reg_write(self.gsm_dc_offset_reg, reg)
        return
    
    def debug_gsm( self, num_samples ):
        # #####################################################################
        # use the debug_gsm command to get a number of GSM result samples
        return array(self.phy.debug_gsm(num_samples))

# #############################################################################
# the main program
if __name__ == '__main__':
    print "No standalone functionality"

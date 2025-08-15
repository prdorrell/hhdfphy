"""
-------------------------------------------------------------------------------
Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/frame_plot.py $
$Revision: 6312 $
$Author: pdm $
$Date: 2011-07-15 09:39:50 +0100 (Fri, 15 Jul 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

get rf into frame buffer and plot

-------------------------------------------------------------------------------
"""

print "loading libs...",

from source import uplink_dpcch_source, downlink_cpich_source, composite_source, psch_source
from numpy  import zeros, concatenate, copy, conj, argmax, max, min, argmin, log10, kaiser
from numpy  import real, imag, array, round, all, equal, tile, ones, e, pi, arange, repeat
from numpy  import roll
from numpy.fft import fft, fftshift
from pylab import show, plot, xlim, ylim, legend, title, figure

import get_defaults
import phyint
import sys
import struct
print "done.\n"

if len(sys.argv) > 1:
    address = sys.argv[1]
    print "Target IP address is %s\n" % address 
    
else :
    address = get_defaults.getDefaultHhdfIpAddress()

phy = phyint.phyint(address,55555)
print "reg0 version id: 0x%04x"%( phy.debug_reg_read( 0 ) )

filtered_data_even_base = 0x00000000
filtered_data_odd_base =  0x00400000
coef_block_ram_bases = \
    (
    0x00010000, 0x00000000, #1, 2
    0x00000000, 0x00000000, #3, 4
    0x00000000, 0x00000000, #5, 6
    0x00000000, 0x00000000, #7, 8
    0x00000000, 0x00000000, #9, 10
    0x00000000, 0x00000000, #11, 12
    0x00000000, 0x00000000, #13, 14
    0x00000000, 0x00000000  #15
    )
correlator_mode_reg = 0x02
even_odd_sync_reg = 0x07
even_buffer_valid = 0x01
odd_buffer_valid = 0x02
finished_with_even = 0x04
finished_with_odd = 0x08
correlation_result_even_base = 0x00800000
correlation_result_odd_base = 0x00c00000
corr_mode_ch_filt_en    = 0x100
corr_mode_corr_en       = 0x080
corr_mode_test_en       = 0x200
corr_mode_512           = 0x01
corr_mode_4096_full     = 0x02
corr_mode_4096          = 0x03
corr_mode_reset         = 0x200
corr_mode_idle          = 0


def read_back_samples( address, length ):
    # read back check
    print "Read back...",
    ans = phy.mem_raw_read( address, length )
    c = []
    for a in ans:
        (re,im)=struct.unpack("<hh", struct.pack( "<I", a ) )
        c.append( re + 1j*im )
    return array(c)

# set up rf
phy.debug_switches( band="wcdma_1", ad_input="RXLBRF", ad_gpio=0xa )
phy.debug_set_frequency( 950.0e6, band="low" )
phy.debug_set_gain( 50 )

# reset correlator and set mode
phy.debug_reg_write( correlator_mode_reg, corr_mode_reset )
phy.debug_reg_write( correlator_mode_reg, corr_mode_idle )
phy.debug_reg_write( correlator_mode_reg, corr_mode_ch_filt_en 
                                        | corr_mode_corr_en 
                                        | corr_mode_512 )

for i in range( 100 ):
    if phy.debug_reg_read( even_odd_sync_reg ) & even_buffer_valid:
        print "even buf ready.\nRead samples..."
        break
else:
    print "Giving up waiting for even/odd sync"
    #sys.exit()

phy.debug_reg_write( correlator_mode_reg, corr_mode_idle ) #hmm

d = read_back_samples( filtered_data_even_base, 76800 )

print "\nplot..."

plot( real(d), color='red', label="real" )
plot( imag(d), color='blue', label="imag" )
title("raw filtered samples")
legend()
show()

"""
-------------------------------------------------------------------------------
Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/test_ul_search.py $
$Revision: 7552 $
$Author: pdm $
$Date: 2011-10-11 09:13:34 +0100 (Tue, 11 Oct 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the uplink search method

-------------------------------------------------------------------------------
"""

print "loading libs...",

from numpy     import zeros, zeros_like, arange
from numpy     import round
from numpy     import minimum, maximum, argmax
from numpy     import concatenate
from numpy     import real, imag, angle 
from numpy     import all, equal
from numpy     import mod
from numpy     import sqrt
from numpy     import absolute
from numpy     import pi
from pylab     import show, plot, subplot, xlim, ylim, legend, title, suptitle

import sys
import time

import get_defaults
import phyint
print "done.\n"

def main(address, band, freq_Hz, code_num, num_pilots) :
    """The main function of the test script"""
    # ###############################################################
    # Initialise the PHY interface
    phy = phyint.phyint(address,55555)

    phy.set_mode("3G");
    _ = phy.wait_for_msg()
    print "    PCB Version  = %s\n" \
          "    FPGA Version = %s\n" \
          "    SW Version   = %s\n" % phy.get_version()
    
#    phy.set_vctcxo(1913)
#    phy.set_vctcxo(1875)
#    phy.set_vctcxo(1874)
    phy.set_vctcxo(1914)
#    phy.set_vctcxo(1990)

    # Set the band
    phy.set_band(band)
    
    # Set the threshold adjustment factors
    ref_cal_thresh_factor_dB = 0.0  
    search_thresh_factor_dB  = 3.0
    track_thresh_factor_dB   = 1.0 
    phy.debug_set_ref_cal_thresh_factor_dB(ref_cal_thresh_factor_dB)
    phy.debug_set_search_thresh_factor_dB(search_thresh_factor_dB)
    phy.debug_set_track_thresh_factor_dB(track_thresh_factor_dB)

    # Select the low-power FPGA configuration
    phy.set_low_power("LOW_POWER");
    
    # Select the pauses
    phy.set_pauses(searching_pauses = 86, tracking_pauses = 8);

    # Start the uplink search
    phy.start_ul_search(code_num, freq_Hz, num_pilots)
    
    aborted = False
    while True :
#    for _ in range(1000) :
        msg = phy.wait_for_msg()
        print "%s" % msg,
        if (msg.find("UAT") == 0):
            aborted = True
            break
    
    print("\n")
    if not aborted :
        time.sleep(5.0)
        phy.abort()
    

# ###################################################################
# the main program
if __name__ == '__main__':
    #
    # parse the command-line arguments: if there is one it is the IP
    # address of the server
    #
    if len(sys.argv) > 1:
        address = sys.argv[1]
        print "Target IP address is %s\n" % address 
        
    else :
        address = get_defaults.getDefaultHhdfIpAddress()

    band = "AUTO"
    
    freq_Hz = 1930000000
#    freq_Hz =  835000000   # UL850
#    freq_Hz =  880000000   # DL850
#    freq_Hz =  890000000   # UL900
#    freq_Hz =  940000000   # DL900
#    freq_Hz = 1730000000   # UL1700
#    freq_Hz = 2130000000   # DL1700
#    freq_Hz = 1750000000   # UL1800
#    freq_Hz = 1860000000   # DL1800    
#    freq_Hz = 1880000000   # UL1900
#    freq_Hz = 1960000000   # DL1900
#    freq_Hz = 1950000000   # UL2100
#    freq_Hz = 2150000000   # DL2100
    code_num = 0
    
    num_pilots = 8
    
    main(address, band, freq_Hz, code_num, num_pilots)

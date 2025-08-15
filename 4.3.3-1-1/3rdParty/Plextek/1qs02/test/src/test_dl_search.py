"""
-------------------------------------------------------------------------------
Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/test_dl_search.py $
$Revision: 7552 $
$Author: pdm $
$Date: 2011-10-11 09:13:34 +0100 (Tue, 11 Oct 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the downlink search method

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

def main(address, band, freq_Hz, code_num, num_syms) :
    """The main function of the test script"""
    # ###############################################################
    # Initialise the PHY interface
    phy = phyint.phyint(address,55555)
    
    phy.set_mode("3G");
    _ = phy.wait_for_msg()
    print "    PCB Version  = %s\n" \
          "    FPGA Version = %s\n" \
          "    SW Version   = %s\n" % phy.get_version()

    antenna = 2
    phy.set_antenna( antenna )
    
    if   (freq_Hz == 2162200000) :
        if   (code_num == 7872) :
            phy.set_vctcxo(2031)
        
        elif (code_num == 3584) :
            phy.set_vctcxo(2028)
            
        else :
            phy.set_vctcxo(2031)

    elif (freq_Hz == 2152200000) :
        if   (code_num == 7776) :
            phy.set_vctcxo(2027)
        
        elif (code_num == 5216) :
            phy.set_vctcxo(2030)
            
        else :
            phy.set_vctcxo(2027)

    elif (freq_Hz == 2112800000) :
        if   (code_num == 3408) :
            phy.set_vctcxo(2019)
        
        elif (code_num ==  608) :
            phy.set_vctcxo(2027)
            
        else :
            phy.set_vctcxo(2027)

    elif (freq_Hz == 1930000000) :
        phy.set_vctcxo(1929)

    else :
        phy.set_vctcxo(1929)
        
    phy.set_vctcxo(1914)

    # Set the band
    phy.set_band(band)

    # Set the threshold adjustment factors
    ref_cal_thresh_factor_dB = 0.0  
    search_thresh_factor_dB  = 5.0
    track_thresh_factor_dB   = 1.0 
    phy.debug_set_ref_cal_thresh_factor_dB(ref_cal_thresh_factor_dB)
    phy.debug_set_search_thresh_factor_dB(search_thresh_factor_dB)
    phy.debug_set_track_thresh_factor_dB(track_thresh_factor_dB)
    
    # Start the downlink search
    phy.start_dl_search(code_num, freq_Hz, num_syms)
    
    aborted = False
    while True :
        msg = phy.wait_for_msg()
        print "%s" % msg,
        if (msg.find("DAT") == 0):
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

    freq_Hz     = 2162200000    #    Off-air
#    code_num = 7872         #    On 2162200000
    code_num = 3584         #    On 2162200000

#    freq_Hz = 2152200000    #    Off-air  
#    code_num = 7776         #    On 2152200000
#    code_num = 5216         #    On 2152200000
    
#    freq_Hz = 2112800000    #    Off-air  
#    code_num = 3408         #    On 2112800000
#   code_num =  608         #    On 2112800000
    
    freq_Hz = 1930000000    #    Sig. gen.  
    code_num = 256          #    On 1930000000 
        
    num_syms = 8
    
    main(address, band, freq_Hz, code_num, num_syms)

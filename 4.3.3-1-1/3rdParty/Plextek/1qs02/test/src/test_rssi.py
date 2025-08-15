"""
-------------------------------------------------------------------------------
Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/test_rssi.py $
$Revision: 6419 $
$Author: pdm $
$Date: 2011-07-22 11:19:49 +0100 (Fri, 22 Jul 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the RSSI calculation

-------------------------------------------------------------------------------
"""

print "loading libs...",

import sys

from numpy  import  arange
from numpy  import  log10
from numpy  import  zeros_like

from pylab  import  plot
from pylab  import  show

import corrint
import get_defaults
import phyint

print "done."

# #############################################################################
# the main program
if __name__ == '__main__':
    
    # #########################################################################
    # parse the command-line arguments: if there is one it is the IP address of
    # the server.
    if len(sys.argv) > 1:
        address = sys.argv[1]
        print "Target IP address is %s\n" % address 
        
    else :
        address = get_defaults.getDefaultHhdfIpAddress()
        
    # #########################################################################
    # create the PHY interface instance.
    phy = phyint.phyint(address,55555)
    
    phy.set_mode("3G");
    _ = phy.wait_for_msg()
    print "    PCB Version  = %s\n" \
          "    FPGA Version = %s\n" \
          "    SW Version   = %s\n" % phy.get_version()
    
    freqRange_Hz = arange(1920e6, 1980e6, 1e5)
    pwr_dB = zeros_like(freqRange_Hz)
    for n in range(0, len(freqRange_Hz)) :
        
        # #####################################################################
        # setup the frequency and gain.
        phy.debug_switches( band="wcdma_3", ad_input="RXHB2RF", ad_gpio=0xc )
        # 1920
        # 1980
        phy.debug_set_frequency( freqRange_Hz[n], band="high" )
        phy.debug_set_gain( 100 )
        
        # #####################################################################
        # initialise the correlator and start it.
        filter_len   = 256
        check_io     = False
        zero_results = False
        frames_to_average = 1
        slots_to_average  = 1
        cin = corrint.correlation_interface(phy, \
                                            filter_len, \
                                            frames_to_average, slots_to_average, \
                                            check_io, zero_results)
        cin.run()
        
        # #####################################################################
        # get the correlator results, which indicates when the RSSI result will
        # be ready. 
        results, num_samples_over_threshold, peak_offset, peak_power, frame_number = cin.get(100)
            
        # #####################################################################
        # read the RSSI result until the same result is received twice (handles 
        # firmware feature in which writes to the result register may be
        # incomplete when a read occurs). 
        res1 = phy.debug_reg_read(13)
        res2 = phy.debug_reg_read(13)
        timeout = 10
        while res1 != res2 :
            timeout = timeout-1
            if timeout < 0 :
                raise Exception("RSSI not stable")
     
            res1 = res2
            res2 = phy.debug_reg_read(13)
            
        pwr_dB[n] = 10*log10(res1)
            
        print "Frequency = %10.0f, Power = %5.1f dB" \
              % (freqRange_Hz[n], pwr_dB[n])

    plot(freqRange_Hz, pwr_dB)
    show()


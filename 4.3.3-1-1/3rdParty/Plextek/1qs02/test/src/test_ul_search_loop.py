"""
-------------------------------------------------------------------------------
Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/test_ul_search_loop.py $
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

print_all          = True
print_version_once = True

num_tests                = 100
max_num_iters_per_test   = 50
max_num_results_per_test = 20

def main(address, band, freq_Hz, code_num, num_pilots) :
    """The main function of the test script"""
    
    version_printed = False
    
    # ###############################################################
    # Repeat the basic test to gather statistics
    result_strings = []
    for n in range(num_tests) :
        
        # ###############################################################
        # Initialise the PHY interface
        phy = phyint.phyint(address,55555)
        
        phy.set_mode("3G");
        _ = phy.wait_for_msg()
        if (not print_version_once) or (not version_printed) :
            print "    PCB Version  = %s\n" \
                  "    FPGA Version = %s\n" \
                  "    SW Version   = %s\n" % phy.get_version()
            version_printed = True
        
#        phy.set_vctcxo(1913)
#        phy.set_vctcxo(1875)
        phy.set_vctcxo(1914)
#        phy.set_vctcxo(1990)
    
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
        #phy.set_low_power("LOW_POWER");
    
        # Select the pauses
        phy.set_pauses(searching_pauses = 53, tracking_pauses = 3);
        
        # Start the uplink search
        phy.start_ul_search(code_num, freq_Hz, num_pilots)
        
        num_search_restarts = 0
        num_results         = 0
        aborted = False
        for _ in range(max_num_iters_per_test) :
            msg = phy.wait_for_msg()
            if (msg.find("UAT") == 0) :
                if print_all :
                    print "%s" % msg,
                    
                aborted = True
                break
    
            elif (msg.find("US1") == 0) :
                if print_all :
                    print "%s" % msg,
                    
                num_search_restarts += 1
    
            elif (msg.find("US2") == 0) :
                if print_all :
                    print "%s" % msg,
    
            elif (msg.find("UU") == 0) :
                if print_all :
                    print "%s" % msg,
                    
                num_results += 1
                if num_results >= max_num_results_per_test :
                    break
        
        if print_all :
            print("\n")
            
        if not aborted :
            phy.abort()
            
            
        if aborted :
            result_string = "Number of restarts = %3u, number of results = %5u, aborted" \
                            % (num_search_restarts, num_results),
            
        else :
            result_string = "Number of restarts = %3u, number of results = %5u" \
                            % (num_search_restarts, num_results),
            
        print "%s" % result_string
        
        result_strings.append(result_string)
    
    print "Summary:\n"
    for n in range(num_tests) :
        print "%3u : %s" % (n, "".join(result_strings[n]))
    

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

    band = "UL2100"
    
    freq_Hz = 1930000000  
    
    code_num = 0
    
    num_pilots = 8
    
    main(address, band, freq_Hz, code_num, num_pilots)

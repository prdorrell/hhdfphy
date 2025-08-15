"""
-------------------------------------------------------------------------------
Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/test_detector_impulse.py $
$Revision: 6412 $
$Author: pdm $
$Date: 2011-07-22 10:33:06 +0100 (Fri, 22 Jul 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the channel filter and correlator using an impulse filter

-------------------------------------------------------------------------------
"""

print "loading libs...",

from numpy     import zeros, zeros_like, arange
from numpy     import minimum, maximum, argmax
from numpy     import concatenate
from numpy     import real, imag, angle 
from numpy     import all, equal
from numpy     import mod
from numpy     import sqrt, log10
from numpy     import absolute
from numpy     import pi
from pylab     import show, plot, subplot, xlim, ylim, legend, title, suptitle

import sys

import corrint
import get_defaults
import memint
import phyint
print "done.\n"

check_io        = False
zero_output_mem = False

num_frames   = 1
num_iters    = 1
filter_len   = 256
filter_delay = 32

def main(address) :
    """The main function of the test script"""
    # ###############################################################
    # define the impulse response
    ref_ir               = zeros( filter_len, dtype = complex )
    ref_ir[filter_delay] = 1
    #print ref_ir
        
    # ###############################################################
    # create the PHY interface instance
    phy = phyint.phyint(address,55555)

    # ###############################################################
    # force the 3G mode
    phy.set_mode("3G");
    _ = phy.wait_for_msg()
    print "    PCB Version  = %s\n" \
          "    FPGA Version = %s\n" \
          "    SW Version   = %s\n" % phy.get_version()
        
    # ###############################################################
    # setup the frequency and gain.
    phy.debug_switches( band="wcdma_3", ad_input="RXHB2RF", ad_gpio=0xc )
    phy.debug_set_frequency( 1930000000, band="high" )
    phy.debug_set_gain( 90 )

    # ###############################################################
    # initialise the correlator and leave it idle.
    frames_to_average = 1
    slots_to_average  = 1
    cin = corrint.correlation_interface(phy, \
                                        filter_len, \
                                        frames_to_average, slots_to_average, \
                                        check_io, zero_output_mem)

    # ###############################################################
    # write correlation coefficents
    print ""
    print "Writing the filter coefficients"
    filter_num = 0
    scale_by_fft_size = False
    cin.define_filter( filter_num, ref_ir, scale_by_fft_size, check_io )

    # ###############################################################
    # start the correlator
    print "Running the correlation"
    cin.run(channel_filter_active = True)

    # ###############################################################
    # iterate through the input offsets
    for n in range(num_iters) :
        # ###########################################################
        # get the results
        results, num_samples_over_threshold, peak_offset, peak_power, frame_number = cin.get(num_frames*76800)
            
        # ###########################################################
        # read the RSSI result until the same result is received 
        # twice (handles firmware feature in which writes to the
        # result register may be incomplete when a read occurs). 
        res1 = phy.debug_reg_read(13)
        res2 = phy.debug_reg_read(13)
        timeout = 10
        while res1 != res2 :
            timeout = timeout-1
            if timeout < 0 :
                raise Exception("RSSI not stable")
     
            res1 = res2
            res2 = phy.debug_reg_read(13)
            
        rms    = sqrt(res1)            
        pwr_dB = 10*log10(res1)

        # ###########################################################
        # calculate the difference in the offsets and the magnitude
        # of the output impulse
        plot( real(results), label="result re" )
        plot( imag(results), label="result im" )
        legend()
        title("RSSI = %.1f dB (%.0f)" % (pwr_dB, rms))
        show()
        
        # ###########################################################
        # generate the test data
        cin.switch_buffers()

# ###################################################################
# the main program
if __name__ == '__main__':
    #
    # parse the command-line arguments: if there is one it is the IP
    # address of the server
    if len(sys.argv) > 1:
        address = sys.argv[1]
        print "Target IP address is %s\n" % address 
        
    else :
        address = get_defaults.getDefaultHhdfIpAddress()
    
    main(address)

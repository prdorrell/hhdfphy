"""
-------------------------------------------------------------------------------
Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/test_frame_counter.py $
$Revision: 6412 $
$Author: pdm $
$Date: 2011-07-22 10:33:06 +0100 (Fri, 22 Jul 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the frame counter

-------------------------------------------------------------------------------
"""

print "loading libs...",

from numpy      import zeros, zeros_like, arange
from numpy      import minimum, maximum, argmax
from numpy      import concatenate, flipud, reshape
from numpy      import real, imag, angle, conj, sum, mean 
from numpy      import all, equal
from numpy      import mod
from numpy      import sqrt, log10
from numpy      import absolute
from numpy      import pi
from pylab      import figure, show, plot, subplot, xlim, ylim, legend, title, suptitle
from pylab      import get_current_fig_manager
from scipy      import signal

import sys

import corrint
import gencodes
import get_defaults
import memint
import phyint
print "done.\n"

check_io        = False
zero_output_mem = False
power_results   = False

num_iters          = 20
filter_len         = 256
frames_to_average  = 1

samplesPerChip     = 2
chipsPerSlot       = 2560
slotsPerFrame      = 15 
samplesPerSlot     = samplesPerChip*chipsPerSlot
samplesPerFrame    = samplesPerSlot*slotsPerFrame

num_output_samples = samplesPerFrame

def main(address) :
    """The main function of the test script"""
    # ###############################################################
    # define the impulse response
    ref_ir = flipud(conj(gencodes.gen_psc()))
    #print ref_ir
        
    # ###############################################################
    # create the PHY interface instance
    phy = phyint.phyint(address,55555)
    
    phy.set_mode("3G");
    _ = phy.wait_for_msg()
    print "    PCB Version  = %s\n" \
          "    FPGA Version = %s\n" \
          "    SW Version   = %s\n" % phy.get_version()
    
    phy.set_vctcxo(1913)
    phy.set_vctcxo(1865)
    phy.set_vctcxo(1990)    # 10.10.7.100 tuned to 2162200000
    phy.set_vctcxo(1956)    # 10.10.7.102 tuned to signal generator
    phy.set_vctcxo(1919)    # 10.10.7.102 tuned to signal generator
     
    #phy.set_vctcxo(2047)
        
    # ###############################################################
    # setup the frequency and gain.
    phy.debug_switches( band="wcdma_3", ad_input="RXHB2RF", ad_gpio=0xa )
#    phy.debug_set_frequency( 2112800000, band="high" )
    phy.debug_set_frequency( 2162200000, band="high" )
#    phy.debug_set_frequency( 2142400000, band="high" )
#    phy.debug_set_frequency( 2156600000, band="high" )
    phy.debug_set_gain( 100 )

    # ###############################################################
    # initialise the correlator and leave it idle.
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
    scale_by_fft_size = True
    cin.define_filter( filter_num, ref_ir, scale_by_fft_size, check_io )

    # ###############################################################
    # start the correlator
    print "Running the correlation"
    channel_filter_active = True
    cin.run(channel_filter_active, power_results)

    # ###############################################################
    # iterate through the input offsets
    peak_pos_meas = zeros(num_iters)
    peak_val_meas = zeros(num_iters)
    for n in range(num_iters) :
        # ###########################################################
        # get the results
        num_input_samples = num_output_samples
        inp_samples, out_samples, num_samples_over_threshold, peak_offset, peak_power, frame_number = \
            cin.get_with_input(num_input_samples, num_output_samples)
        
        print "Frame number = %d" % mean(frame_number)
        
        # ###########################################################
        # generate the test data
        cin.switch_buffers()
        
#    figure()
#    subplot(2, 1, 1)
#    plot(peak_pos_meas)
#    subplot(2, 1, 2)
#    plot(peak_val_meas)
#    show()

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

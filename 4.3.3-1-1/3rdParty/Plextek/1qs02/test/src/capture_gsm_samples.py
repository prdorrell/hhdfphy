"""
-------------------------------------------------------------------------------
Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/capture_gsm_samples.py $
$Revision: 6380 $
$Author: pdm $
$Date: 2011-07-20 09:32:59 +0100 (Wed, 20 Jul 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Capture the samples spanning the GSM900 downlink band and store them to Matlab files. 

-------------------------------------------------------------------------------
"""

print "loading libs...",

from numpy      import arange
from numpy      import flipud
from numpy      import conj 
from numpy      import array
from scipy      import io

import sys
import time

import corrint
import gencodes
import get_defaults
import phyint
print "done.\n"

check_io        = False
zero_output_mem = False
power_results   = True

samplesPerChip     = 2
chipsPerSlot       = 2560
slotsPerFrame      = 15 
samplesPerSlot     = samplesPerChip*chipsPerSlot
samplesPerFrame    = samplesPerSlot*slotsPerFrame

num_frames   = 50
num_iters    = 2
filter_len   = 2048

num_inp_samples = num_frames*samplesPerFrame
num_out_samples = samplesPerFrame

def main(address) :
    """The main function of the test script"""
    # ###############################################################
    # create the PHY interface instance
    phy = phyint.phyint(address,55555)
    print "FPGA version id: 0x%04x"%( phy.debug_reg_read( 0 ) )
        
    frequencies = arange(935e6+3.6e6/2, 960e6, 3.6e6)
    for frequency in frequencies :
        # ###############################################################
        # setup the frequency and gain.
        phy.set_vctcxo(1942)
        
        phy.debug_switches(band="wcdma_1", ad_input="RXLBRF", ad_gpio=0xa)
        phy.debug_set_frequency(frequency, band="low")
        phy.debug_set_gain(78)
    
        # ###############################################################
        # initialise the correlator and leave it idle.
        frames_to_average = num_frames
        slots_to_average  = 1
        cin = corrint.correlation_interface(phy, \
                                            filter_len, \
                                            frames_to_average, slots_to_average, \
                                            check_io, zero_output_mem)
    
        # ###############################################################
        # start the correlator
        print "Running the correlation"
        channel_filter_active = True
        cin.run(channel_filter_active, power_results)
        time.sleep(5.0)
    
        # ###############################################################
        # iterate through the input offsets
        for n in range(num_iters) :
            # ###########################################################
            # get the results
            time.sleep(num_frames*0.01+0.1)
            inp_samples, _, _, _, _, _ = \
                cin.get_with_input(num_inp_samples, num_out_samples)
    
            variable = "inp_samples_%d_%d" % (frequency, n)
            filename = "C:\Documents and Settings\pdm.000\Desktop\Spies\Ames\GsmSamples\%s" % variable             
    
            matlab_data = {variable : inp_samples}
            io.savemat(filename, matlab_data, oned_as='column')            
            
            # ###########################################################
            # let the FPGA generate the test data for the next iteration
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

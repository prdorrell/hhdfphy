"""
-------------------------------------------------------------------------------
Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/capture_samples.py $
$Revision: 6397 $
$Author: pdm $
$Date: 2011-07-21 12:26:29 +0100 (Thu, 21 Jul 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Capture the samples and store them to Matlab files. 

-------------------------------------------------------------------------------
"""

print "loading libs...",

from numpy      import zeros
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

off_air = False

num_inp_samples = num_frames*samplesPerFrame
num_out_samples = samplesPerFrame

def main(address) :
    """The main function of the test script"""
    # ###############################################################
    # define the impulse response
    scrambling_code = 0
    pilots_per_slot = 8
    
    uplink_scram_code = array(gencodes.gen_uplink_scramble_long(scrambling_code))
    slot0_pilot_field_code  = uplink_scram_code[:pilots_per_slot*256]
    slot0_pilot_field_chips = 1j*slot0_pilot_field_code
    slot0_pilot_field_chips[0:-256] = -slot0_pilot_field_chips[0:-256]
    slot0_pilot_field_chips[-256:]  = +slot0_pilot_field_chips[-256:]

#    slot0_pilot_field_chips = -slot0_pilot_field_chips
    ref_ir = zeros(filter_len)+0j
    ref_ir[:pilots_per_slot*256] = flipud(conj(slot0_pilot_field_chips))
        
    # ###############################################################
    # create the PHY interface instance
    phy = phyint.phyint(address,55555)
    print "FPGA version id: 0x%04x"%( phy.debug_reg_read( 0 ) )

    if off_air :    
        phy.set_vctcxo(2004)
        
    else :
        phy.set_vctcxo(1929)
        
    # ###############################################################
    # setup the frequency and gain.
    if off_air :    
        phy.set_vctcxo(2004)    # 10.10.7.100 tuned to 2162200000
        phy.debug_switches( band="wcdma_3", ad_input="RXHB2RF", ad_gpio=0xa )
        phy.debug_set_frequency( 2162200000, band="high" )
        phy.debug_set_gain( 78 )
        
    else :
        phy.set_vctcxo(1916)    # sig. gen.
        phy.debug_switches( band="load50", ad_input="RXHB2RF", ad_gpio=0xc )
        phy.debug_set_frequency( 1930000000, band="high" )
        phy.debug_set_gain( 0 )
    

    # ###############################################################
    # initialise the correlator and leave it idle.
    frames_to_average = num_frames
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
    time.sleep(5.0)

    # ###############################################################
    # iterate through the input offsets
    for n in range(num_iters) :
        # ###########################################################
        # get the results
        time.sleep(num_frames*0.01+0.1)
        inp_samples, _, _, _, _, _ = \
            cin.get_with_input(num_inp_samples, num_out_samples)

        variable = "inp_samples_%d" % n
        filename = "C:\Documents and Settings\pdm.000\Desktop\Spies\Ames\AnalyseDcOffset\log_%d" % n             

        matlab_data = {variable : inp_samples}
        io.savemat(filename, matlab_data)            
        
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

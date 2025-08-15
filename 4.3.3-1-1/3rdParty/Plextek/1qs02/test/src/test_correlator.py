"""
-------------------------------------------------------------------------------
Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/test_correlator.py $
$Revision: 6119 $
$Author: pdm $
$Date: 2011-06-27 14:56:06 +0100 (Mon, 27 Jun 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the correlator using a variety of scripted tests

-------------------------------------------------------------------------------
"""

print "loading libs...",

#from source import uplink_dpcch_source, downlink_cpich_source, composite_source, psch_source
import sys

import get_defaults
import test_correlator_averaging
import test_correlator_gains
import test_correlator_impulse
import test_correlator_psch
import test_correlator_slot_averaging
import test_correlator_ul
import test_correlator_ul_track

print "done.\n"

# ###################################################################
# the main program
if __name__ == '__main__':

    overall_test_pass = True
    
    #
    # parse the command-line arguments: if there is one it is the IP
    # address of the server
    if len(sys.argv) > 1:
        address = sys.argv[1]
        print "Target IP address is %s\n" % address 
        
    else :
        address = get_defaults.getDefaultHhdfIpAddress()
    
    #
    #    Carry out the tests with a simple impulse filter.
    #
    print ""    
    print "================================================================================"    
    print "Testing the correlator functionality with simple impulse filters"    
    print ""
    
    offsets      = [0, 38400, 76799]
    filter_len   = 256
    filter_delay = 0
    test_result  = test_correlator_impulse.main(address, \
                                                offsets, filter_len, filter_delay)
    overall_test_pass = overall_test_pass and test_result

    offsets      = [0, 38400, 76799]
    filter_len   = 2048
    filter_delay = 0
    test_result  = test_correlator_impulse.main(address, \
                                                offsets, filter_len, filter_delay)
    overall_test_pass = overall_test_pass and test_result
    
    #
    #    Carry out the tests with the uplink DPCCH filter.
    #
    print ""    
    print "================================================================================"    
    print "Testing the correlator functionality with the uplink DPCCH filter"    
    print ""
    
    offsets     = [38400, 76798, 76799, 0, 1]
    test_result = test_correlator_ul.main(address, offsets)
    overall_test_pass = overall_test_pass and test_result
    
    #
    #    Carry out the tests with the PSCH filter.
    #
    print ""    
    print "================================================================================"    
    print "Testing the correlator functionality with the PSCH filter"    
    print ""
    
    offsets     = [0, 2560, 5119]
    test_result = test_correlator_psch.main(address, offsets)
    overall_test_pass = overall_test_pass and test_result
    
    #
    #    Carry out the tests of the gain settings.
    #
    print ""    
    print "================================================================================"    
    print "Testing the correlator functionality for the gain settings"    
    print ""

    signal_power = get_defaults.getDefaultHighSignalPower()
    test_correlator_gains.main(address, signal_power)
    overall_test_pass = overall_test_pass and test_result

    signal_power = get_defaults.getDefaultMidSignalPower()
    test_correlator_gains.main(address, signal_power)
    overall_test_pass = overall_test_pass and test_result

    signal_power = get_defaults.getDefaultLowSignalPower()
    test_correlator_gains.main(address, signal_power)
    overall_test_pass = overall_test_pass and test_result
    
    #
    #    Carry out the tests of the frame averaging.
    #
    print ""    
    print "================================================================================"    
    print "Testing the correlator functionality for frame averaging"    
    print ""
    
    offsets           = [0, 19200, 38400, 57600]
    filter_len        = 256
    frames_to_average = 4
    overall_test_pass = test_correlator_averaging.main(address, offsets, filter_len, frames_to_average)
    
    offsets           = [0, 19200, 38400, 57600]
    filter_len        = 2048
    frames_to_average = 4
    overall_test_pass = test_correlator_averaging.main(address, offsets, filter_len, frames_to_average)
    
    #
    #    Carry out the tests of the slot averaging.
    #
    print ""    
    print "================================================================================"    
    print "Testing the correlator functionality for slot averaging"    
    print ""
    
    offsets           = [38400, 76798, 76799, 0, 1]
    filter_len        = 256
    frames_to_average = 3
    slots_to_average  = 5
    overall_test_pass = test_correlator_slot_averaging.main(address, offsets, filter_len, frames_to_average, slots_to_average)
    
    offsets           = [38400, 76798, 76799, 0, 1]
    filter_len        = 2048
    frames_to_average = 3
    slots_to_average  = 14
    overall_test_pass = test_correlator_slot_averaging.main(address, offsets, filter_len, frames_to_average, slots_to_average)
    
    #
    #    Carry out the tests of the tracking.
    #
    print ""    
    print "================================================================================"    
    print "Testing the correlator functionality for tracking"    
    print ""

    target_offset     = 512 
    offsets           = [target_offset-512, target_offset-256, target_offset, target_offset+256, target_offset+511]
    frames_to_average = 2
    slots_to_average  = 15
    overall_test_pass = test_correlator_ul_track.main(address, target_offset, offsets, frames_to_average, slots_to_average)

    #
    #    Print the summary.
    #
    if overall_test_pass :
        print "Summary : Test Passed"
        
    else :
        print "Summary : Test Failed"

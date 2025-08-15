"""
-------------------------------------------------------------------------------
Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/test_correlator_ul_track.py $
$Revision: 6380 $
$Author: pdm $
$Date: 2011-07-20 09:32:59 +0100 (Wed, 20 Jul 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the correlator track mode using the uplink filters

-------------------------------------------------------------------------------
"""

print "loading libs...",

from numpy      import zeros, zeros_like, arange
from numpy      import minimum, maximum, argmax
from numpy      import concatenate, fliplr, flipud, reshape
from numpy      import real, imag, angle, conj 
from numpy      import all, equal
from numpy      import mod
from numpy      import sqrt
from numpy      import absolute, floor
from numpy      import pi
from numpy      import array, roll, vstack
from pylab      import show, plot, subplot, xlim, ylim, legend, title, suptitle, figure, draw
from scipy      import signal

import sys

import corrint
import gencodes
import get_defaults
import memint
import phyint
import sim_mf_sum
import source
print "done.\n"

check_io        = False
zero_output_mem = True
power_results   = True

samplesPerChip     = 2
chipsPerSlot       = 2560
slotsPerFrame      = 15 
samplesPerSlot     = samplesPerChip*chipsPerSlot
samplesPerFrame    = samplesPerSlot*slotsPerFrame

fft_len = 4096

num_track_mode_results = samplesPerSlot-fft_len
track_mode_skew        = num_track_mode_results/2

signal_power = get_defaults.getDefaultHighSignalPower()

def show_plots(ref_peak_mag, ref_peak_pos, ref_mag_samples, \
               fpga_peak_mag, fpga_peak_pos, fpga_mag_samples) :
    """Plot the reference and FPGA results"""
    figure
    
    subplot(2, 1, 1)
#    plot(ref_mag_samples[:num_track_mode_results])
    plot(ref_mag_samples)
    title("Reference, peak of %.1f at %d" % (ref_peak_mag, ref_peak_pos))
    
    subplot(2, 1, 2)
#    plot(fpga_mag_samples[:num_track_mode_results])
    plot(fpga_mag_samples)
    title("Output, impulse of %d at %d" % (fpga_peak_mag, fpga_peak_pos))
    
    suptitle(  "Reference at %d, FPGA at %d, error = %d" \
             % ( ref_peak_pos, fpga_peak_pos, fpga_peak_pos-ref_peak_pos ))
    
    show()

def analyse(inp_samples, ref_ir, \
            fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
            plot_results = False) :
    """Analyse the FPGA results"""

    #
    #    Calculate the reference results.
    #
    accumulate_by_frame = True
    ref_pwr_samples, ref_peak_pos, ref_peak_pwr = \
        sim_mf_sum.fir_pwr(ref_ir, inp_samples, accumulate_by_frame)
    
    ref_mag_samples = sqrt(ref_pwr_samples)
    ref_peak_mag    = sqrt(ref_peak_pwr)

#    #
#    #    Adjust the results for the target offset.
#    #
#    ref_pwr_samples = roll(ref_pwr_samples, -target_offset+track_mode_skew)
#    ref_mag_samples = roll(ref_mag_samples, -target_offset+track_mode_skew)
#    ref_peak_pos    = (ref_peak_pos-target_offset+track_mode_skew+samplesPerFrame) % samplesPerFrame

    #
    #    Determine if the test passed.
    #
    if (    (fpga_peak_pos == ref_peak_pos) \
        and (sqrt(fpga_peak_pwr) >= (0.975*ref_peak_mag)) \
        and (sqrt(fpga_peak_pwr) <= (1.025*ref_peak_mag))) :    
        test_passed = True
        
    else :
        test_passed = False
         
    test_description = "    FPGA pos = %5u, Ref pos = %5u, FPGA mag = %5.0f, Ref mag = %5.0f" \
                       % (fpga_peak_pos, ref_peak_pos, sqrt(fpga_peak_pwr), ref_peak_mag) 

    #
    #    Plot the results
    #    
    if plot_results :
        show_plots(ref_peak_mag, ref_peak_pos, ref_mag_samples, \
                   sqrt(fpga_peak_pwr), fpga_peak_pos, fpga_mag_samples)
    
    return test_passed, test_description

def main(address, target_offset, offsets, frames_to_average, slots_to_average, plot_results = False) :
    """The main function of the test script"""
    
    print ""
    print "    target_offset     = %5u" % target_offset
    print "    offsets           = [",
    for offset in offsets :
        print " %5u" % offset,
        
    print " ]"
    print "    frames_to_average = %u" % frames_to_average
    print "    slots_to_average  = %u" % slots_to_average
    print ""

    overall_test_pass = True
    
    # ###############################################################
    # create the signal source
    samplesPerChip  = 2
    scrambling_code = 0
    pilots_per_slot = 8
    ref_source = source.uplink_dpcch_source(scrambling_code, pilots_per_slot)
    sig_src = source.composite_source(samplesPerChip, (ref_source,))
    
    # ###############################################################
    # define the impulse response
    ref_ir = ref_source.get_fir_coefs(0)
    filter_len = len(ref_ir)
    for slot_num in range(1, slots_to_average) :
        slot_ref_ir = ref_source.get_fir_coefs(slot_num)
        slot_ref_ir = roll(slot_ref_ir, slot_num*0)
        ref_ir = vstack((ref_ir, slot_ref_ir))
        
    ref_ir.shape = (slots_to_average, filter_len)

    # ###############################################################
    # calculate the number of input samples, allowing for the
    # overlap-add implementation which requires an integer number of
    # blocks (1 block = filter_len samples)
    num_input_samples      = frames_to_average*samplesPerFrame+2*filter_len
    num_fpga_input_samples = (frames_to_average+2)*samplesPerFrame+2*filter_len
        
    # ###############################################################
    # create the PHY interface instance
    phy = phyint.phyint(address,55555)
    print "    PCB Version  = %s\n" \
          "    FPGA Version = %s\n" \
          "    SW Version   = %s\n" % phy.get_version()

    # ###############################################################
    # initialise the correlator and leave it idle.
    cin = corrint.correlation_interface(phy, \
                                        filter_len, \
                                        frames_to_average, slots_to_average, \
                                        check_io, zero_output_mem)
    
    # ###############################################################
    # write correlation coefficents
    scale_by_fft_size = True
    for slot_num in range(slots_to_average) :
        cin.define_filter( slot_num, ref_ir[slot_num, :], scale_by_fft_size, check_io )

    # ###############################################################
    # generate the test data for the first correlation
    n = 0
    offset = offsets[n]

    frame      = sqrt(signal_power)*sig_src.get_frame(frames_to_average+3, offset+2, noise_pwr = 0 , ghosts = False)
    even_frame = frame[:num_fpga_input_samples]
    cin.set(even_frame, check_io)

    # ###############################################################
    # generate the test data for the second correlation
    n = n+1
    if len(offsets) > 1 :
        offset = offsets[n]

        frame     = sqrt(signal_power)*sig_src.get_frame(frames_to_average+3, offset+2, noise_pwr = 0 , ghosts = False)
        odd_frame = frame[:num_fpga_input_samples]
        cin.set(odd_frame, check_io)
    
        # ###############################################################
        # set the start offset so that the expected result is centred
        start_offset = target_offset-track_mode_skew
        if start_offset < 0 :
            start_offset = start_offset+samplesPerFrame
            
        cin.set_start_offset(start_offset)
    
        # ###############################################################
        # start the correlator
        channel_filter_active = False
        cin.run_track(channel_filter_active, power_results)
    
        # ###############################################################
        # iterate through the input offsets
        for n in range(2, len(offsets)) :
            # ###########################################################
            # get the results
            fpga_samples, _, fpga_peak_pos, fpga_peak_pwr, _ = cin.get(samplesPerFrame)
            
            if power_results :
                fpga_mag_samples = sqrt(fpga_samples)
                
            else :
                fpga_mag_samples = absolute(fpga_samples)
            
            # ###########################################################
            # analyse the results
            if mod(n, 2) == 0 :
                test_passed, test_description = \
                    analyse(even_frame[start_offset:start_offset+num_input_samples], ref_ir, \
                            fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
                            plot_results)
                
            else :
                test_passed, test_description = \
                    analyse(odd_frame[start_offset:start_offset+num_input_samples], ref_ir, \
                            fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
                            plot_results)

            if test_passed:
                print "    %2u) %s Pass" % (n-2, test_description)
                
            else:
                print "    %2u) %s Fail" % (n-2, test_description)
                overall_test_pass = False
            
            # ###########################################################
            # generate the test data
            offset = offsets[n]
            if mod(n, 2) == 0 :
                frame      = sqrt(signal_power)*sig_src.get_frame(frames_to_average+3, offset+2, noise_pwr = 0 , ghosts = False)
                even_frame = frame[:num_fpga_input_samples]
                cin.set(even_frame, check_io)
                
            else :
                frame     = sqrt(signal_power)*sig_src.get_frame(frames_to_average+3, offset+2, noise_pwr = 0 , ghosts = False)
                odd_frame = frame[:num_fpga_input_samples]
                cin.set(odd_frame, check_io)
    
        # ###########################################################
        # get the results for the penultimate correlation
        n = n+1
        fpga_samples, _, fpga_peak_pos, fpga_peak_pwr, _ = cin.get(samplesPerFrame)
        
        if power_results :
            fpga_mag_samples = sqrt(fpga_samples)
            
        else :
            fpga_mag_samples = absolute(fpga_samples)
        
        # ###########################################################
        # analyse the results
        if mod(n, 2) == 0 :
            test_passed, test_description = \
                analyse(even_frame[start_offset:start_offset+num_input_samples], ref_ir, \
                        fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
                        plot_results)
            
        else :
            test_passed, test_description = \
                analyse(odd_frame[start_offset:start_offset+num_input_samples], ref_ir, \
                        fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
                        plot_results)

        if test_passed:
            print "    %2u) %s Pass" % (n-2, test_description)
            
        else:
            print "    %2u) %s Fail" % (n-2, test_description)
            overall_test_pass = False

    else :
        # ###############################################################
        # set the start offset so that the expected result is centred
        start_offset = target_offset-track_mode_skew
        if start_offset < 0 :
            start_offset = start_offset+samplesPerFrame
            
        cin.set_start_offset(start_offset)
    
        # ###############################################################
        # start the correlator
        channel_filter_active = False
        cin.run_track(channel_filter_active, power_results)

    # ###########################################################
    # get the results for last correlation
    n = n+1
    fpga_samples, _, fpga_peak_pos, fpga_peak_pwr, _ = cin.get(samplesPerFrame)
    
    if power_results :
        fpga_mag_samples = sqrt(fpga_samples)
        
    else :
        fpga_mag_samples = absolute(fpga_samples)
    
    # ###########################################################
    # analyse the results
    if mod(n, 2) == 0 :
        test_passed, test_description = \
            analyse(even_frame[start_offset:start_offset+num_input_samples], ref_ir, \
                    fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
                    plot_results)
        
    else :
        test_passed, test_description = \
            analyse(odd_frame[start_offset:start_offset+num_input_samples], ref_ir, \
                    fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
                    plot_results)

    if test_passed:
        print "    %2u) %s Pass" % (n-2, test_description)
        
    else:
        print "    %2u) %s Fail" % (n-2, test_description)
        overall_test_pass = False

    #
    #    Print the test summary.
    #
    print ""
    if overall_test_pass :
        print "    Test Passed"
        
    else :
        print "    Test Failed"
        
    print ""
    
    return overall_test_pass

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

    target_offset     = 512 
    offsets           = [target_offset-512, target_offset-256, target_offset, target_offset+256, target_offset+511]
    frames_to_average = 2
    slots_to_average  = 15
    main(address, target_offset, offsets, frames_to_average, slots_to_average, plot_results = True)

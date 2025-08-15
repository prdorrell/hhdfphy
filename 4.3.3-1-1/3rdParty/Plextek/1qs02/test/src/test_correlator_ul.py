"""
-------------------------------------------------------------------------------


-------------------------------------------------------------------------------
  Filename:   test_correlator_ul.py
  Author(s):  pdm
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the correlator using the uplink filters

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
from numpy      import array
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
zero_output_mem = False
power_results   = True

frames_to_average = 2
slots_to_average  = 1

samplesPerChip     = 2
chipsPerSlot       = 2560
slotsPerFrame      = 15 
samplesPerSlot     = samplesPerChip*chipsPerSlot
samplesPerFrame    = samplesPerSlot*slotsPerFrame

signal_power = get_defaults.getDefaultHighSignalPower()

def show_plots(ref_peak_mag, ref_peak_pos, ref_mag_samples, \
               fpga_peak_mag, fpga_peak_pos, fpga_mag_samples) :
    """Plot the reference and FPGA results"""
    figure
    
    subplot(2, 1, 1)
    
    plot(ref_mag_samples)
    title("Reference, peak of %.1f at %d" % (ref_peak_mag, ref_peak_pos))
    
    subplot(2, 1, 2)
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

def main(address, offsets, plot_results = False) :
    """The main function of the test script"""
    
    print ""
    print "    offsets       = [",
    for offset in offsets :
        print " %5u" % offset,
        
    print " ]"
    print ""

    overall_test_pass = True
    
    # ###############################################################
    # create the signal source
    samplesPerChip  = 2
    scrambling_code = 0
    pilots_per_slot = 8
    uplink_dpcch_source = source.uplink_dpcch_source(scrambling_code, pilots_per_slot)
    sig_src = source.composite_source(samplesPerChip, (uplink_dpcch_source,))
    
    # ###############################################################
    # define the impulse response
    ref_ir = uplink_dpcch_source.get_fir_coefs(0)
    filter_len = len(ref_ir)

    # ###############################################################
    # calculate the number of input samples, allowing for the
    # overlap-add implementation which requires an integer number of
    # blocks (1 block = filter_len samples)
    num_input_samples      = frames_to_average*samplesPerFrame+2*filter_len
    num_fpga_input_samples = int(  (frames_to_average-1)*samplesPerFrame \
                                 + filter_len*floor((samplesPerFrame+2*filter_len+filter_len-1)/filter_len))
        
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
    filter_num = 0
    scale_by_fft_size = True
    cin.define_filter( filter_num, ref_ir, scale_by_fft_size, check_io )

    # ###############################################################
    # generate the test data for the first correlation
    n = 0
    offset = offsets[n]

    frame      = sqrt(signal_power)*sig_src.get_frame(frames_to_average+1, offset+2*filter_len, noise_pwr = 0 , ghosts = False)
    even_frame = frame[:num_fpga_input_samples]
    cin.set(even_frame, check_io)

    # ###############################################################
    # generate the test data for the second correlation
    n = n+1
    if len(offsets) > 1 :
        offset = offsets[n]

        frame     = sqrt(signal_power)*sig_src.get_frame(frames_to_average+1, offset+2*filter_len, noise_pwr = 0 , ghosts = False)
        odd_frame = frame[:num_fpga_input_samples]
        cin.set(odd_frame, check_io)
    
        # ###############################################################
        # start the correlator
        channel_filter_active = False
        cin.run(channel_filter_active, power_results)
    
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
                    analyse(even_frame[:num_input_samples], ref_ir, \
                            fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
                            plot_results)
                
            else :
                test_passed, test_description = \
                    analyse(odd_frame[:num_input_samples], ref_ir, \
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
                frame      = sqrt(signal_power)*sig_src.get_frame(frames_to_average+1, offset+2*filter_len, noise_pwr = 0 , ghosts = False)
                even_frame = frame[:num_fpga_input_samples]
                cin.set(even_frame, check_io)
                
            else :
                frame     = sqrt(signal_power)*sig_src.get_frame(frames_to_average+1, offset+2*filter_len, noise_pwr = 0 , ghosts = False)
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
                analyse(even_frame[:num_input_samples], ref_ir, \
                        fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
                        plot_results)
            
        else :
            test_passed, test_description = \
                analyse(odd_frame[:num_input_samples], ref_ir, \
                        fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
                        plot_results)

        if test_passed:
            print "    %2u) %s Pass" % (n-2, test_description)
            
        else:
            print "    %2u) %s Fail" % (n-2, test_description)
            overall_test_pass = False

    else :
        # ###############################################################
        # start the correlator
        channel_filter_active = False
        cin.run(channel_filter_active, power_results)

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
            analyse(even_frame[:num_input_samples], ref_ir, \
                    fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
                    plot_results)
        
    else :
        test_passed, test_description = \
            analyse(odd_frame[:num_input_samples], ref_ir, \
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

    offsets      = [0, 38400, 76799, 0, 38400, 76799]
    offsets      = [38400, 76798, 76799, 0, 1]
    offsets = [0, 2560, 5119, 38400]
    main(address, offsets, plot_results = True)

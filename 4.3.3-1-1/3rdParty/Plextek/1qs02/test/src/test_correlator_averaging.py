"""
-------------------------------------------------------------------------------


-------------------------------------------------------------------------------
  Filename:   test_correlator_averaging.py
  Author(s):  pdm
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the correlator and the averaging using impulses and simple delay filters

-------------------------------------------------------------------------------
"""

print "loading libs...",

from numpy     import zeros, zeros_like, arange
from numpy     import minimum, maximum, argmax, argsort
from numpy     import concatenate
from numpy     import real, imag, angle 
from numpy     import all, equal
from numpy     import mod
from numpy     import sqrt
from numpy     import absolute, floor
from numpy     import pi
from pylab     import show, plot, subplot, xlim, ylim, legend, title, suptitle, figure, get_current_fig_manager

import sys

import corrint
import get_defaults
import memint
import phyint
import sim_mf_sum
print "done.\n"

check_io        = False
zero_output_mem = False
power_results   = True

samplesPerChip     = 2
chipsPerSlot       = 2560
slotsPerFrame      = 15 
samplesPerSlot     = samplesPerChip*chipsPerSlot
samplesPerFrame    = samplesPerSlot*slotsPerFrame

def show_plots(inp_samples, \
               ref_peak_mag, ref_peak_pos, ref_mag_samples, \
               fpga_peak_mag, fpga_peak_pos, fpga_mag_samples) :
    """Plot the reference and FPGA results"""
    figure
    get_current_fig_manager().window.setGeometry(20, 40, 600, 800)
    
    subplot(3, 1, 1)
    plot(abs(inp_samples))
    _, ymax = ylim()
    xlim(xmin = -10000)
    _, ymax = ylim()
    ylim(ymin = -ymax)
    title("Input samples")
    
    subplot(3, 1, 2)
    plot(ref_mag_samples)
    xlim(-10000, +90000)
    _, ymax = ylim()
    ylim(ymin = -ymax)
    title("Reference, peak of %.1f at %d" % (ref_peak_mag, ref_peak_pos))
    
    subplot(3, 1, 3)
    plot(fpga_mag_samples)
    xlim(-10000, +90000)
    _, ymax = ylim()
    ylim(ymin = -ymax)
    title("FPGA, peak of %.1f at %d" % (fpga_peak_mag, fpga_peak_pos))
    
    suptitle(  "Reference at %d, output at %d, error = %d" \
             % (ref_peak_pos, fpga_peak_pos, fpga_peak_pos-ref_peak_pos))
    
    show()

def analyse(inp_samples, ref_ir, frames_to_average, \
            fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
            plot_results = False, prefix = "") :
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
    #    Identify the peaks.
    #    
    fpga_peak_pos_list = zeros(frames_to_average)
    fpga_peak_mag_list = zeros(frames_to_average)
    
    fpga_mag_samples_temp = fpga_mag_samples.copy()
    fpga_mag_samples_temp[fpga_peak_pos] = 0
    fpga_peak_pos_list[0] = fpga_peak_pos
    fpga_peak_mag_list[0] = sqrt(fpga_peak_pwr)
    
    for n in range(1, frames_to_average) :
        fpga_peak_pos_temp = argmax(fpga_mag_samples_temp)
        fpga_peak_peak_mag = abs(fpga_mag_samples_temp[fpga_peak_pos_temp])
        fpga_mag_samples_temp[fpga_peak_pos_temp] = 0
        fpga_peak_pos_list[n] = fpga_peak_pos_temp
        fpga_peak_mag_list[n] = fpga_peak_peak_mag

    fpga_sort_indeces = argsort(fpga_peak_pos_list)
    fpga_peak_pos_list = fpga_peak_pos_list[fpga_sort_indeces]
    fpga_peak_mag_list = fpga_peak_mag_list[fpga_sort_indeces]

    ref_peak_pos_list = zeros(frames_to_average)
    ref_peak_mag_list = zeros(frames_to_average)
    
    ref_mag_samples_temp = ref_mag_samples.copy()
    ref_mag_samples_temp[ref_peak_pos] = 0
    ref_peak_pos_list[0] = ref_peak_pos
    ref_peak_mag_list[0] = sqrt(ref_peak_pwr)
    
    for n in range(1, frames_to_average) :
        ref_peak_pos_temp = argmax(ref_mag_samples_temp)
        ref_peak_peak_mag = abs(ref_mag_samples_temp[ref_peak_pos_temp])
        ref_mag_samples_temp[ref_peak_pos_temp] = 0
        ref_peak_pos_list[n] = ref_peak_pos_temp
        ref_peak_mag_list[n] = ref_peak_peak_mag

    ref_sort_indeces = argsort(ref_peak_pos_list)
    ref_peak_pos_list = ref_peak_pos_list[ref_sort_indeces]
    ref_peak_mag_list = ref_peak_mag_list[ref_sort_indeces]

    #
    #    Determine if the test passed.
    #
    if (    all(fpga_peak_pos_list == ref_peak_pos_list) \
        and all(fpga_peak_mag_list >= (0.975*ref_peak_mag_list)) \
        and all(fpga_peak_mag_list <= (1.025*ref_peak_mag_list))) :    
        test_passed = True
        
    else :
        test_passed = False

    test_description  = "    FPGA pos =" 
    for peak_pos in fpga_peak_pos_list :
        test_description += " %5u" % peak_pos 
    test_description += "\n"
     
    test_description += prefix
    test_description += "    Ref pos  =" 
    for peak_pos in ref_peak_pos_list :
        test_description += " %5u" % peak_pos 
    test_description += "\n"
        
    test_description += prefix
    test_description += "    FPGA mag =" 
    for peak_mag in fpga_peak_mag_list :
        test_description += " %5.0f" % peak_mag 
    test_description += "\n"
     
    test_description += prefix
    test_description += "    Ref mag  =" 
    for peak_mag in ref_peak_mag_list :
        test_description += " %5.0f" % peak_mag 

    #
    #    Plot the results
    #    
    if plot_results :
        show_plots(inp_samples[2*len(ref_ir):], \
                   ref_peak_mag, ref_peak_pos, ref_mag_samples, \
                   sqrt(fpga_peak_pwr), fpga_peak_pos, fpga_mag_samples)
    
    return test_passed, test_description

def main(address, offsets, filter_len, frames_to_average, plot_results = False) :
    """The main function of the test script"""
    
    print ""
    print "    offsets           = [",
    for offset in offsets :
        print " %5u" % offset,
        
    print " ]"
    print "    filter_len        = %4d" % filter_len
    print "    frames_to_average = %d" % frames_to_average
    print ""

    overall_test_pass = True

    # ###############################################################
    # define the impulse response
    filter_delay         = filter_len-1
    ref_ir               = zeros( filter_len, dtype = complex )
    ref_ir[filter_delay] = sqrt(2*filter_len) 
        
    # ###############################################################
    # create the PHY interface instance
    phy = phyint.phyint(address,55555)
    print "    PCB Version  = %s\n" \
          "    FPGA Version = %s\n" \
          "    SW Version   = %s\n" % phy.get_version()

    # ###############################################################
    # initialise the correlator and leave it idle.
    slots_to_average  = 1
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
    # calculate the number of input samples, allowing for the
    # overlap-add implementation which requires an integer number of
    # blocks (1 block = filter_len samples)
    num_input_samples      = frames_to_average*samplesPerFrame+2*filter_len
    num_fpga_input_samples = int(  (frames_to_average-1)*samplesPerFrame \
                                 + filter_len*floor((samplesPerFrame+2*filter_len+filter_len-1)/filter_len))

    # ###############################################################
    # generate the test data for the first correlation
    n = 0
    offset = offsets[n]

    impulse_val        = 5811*(1+1j)/sqrt(2)

    even_frame = zeros(num_fpga_input_samples)+1j*zeros(num_fpga_input_samples)
    even_frame[offset+2*filter_len] = impulse_val
    for k in range(1, frames_to_average) :
        impulse1_pos = (offset+2*filter_len+k*(samplesPerFrame+   0)) % num_input_samples
        impulse2_pos = (offset+2*filter_len+k*(samplesPerFrame+1000)) % num_input_samples
        even_frame[impulse1_pos] = impulse_val
        even_frame[impulse2_pos] = impulse_val

    cin.set(even_frame, check_io)

    # ###############################################################
    # generate the test data for the second correlation
    n = n+1
    if len(offsets) > 1 :
        offset = offsets[n]

        odd_frame = zeros(num_fpga_input_samples)+1j*zeros(num_fpga_input_samples)
        odd_frame[offset+2*filter_len] = impulse_val
        for k in range(1, frames_to_average) :
            impulse1_pos = (offset+2*filter_len+k*(samplesPerFrame+   0)) % num_input_samples
            impulse2_pos = (offset+2*filter_len+k*(samplesPerFrame+1000)) % num_input_samples
            odd_frame[impulse1_pos] = impulse_val
            odd_frame[impulse2_pos] = impulse_val
    
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
                    analyse(even_frame[:num_input_samples], ref_ir, frames_to_average, \
                            fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
                            plot_results, prefix = "        ")
                
            else :
                test_passed, test_description = \
                    analyse(odd_frame[:num_input_samples], ref_ir, frames_to_average, \
                            fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
                            plot_results, prefix = "        ")

            if test_passed:
                print "    %2u) %s Pass" % (n-2, test_description)
                
            else:
                print "    %2u) %s Fail" % (n-2, test_description)
                overall_test_pass = False
            
            # ###########################################################
            # generate the test data
            offset = offsets[n]

            if mod(n, 2) == 0 :
                even_frame = zeros(num_fpga_input_samples)+1j*zeros(num_fpga_input_samples)
                even_frame[offset+2*filter_len] = impulse_val
                for k in range(1, frames_to_average) :
                    impulse1_pos = (offset+2*filter_len+k*(samplesPerFrame+   0)) % num_input_samples
                    impulse2_pos = (offset+2*filter_len+k*(samplesPerFrame+1000)) % num_input_samples
                    even_frame[impulse1_pos] = impulse_val
                    even_frame[impulse2_pos] = impulse_val
    
                cin.set(even_frame, check_io)
                
            else :
                odd_frame = zeros(num_fpga_input_samples)+1j*zeros(num_fpga_input_samples)
                odd_frame[offset+2*filter_len] = impulse_val
                for k in range(1, frames_to_average) :
                    impulse1_pos = (offset+2*filter_len+k*(samplesPerFrame+   0)) % num_input_samples
                    impulse2_pos = (offset+2*filter_len+k*(samplesPerFrame+1000)) % num_input_samples
                    odd_frame[impulse1_pos] = impulse_val
                    odd_frame[impulse2_pos] = impulse_val
    
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
                analyse(even_frame[:num_input_samples], ref_ir, frames_to_average, \
                        fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
                        plot_results, prefix = "        ")
            
        else :
            test_passed, test_description = \
                analyse(odd_frame[:num_input_samples], ref_ir, frames_to_average, \
                        fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
                        plot_results, prefix = "        ")

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
    # get the results for the last correlation
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
            analyse(even_frame[:num_input_samples], ref_ir, frames_to_average, \
                    fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
                    plot_results, prefix = "        ")
        
    else :
        test_passed, test_description = \
            analyse(odd_frame[:num_input_samples], ref_ir, frames_to_average, \
                    fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
                    plot_results, prefix = "        ")

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
    
    offsets           = [0, 19200, 38400, 57600]
    filter_len        = 256
    frames_to_average = 4
    main(address, offsets, filter_len, frames_to_average, plot_results = True)
    
    offsets           = [0, 19200, 38400, 57600]
    filter_len        = 2048
    frames_to_average = 4
    main(address, offsets, filter_len, frames_to_average, plot_results = True)

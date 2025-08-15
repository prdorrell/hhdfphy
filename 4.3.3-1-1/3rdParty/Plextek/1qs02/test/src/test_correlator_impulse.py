"""
-------------------------------------------------------------------------------


-------------------------------------------------------------------------------
  Filename:   test_correlator_impulse.py
  Author(s):  pdm
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the correlator using impulses and simple delay filters

-------------------------------------------------------------------------------
"""

print "loading libs...",

from numpy          import zeros, zeros_like, arange
from numpy          import minimum, maximum, argmax
from numpy          import concatenate
from numpy          import real, imag, angle 
from numpy          import all, equal
from numpy          import mod
from numpy          import sqrt
from numpy          import absolute, floor
from numpy          import pi
from numpy.random   import randint  
from pylab          import show, plot, subplot, xlim, ylim, legend, title, suptitle
from pylab          import figure, get_current_fig_manager

import sys

import corrint
import get_defaults
import memint
import phyint
import sim_mf_sum
print "done.\n"

check_io        = False
zero_output_mem = False

frames_to_average = 1
slots_to_average  = 1

samplesPerChip     = 2
chipsPerSlot       = 2560
slotsPerFrame      = 15 
samplesPerSlot     = samplesPerChip*chipsPerSlot
samplesPerFrame    = samplesPerSlot*slotsPerFrame

def show_plots(inp_peak_mag, inp_peak_pos, inp_samples, \
               ref_peak_mag, ref_peak_pos, ref_mag_samples, \
               fpga_peak_mag, fpga_peak_pos, fpga_samples) :
    """Plot the reference and FPGA results"""
    figure
    get_current_fig_manager().window.setGeometry(20, 40, 1000, 900)

    subplot(3, 1, 1)
    
    plot( real( inp_samples ), label="orig data re" )
    plot( imag( inp_samples ), label="orig data im" )
    xlim(xmin = -10000)
    _, ymax = ylim()
    ylim(ymin = -ymax)
    legend(loc="best")
    title("Input, impulse of %d at %d" % (inp_peak_mag, inp_peak_pos))

    subplot(3, 1, 2)
    
    plot(ref_mag_samples)
    xlim(xmin = -10000)
    _, ymax = ylim()
    ylim(ymin = -ymax)
    title("Reference, peak of %.1f at %d" % (ref_peak_mag, ref_peak_pos))
    
    subplot(3, 1, 3)
    plot( real(fpga_samples), label="result re" )
    plot( imag(fpga_samples), label="result im" )
    xlim(xmin = -10000)
    _, ymax = ylim()
    ylim(ymin = -ymax)
    legend(loc="best")
    title("Output, impulse of %d at %d" % (fpga_peak_mag, fpga_peak_pos))
    
    suptitle(  "Input at %d, output at %d, error = %d" \
             % ( inp_peak_pos, fpga_peak_pos, fpga_peak_pos-inp_peak_pos ))
    
    show()

def analyse(inp_samples, ref_ir, \
            fpga_samples, fpga_peak_pos, fpga_peak_pwr, \
            plot_results = False) :
    """Analyse the results"""
    
    #
    #    Get the ideal position of the impulse from the input data, 
    #    ignoring the lead-in.
    #
    inp_peak_pos = argmax(absolute(inp_samples[2*len(ref_ir):]))
    inp_peak_mag = max(absolute(inp_samples[2*len(ref_ir):]))

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
        show_plots(inp_peak_mag, inp_peak_pos, inp_samples[2*len(ref_ir):], \
                   ref_peak_mag, ref_peak_pos, ref_mag_samples, \
                   sqrt(fpga_peak_pwr), fpga_peak_pos, fpga_samples)
    
    return test_passed, test_description

def main(address, offsets, filter_len, filter_delay, plot_results = False) :
    """The main function of the test script"""
    
    print ""
    print "    offsets       = [",
    for offset in offsets :
        print " %5u" % offset,
        
    print " ]"
    print "    filter length = %4u" % filter_len
    print "    filter delay  = %4u" % filter_delay
    print ""

    overall_test_pass = True
    
    # ###############################################################
    # define the magnitude of the input impulse
    impulse_val = 7500
            
    # ###############################################################
    # define the impulse response
    ref_ir               = zeros( filter_len, dtype = complex )
    ref_ir[filter_delay] = sqrt(2*filter_len)

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
    # Write correlation coefficients.  For the simple delay filter it
    # is best to scale the normalise the coefficients to their
    # maximum magnitude.  
    filter_num = 0
    scale_by_fft_size = True
    cin.define_filter( filter_num, ref_ir, scale_by_fft_size, check_io )

    # ###############################################################
    # generate the test data for the first correlation.
    n = 0
    offset = offsets[n]

    even_frame = zeros(num_fpga_input_samples)+1j*zeros(num_fpga_input_samples)
    even_frame[offset+2*filter_len] = impulse_val

    cin.set(even_frame, check_io)

    # ###############################################################
    # generate the test data for the second correlation.
    n = n+1
    if len(offsets) > 1 :
        offset = offsets[n]

        odd_frame = zeros(num_fpga_input_samples)+1j*zeros(num_fpga_input_samples)
        odd_frame[offset+2*filter_len] = impulse_val
    
        cin.set(odd_frame, check_io)
    
        # ###############################################################
        # start the correlator
        cin.run()
    
        # ###############################################################
        # iterate through the input offsets
        for n in range(2, len(offsets)) :
            # ###########################################################
            # get the results
            fpga_samples, _, fpga_peak_pos, fpga_peak_pwr, _ = cin.get(samplesPerFrame)
    
            # ###########################################################
            # analyse the results
            if mod(n, 2) == 0 :
                test_passed, test_description = \
                    analyse(even_frame[:num_input_samples], ref_ir, \
                            fpga_samples, fpga_peak_pos, fpga_peak_pwr, \
                            plot_results)
                
            else :
                test_passed, test_description = \
                    analyse(odd_frame[:num_input_samples], ref_ir, \
                            fpga_samples, fpga_peak_pos, fpga_peak_pwr, \
                            plot_results)

            if test_passed:
                print "    %2u) %s Pass" % (n-2, test_description)
                
            else:
                print "    %2u) %s Fail" % (n-2, test_description)
                overall_test_pass = False
            
            # ###########################################################
            # generate the test data, padding with random values to show
            # alignment problems.
            offset = offsets[n]

            if mod(n, 2) == 0 :
                even_frame = zeros(num_fpga_input_samples)+1j*zeros(num_fpga_input_samples)
                even_frame[offset+2*filter_len] = impulse_val
    
                cin.set(even_frame, check_io)
                
            else :
                odd_frame = zeros(num_fpga_input_samples)+1j*zeros(num_fpga_input_samples)
                odd_frame[offset+2*filter_len] = impulse_val
    
                cin.set(odd_frame, check_io)
    
        # ###########################################################
        # get the results for the penultimate correlation
        n = n+1
        fpga_samples, _, fpga_peak_pos, fpga_peak_pwr, _ = cin.get(samplesPerFrame)
    
        # ###########################################################
        # analyse the results
        if mod(n, 2) == 0 :
            test_passed, test_description = \
                analyse(even_frame[:num_input_samples], ref_ir, \
                        fpga_samples, fpga_peak_pos, fpga_peak_pwr, \
                        plot_results)
            
        else :
            test_passed, test_description = \
                analyse(odd_frame[:num_input_samples], ref_ir, \
                        fpga_samples, fpga_peak_pos, fpga_peak_pwr, \
                        plot_results)

        if test_passed:
            print "    %2u) %s Pass" % (n-2, test_description)
            
        else:
            print "    %2u) %s Fail" % (n-2, test_description)
            overall_test_pass = False

    else :
        # ###############################################################
        # start the correlator
        cin.run()

    # ###########################################################
    # get the results for the last correlation
    n = n+1
    fpga_samples, _, fpga_peak_pos, fpga_peak_pwr, _ = cin.get(samplesPerFrame)

    # ###########################################################
    # analyse the results
    if mod(n, 2) == 0 :
        test_passed, test_description = \
            analyse(even_frame[:num_input_samples], ref_ir, \
                    fpga_samples, fpga_peak_pos, fpga_peak_pwr, \
                    plot_results)
        
    else :
        test_passed, test_description = \
            analyse(odd_frame[:num_input_samples], ref_ir, \
                    fpga_samples, fpga_peak_pos, fpga_peak_pwr, \
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
    
    offsets      = [0, 38400, 76799]
    filter_len   = 256
    filter_delay = 0
    main(address, offsets, filter_len, filter_delay, plot_results = True)
    
    offsets      = [0, 38400, 76799]
    filter_len   = 2048
    filter_delay = 0
    main(address, offsets, filter_len, filter_delay, plot_results = True)

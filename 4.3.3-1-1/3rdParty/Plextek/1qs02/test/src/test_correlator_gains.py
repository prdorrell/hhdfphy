"""
-------------------------------------------------------------------------------


-------------------------------------------------------------------------------
  Filename:   test_correlator_gains.py
  Author(s):  pdm
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the correlator gains using the uplink filters and signals

-------------------------------------------------------------------------------
"""

print "loading libs...",

from numpy      import zeros, zeros_like, arange
from numpy      import minimum, maximum, argmax
from numpy      import concatenate, fliplr, flipud
from numpy      import real, imag, angle, conj 
from numpy      import all, equal
from numpy      import mod
from numpy      import sqrt, mean
from numpy      import absolute, floor
from numpy      import pi
from numpy      import array
from pylab      import show, plot, subplot, xlim, ylim, legend, title, suptitle
from pylab      import figure, get_current_fig_manager
from scipy      import signal

import sys

import corrint
import gencodes
import get_defaults
import memint
import phyint
import source
import sim_mf_sum
print "done.\n"

check_io        = False
zero_output_mem = False

samplesPerChip     = 2
chipsPerSlot       = 2560
slotsPerFrame      = 15 
samplesPerSlot     = samplesPerChip*chipsPerSlot
samplesPerFrame    = samplesPerSlot*slotsPerFrame

frames_to_average = 1
slots_to_average  = 1

offset_samples = 10000

def main(address, signal_power, plot_results = False) :
    """The main function of the test script"""
    
    # ###############################################################
    # define the signal types
    scrambling_code = 0
    pilots_per_slot = 8
    
    srcs = []
    srcs.append(("Uplink DPCCH", source.uplink_dpcch_source(scrambling_code, pilots_per_slot)))
    srcs.append(("PSCH", source.psch_source()))
    srcs.append(("P-CPICH", source.downlink_cpich_source(scrambling_code, antenna=1)))
    
    print ""
    print "    signal_power = %5.0f^2" % sqrt(signal_power)
    print "    sources      = [",
    for src in srcs :
        print " \"%s\"" % src[0],
        
    print " ]"
    print ""

    overall_test_pass = True
    
    # ###############################################################
    # create the PHY interface instance
    phy = phyint.phyint(address,55555)
    print "    PCB Version  = %s\n" \
          "    FPGA Version = %s\n" \
          "    SW Version   = %s\n" % phy.get_version()
    
    # ###############################################################
    # analyse each source in turn
    samplesPerChip  = 2
    scrambling_code = 0
    pilots_per_slot = 8

    for n in range(len(srcs)) :
        src_def = srcs[n]
        name = src_def[0]
        src  = src_def[1]
        
        # ###############################################################
        # define the impulse response
        ref_ir = src.get_fir_coefs(0)

        sig_src = source.composite_source(samplesPerChip, (src,))
        
        filter_len = len(ref_ir)
        
        # ###############################################################
        # calculate the number of input samples, allowing for the
        # overlap-add implementation which requires an integer number of
        # blocks (1 block = filter_len samples)
        num_input_samples      = frames_to_average*samplesPerFrame+2*filter_len
        num_fpga_input_samples = int(  (frames_to_average-1)*samplesPerFrame \
                                     + filter_len*floor((samplesPerFrame+2*filter_len+filter_len-1)/filter_len))
    
        # ###############################################################
        # initialise the correlator and leave it idle.
        cin = corrint.correlation_interface(phy, \
                                            filter_len, \
                                            frames_to_average, slots_to_average, \
                                            check_io, zero_output_mem)
        
        # ###############################################################
        # configure the correlator gain schedule
        #
        #    The LS 16 bits control the forward FFT and the MS 16 bits
        #    control the inverse FFT.
        #
        #    The schedule is split into 2-bit shift values that are
        #    applied after a pair of radix-2 stages.  i.e. bits 0 and 1
        #    represent a shift applied after the second stage of the 
        #    forward FFT.
        #
        #    The gain of an N-point FFT/IFFT pair always N.  If the
        #    signal is noise-like then this gain is equally distributed 
        #    between the forward and inverse FFTs.  However the
        #    application of the coefficient changes the signal for the
        #    inverse FFT and in the ideal case.
        #
        #    (Note that the application of the gain in the FPGA currently
        #    introduces a factor of 0.5, but if the software can
        #    guarantee that the coefficients do not include -128 this can
        #    be removed.)
        #
        if filter_len == 256 :
            fwd_fft_gains =   (3 <<  0)  \
                            | (0 <<  2)
            inv_fft_gains =   (2 <<  0)  \
                            | (1 <<  2)  \
                            | (1 <<  4)  \
                            | (1 <<  6)  \
                            | (1 <<  8)  \
                            | (0 << 10)
                            
        else :
            fwd_fft_gains =   (3 <<  0)  \
                            | (2 <<  2)
            inv_fft_gains =   (2 <<  0)  \
                            | (1 <<  2)  \
                            | (1 <<  4)  \
                            | (1 <<  6)  \
                            | (1 <<  8)  \
                            | (2 << 10)
                            
        cin.set_gain_schedule((inv_fft_gains << 16) | fwd_fft_gains)
    
        # ###############################################################
        # write correlation coefficents
        filter_num = 0
        scale_by_fft_size = True
        cin.define_filter( filter_num, ref_ir, scale_by_fft_size, check_io )
    
        # ###############################################################
        # generate the test data for the first correlation
        frame_ref = sqrt(signal_power)*sig_src.get_frame(frames_to_average+1, \
                                                         offset_samples, \
                                                         noise_pwr = 0, \
                                                         ghosts = False)
        
        frame = frame_ref[:num_fpga_input_samples]
        cin.set(frame, check_io)
    
        # ###############################################################
        # start the correlator
        cin.run()
    
        # ###########################################################
        # get the results for last correlation
        fpga_samples, _, fpga_peak_pos, fpga_peak_pwr, _ = cin.get(samplesPerFrame)

        # ###########################################################
        # calculate the threshold
        if filter_len == 256 :
            coherent_gain    = filter_len/64
            threshold_factor = 10**(-12.1/20.0) 
            
        else:
            coherent_gain = sqrt(2)*filter_len/512
            threshold_factor = 10**(-21.1/20.0) 
            
        expected_peak = coherent_gain*sqrt(signal_power)
        threshold     = threshold_factor*expected_peak

        # ###########################################################
        # calculate the reference results.
        accumulate_by_frame = True
        ref_pwr_samples, ref_peak_pos, ref_peak_pwr = \
            sim_mf_sum.fir_pwr(ref_ir, frame[:num_input_samples], accumulate_by_frame)
        
        ref_peak_mag    = sqrt(ref_peak_pwr)
        
        # ###########################################################
        # allow the PSCH peak position to be in any slot
        if name == "PSCH":
            fpga_peak_pos = mod(fpga_peak_pos, samplesPerSlot)
            ref_peak_pos  = mod(ref_peak_pos,  samplesPerSlot)

        # ###########################################################
        # determine if the test passed.
        if (    (fpga_peak_pos == ref_peak_pos) \
            and (sqrt(fpga_peak_pwr) >= (0.975*ref_peak_mag)) \
            and (sqrt(fpga_peak_pwr) <= (1.025*ref_peak_mag))) :    
            test_passed = True
            
        else :
            test_passed = False
             
        test_description = "    FPGA pos = %5u, Ref pos = %5u, FPGA mag = %5.0f, Ref mag = %5.0f" \
                           % (fpga_peak_pos, ref_peak_pos, sqrt(fpga_peak_pwr), ref_peak_mag) 

        if test_passed:
            print "    %2u) %s Pass" % (n, test_description)
            
        else:
            print "    %2u) %s Fail" % (n, test_description)
            overall_test_pass = False
            
        # ###########################################################
        # plot the results
        if plot_results :
            figure()
            get_current_fig_manager().window.setGeometry(20, 40, 1000, 900)
            
            subplot(3, 1, 1)
            plot(real( frame ), label="real")
            plot(imag( frame ), label="imag")
            mean_input_pwr = mean(abs(frame)**2)
            max_input_component = max(max(abs(real(frame))), max(abs(imag(frame))))
            title(  "Input components, mean power = %.0f (%.0f^2), peak component = %.0f" \
                  % (mean_input_pwr, sqrt(mean_input_pwr), max_input_component))
            legend(loc="best")
            
            subplot(3, 1, 2)
            plot(real(fpga_samples), label="real")
            plot(imag(fpga_samples), label="imag")
            mean_output_pwr      = mean(abs(fpga_samples)**2)
            max_output_component = max(max(abs(real(fpga_samples[:20000]))), max(abs(imag(fpga_samples[:20000]))))
            max_output_magnitude = max(abs(fpga_samples[:20000]))
            title(  "FPGA output components, mean power = %.0f (%.0f^2), peak component = %.0f" \
                  % (mean_output_pwr, sqrt(mean_output_pwr), max_output_component))
            legend(loc="best")
            xlim(offset_samples-20, offset_samples+20)
            
            subplot(3, 1, 3)
            plot(abs(fpga_samples),     label="FPGA")
            plot(sqrt(ref_pwr_samples), label="Ref.")
            plot([0, len(fpga_samples)-1], [expected_peak, expected_peak])
            plot([0, len(fpga_samples)-1], [threshold,     threshold])
            title(  "Output magnitude, mean power = %.0f (%.0f^2), peak magnitude = %.0f" \
                  % (mean_output_pwr, sqrt(mean_output_pwr), max_output_magnitude))
            legend(loc="best")
            xlim(offset_samples-20, offset_samples+20)
            
            suptitle(  "%s : Power gain = %f, Magnitude gain = %f" \
                     % (name, \
                        mean_output_pwr/mean_input_pwr, \
                        sqrt(mean_output_pwr/mean_input_pwr)))
            
            show()

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

    signal_power = get_defaults.getDefaultHighSignalPower()
    main(address, signal_power, plot_results = True)

    signal_power = get_defaults.getDefaultMidSignalPower()
    main(address, signal_power, plot_results = True)

    signal_power = get_defaults.getDefaultLowSignalPower()
    main(address, signal_power, plot_results = True)

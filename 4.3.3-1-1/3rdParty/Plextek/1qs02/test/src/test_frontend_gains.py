"""
-------------------------------------------------------------------------------


-------------------------------------------------------------------------------
  Filename:   test_frontend_gains.py
  Author(s):  pdm
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the RF frontend gains

-------------------------------------------------------------------------------
"""
from cProfile import label

print "loading libs...",

from numpy      import zeros, zeros_like, arange
from numpy      import minimum, maximum, argmax
from numpy      import concatenate, flipud, reshape, roll
from numpy      import real, imag, angle, conj, sum, mean 
from numpy      import all, equal
from numpy      import mod
from numpy      import sqrt, log10
from numpy      import absolute
from numpy      import pi
from numpy      import array
from pylab      import figure, show, draw, plot, subplot, xlim, ylim, legend, title, suptitle, xlabel, ylabel
from pylab      import get_current_fig_manager
from scipy      import signal, fft

import sys

import corrint
import gencodes
import get_defaults
import memint
import phyint
import source
print "done.\n"

check_io        = False
zero_output_mem = False

num_frames   = 1
gains        = arange(0, 101, 10)
#gains        = zeros(10)

def main(address) :
    """The main function of the test script"""
    # ###############################################################
    # define the impulse response
    filter_len = 256
    ref_ir = zeros(filter_len)
        
    # ###############################################################
    # create the PHY interface instance
    phy = phyint.phyint(address,55555)

    phy.set_mode("3G");
    _ = phy.wait_for_msg()
    print "    PCB Version  = %s\n" \
          "    FPGA Version = %s\n" \
          "    SW Version   = %s\n" % phy.get_version()
    
    phy.set_vctcxo(1913)

    fpga_rms    = 1.0*zeros_like(gains)            
    fpga_pwr_dB = 1.0*zeros_like(gains)
    
    meas_rms    = 1.0*zeros_like(gains)
    meas_pwr_dB = 1.0*zeros_like(gains)
    
    meas_dc_inph = 1.0*zeros_like(gains)
    meas_dc_quad = 1.0*zeros_like(gains)
    
    meas_ac_inph = 1.0*zeros_like(gains)
    meas_ac_quad = 1.0*zeros_like(gains)

    meas_no_dc_rms    = 1.0*zeros_like(gains)
    meas_no_dc_pwr_dB = 1.0*zeros_like(gains)
    
    for n in range(len(gains)) :
        gain = gains[n]
                
        # ###############################################################
        # setup the frequency and gain.
        phy.debug_switches( band="wcdma_3", ad_input="RXHB2RF", ad_gpio=0xc )
        phy.debug_set_frequency( 1930000000, band="high" )
        phy.debug_set_gain( gain )
    
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
        scale_by_fft_size = True
        cin.define_filter( filter_num, ref_ir, scale_by_fft_size, check_io )
    
        # ###############################################################
        # start the correlator
        print "Running the correlation"
        cin.run(channel_filter_active = True)

        # ###########################################################
        # get the results
        print "Getting the results"
        num_input_samples = num_frames*76800
        num_output_samples = num_frames*76800
        cin.get_with_input(num_input_samples, num_output_samples)
        cin.get_with_input(num_input_samples, num_output_samples)
        inp_samples, out_samples, num_samples_over_threshold, peak_offset, peak_power, frame_number = \
            cin.get_with_input(num_input_samples, num_output_samples)
            
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

        fpga_rms[n]     = sqrt(res1)            
        fpga_pwr_dB[n]  = 10*log10(res1)
        
        meas_rms[n]     = sqrt(mean(real(inp_samples)**2+imag(inp_samples)**2))
        meas_pwr_dB[n]  = 20*log10(meas_rms[n])
        
        meas_dc_inph[n] = mean(real(inp_samples))
        meas_dc_quad[n] = mean(imag(inp_samples))
        
        meas_ac_inph[n] = sqrt(mean((real(inp_samples)-meas_dc_inph[n])**2))
        meas_ac_quad[n] = sqrt(mean((imag(inp_samples)-meas_dc_quad[n])**2))

        meas_no_dc_rms[n]    = sqrt(mean(  (real(inp_samples)-meas_dc_inph[n])**2 \
                                         + (imag(inp_samples)-meas_dc_quad[n])**2))
        meas_no_dc_pwr_dB[n] = 20*log10(meas_no_dc_rms[n])
        
        # ###########################################################
        # plot the results
        figure()
        get_current_fig_manager().window.setGeometry(20, 40, 600, 800)

        subplot(3, 1, 1)
        plot(absolute(inp_samples))
        xlabel("Time (samples)")
        ylabel("Magnitude (LSB)")
        
        subplot(3, 1, 2)
        plot(real(inp_samples), label="Real")
        plot(imag(inp_samples), label="Imag")
        legend()
        xlabel("Time (samples)")
        ylabel("Real/Imag (LSB)")
        
        subplot(3, 1, 3)
        spectrum = 20*log10(abs(fft(inp_samples)))
        spectrum = roll(spectrum, len(spectrum)/2)
        freqs_Hz = 7.68*arange(-len(spectrum)/2, +len(spectrum)/2, 1)/len(spectrum)
        plot(freqs_Hz, spectrum)
        xlabel("Frequency (Hz)")
        ylabel("dB")
        
        suptitle("Gain = %.0f dB, FPGA RSSI = %.1f dB, Meas. RSSI = %.1f dB" % (gain, fpga_pwr_dB[n], meas_pwr_dB[n]))
        
        show()

    # ###########################################################
    # plot the final results
    figure()
    get_current_fig_manager().window.setGeometry(20, 40, 600, 800)

    subplot(3, 1, 1)
    plot( gains, fpga_pwr_dB,       label="FPGA RSSI (dB)" )
    plot( gains, meas_pwr_dB,       label="Meas RSSI (dB)" )
    plot( gains, meas_no_dc_pwr_dB, label="Meas RSSI (dB, w/o DC)" )

    legend(loc="upper left")
    xlabel("Gain Setting (dB)")
    ylabel("RSSI (dB)")

    subplot(3, 1, 2)
    plot( gains, meas_dc_inph, label="Inphase" )
    plot( gains, meas_dc_quad, label="Quadrature" )
    legend()
    xlabel("Gain Setting (dB)")
    ylabel("Average DC Offset")

    subplot(3, 1, 3)
    plot( gains, meas_ac_inph, label="Inphase" )
    plot( gains, meas_ac_quad, label="Quadrature" )
    legend()
    xlabel("Gain Setting (dB)")
    ylabel("RMS AC Level")
    
    show()
    
    for n in range(len(meas_rms)) :
        print meas_rms[n]/fpga_rms[n]

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

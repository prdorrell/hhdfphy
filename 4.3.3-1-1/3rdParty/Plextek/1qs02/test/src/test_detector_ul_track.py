"""
-------------------------------------------------------------------------------


-------------------------------------------------------------------------------
  Filename:   test_detector_ul_track.py
  Author(s):  pdm
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the channel filter and correlator using the uplink filters

-------------------------------------------------------------------------------
"""

print "loading libs...",

from numpy      import zeros, zeros_like, arange
from numpy      import amin, amax, argmax
from numpy      import concatenate, flipud, reshape
from numpy      import real, imag, angle, conj, sum, mean 
from numpy      import all, equal
from numpy      import mod
from numpy      import sqrt, log10
from numpy      import absolute
from numpy      import pi
from numpy      import array
from numpy      import roll, vstack
from pylab      import figure, show, draw, plot, subplot, xlim, ylim, legend, title, suptitle
from pylab      import get_current_fig_manager
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

samplesPerChip     = 2
chipsPerSlot       = 2560
slotsPerFrame      = 15 
samplesPerSlot     = samplesPerChip*chipsPerSlot
samplesPerFrame    = samplesPerSlot*slotsPerFrame

frames_to_average = 2
slots_to_average  = 15

num_iters = 10

def show_plots(inp_samples, ref_rssi, fpga_rssi, \
               fpga_peak_mag, fpga_peak_pos, fpga_mag_samples, \
               expected_peak, threshold) :
    """Plot the reference and FPGA results"""

    # ###########################################################
    # plot the results
    figure()
    get_current_fig_manager().window.setGeometry(20, 40, 600, 800)

    subplot(2, 1, 1)
    time = arange(0.0, inp_samples.size)/samplesPerFrame
    plot(time, absolute(inp_samples))
    title(  "FPGA: RSSI = %.1f dB (rms = %.0f), Ref.: RSSI = %.1f dB (rms = %.0f)" \
          % (20*log10(fpga_rssi), fpga_rssi, 20*log10(fpga_rssi), fpga_rssi))

    subplot(2, 1, 2)
    time = arange(0.0, fpga_mag_samples.size)/samplesPerFrame
    time.shape = fpga_mag_samples.shape 
    plot(time, 20*log10(fpga_mag_samples))
    title("Output, impulse of %d at %d" % (fpga_peak_mag, fpga_peak_pos))
    plot([amin(time), amax(time)], [20*log10(expected_peak), 20*log10(expected_peak)])
    plot([amin(time), amax(time)], [20*log10(threshold),     20*log10(threshold)])
    
    show()

def analyse(inp_samples, ref_rssi, fpga_rssi, ref_ir, \
            fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
            expected_peak, threshold, \
            plot_results = False) :
    """Analyse the FPGA results"""

    #
    #    Plot the results
    #    
    if plot_results :
        show_plots(inp_samples, ref_rssi, fpga_rssi, \
                   sqrt(fpga_peak_pwr), fpga_peak_pos, fpga_mag_samples, \
                   expected_peak, threshold)
    
    return

def main(address, plot_results = False) :
    """The main function of the test script"""
    
    # ###############################################################
    # create the signal source
    scrambling_code = 0
    pilots_per_slot = 8
    ref_source = source.uplink_dpcch_source(scrambling_code, pilots_per_slot)
    
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
    # create the PHY interface instance
    phy = phyint.phyint(address,55555)
    
    # ###############################################################
    # force the 3G mode
    phy.set_mode("3G");
    _ = phy.wait_for_msg()
    print "    PCB Version  = %s\n" \
          "    FPGA Version = %s\n" \
          "    SW Version   = %s\n" % phy.get_version()
        
    # ###############################################################
    # setup the frequency and gain.
    phy.debug_switches( band="wcdma_3", ad_input="RXHB2RF", ad_gpio=0xc )
    phy.debug_set_frequency( 1930000000, band="high" )
    phy.debug_set_gain( 78 )
        
    phy.set_vctcxo(1929)
        
    # #####################################################################
    # zero the input memory
    print ""
    print "Zeroing the input memory"
    fill = zeros((frames_to_average+4)*76800)+1j*zeros((frames_to_average+4)*76800)
    print "    The even buffer"
    sample_input_even_base = 0x00000000
    memint.write_samples( phy, \
                          fill, \
                          sample_input_even_base, \
                          False )
    print "    The odd buffer"
    sample_input_odd_base  = 0x00400000
    memint.write_samples( phy, \
                          fill, \
                          sample_input_odd_base, \
                          False  )

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
    # start the correlator
    channel_filter_active = True
    cin.run_track(channel_filter_active, power_results)

    # ###############################################################
    # iterate through the input offsets
    for n in range(num_iters) :
        # ###########################################################
        # get the results
        num_input_samples  = (frames_to_average+4)*samplesPerFrame
        num_output_samples = samplesPerFrame
        inp_samples, out_samples, fpga_num_samples_over_threshold, fpga_peak_pos, fpga_peak_pwr, _ = \
            cin.get_with_input(num_input_samples, num_output_samples)
        
        if power_results :
            fpga_mag_samples = sqrt(out_samples)
            
        else :
            fpga_mag_samples = absolute(out_samples)
            
        fpga_pwr_samples = fpga_mag_samples**2

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

        fpga_rssi = sqrt(res1*262144.0/samplesPerFrame)            
        
        ref_rssi  = sqrt(mean(abs(inp_samples)**2))
        
        print "RSSI power results: FPGA = %10.0f, Measured = %10.0f, Ratio = %6.3f" \
              % (fpga_rssi**2, ref_rssi**2, fpga_rssi**2/ref_rssi**2)
        
        # ###########################################################
        # calculate the threshold
        if filter_len == 256 :
            coherent_gain    = filter_len/64
            if (frames_to_average*slots_to_average == 1*1) :
                    threshold_factor = 10**(-12.1/20.0) 
                     
            elif (frames_to_average*slots_to_average == 1*15) :
                if filter_len == 256 :
                    threshold_factor = 10**(-19.0/20.0) 
                    
            else :
                raise Exception("Threshold factor is unknown")
            
        else:
            coherent_gain    = sqrt(2)*filter_len/512
            if (frames_to_average*slots_to_average == 1*1) :
                    threshold_factor = 10**(-21.1/20.0)
                     
            elif (frames_to_average*slots_to_average == 3*15) :
                    threshold_factor = 10**((-30.3-4.0)/20.0) 
                     
            elif (frames_to_average*slots_to_average == 2*15) :
                    threshold_factor = 10**((-30.3-2.0)/20.0) 
                    
            else :
                raise Exception("Threshold factor is unknown")
            
        expected_peak = sqrt(frames_to_average*slots_to_average)*coherent_gain*ref_rssi
        threshold     = threshold_factor*expected_peak
        
        print "expected peak = %f, threshold = %f\n" % (expected_peak, threshold)
        
        pwr_threshold = threshold*threshold
        cin.set_threshold(pwr_threshold)
        
        # ###########################################################
        # check the threshold and peak detection
        meas_num_samples_over_threshold = sum(fpga_mag_samples > threshold)
        meas_peak_offset                = argmax(fpga_mag_samples)
        meas_peak_power                 = fpga_pwr_samples[meas_peak_offset]
        
        print "FPGA: number samples = %d, peak offset = %d, peak data = %d" \
              %(fpga_num_samples_over_threshold, fpga_peak_pos, fpga_peak_pwr)           
        print "Meas: number samples = %d, peak offset = %d, peak data = %d" \
              %(meas_num_samples_over_threshold, meas_peak_offset, meas_peak_power)
                         
        # ###########################################################
        # analyse the results
        analyse(inp_samples, ref_rssi, fpga_rssi, ref_ir, \
                fpga_mag_samples, fpga_peak_pos, fpga_peak_pwr, \
                expected_peak, threshold, \
                plot_results)
        
        # ###########################################################
        # generate the test data
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
    
    main(address, plot_results = True)

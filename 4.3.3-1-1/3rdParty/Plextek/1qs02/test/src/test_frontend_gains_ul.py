"""
-------------------------------------------------------------------------------


-------------------------------------------------------------------------------
  Filename:   test_frontend_gains_ul.py
  Author(s):  pdm
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the frontend gains using the channel filter and correlator configured with the uplink filters

-------------------------------------------------------------------------------
"""

print "loading libs...",

from numpy      import zeros, zeros_like, arange
from numpy      import minimum, maximum, argmax
from numpy      import concatenate, flipud, reshape
from numpy      import real, imag, angle, conj, sum, mean 
from numpy      import all, equal
from numpy      import mod
from numpy      import sqrt, log10
from numpy      import absolute
from numpy      import pi
from numpy      import array
from numpy      import float64
from pylab      import figure, show, draw, plot, subplot, xlim, ylim, legend, title, suptitle
from pylab      import get_current_fig_manager
from scipy      import signal

import sys

import corrint
import get_defaults
import gencodes
import memint
import phyint
print "done.\n"

check_io        = False
zero_output_mem = False
power_results   = True

chipsPerSlot       = 2560
slotsPerFrame      = 15
 
chanSamplesPerChip  = 2
chanSamplesPerSlot  = chanSamplesPerChip*chipsPerSlot
chanSamplesPerFrame = chanSamplesPerSlot*slotsPerFrame
 
adcSamplesPerChip   = 5
adcSamplesPerSlot   = adcSamplesPerChip*chipsPerSlot
adcSamplesPerFrame  = adcSamplesPerSlot*slotsPerFrame

num_frames   = 1
filter_len   = 2048

gains = arange(0, 101, 1)

def main(address) :
    """The main function of the test script"""
    # ###############################################################
    # define the impulse response
    samplesPerChip  = 2
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
    
    # ###############################################################
    # force the 3G mode
    phy.set_mode("3G");
    _ = phy.wait_for_msg()
    print "    PCB Version  = %s\n" \
          "    FPGA Version = %s\n" \
          "    SW Version   = %s\n" % phy.get_version()
        
    # ###############################################################
    # setup the frequency.
    phy.debug_switches( band="wcdma_3", ad_input="RXHB2RF", ad_gpio=0xc )
    phy.debug_set_frequency( 1930000000, band="high" )
    phy.debug_set_gain( 78 )
    
    phy.set_vctcxo(1913)
    phy.set_vctcxo(1865)

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
    channel_filter_active = True
    cin.run(channel_filter_active, power_results)

    # ###############################################################
    # iterate through the gains
    fpga_adc_rssi_pwr_dB  = zeros(len(gains))
    fpga_chan_rssi_pwr_dB = zeros(len(gains))
    fpga_peak_pwr_dB      = zeros(len(gains))
    meas_rssi_pwr_dB      = zeros(len(gains))
    meas_peak_pwr_dB      = zeros(len(gains))
    for n in range(len(gains)) :
        # ###############################################################
        # setup the gain.
        phy.debug_set_gain(gains[n])
        
        # ###########################################################
        # discard two sets of results
        num_input_samples = num_frames*76800
        num_output_samples = num_frames*76800
        cin.get(1)
        cin.switch_buffers()
        cin.get(1)
        cin.switch_buffers()
        
        # ###########################################################
        # get the results
        inp_samples, out_samples, num_samples_over_threshold, fpga_peak_offset, fpga_peak_power, frame_number = \
            cin.get_with_input(num_input_samples, num_output_samples)
        
        if power_results :
            magnitude_samples = sqrt(out_samples)
            
        else :
            magnitude_samples = absolute(out_samples)
            
        power_samples = magnitude_samples**2
        
        fpga_peak_pwr_dB[n] = 10.0*log10(fpga_peak_power)

        # ###########################################################
        # read the RSSI result until the same result is received 
        # twice (handles firmware feature in which writes to the
        # result register may be incomplete when a read occurs). 
        res1 = phy.debug_reg_read(22)
        res2 = phy.debug_reg_read(22)
        timeout = 10
        while res1 != res2 :
            timeout = timeout-1
            if timeout < 0 :
                raise Exception("RSSI not stable")
     
            res1 = res2
            res2 = phy.debug_reg_read(22)

        fpga_adc_rssi_pwr       = res1*512.0/adcSamplesPerFrame
        fpga_adc_rssi_pwr_dB[n] = 10*log10(fpga_adc_rssi_pwr)
        
        res1 = phy.debug_reg_read(13)
        res2 = phy.debug_reg_read(13)
        timeout = 10
        while res1 != res2 :
            timeout = timeout-1
            if timeout < 0 :
                raise Exception("RSSI not stable")
     
            res1 = res2
            res2 = phy.debug_reg_read(13)

        fpga_chan_rssi_pwr       = res1*262144.0/chanSamplesPerFrame
        fpga_chan_rssi_pwr_dB[n] = 10*log10(fpga_chan_rssi_pwr)
        
        meas_rssi_pwr       = mean(abs(inp_samples)**2)
        meas_rssi_pwr_dB[n] = 10*log10(meas_rssi_pwr)
        
        # ###########################################################
        # start generating the next test data
        cin.switch_buffers()
        
        # ###########################################################
        # check the threshold and peak detection
        meas_peak_offset = argmax(magnitude_samples)
        meas_peak_pwr_dB[n]  = 10.0*log10(power_samples[meas_peak_offset])
        
        print "Gain = %3d dB : FPGA ADC RSSI = %4.1f dB, Channel RSSI = %4.1f dB, Peak of %4.1f dB at %5d : Script RSSI = %4.1f dB, Peak of %4.1f dB at %5d" \
              % (gains[n], \
                 fpga_adc_rssi_pwr_dB[n], fpga_chan_rssi_pwr_dB[n], \
                 fpga_peak_pwr_dB[n], fpga_peak_offset, \
                 meas_rssi_pwr_dB[n], meas_peak_pwr_dB[n], meas_peak_offset)

#        # ###########################################################
#        # plot the results
#        figure()
#        get_current_fig_manager().window.setGeometry(20, 40, 600, 800)
#
#        subplot(3, 1, 1)
#        plot( absolute(inp_samples) )
#        title(  "FPGA: RSSI = %.1f dB (rms = %.0f), Meas: RSSI = %.1f dB (rms = %.0f)" \
#              % (fpga_pwr_dB, fpga_rms, meas_pwr_dB, meas_rms))
#
#        subplot(3, 1, 2)
#        b = zeros(samplesPerChip*len(ref_ir))*1j
#        b[::samplesPerChip] = ref_ir
#        out_samples_ref = signal.lfilter(b, 1, inp_samples)
#        peak_pos_ref = argmax(abs(out_samples_ref))
#        peak_val_ref = abs(out_samples_ref[peak_pos_ref])
#        plot(abs(out_samples_ref))
#        title("Peak of %.1f at %d" % (peak_val_ref, peak_pos_ref))
#
#        subplot(3, 1, 3)
#        plot( magnitude_samples )
#        title("Peak of %.1f at %d" % (sqrt(meas_peak_power), meas_peak_offset))
#        
#        show()

    # ###########################################################
    # plot the results
    figure()
    plot(gains, meas_rssi_pwr_dB, label="RSSI")
    plot(gains, meas_peak_pwr_dB, label="RSCP")
    legend(loc="best")
    show()
        
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

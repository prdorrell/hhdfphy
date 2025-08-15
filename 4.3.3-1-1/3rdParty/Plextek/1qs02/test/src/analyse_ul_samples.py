"""
-------------------------------------------------------------------------------


-------------------------------------------------------------------------------
  Filename:   analyse_ul_samples.py
  Author(s):  pdm
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Analyse the samples used for an uplink search 

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
from numpy      import absolute, angle, unwrap
from numpy      import pi
from numpy      import array
from pylab      import figure, show, draw, plot, subplot, xlim, ylim, legend, title, suptitle
from pylab      import get_current_fig_manager
from scipy      import signal, polyfit, io

import sys
import time

import corrint
import gencodes
import get_defaults
import memint
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
    
    phy.set_vctcxo(1916)
        
    # ###############################################################
    # setup the frequency and gain.
    phy.debug_switches( band="wcdma_3", ad_input="RXHB2RF", ad_gpio=0xc )
    phy.debug_set_frequency( 1930000000, band="high" )
    phy.debug_set_gain( 78 )

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
    true_peak_pos_ref_by_frame = []
    for n in range(num_iters) :
        # ###########################################################
        # get the results
        time.sleep(num_frames*0.01+0.1)
        inp_samples, out_samples, num_samples_over_threshold, peak_offset, peak_power, frame_number = \
            cin.get_with_input(num_inp_samples, num_out_samples)

#        for m in range(num_frames*slotsPerFrame/3) :          
#            plot(real(inp_samples[m*3*5120:(m+1)*3*5120]), imag(inp_samples[m*3*5120:(m+1)*3*5120]))
#            show()
            
#        matlab_data = {'inp_samples' : inp_samples}            
#        io.savemat("C:\Documents and Settings\pdm.000\Desktop\Spies\Ames\AnalyseSampling\log2", matlab_data)            
            
#        return
        
        if power_results :
            magnitude_samples = sqrt(out_samples)
            
        else :
            magnitude_samples = absolute(out_samples)
            
        power_samples = magnitude_samples**2
        
        print mean(power_samples)

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

        fpga_rms    = sqrt(res1)            
        fpga_pwr_dB = 20*log10(fpga_rms)
        
        meas_rms    = sqrt(mean(abs(inp_samples)**2))
        meas_pwr_dB = 20*log10(meas_rms)
        
        print "RSSI power results: FPGA = %10.0f, Measured = %10.0f, Ratio = %6.3f" \
              % (fpga_rms**2, meas_rms**2, fpga_rms**2/meas_rms**2)
        
        # ###########################################################
        # let the FPGA generate the test data for the next iteration
        cin.switch_buffers()
        
        # ###########################################################
        # calculate the threshold
        if filter_len == 256 :
            coherent_gain    = filter_len/128
            threshold_factor = 10**(-12.1/20.0) 
            
        else:
            coherent_gain = sqrt(2)*filter_len/1024
            threshold_factor = 10**(-21.1/20.0) 
            
        expected_peak = coherent_gain*meas_rms
        threshold     = threshold_factor*expected_peak
        
        print "expected peak = %f, threshold = %f\n" % (expected_peak, threshold)
        
        pwr_threshold = threshold*threshold
        cin.set_threshold(pwr_threshold)
        
        # ###########################################################
        # check the threshold and peak detection
        meas_num_samples_over_threshold = sum(magnitude_samples > threshold)
        meas_peak_offset                = argmax(magnitude_samples)
        meas_peak_power                 = power_samples[meas_peak_offset]
        
        print "FPGA: number samples = %d, peak offset = %d, peak data = %d" \
              %(num_samples_over_threshold, peak_offset, peak_power)           
        print "Meas: number samples = %d, peak offset = %d, peak data = %d" \
              %(meas_num_samples_over_threshold, meas_peak_offset, meas_peak_power)           

        # ###########################################################
        # apply the reference filter
        b = zeros(samplesPerChip*len(ref_ir))*1j
        b[::samplesPerChip] = ref_ir
        out_samples_ref = signal.lfilter(b, 1, inp_samples)

        # ###########################################################
        # rearrange the reference result into frames and identify the peak in each frame
        samples_ref_by_frame = reshape(out_samples_ref, (-1, samplesPerFrame))
        
        true_peak_pos_ref_by_frame.append(argmax(abs(samples_ref_by_frame), axis=1))
        
        peak_phase_ref_by_frame = unwrap(angle(samples_ref_by_frame[arange(num_frames), \
                                                                    true_peak_pos_ref_by_frame[n]]))
        
        # ###########################################################
        # sum the powers by frames
        frame_pwr_samples_ref = sum(abs(samples_ref_by_frame)**2, axis=0)
        frame_mag_samples_ref = sqrt(frame_pwr_samples_ref)
        
        # ###########################################################
        # find the peak in the summed results and get magnitude and phase of the corresponding sample in each frame 
        peak_pos_ref = argmax(frame_mag_samples_ref)
        peak_val_ref = frame_mag_samples_ref[peak_pos_ref]
        
        peak_samples_ref_by_frame = samples_ref_by_frame[:, peak_pos_ref]
        peak_mag_samples_ref_by_frame = abs(peak_samples_ref_by_frame)
        peak_phase_samples_ref_by_frame = unwrap(angle(peak_samples_ref_by_frame))

        # ###########################################################
        # plot the results
        figure()
        get_current_fig_manager().window.setGeometry(20, 40, 1200, 800)

        subplot(2, 3, 1)
        
        plot(abs(out_samples_ref))
        title("Ideal matched-filter output")

        subplot(2, 3, 4)
        
        num_samples_to_show = min(len(frame_mag_samples_ref), 5*samplesPerFrame)
        plot(frame_mag_samples_ref[:num_samples_to_show])
        title("Ideal summed matched-filter output")

        subplot(2, 3, 2)
        
        plot(peak_mag_samples_ref_by_frame)
        title("Peak magnitude by frame (based on summed peak)")

        subplot(2, 3, 5)
        
        plot(peak_phase_samples_ref_by_frame*180.0/pi)
        title("Peak phase by frame (based on summed peak)")

        subplot(2, 3, 3)

        plot(true_peak_pos_ref_by_frame[n])
        title("True peak position")
        
        subplot(2, 3, 6)
        
        plot(peak_phase_ref_by_frame*180.0/pi)
        title("True peak phase")
        
        (ar, br) = polyfit(0.01*arange(num_frames), peak_phase_ref_by_frame/2.0/pi, 1)
        print ar
        print br

        suptitle("Input: RSSI = %.1f dB (rms = %.0f), Matched Filter: Peak of %.1f at %d" \
                 % (meas_pwr_dB, meas_rms, peak_val_ref, peak_pos_ref))
        
        show()
        
    figure
    for n in range(num_iters) :
        plot(true_peak_pos_ref_by_frame[n], label="%d" % n)
        
    legend()
    title("True peak position for multiple runs")
        
    show()

# ###################################################################
# the main program
if __name__ == '__main__':
    #
    # parse the command-line arguments: if there is one it is the IP
    # address of the server
    address = get_defaults.getDefaultHhdfIpAddress()
    if len(sys.argv) > 1:
        address = sys.argv[1]
    #print "using address:", address
    
    main(address)

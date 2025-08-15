"""
-------------------------------------------------------------------------------


-------------------------------------------------------------------------------
  Filename:   corrint.py
  Author(s):  pdm
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Interface to the FPGA correlator.

-------------------------------------------------------------------------------
"""

from numpy      import array
from numpy      import round, minimum, maximum
from numpy      import concatenate, fliplr, flipud
from numpy      import zeros, zeros_like
from numpy      import real, imag, conj
from numpy      import all, equal
from numpy      import sqrt
from numpy.fft  import fft
import struct

import memint
import phyint


# #############################################################################
# test correlator interface
class correlation_interface:
    def __init__( self, phy, filter_len, frames_to_average, slots_to_average, readback_test, zero_results, power_results = False):
        # #####################################################################
        # register numbers and fields
        
        # #######################################
        # mode control register number
        self.corr_mode_reg              = 0x02
        
        # operational mode field values (bits 0:1)
        self.corr_mode_idle             = 0x000
        self.corr_mode_512              = 0x001
        self.corr_mode_4096_full        = 0x002
        self.corr_mode_4096             = 0x003

        # antenna mode field values (bits 2:3)
        self.antenna_mode_swap12        = 0x000    # only valid with operational mode = corr_mode_4096
        self.antenna_mode_ant1          = 0x004
        self.antenna_mode_ant2          = 0x008
        self.antenna_mode_swap21        = 0x00C    # only valid with operational mode = corr_mode_4096

        # test mode field values (bits 4:5)
        
        # other mode field values (bits 7:9)
        self.corr_mode_corr_en          = 0x080
        self.corr_mode_ch_filt_en       = 0x100
        self.corr_mode_reset            = 0x200
        self.corr_mode_power_results    = 0x400
        
        # #######################################
        # frames-to-average register number
        self.average_ctrl_reg           = 0x03
        
        # #######################################
        # correlation start offset register number
        self.corr_start_offset_reg      = 0x04
        
        # #######################################
        # correlation length register number
        self.corr_len_reg               = 0x05
        
        # #######################################
        # correlation peak threshold register number
        self.corr_peak_thresh_reg       = 0x06
        
        # #######################################
        # flip control register number
        self.even_odd_sync_reg          = 0x07

        # odd/even status set by FPGA (bits 0:1)    
        self.even_buffer_valid          = 0x01
        self.odd_buffer_valid           = 0x02
        
        # odd/even release set by software (bits 2:3)
        self.finished_with_even         = 0x04
        self.finished_with_odd          = 0x08
        
        # FFT overflow (bit 4)
        self.fft_overflow               = 0x10
        
        # #######################################
        # even peak address register number
        self.even_frame_number_reg      = 0x09
        
        # #######################################
        # even peak address register number
        self.even_peak_address_reg      = 0x0A
        
        # #######################################
        # even peak power register number
        self.even_peak_power_reg        = 0x0B
        
        # #######################################
        # even number of peaks register number
        self.even_number_of_peaks_reg   = 0x0C
        
        # #######################################
        # RSSIs register number
        self.rssi_reg                   = 0x0D
        
        # #######################################
        # FSM status register
        self.fsm_status1_reg            = 0x0E
        
        # #######################################
        # gain control register number
        self.gain_ctrl_reg              = 0x10
        
        # #######################################
        # FSM status register
        self.fsm_status2_reg            = 0x11
        
        # #######################################
        # odd peak address register number
        self.odd_frame_number_reg       = 0x12
        
        # #######################################
        # odd peak address register number
        self.odd_peak_address_reg       = 0x13
        
        # #######################################
        # odd peak power register number
        self.odd_peak_power_reg         = 0x14
        
        # #######################################
        # odd number of peaks register number
        self.odd_number_of_peaks_reg    = 0x15
        
        # #######################################
        # ADC RSSI register number
        self.adc_rssi_reg               = 0x16
        
        # #####################################################################
        # base addresses of the coefficient RAM
        self.coef_block_ram_base = 0x00010000
        
        # #####################################################################
        # base addresses of the input and output sample memory
        self.sample_input_even_base = 0x00000000
        self.sample_input_odd_base  = 0x00400000

        self.sample_result_even_base = 0x00800000
        self.sample_result_odd_base  = 0x00c00000

        # #####################################################################
        # the size of the coefficients
        self.coeff_size_bits = 8

        # #####################################################################
        # the selected antenna
        self.antenna = 1
        
        # #####################################################################
        # make local copies of the controls
        self.phy = phy
        
        if (filter_len == 256) or (filter_len == 2048) :
            self.filter_len = filter_len
        else :
            raise Exception("Unsupported filter length = %d" % filter_len)
        self.fft_size   = 2*filter_len
         
        self.readback_test = readback_test
                
        # #####################################################################
        # reset correlator and set mode
        self.phy.debug_reg_write( self.corr_mode_reg, self.corr_mode_reset )
        self.phy.debug_reg_write( self.corr_mode_reg, self.corr_mode_idle )
        
        self.phy.debug_reg_write( self.average_ctrl_reg,        ((slots_to_average-1) << 8) \
                                                              | (frames_to_average-1))
        self.phy.debug_reg_write( self.corr_start_offset_reg, 0)
        self.phy.debug_reg_write( self.corr_len_reg,          0)
        self.phy.debug_reg_write( self.corr_peak_thresh_reg,  0)
        self.phy.debug_reg_write( self.even_odd_sync_reg,     0)
        
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
                            
        self.phy.debug_reg_write( self.gain_ctrl_reg, (inv_fft_gains << 16) | fwd_fft_gains )
        
        # #####################################################################
        # correlator starts using the even buffer
        self.write_samples_to_even  = True
        self.read_samples_from_even = True

        # #####################################################################
        # default to using IQ results
        self.power_results = power_results
        
        # #####################################################################
        # zero the result memory
        if zero_results :
            print ""
            print "Zeroing the result memory"
            fill = zeros(2*76800)+1j*zeros(2*76800)
            print "    The even buffer"
            memint.write_samples( phy, \
                                  fill, \
                                  self.sample_result_even_base, \
                                  self.readback_test )
            print "    The odd buffer"
            memint.write_samples( phy, \
                                  fill, \
                                  self.sample_result_odd_base, \
                                  self.readback_test  )

    def set_gain_schedule( self, gains ):
        # #####################################################################
        # set the gain schedule
        self.phy.debug_reg_write( self.gain_ctrl_reg, gains )

    def set_threshold( self, pwr_threshold ):
        # #####################################################################
        # set the power threshold
        if self.power_results :
            self.phy.debug_reg_write( self.corr_peak_thresh_reg, pwr_threshold/512 )
            
        else :
            self.phy.debug_reg_write( self.corr_peak_thresh_reg, pwr_threshold/2 )

    def set_antenna( self, antenna ):
        # #####################################################################
        # set the antenna to be used
        if (antenna != 1) and (antenna != 2) :
            raise Exception("Invalid antenna number (%u)" % antenna)
        self.antenna = antenna

    def read_coeffs( self, address, num_coeffs ):
        # #####################################################################
        # read coefficient data
        coeffs = []
        self.phy.debug_reg_read( address ) # prime the block ram read (delayed by one read)
        if self.coeff_size_bits == 16 :
            for i in range(num_coeffs):
                i = (i+1) % self.fft_size
                d = self.phy.debug_reg_read( address+i )
                (re, im)=struct.unpack("<hh", struct.pack( "<I", d ) )
                coeffs.append( re + 1j*im )
                
        elif self.coeff_size_bits == 8 :
            for i in range(0, num_coeffs, 2):
                i = (i/2+1) % self.fft_size
                d = self.phy.debug_reg_read( address+i )
                (re0, im0, re1, im1)=struct.unpack("<bbbb", struct.pack( "<I", d ) )
                coeffs.append( re0 + 1j*im0 )
                coeffs.append( re1 + 1j*im1 )
            
        else :
            raise Exception("Unsupported coefficient size =%d" % self.coeff_size_bits)

        coeffs = array(coeffs)

        return coeffs

    def write_coeffs( self, coeffs, block_num, readback_test ):
        # #####################################################################
        # write sample data
        if self.coeff_size_bits == 16 :
            address = self.coef_block_ram_base+block_num*4096
            
        else :
            address = self.coef_block_ram_base+block_num*2048
        
#        coeff_file = open("coeff_file.txt", "w+")
#        for z in coeffs :
#            coeff_file.write("%+6d %+6d\n" % (real(z), imag(z)))
#        coeff_file.close()
        
        i = 0
        if self.coeff_size_bits == 16 :
            for c in coeffs:
                re = int(real(c))
                im = int(imag(c))
                d = struct.unpack( "<I", struct.pack( "<hh", re, im ) )[0]
                self.phy.debug_reg_write( address+i, d )
                i += 1
                
        elif self.coeff_size_bits == 8 :
            for n in range(0, len(coeffs), 2) :
                re0 = int(real(coeffs[n+0]))
                im0 = int(imag(coeffs[n+0]))
                re1 = int(real(coeffs[n+1]))
                im1 = int(imag(coeffs[n+1]))
                d = struct.unpack( "<I", struct.pack( "<bbbb", re0, im0, re1, im1 ) )[0]
                self.phy.debug_reg_write( address+i, d )
                i += 1
            
        else :
            raise Exception("Unsupported coefficient size =%d" % self.coeff_size_bits)
            
        if readback_test :
            # read back check
            readback = self.read_coeffs( address, len(coeffs) )

            if not all( equal( coeffs, readback ) ) :
                raise Exception("Coefficient readback failed (address = 0x%x)" % address)

    def define_filter( self, filter_num, impulse_response, scale_by_fft_size, readback ):
        # #####################################################################
        # write sample data
        if len(impulse_response) != self.filter_len :
            raise Exception("Impulse response has wrong length (%d)" % len(impulse_response))
        
        extended_impulse_response = concatenate( (impulse_response, \
                                                  zeros_like(impulse_response)), \
                                                axis=0)
        freq_response = fft(extended_impulse_response)
        
        # set teh DC and adjacent bins to 0 gain
        freq_response[-1] = 0.0
        freq_response[ 0] = 0.0
        freq_response[ 1] = 0.0
        
        
        max_abs_coeff_val = 2**(self.coeff_size_bits-1)-1.0
        if scale_by_fft_size :
            norm_factor = max_abs_coeff_val/(2.0*sqrt(self.fft_size))
            
        else :
            norm_factor = max_abs_coeff_val/max(abs(freq_response))

        freq_response = norm_factor*freq_response
        freq_response.real = maximum(-max_abs_coeff_val, \
                                     minimum(+max_abs_coeff_val, \
                                             round(freq_response.real)))
        freq_response.imag = maximum(-max_abs_coeff_val, \
                                     minimum(+max_abs_coeff_val, \
                                             round(freq_response.imag)))
        #print freq_response
        
        self.write_coeffs( freq_response, filter_num, readback )
    
    def run( self, channel_filter_active = False, power_results = False ):
        # #####################################################################
        # Let the correlator run
        enable_flag = self.corr_mode_corr_en
        
        if self.filter_len == 256 :
            enable_flag = enable_flag | self.corr_mode_512
        elif self.filter_len == 2048 :
            enable_flag = enable_flag | self.corr_mode_4096_full
            
        if self.antenna == 1 :            
            enable_flag = enable_flag | self.antenna_mode_ant1
            
        else :
            enable_flag = enable_flag | self.antenna_mode_ant2
            
        if channel_filter_active :
            enable_flag = enable_flag | self.corr_mode_ch_filt_en

        self.power_results = power_results
        if self.power_results :
            enable_flag = enable_flag | self.corr_mode_power_results
                    
        self.phy.debug_reg_write( self.corr_mode_reg, enable_flag )
    
    def run_track( self, channel_filter_active = False, power_results = False ):
        # #####################################################################
        # Let the correlator run
        enable_flag = self.corr_mode_corr_en
        
        if self.filter_len == 2048 :
            enable_flag = enable_flag | self.corr_mode_4096
            
        else :
            raise Exception("Unsupported filter length = %d" % self.filter_len)
            
        if self.antenna == 1 :            
            enable_flag = enable_flag | self.antenna_mode_ant1
            
        else :
            enable_flag = enable_flag | self.antenna_mode_ant2
            
        if channel_filter_active :
            enable_flag = enable_flag | self.corr_mode_ch_filt_en

        self.power_results = power_results
        if self.power_results :
            enable_flag = enable_flag | self.corr_mode_power_results
                    
        self.phy.debug_reg_write( self.corr_mode_reg, enable_flag )

    def set_start_offset( self, start_offset ):
        # #####################################################################
        # set the start offset value
        self.phy.debug_reg_write( self.corr_start_offset_reg, start_offset)
    
    def switch_buffers( self ):
        # #####################################################################
        # switch the even/odd buffers
        if self.write_samples_to_even:
            done_flag = self.finished_with_even
        else:
            done_flag = self.finished_with_odd
        
        self.write_samples_to_even = not self.write_samples_to_even
        
        # mark done with even /odd
        self.phy.debug_reg_write( self.even_odd_sync_reg, done_flag )
    
    def set( self, samples, readback ):
        # #####################################################################
        # write new input samples and switch the even/odd buffers
#        sample_file = open("samples_file.txt", "w+")
#        for z in samples :
#            sample_file.write("%+6d %+6d\n" % (real(z), imag(z)))
#        sample_file.close()
        
        if self.write_samples_to_even:
            address   = self.sample_input_even_base
            done_flag = self.finished_with_even
        else:
            address   = self.sample_input_odd_base
            done_flag = self.finished_with_odd
            
        memint.write_samples( self.phy, \
                              samples, \
                              address, \
                              readback )
        
        self.write_samples_to_even = not self.write_samples_to_even
        
        # mark done with even /odd
        self.phy.debug_reg_write( self.even_odd_sync_reg, done_flag )
    
    def get( self, num_samples ):
        # #####################################################################
        # wait for the correlation to complete and get the samples and peak
        # detector information
    
        # decide which buffer is in use
        #print "wait for ", self.fpga_using_odd
        if self.read_samples_from_even:
            # fpga using odd
            flag                = self.even_buffer_valid
            address             = self.sample_result_even_base
            peak_address_reg    = self.even_peak_address_reg
            peak_power_reg      = self.even_peak_power_reg
            number_of_peaks_reg = self.even_number_of_peaks_reg
            frame_number_reg    = self.even_frame_number_reg
            
        else:
            #fpga using even
            flag                = self.odd_buffer_valid
            address             = self.sample_result_odd_base
            peak_address_reg    = self.odd_peak_address_reg
            peak_power_reg      = self.odd_peak_power_reg
            number_of_peaks_reg = self.odd_number_of_peaks_reg
            frame_number_reg    = self.odd_frame_number_reg
            
        # wait for the correlation to be completed
        status_reg = 0
        for i in range( 100 ):
            status_reg = self.phy.debug_reg_read( self.even_odd_sync_reg )
            if status_reg & flag :
                break
            
        if not status_reg & flag :
            #print "Giving up waiting for even/odd sync %d" % self.read_samples_from_even
            fsm_status1 = self.phy.debug_reg_read( self.fsm_status1_reg )
            fsm_status2 = self.phy.debug_reg_read( self.fsm_status2_reg )
            raise Exception( "Giving up waiting for even/odd sync %d (reg = %x, fsm status 1 = 0x%x, fsm status 2 = 0x%x)" \
                            % (self.read_samples_from_even, status_reg, fsm_status1, fsm_status2) )
        
        if status_reg & self.fft_overflow :
            print "FFT overflow occurred"        
        
        # read the output samples from memory
        samples = memint.read_samples( self.phy, address, num_samples, self.power_results )
        if self.power_results :
            samples = 512.0*samples
        
        # read the peak detector information
        peak_address = self.phy.debug_reg_read( peak_address_reg )
        peak_offset  = peak_address-address
        peak_power   = self.phy.debug_reg_read( peak_power_reg )
        if self.power_results :
            peak_power = 512.0*peak_power
        
        else :
            peak_power =   2.0*peak_power
        
        num_samples_over_threshold = self.phy.debug_reg_read( number_of_peaks_reg )
        
        # read the frame number
        frame_number = self.phy.debug_reg_read( frame_number_reg )

        # switch buffers        
        self.read_samples_from_even = not self.read_samples_from_even
        
        return samples, num_samples_over_threshold, peak_offset, peak_power, frame_number
    
    def get_with_input( self, num_inp_samples, num_out_samples ):
        # #####################################################################
        # wait for the correlation to complete and get the samples and peak
        # detector information
    
        # decide which buffer is in use
        #print "wait for ", self.fpga_using_odd
        if self.read_samples_from_even:
            # fpga using odd
            flag                = self.even_buffer_valid
            inp_address         = self.sample_input_even_base
            out_address         = self.sample_result_even_base
            peak_address_reg    = self.even_peak_address_reg
            peak_power_reg      = self.even_peak_power_reg
            number_of_peaks_reg = self.even_number_of_peaks_reg
            frame_number_reg    = self.even_frame_number_reg
            
        else:
            #fpga using even
            flag                = self.odd_buffer_valid
            inp_address         = self.sample_input_odd_base
            out_address         = self.sample_result_odd_base
            peak_address_reg    = self.odd_peak_address_reg
            peak_power_reg      = self.odd_peak_power_reg
            number_of_peaks_reg = self.odd_number_of_peaks_reg
            frame_number_reg    = self.odd_frame_number_reg
            
        # wait for the correlation to be completed
        status_reg = 0
        for i in range( 100 ):
            status_reg = self.phy.debug_reg_read( self.even_odd_sync_reg )
            if status_reg & flag :
                break
            
        if not status_reg & flag :
            #print "Giving up waiting for even/odd sync %d" % self.read_samples_from_even
            fsm_status1 = self.phy.debug_reg_read( self.fsm_status1_reg )
            fsm_status2 = self.phy.debug_reg_read( self.fsm_status2_reg )
            raise Exception( "Giving up waiting for even/odd sync %d (reg = %x, fsm status 1 = 0x%x, fsm status 2 = 0x%x)" \
                            % (self.read_samples_from_even, status_reg, fsm_status1, fsm_status2) )
        
        # read the input and output samples from memory
        inp_samples = memint.read_samples( self.phy, inp_address, num_inp_samples, False )
        out_samples = memint.read_samples( self.phy, out_address, num_out_samples, self.power_results )
        if self.power_results :
            out_samples = 512.0*out_samples
        
        # read the peak detector information
        peak_address = self.phy.debug_reg_read( peak_address_reg )
        peak_offset  = peak_address-out_address
        peak_power   = self.phy.debug_reg_read( peak_power_reg )
        if self.power_results :
            peak_power = 512.0*peak_power
        
        else :
            peak_power =   2.0*peak_power
        
        num_samples_over_threshold = self.phy.debug_reg_read( number_of_peaks_reg )
        
        # read the frame number
        frame_number = self.phy.debug_reg_read( frame_number_reg )

        # switch buffers        
        self.read_samples_from_even = not self.read_samples_from_even
        
        return inp_samples, out_samples, num_samples_over_threshold, peak_offset, peak_power, frame_number


# #############################################################################
# the main program
if __name__ == '__main__':
    print "No standalone functionality"

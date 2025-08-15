"""
-------------------------------------------------------------------------------


-------------------------------------------------------------------------------
  Filename:   phyint.py
  Author(s):  pdm
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

python interface to the phy program

-------------------------------------------------------------------------------
"""

import socket
import re
import struct

class phyint:
    def __init__( self, host, port ):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM )
        self.s.connect(( host, port ))
    
    def __del__(self):
        self.s.close()

    def wait_for_binary_reply(self, length):
        "wait for a fixed-length reply"
        msg = self.s.recv(length)
        return msg

    def wait_for_text_reply( self, match):
        "wait for a reply starting with any string terminated by '\n' that is in the list 'match' "
        if type(match) != list : match = [match]
            
        while( 1 ):
            msg = self.wait_for_msg()
                
            msg.strip()
            if msg == "?\n" :
                raise Exception("Interface error has been reported")
            
            for cmd in match:
                if msg.find( cmd ) == 0:
                    # found, split it up
                    ans = [ msg[:len(cmd)] ]
                    ans.extend( re.split( "[ ,]+", msg[len(cmd):].strip() ) )
                    #print ans
                    return ans
            #else keep trying until match (timeout needed?)
    
    def wait_for_msg( self ):
        "wait for a message "
        
        msg       = ""
        next_char = ''
        while next_char != '\n' :
            next_char = self.s.recv(1)
            msg += next_char
            
        msg.strip()
        return msg
    
    def set_vctcxo( self, dac ):
        if dac<0 or dac > 0x0fff:
            raise ValueError( "dac value out of range" )
        self.s.send("#debug_vctcxo %d\n"%(dac) )
        result = self.wait_for_text_reply( "#debug_vctcxo" )
        if not result or result[1] != "ok":
            raise Exception( "set_vctcxo_dac error", result )
    
    def set_debug_print( self, b ):
        if b : str = "on"
        else : str = "off"
        self.s.send( "#debug_print %s\n"%(str) )
        result = self.wait_for_text_reply( "#debug_print" )
        if not result or result[1] != "ok":
            raise Exception( "debug_print error", result )
    
    def debug_switches( self, band, ad_input, ad_gpio ):
        self.s.send( "#debug_switches %s,%s,%s\n"%(band, ad_input, str(ad_gpio)) )
        result = self.wait_for_text_reply("#debug_switches")
        if not result or result[1] != "ok":
            raise Exception( "debug_switches error", result )
    
    def abort( self ):
        self.s.send( "#abort\n" )
        result = self.wait_for_text_reply( "a" )
        if result[1] != '0':
            raise Exception( "abort error", result )
    
    def set_band( self, band ):
        band = band.upper()
        if band not in ["UL850","DL850","UL900","DL900","UL1700","DL1700","UL1800","DL1800","UL1900","DL1900","UL2100","DL2100","AUTO","LOAD"]:
            raise Exception( "invalid band arg", band )
        self.s.send( "#set_band %s\n"%(band) )
        result = self.wait_for_text_reply( "b" )
        if result[1] != '0':
            raise Exception( "band error", result )
    
    def cal_ref_clock( self, mode, freq_Hz=0, code=-1, antenna=1 ):
        mode = mode.lower()
        if mode not in ["gps","cpich","psch"]:
            raise Exception( "invalid cal mode", mode )
        if mode == "cpich":
            self.s.send( "#cal_ref %s %u %d %d \n"%(mode, freq_Hz, code, antenna) )
        elif mode == "psch":
            self.s.send( "#cal_ref %s %u\n"%(mode, freq_Hz ) )
        else:
            self.s.send( "#cal_ref %s\n"%(mode) )
        result = self.wait_for_text_reply( "c" )
        if result[1] != '0':
            raise Exception( "cal ref clock error", result )
    
    def start_ul_search( self, code, freq_Hz, pilots ):
        self.s.send( "#ul_search %d,%d,%d\n"%(code,freq_Hz,pilots) )
        result = self.wait_for_text_reply( "u" )
        if result[1] != '0':
            raise Exception( "ul search error", result )
    
    def start_dl_search( self, code, freq_Hz, syms ):
        self.s.send( "#dl_search %d,%d,%d\n"%(code,freq_Hz,syms) )
        result = self.wait_for_text_reply( "d" )
        if result[1] != '0':
            raise Exception( "dl search error", result )
    
    def set_standby( self, standby ):
        standby = standby.upper()
        if standby not in ["ACTIVE","FPGA_IDLE","ADC_IDLE","FPGA_ADC_IDLE","RADIO_FPGA_ADC_IDLE"]:
            raise Exception( "invalid mode arg", standby )
        self.s.send( "#set_standby %s\n"%(standby) )
        result = self.wait_for_text_reply( "s" )
        if result[1] != '0':
            raise Exception( "mode error", result )
    
    def set_pauses( self, searching_pauses, tracking_pauses ):
        self.s.send( "#set_pauses %d,%d\n" % (searching_pauses, tracking_pauses) )
        result = self.wait_for_text_reply( "p" )
        if result[1] != '0':
            raise Exception( "pauses error", result )
    
    def set_low_power( self, low_power ):
        low_power = low_power.upper()
        if low_power not in ["LOW_POWER","HIGH_SENS"]:
            raise Exception( "invalid mode arg", low_power )
        self.s.send( "#set_low_power %s\n" % (low_power) )
        result = self.wait_for_text_reply( "w" )
        if result[1] != '0':
            raise Exception( "low power error", result )
    
    def set_mode( self, mode ):
        mode = mode.upper()
        if mode not in ["4G","3G","GSM"]:
            raise Exception( "invalid mode arg", mode )
        self.s.send( "#set_mode %s\n"%(mode) )
        result = self.wait_for_text_reply( "m" )
        if result[1] != '0':
            raise Exception( "mode error", result )
    
    def set_antenna( self, antenna ):
        self.s.send( "#set_antenna %d\n" % antenna )
        result = self.wait_for_text_reply( "y" )
        if result[1] != '0':
            raise Exception( "antenna error", result )
    
    def get_version( self ):
        self.s.send( "#get_version\n" )
        result = self.wait_for_text_reply( "v" )
        return (result[2], result[3], result[4])
    
    def start_gsm_tc_track( self, freq_Hz ):
        self.s.send( "#chan_pwr %d\n"%(freq_Hz) )
        result = self.wait_for_text_reply( "p" )
        if result[1] != '0':
            raise Exception( "GSM TC track error", result )
    
    def mem_raw_read( self, address, length ):
        ans = []
        block_size = 2000
        while( length > 0 ):
            words_to_get = length
            if words_to_get > block_size:
                words_to_get = block_size
            length -= words_to_get

            cmd = "#debug_peek"
            ok_ack = cmd+" ok,"  #   All other responses are longer
            self.s.send( "%s %d, %d\n"%(cmd,address,words_to_get) )
            result = self.wait_for_binary_reply( len(ok_ack) )
            if result.find(ok_ack) != 0:
                end_of_err_msg = self.wait_for_binary_reply( 100 )  # Arbitrary limit
                raise Exception( "debug_peek error", ( result + end_of_err_msg) )

            bytes_to_get = 4*words_to_get
            result = self.wait_for_binary_reply( bytes_to_get )
            while( len(result) < bytes_to_get ):
                result = result + self.wait_for_binary_reply( bytes_to_get-len(result) )

            self.wait_for_binary_reply(1)   # Discard the trailing '\n'

            #print "raw read result len:",len(result)
            address += words_to_get
            #print "words_to_get:",words_to_get
            for i in range(words_to_get):
                r = result[i*4:i*4+4]
                if len(r) != 4 : print i,len(r)
                ans.append( struct.unpack( "<I", r )[0] )
        return ans
    
    def mem_raw_write( self, address, data ):
        block_size = 500
        length = len(data)
        pos = 0
        while( pos < length ):
            words_to_put = length-pos
            if words_to_put > block_size:
                words_to_put = block_size
            
            string = ""
            for i in range(0,words_to_put):
                string += str(data[i+pos]) + ' '
            
            #print "debug_poke address: %x, %d words, block: %d"%(address,words_to_put,pos/block_size)
            self.s.send( "#debug_poke %d,%s\n"%(address,string) )
            result = self.wait_for_text_reply( "#debug_poke" )
            if result[1] != "ok":
                raise Exception( "debug_poke error", result )
            
            pos += words_to_put
            address += words_to_put
        return
    
    def debug_ADF4602( self, wr, address, data=0 ):
        # only write implemented at the moment
        self.s.send( "#debug_ADF4602 write %f %d\n"%(address, data) )
        result = self.wait_for_text_reply( "#debug_ADF4602" )
        if result[1] != 'ok':
            raise Exception( "debug_ADF4602 error", result )
    
    def debug_ranging( self, on_off, value=None ):
        # to do, check string = on/off/now
        if value == None:
            self.s.send( "#debug_ranging %s\n"%(on_off) )
        else:
            self.s.send( "#debug_ranging %s %d\n"%(on_off, value) )
        
        result = self.wait_for_text_reply( "#debug_ranging" )
        if result[1] != 'ok':
            raise Exception( "debug_ranging error", result )
    
    def debug_set_frequency( self, frequency, band="high" ):
        if band == "high":
            self.debug_ADF4602( "write", 10, frequency/50e3 )
        else:
            self.debug_ADF4602( "write", 10, frequency*2/50e3 )
    
    def debug_set_gain( self, gain ):
        self.debug_ADF4602( "write", 11, gain )
    
    def debug_set_ref_cal_thresh_factor_dB( self, thresh_factor_dB ):
        self.s.send( "#debug_ref_cal_threshold %f\n" % thresh_factor_dB )
        result = self.wait_for_text_reply( "#debug_ref_cal_threshold" )
        if result[1] != 'ok':
            raise Exception( "reference calibration threshold factor error", result )
    
    def debug_set_search_thresh_factor_dB( self, thresh_factor_dB ):
        self.s.send( "#debug_search_threshold %f\n" % thresh_factor_dB )
        result = self.wait_for_text_reply( "#debug_search_threshold" )
        if result[1] != 'ok':
            raise Exception( "search mode threshold factor error", result )
    
    def debug_set_track_thresh_factor_dB( self, thresh_factor_dB ):
        self.s.send( "#debug_track_threshold %f\n" % thresh_factor_dB )
        result = self.wait_for_text_reply( "#debug_track_threshold" )
        if result[1] != 'ok':
            raise Exception( "track mode threshold factor error", result )
    
    def debug_reg_write( self, address, data ):
        self.s.send( "#debug_reg W %d %d\n"%(address, data) )
        result = self.wait_for_text_reply( "#debug_reg" )
        if result[1] != 'ok':
            raise Exception( "debug_reg W error", result )
    
    def debug_reg_read( self, address ):
        self.s.send( "#debug_reg R %d\n"%(address) )
        result = self.wait_for_text_reply( "#debug_reg" )
        if result[1] != 'ok':
            raise Exception( "debug_reg R error", result )
        return int(result[2])
    
    def debug_gpio_set( self, file_number, value ):
        if value :
            self.s.send( "#debug_gpio high %d\n"%(file_number) )
        else :
            self.s.send( "#debug_gpio low %d\n"%(file_number) )
        
        result = self.wait_for_text_reply( "#debug_gpio" )
        if result[1] != 'ok':
            raise Exception( "debug_gpio set error", result )
    
    def debug_gpio_get( self, file_number ):
        self.s.send( "#debug_gpio get %d\n"%(file_number) )
        result = self.wait_for_text_reply( "#debug_gpio" )
        if result[1] != 'ok':
            raise Exception( "debug_gpio get error", result )
        return int(result[2])
    
    def debug_adc_set( self, register, value ):
        self.s.send( "#debug_ADC write %d %d\n"%(register, value) )
        result = self.wait_for_text_reply( "#debug_ADC" )
        if result[1] != 'ok':
            raise Exception( "debug_ADC set error", result )
        
    def debug_gsm( self, num_samples ):
        self.s.send( "#debug_gsm %u\n"%(num_samples) )
        
        result = self.wait_for_text_reply( "#debug_gsm" )
        
        if result[1] != 'ok':
            raise Exception( "debug_gsm error", result )
        
        ans = []
        for n in range(num_samples) :
            ans.append(float(result[n+2]))
        
        return ans

    def lte_search( self, freq_Hz, chan1_filt, chan2_filt, correl_coeffs ):
        self.s.send( "#lte_search %d,%s,%s,%s\n"%(freq_Hz,chan1_filt, chan2_filt, correl_coeffs) )
        result = self.wait_for_text_reply( "l" )
        if result[1] != '0':
            raise Exception( "lte_search error", result )

    def regression_4g(self, chan1_filt, chan2_filt, correl_coeffs, test_vec ):
        """Send the #regression_4g command to the unit under test and report
        results/errors
        
        @param chan1_filt filename for channel 1 filter coefficients
        @param chan2_file filename for channel 2 filter coefficients
        @param correl_coeffs filename for the correlator coefficients
        @param test_vec filename for the ADC samples test vector
        @returns list of reported correlation results.  Each result is a list
                 [time, corrPeakNum, corrPeakDecSqrd, sumCorrIntAtPeak]
                 If an error occurs a #regression_4g exception is raised
        """
        statistics = []
        results = []
        self.s.send( "#regression_4g %s,%s,%s,%s\n"%(chan1_filt, chan2_filt, correl_coeffs, test_vec) )
        finished = False;
        while (not finished):
            result = self.wait_for_text_reply( ["r", "R"] )
            if result[0] == 'r':
                if result[1] != '0':
                    # Problem (e.g. parameter errors, runtime error)
                    raise Exception( "#regression_4g r error", result )
                else:
                    # process r0 normal finish
                    print("#regression_4g: normal finish")
                    finished = True
            elif result[0] == 'R':
                if result[1][0] == 'S':
                    # process RS (indicates command progress)
                    if result[1][1] == '0':
                        print("#regression_4g: command accepted")
                    elif result[1][1] == '1':
                        print("#regression_4g: running")
                    elif result[1][1] == '2':
                        print("#regression_4g: ADC buffer now empty")
                    elif result[1][1] == '2':
                        print("#regression_4g: !!Correlator FIFO OVERFLOW!!")
                    elif result[1][1] == '4':
                        # process RS3 result.  Fields following 'R' 'S3' are:
                        #     result[2] = 4g_stats.NumVals
                        #     result[3] = 4g_stats.PeakPwr
                        #     result[4] = 4g_stats.Sum
                        numvals = int(result[2])
                        peakPwr = float(result[3])
                        sum = float(result[4])
                        statistics.append([numvals, peakPwr, sum])
                    else:
                        raise Exception( "#regression_4g RS error", result )
                elif result[1][0] == 'R':
                    # process RR result.  Fields following 'R' 'R' are:
                    #     result[2] = peaks[index].time
                    #     result[3] = peaks[index].CorrPeakNum
                    #     result[4] = peaks[index].CorrPeakDenSqrd
                    #     result[5] = peaks[index].SumCorrInpAtPeak).str();
                    # Pack fields into a list and append to the results list.
                    time = int(result[2])
                    corrPeakNum = float(result[3])
                    corrPeakDecSqrd = float(result[4])
                    sumCorrIntAtPeak = float(result[5])
                    results.append([time, corrPeakNum, corrPeakDecSqrd, sumCorrIntAtPeak])
                elif result[1][0] == 'A' and result[1][1] == 'T':
                    # process RAT timeout
                    print("#regression_4g: Timeout without seeing any peaks\n")
                else:
                    raise Exception( "#regression_4g RS error", result )
            else:
                raise Exception( "#regression_4g R error", result )
        return statistics, results

#test
if __name__ == "__main__":
    p = phyint( "localhost", 55555 )

    try:
        p.set_debug_print( False )
        p.set_debug_print( True )
        p.set_vctcxo( 1234 )
        
        p.set_band( "UL900" )
        p.cal_ref_clock( "gps" )
        
    except ValueError as e:
        print e
        print e.args

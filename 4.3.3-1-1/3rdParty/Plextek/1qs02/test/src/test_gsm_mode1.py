"""
-------------------------------------------------------------------------------


-------------------------------------------------------------------------------
  Filename:   test_gsm_mode1.py
  Author(s):  pdm
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the GSM mode

-------------------------------------------------------------------------------
"""

print "loading libs...",

import numpy
import pylab
import sys
import time

import get_defaults
import gsmint
import phyint
print "done.\n"

def main(address) :
    """The main function of the test script"""
    # ###############################################################
    # Initialise the PHY interface
    phy = phyint.phyint(address,55555)
    print "    PCB Version  = %s\n" \
          "    FPGA Version = %s\n" \
          "    SW Version   = %s\n" % phy.get_version()

    # ###############################################################
    # Set the GSM mode
    #
    phy.set_mode("GSM")
    msg = phy.wait_for_msg()
    print "%s" % msg,
    
    # ###############################################################
    # Set the VCTCXO DAC
    #
    phy.set_vctcxo(1929)
        
    # ###############################################################
    # Setup the frequency and gain
#    phy.debug_switches( band="wcdma_3", ad_input="RXHB2RF", ad_gpio=0xc )
    phy.debug_switches( band="load50", ad_input="RXHB2RF", ad_gpio=0xc )
    phy.debug_set_frequency( 1930000000, band="high" )
    phy.debug_set_gain( 100 )

    # ###############################################################
    # Initialise the interface to the GSM functionality
    gsm = gsmint.gsm_interface(phy)
    
    once = True
    forever = True
    while forever :
        once = False    
        inph_dc = 0.0
        quad_dc = 0.0
        set_inph_dc = 0
        set_quad_dc = 0
        gsm.set_dc_offset(set_inph_dc, set_quad_dc)
            
        # ###############################################################
        # Start the GSM measurements
        one_result_per_slot = False
        enable_dc_offset    = True
        antenna1            = True
        
        if one_result_per_slot :
            results_per_slot = 1
            
        else :
            results_per_slot = 3

        gsm.run( one_result_per_slot, enable_dc_offset, antenna1 )
    
        # ###############################################################
        # Get the GSM measurement results
        results = gsm.debug_gsm(10000)
        rssi    = gsm.get_adc_rssi()
        (inph_dc, quad_dc) = gsm.get_dc_offset()
#        gsm.run( one_result_per_slot, enable_dc_offset, antenna1 )
        
        gsm.stop()
        
        inph_dc /= 1.0
        quad_dc /= 1.0
        
        print "%3d, %3d : RSSI = %10u, DC = %6.1f, %6.1f, RMS = %10.1f" \
               % (set_inph_dc, set_quad_dc, rssi, inph_dc, quad_dc, numpy.sqrt(numpy.mean(results)))
                
        num_results = results.size
        num_slots   = int(numpy.floor(num_results/results_per_slot/8))
        num_results = results_per_slot*8*num_slots
        results = results[:num_results]
        slot_results = results.copy()
        slot_results.shape = (num_slots, results_per_slot*8)

        pylab.subplot(2, 1, 1)            
        pylab.plot(numpy.sqrt(results))

        pylab.subplot(2, 2, 3)            
        pylab.plot(numpy.sqrt(numpy.transpose(slot_results)))
    
        pylab.subplot(2, 2, 4)            
        pylab.plot(numpy.sqrt(slot_results))
        pylab.show()
    
        time.sleep(0.1)

# ###################################################################
# the main program
if __name__ == '__main__':
    #
    # parse the command-line arguments: if there is one it is the IP
    # address of the server
    #
    if len(sys.argv) > 1:
        address = sys.argv[1]
        print "Target IP address is %s\n" % address 
        
    else :
        address = get_defaults.getDefaultHhdfIpAddress()
    
    main(address)

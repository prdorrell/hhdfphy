"""
-------------------------------------------------------------------------------
Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/test_gsm_mode_dc_offsets.py $
$Revision: 6419 $
$Author: pdm $
$Date: 2011-07-22 11:19:49 +0100 (Fri, 22 Jul 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the GSM mode DC offset measurement and correction

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

    # ###############################################################
    # Set the GSM mode
    #
    phy.set_mode("GSM")
    _ = phy.wait_for_msg()
    print "    PCB Version  = %s\n" \
          "    FPGA Version = %s\n" \
          "    SW Version   = %s\n" % phy.get_version()
    
    # ###############################################################
    # Set the VCTCXO DAC
    #
    phy.set_vctcxo(1929)
        
    # ###############################################################
    # Setup the frequency and gain
#    phy.debug_switches( band="wcdma_3", ad_input="RXHB2RF", ad_gpio=0xc )
    if False :
        freq_Hz = 949200000    
        phy.debug_switches( band="load50", ad_input="RXLBRF", ad_gpio=0xA )
        phy.debug_set_frequency( freq_Hz, band="low" )

    else :    
        freq_Hz = 1930000000    
        phy.debug_switches( band="load50", ad_input="RXHB2RF", ad_gpio=0xc )
        phy.debug_set_frequency( freq_Hz, band="high" )
    
    phy.debug_set_gain( 0 )

    # ###############################################################
    # Initialise the interface to the GSM functionality
    gsm = gsmint.gsm_interface(phy)
    
    once = True
    while once :
        once = False    
        
        skip = 5
        set_inph_dc = 0.0
        set_quad_dc = 0.0
        for _ in range(10) :
            gsm.set_dc_offset(int(set_inph_dc), int(set_quad_dc))
        
            # ###############################################################
            # Start the GSM measurements
            one_result_per_slot = False
            enable_dc_offset    = True
            antenna1            = True
            gsm.run( one_result_per_slot, enable_dc_offset, antenna1 )
            
            if one_result_per_slot :
                results_per_slot = 1
                
            else :
                results_per_slot = 3
        
            # ###############################################################
            # Get the GSM measurement results
            results = gsm.debug_gsm(10000)
            rssi    = gsm.get_adc_rssi()
            (inph_dc, quad_dc) = gsm.get_dc_offset()
            inph_dc /= 4.0
            quad_dc /= 4.0
    
            gsm.stop()
            
            print "%3d, %3d : RSSI = %10u, DC = %6.1f, %6.1f, RMS = %10.1f" \
                   % (set_inph_dc, set_quad_dc, rssi, inph_dc, quad_dc, numpy.sqrt(numpy.mean(results)))

            if skip > 0 :
                skip -= 1
                
            else :
                set_inph_dc = inph_dc
                set_quad_dc = quad_dc
        
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

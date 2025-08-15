"""
-------------------------------------------------------------------------------
Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/test_gsm_tc_track.py $
$Revision: 7552 $
$Author: pdm $
$Date: 2011-10-11 09:13:34 +0100 (Tue, 11 Oct 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the GSM mode's time-continuous tracking

-------------------------------------------------------------------------------
"""

print "loading libs...",

import sys
import time

import get_defaults
import phyint
print "done.\n"

def main(address, band, freq_Hz) :
    """The main function of the test script"""
    # ###############################################################
    # Initialise the PHY interface
    phy = phyint.phyint(address,55555)
    
    phy.set_mode("GSM");
    _ = phy.wait_for_msg()
    print "    PCB Version  = %s\n" \
          "    FPGA Version = %s\n" \
          "    SW Version   = %s\n" % phy.get_version()
    
#    phy.set_vctcxo(1913)
#    phy.set_vctcxo(1875)
#    phy.set_vctcxo(1874)
#    phy.set_vctcxo(1929)
    phy.set_vctcxo(1914)
#    phy.set_vctcxo(1990)

    phy.set_band(band)
    
    for _ in range(100) :
        phy.start_gsm_tc_track(freq_Hz)
        
        for _ in range(100) :
            msg = phy.wait_for_msg()
            print "%s" % msg,
    
        print("\n")
        phy.abort()
    

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

    band = "UL2100"
    
    freq_Hz = 1930000000
    main(address, band, freq_Hz)

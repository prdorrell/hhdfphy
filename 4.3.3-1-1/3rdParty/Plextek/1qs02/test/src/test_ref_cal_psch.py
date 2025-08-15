"""
-------------------------------------------------------------------------------
Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/test_ref_cal_psch.py $
$Revision: 7552 $
$Author: pdm $
$Date: 2011-10-11 09:13:34 +0100 (Tue, 11 Oct 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the reference calibration using the PSCH method

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
    
    phy.set_mode("3G");
    _ = phy.wait_for_msg()
    print "    PCB Version  = %s\n" \
          "    FPGA Version = %s\n" \
          "    SW Version   = %s\n" % phy.get_version()

    # Set the band
    phy.set_band(band)
    
    # Calibrate the reference
    start = time.time()
    phy.cal_ref_clock("psch", freq_Hz)
    msg = phy.wait_for_msg()
    end = time.time()
    print "%s    %f s" % (msg.strip(), (end-start))

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

    band = "AUTO"
    
    freq_Hz = 1930000000  
#    freq_Hz = 2162200000  
#    freq_Hz = 2142400000
#    freq_Hz = 2156600000
#    freq_Hz = 2112800000

    main(address, band, freq_Hz)

"""
-------------------------------------------------------------------------------
Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/test_misc_commands.py $
$Revision: 6998 $
$Author: pdm $
$Date: 2011-08-26 13:33:47 +0100 (Fri, 26 Aug 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test miscellaneous commands

-------------------------------------------------------------------------------
"""

print "loading libs...",

import sys
import time

import get_defaults
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

    #
    # Set the GSM mode
    #
    phy.set_mode("GSM")
    msg = phy.wait_for_msg()
    print "%s" % msg,
            
    #
    # Set the 3G mode
    #
    phy.set_mode("3G")
    msg = phy.wait_for_msg()
    print "%s" % msg,

    #
    # Set the FPGA idle
    #
    phy.set_standby("FPGA_IDLE")

    #
    # Set the ADC idle
    #
    phy.set_standby("ADC_IDLE")

    #
    # Set the FPGA and ADC idle
    #
    phy.set_standby("FPGA_ADC_IDLE")

    #
    # Set all active
    #
    phy.set_standby("RADIO_FPGA_ADC_IDLE")

    #
    # Set all active
    #
    phy.set_standby("ACTIVE")

    #
    # Define the pauses.
    #
    searching_pauses = 0
    tracking_pauses  = 0
    phy.set_pauses( searching_pauses, tracking_pauses )
    
    searching_pauses = 53
    tracking_pauses  = 3
    phy.set_pauses( searching_pauses, tracking_pauses )

    #
    # Set the low-power mode.
    #
    phy.set_low_power( "low_power" )
    phy.set_low_power( "high_sens" )

    #
    # Set the antenna.
    #
    antenna = 1
    phy.set_antenna( antenna )

    antenna = 2
    phy.set_antenna( antenna )

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

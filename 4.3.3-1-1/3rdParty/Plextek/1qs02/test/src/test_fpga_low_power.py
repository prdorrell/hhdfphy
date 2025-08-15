"""
-------------------------------------------------------------------------------
Copyright (c) 2022 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/test_mode_changes.py $
$Revision: 6380 $
$Author: pdm $
$Date: 2011-07-20 09:32:59 +0100 (Wed, 20 Jul 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the mode changes

-------------------------------------------------------------------------------
"""
from __future__ import print_function

print ("loading libs...")

import sys
import time

import get_defaults
import phyint
print ("done.\n")

def set_mode_and_wait_for_msg(phy, mode):
    """Set chosen mode and wait for confirmation from uut"""
    print("test_mode({0})".format(mode))
    phy.set_mode(mode)
    msg = phy.wait_for_msg()
    print("\t%s" % msg)

def connect_and_set_mode(uut_address, mode):
    """Connect to the HHDFPHY and set the mode"""
    phy = phyint.phyint(uut_address,55555)
    set_mode_and_wait_for_msg(phy, mode)

def cycle_low_power_by_connecting(uut_address):
    """The main function of the test script
    Sleep for 10 s, connect to the HHDFPHY, write to the test register (so that
    subsequent reads from the version register of the 3G FPGA work), read the
    versions and sleep for 10 s before returning (thereby disconnecting).
    """
    time.sleep(2)

    phy = phyint.phyint(uut_address,55555)

    phy.debug_reg_write(1, 1)

    print("    PCB Version  = %s\n" \
          "    FPGA Version = %s\n" \
          "    SW Version   = %s\n" % phy.get_version())

    time.sleep(2)

def main(uut_address):
    """The main function of the test script
    Test the low power mode in different ways.
    """

    connect_and_set_mode(uut_address, "3G")
    for _ in range(10):
        cycle_low_power_by_connecting(uut_address)

    connect_and_set_mode(uut_address, "4G")
    for _ in range(10):
        cycle_low_power_by_connecting(uut_address)

# ###################################################################
# the main program
if __name__ == '__main__':
    #
    # parse the command-line arguments: if there is one it is the IP
    # address of the server
    #
    if len(sys.argv) > 1:
        address = sys.argv[1]
        print ("Target IP address is %s\n" % address)
        
    else :
        address = get_defaults.getDefaultHhdfIpAddress()

    main(address)

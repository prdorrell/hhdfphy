"""
-------------------------------------------------------------------------------


-------------------------------------------------------------------------------
  Filename:   test_mode_changes.py
  Author(s):  pdm, pmd
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

def test_gsm_commands(phy, should_accept, mode):
    """Test GSM specific commands"""
    #
    # Check that the GSM-specific commands are accepted/rejected
    #
    # Currently do-nothing - no GSM specific tests to run

def test_3g_commands(phy, should_accept, mode):
    """Test 3G specific commands"""
    #
    # Check that the 3G-specific commands are accepted/rejected.
    #
    accepted = True
    freq_Hz  = 1930000000
    try:
        phy.cal_ref_clock("psch", freq_Hz)
        
    except Exception as e:
        if e[1][1] == "1" :
            accepted = False
            
    else:
        phy.abort()

    if accepted != should_accept :
        print("Reference calibration {0} in {1} mode.".format(
            "accepted" if accepted else "rejected", mode))

    accepted = True
    code_num = 0
    freq_Hz  = 1930000000
    num_syms = 8
    try:
        phy.start_ul_search(code_num, freq_Hz, num_syms)
        
    except Exception as e:
        if e[1][1] == "1":
            accepted = False
            
    else:
        phy.abort()

    if accepted != should_accept:
        print("UL Search {0} in {1} mode.".format(
            "accepted" if accepted else "rejected", mode))
    
    accepted = True
    code_num = 0
    freq_Hz  = 1930000000
    num_syms = 8
    try:
        phy.start_dl_search(code_num, freq_Hz, num_syms)
        
    except Exception as e:
        if e[1][1] == "1":
            accepted = False
            
    else:
        phy.abort()

    if accepted != should_accept :
        print("DL Search {0} in {1} mode.".format(
            "accepted" if accepted else "rejected", mode))

def test_4g_commands(phy, should_accept, mode):
    """Test GSM specific commands"""
    #
    # Check that the 4G-specific commands are accepted/rejected
    #
    accepted = True
    freq_Hz  = 1930000000
    ch_filt_1_coeff = "/opt/hhdfphy/coefficient_files/ChanFilt_1_BW_5_0_MHz_v01.dat"
    ch_filt_2_coeff = "/opt/hhdfphy/coefficient_files/ChanFilt_2_BW_5_0_MHz_v01.dat"
    corr_coeff = "/opt/hhdfphy/coefficient_files/Correlator_BW_5_0_MHz_ID_03_CP_144_v01.dat"

    try:
        phy.lte_search(freq_Hz, ch_filt_1_coeff, ch_filt_2_coeff, corr_coeff)

    except Exception as e:
        if e[1][1] != "0":    # bug in hhdf means we are seeing 'l"' rather than 'l1'
            accepted = False
    else:
        phy.abort()

    if accepted != should_accept:
        print("LTE Search {0} in {1} mode.".format(
            "accepted" if accepted else "rejected", mode))


def set_mode_and_wait_for_msg(phy, mode):
    """Set chosen mode and wait for confirmation from uut"""
    print("test_mode({0})".format(mode))
    phy.set_mode(mode)
    msg = phy.wait_for_msg()
    print("\t%s" % msg)


def test_mode(phy, mode, should_accept_gsm, should_accept_3g, should_accept_4g):
    """Set chosen mode and check appropriate functions are accepted/rejected"""
    set_mode_and_wait_for_msg(phy, mode)

    #
    # Check that the GSM-specific commands are accepted/rejected.
    #
    test_gsm_commands(phy, should_accept_gsm, mode)

    #
    # Check that the 3G-specific commands are acceptedrejected.
    #
    test_3g_commands(phy, should_accept_3g, mode)

    #
    # Check that the 4G-specific commands are acceptedrejected.
    #
    test_4g_commands(phy, should_accept_4g, mode)


def test_gsm_mode(phy):
    """Set GSM mode and check appropriate functions are accepted/rejected"""
    test_mode(phy, "GSM", True, False, False)


def test_3g_mode(phy):
    """Set 3G mode and check appropriate functions are accepted/rejected"""
    test_mode(phy, "3G", False, True, False)


def test_4g_mode(phy):
    """Set 4G mode and check appropriate commands are accepted/rejected"""
    test_mode(phy, "4G", False, False, True)


def main(uut_address):
    """The main function of the test script
    Need to test all possible mode change directions and not just
    whether each mode can be run.  In addition to setting an initial
    mode this requires 6 further transitions
        GSM>3G
        3G->4G
        4G>3G
        3G->GSM
        GSM->4G
        4G>GSM
    """
    # ###############################################################
    # Initialise the PHY interface
    phy = phyint.phyint(uut_address,55555)
    print("    PCB Version  = %s\n" \
          "    FPGA Version = %s\n" \
          "    SW Version   = %s\n" % phy.get_version())

    set_mode_and_wait_for_msg(phy, "GSM")  # Define starting mode

    # Test each of the 6-possible mode transition
    test_3g_mode(phy)
    test_4g_mode(phy)
    test_3g_mode(phy)
    test_gsm_mode(phy)
    test_4g_mode(phy)
    test_gsm_mode(phy)


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

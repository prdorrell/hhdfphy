"""
-------------------------------------------------------------------------------
Copyright (c) 2022 Domo Tactical Communications (DTC) Ltd
All Rights Reserved
-------------------------------------------------------------------------------
Author: Peter Debenham
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the correlator using the regression test interface and known ADC sample
inputs.  The channel filter, correlelator coefficients and ADC sample data files
must be on the remote target board in the locations shown at the top of the
test_4g_regression() function.

-------------------------------------------------------------------------------
"""
from __future__ import print_function

print ("loading libs...")

import sys
import time
import argparse

import get_defaults
import phyint
print ("done.\n")

def test_4g_regression(phy, 
                       should_accept,
                       mode,
                       adc_samples):
    """
    Confirm the regression test interface does not report any errors

    For 4G mode run the correlator using the defined ADC Samples and print
    the correlation results returned by the remote unit under test"""

    ch_filt_1_coeff = "/opt/hhdfphy/LTE_CoefficientFiles/ChanFilt_1_BW_5_0_MHz_v01.dat"
    ch_filt_2_coeff = "/opt/hhdfphy/LTE_CoefficientFiles/ChanFilt_2_BW_5_0_MHz_v01.dat"
    corr_coeff = "/opt/hhdfphy/LTE_CoefficientFiles/Correlator_BW_5_0_MHz_ID_03_CP_144_v01.dat"
    if (adc_samples is None):
        adc_samples = "/opt/hhdfphy/regression_adc_samples/AdcOutputSamplesAsInt.dat"
    accepted = True

    print("Starting test_4g_regression")
    print("adc_samples = " + adc_samples)

    statistics = []
    results = []
    try:
        statistics, results = phy.regression_4g(ch_filt_1_coeff, ch_filt_2_coeff, corr_coeff, adc_samples)
        print("peak time corrPeakNum corrPeakDecSqrd sumCorrIntAtPeak")
        item = 1
        for time, corrPeakNum, corrPeakDecSqrd, sumCorrIntAtPeak in results:
            # Output results in Excel friendly format, simple white space delimination
            print(item, time, corrPeakNum, corrPeakDecSqrd, sumCorrIntAtPeak) 
            item += 1
        print("Statistic numVals PeakPwr Sum")
        item = 1
        for numVals, peakPwr, sum in statistics:
            # Output results in Excel friendly format, simple white space delimination
            print(item, numVals, peakPwr, sum) 
            item += 1

    except Exception as e:
        print(e)
        if e[1][1] != "0":    # bug in hhdf means we are seeing 'r"' rather than 'r0'
            accepted = False
    else:
        phy.abort()

    if accepted != should_accept:
        print("regression_4g {0} in {1} mode.".format(
            "accepted" if accepted else "rejected", mode))


def set_mode_and_wait_for_msg(phy, mode):
    """Set chosen mode and wait for confirmation from uut"""
    print("Setting test mode: ({0})".format(mode))
    phy.set_mode(mode)
    msg = phy.wait_for_msg()
    print("\t%s" % msg)

def main(uut_address, adc_samples):
    """The main function of the test script
    Ensure in 4G mode and then run the correlator test
    """
    # ###############################################################
    # Initialise the PHY interface
    phy = phyint.phyint(uut_address,55555)
    print("    PCB Version  = %s\n" \
          "    FPGA Version = %s\n" \
          "    SW Version   = %s\n" % phy.get_version())

    set_mode_and_wait_for_msg(phy, "3G")  # Force FPGA to be reloaded for each iteration
    set_mode_and_wait_for_msg(phy, "4G")  # Define starting mode
    test_4g_regression(phy, True, "4G", adc_samples)


# ###################################################################
# the main program
if __name__ == '__main__':
    # At time of writing the available sample files are:
    #adc_samples = "/opt/hhdfphy/regression_adc_samples/AdcOutputSamplesAsInt.dat"

    #adc_samples = "/opt/hhdfphy/regression_adc_samples/AdcOutputSamplesImpulseAsInt.dat"
    #adc_samples = "/opt/hhdfphy/regression_adc_samples/AdcOutputSamplesImpulse10AsInt.dat"
    #adc_samples = "/opt/hhdfphy/regression_adc_samples/AdcOutputSamplesImpulse100AsInt.dat"

    #adc_samples = "/opt/hhdfphy/regression_adc_samples/AdcOutputSamplesRealImpulseAsInt.dat"
    #adc_samples = "/opt/hhdfphy/regression_adc_samples/AdcOutputSamplesRealImpulse10AsInt.dat"
    #adc_samples = "/opt/hhdfphy/regression_adc_samples/AdcOutputSamplesRealImpulse100AsInt.dat"

    parser = argparse.ArgumentParser(description ='Run 4g_regression test')
    
    parser.add_argument('-a', '--addr', '--ipaddr', metavar ='ipaddr', 
                    required = False, dest ='ipaddrs', 
                    action ='store', 
                    help ='Target ip address if not using default')
    parser.add_argument('-f', '--file', '--adcfile', metavar ='filename', 
                    required = False, dest ='filenames', 
                    action ='store', 
                    help ='ADC sample files if not using default.  '
                          'Default = /opt/hhdfphy/regression_adc_samples/AdcOutputSamplesAsInt.dat')
    #
    # parse the command-line arguments:
    args = parser.parse_args()

    # Do we have a new ip address of the server?
    #
    if (args.ipaddrs is not None):
        address = args.ipaddrs
        print ("Target IP address is %s\n" % address)
        
    else :
        address = get_defaults.getDefaultHhdfIpAddress()

    if (args.filenames is None):
        # Use all the cases, repeating them to get different decimation phases
        all_cases = ["/opt/hhdfphy/regression_adc_samples/AdcOutputSamplesAsInt.dat",
                     "/opt/hhdfphy/regression_adc_samples/AdcOutputSamplesImpulseAsInt.dat",
                     "/opt/hhdfphy/regression_adc_samples/AdcOutputSamplesImpulse10AsInt.dat",
                     "/opt/hhdfphy/regression_adc_samples/AdcOutputSamplesImpulse100AsInt.dat",
                     "/opt/hhdfphy/regression_adc_samples/AdcOutputSamplesRealImpulseAsInt.dat",
                     "/opt/hhdfphy/regression_adc_samples/AdcOutputSamplesRealImpulse10AsInt.dat",
                     "/opt/hhdfphy/regression_adc_samples/AdcOutputSamplesRealImpulse100AsInt.dat"]
        for adc_samples in all_cases:
            print("\n")
            print("--------------------------------------------------------------------------------------------------------------\n")
            print(adc_samples)
            print("\n")
            print("\n")

            for _ in range(10) :
                main(address, adc_samples)

    else :
        main(address, args.filenames)


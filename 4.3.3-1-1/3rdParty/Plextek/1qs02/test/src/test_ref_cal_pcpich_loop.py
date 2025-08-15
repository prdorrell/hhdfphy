"""
-------------------------------------------------------------------------------
Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/test_ref_cal_pcpich_loop.py $
$Revision: 6452 $
$Author: pdm $
$Date: 2011-07-22 18:03:32 +0100 (Fri, 22 Jul 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Test the reference calibration using the PCPICH method

-------------------------------------------------------------------------------
"""

print "loading libs...",

import re
import sys
import time

import get_defaults
import phyint

print "done.\n"

print_all          = True
print_version_once = True

num_tests = 50

def main(address, band, freq_Hz, code) :
    """The main function of the test script"""
    
    version_printed = False
    
    # ###############################################################
    # Repeat the basic test to gather statistics
    dac_vals = []
    result_strings = []
    for n in range(num_tests) :
        # ###############################################################
        # Initialise the PHY interface
        phy = phyint.phyint(address,55555)

        phy.set_mode("3G");
        _ = phy.wait_for_msg()
        if (not print_version_once) or (not version_printed) :
            print "    PCB Version  = %s\n" \
                  "    FPGA Version = %s\n" \
                  "    SW Version   = %s\n" % phy.get_version()
            version_printed = True
    
        # Set the band
        phy.set_band(band)
        
        # Calibrate the reference
        start = time.time()
        phy.cal_ref_clock("cpich", freq_Hz, code)
        msg = phy.wait_for_msg()
        end = time.time()

        if (msg.find("CAT") == 0) :
            if print_all :
                print "%s" % msg,
    
        elif (msg.find("CC0") == 0) :
            split_msg = re.split( "[ ,]+", msg.strip() )
            dac_vals.append(split_msg[1])
            if print_all :
                print "%s    %f s" % (msg.strip(), (end-start))
                
        result_strings.append(msg.strip('\n'))                
    
    print "Summary:\n"
    for n in range(num_tests) :
        print "%3u : %s" % (n, "".join(result_strings[n]))

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
    
    freq_Hz = 2162200000    #    Off-air
    code_num = 7872         #    On 2162200000,  24/06/2011 : 2031
#    code_num = 3584         #    On 2162200000,  24/06/2011 : 2028

#    freq_Hz = 2152200000    #    Off-air  
#    code_num = 7776         #    On 2152200000,  24/06/2011 : 2027
#    code_num = 5216         #    On 2152200000,  24/06/2011 : 2030
    
#    freq_Hz = 2112800000    #    Off-air  
#    code_num = 3408         #    On 2112800000,  24/06/2011 : 2019
#    code_num =  608         #    On 2112800000,  24/06/2011 : 2027
    
#    freq_Hz = 1930000000    #    Sig. gen.  
#    code_num = 256          #    On 1930000000 
    
    main(address, band, freq_Hz, code_num)

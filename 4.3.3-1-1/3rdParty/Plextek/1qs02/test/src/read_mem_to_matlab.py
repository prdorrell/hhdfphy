"""
-------------------------------------------------------------------------------
Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/read_mem_to_matlab.py $
$Revision: 6380 $
$Author: pdm $
$Date: 2011-07-20 09:32:59 +0100 (Wed, 20 Jul 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Read the contents of the input and output memory. 

-------------------------------------------------------------------------------
"""

print "loading libs...",

from scipy      import io

import get_defaults
import memint
import phyint
import sys
print "done.\n"

def main(address, num_inp_samples, num_out_samples, power_samples) :
    """The main function of the test script"""
    
    # ###############################################################
    # create the PHY interface instance
    phy = phyint.phyint(address, 55555)
    print "FPGA version id: 0x%04x"%( phy.debug_reg_read( 0 ) )

    # ###############################################################
    # read the input sample buffer
    sample_input_even_base = 0x00000000
    sample_input_odd_base  = 0x00400000

    inp_samples_even = memint.read_samples( phy, sample_input_even_base, num_inp_samples, False )

    variable = "inp_samples_even"
    filename = "C:\Documents and Settings\pdm.000\Desktop\Spies\Ames\SampleLogs\inp_samples_even"             

    matlab_data = {variable : inp_samples_even}
    io.savemat(filename, matlab_data, oned_as='column')            

    inp_samples_odd = memint.read_samples( phy, sample_input_odd_base, num_inp_samples, False )

    variable = "inp_samples_odd"
    filename = "C:\Documents and Settings\pdm.000\Desktop\Spies\Ames\SampleLogs\inp_samples_odd"             

    matlab_data = {variable : inp_samples_odd}
    io.savemat(filename, matlab_data, oned_as='column')            

    # ###############################################################
    # read the output sample buffer
    sample_result_even_base = 0x00800000
    sample_result_odd_base  = 0x00c00000

    out_samples_even = memint.read_samples( phy, sample_result_even_base, num_out_samples, power_samples )
    out_samples_even = out_samples_even.astype(float)
    
    variable = "out_samples_even"
    filename = "C:\Documents and Settings\pdm.000\Desktop\Spies\Ames\SampleLogs\out_samples_even"             

    matlab_data = {variable : out_samples_even}
    io.savemat(filename, matlab_data, oned_as='column')            

    out_samples_odd = memint.read_samples( phy, sample_result_odd_base, num_out_samples, power_samples )
    out_samples_odd = out_samples_odd.astype(float)

    variable = "out_samples_odd"
    filename = "C:\Documents and Settings\pdm.000\Desktop\Spies\Ames\SampleLogs\out_samples_odd"             

    matlab_data = {variable : out_samples_odd}
    io.savemat(filename, matlab_data, oned_as='column')            

# ###################################################################
# the main program
if __name__ == '__main__':
    #
    # parse the command-line arguments: if there is one it is the IP
    # address of the server
    if len(sys.argv) > 1:
        address = sys.argv[1]
        print "Target IP address is %s\n" % address 
        
    else :
        address = get_defaults.getDefaultHhdfIpAddress()
    
    num_inp_samples = 5*76800
    num_out_samples = 76800
    power_samples   = True
    main(address, num_inp_samples, num_out_samples, power_samples)

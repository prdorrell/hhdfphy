"""
-------------------------------------------------------------------------------
Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/memint.py $
$Revision: 6998 $
$Author: pdm $
$Date: 2011-08-26 13:33:47 +0100 (Fri, 26 Aug 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Interfaces to the DDR memory for high-level data types.

-------------------------------------------------------------------------------
"""

from numpy  import array
from numpy  import real, imag
from numpy  import all, equal
import struct

import phyint

# #####################################################################
# read sample data
def read_samples( phy, address, num_samples, power_samples = False ):
    raw_data = phy.mem_raw_read( address, num_samples )
    
    samples = []
    if power_samples :
        for a in raw_data:
            pwr=struct.unpack("<I", struct.pack( "<I", a ) )
            samples.append( pwr )

    else :
        for a in raw_data:
            (re,im)=struct.unpack("<hh", struct.pack( "<I", a ) )
            samples.append( re + 1j*im )
                
    samples = array(samples)
    
    return samples

# #####################################################################
# write sample data
def write_samples( phy, samples, address, readback_test ):
    raw_data = []
    ref_data = []
    for c in samples:
        re = int(real(c))
        im = int(imag(c))
        raw_data.append( struct.unpack("<I", struct.pack( "<hh", re, im ) )[0] )
        ref_data.append(re + 1j*im)
        
    phy.mem_raw_write( address, raw_data )
    ref_data = array(ref_data)
    
    if readback_test :
        # read back check
        readback = read_samples( phy, address, len(samples) )
        
        if not all( equal(readback, ref_data) ) :
            raise Exception("Sample readback failed (address = 0x%x)" % address)

# #############################################################################
# the main program
if __name__ == '__main__':
    print "No standalone functionality"

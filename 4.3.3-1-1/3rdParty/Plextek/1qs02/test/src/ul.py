"""
-------------------------------------------------------------------------------
Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/ul.py $
$Revision: 4448 $
$Author: pdm $
$Date: 2011-01-27 10:55:09 +0000 (Thu, 27 Jan 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

test script to send set of command to phy program

-------------------------------------------------------------------------------
"""

import phyint
import time

phy = phyint.phyint("localhost",55555)

phy.set_band( "ul900" )

phy.set_vctcxo( 1500 )
#phy.cal_ref_clock("gps")
#phy.cal_ref_clock("cpich",4500,123,1)
#phy.cal_ref_clock("psch",4500)
phy.start_ul_search( 123, 4500, 4 )
#time.sleep(2.5)

try:
    while(1):  pass
except KeyboardInterrupt:
    phy.abort()



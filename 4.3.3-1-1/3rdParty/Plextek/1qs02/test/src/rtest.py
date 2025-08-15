"""
-------------------------------------------------------------------------------
Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/rtest.py $
$Revision: 6312 $
$Author: pdm $
$Date: 2011-07-15 09:39:50 +0100 (Fri, 15 Jul 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

test script to send set of command to phy program

-------------------------------------------------------------------------------
"""

import phyint
#import time

phy = phyint.phyint("10.10.7.98",55555)


phy.debug_switches( band="wcdma_1", ad_input="RXLBRF", ad_gpio=0xa )
phy.debug_set_frequency( 949.7e6, band="low" )
phy.debug_set_gain( 60 )

#~ try:
    #~ while(1):  pass
#~ except KeyboardInterrupt:
    #~ phy.abort()



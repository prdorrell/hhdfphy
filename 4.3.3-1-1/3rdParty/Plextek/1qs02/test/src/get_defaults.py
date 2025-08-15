"""
-------------------------------------------------------------------------------


-------------------------------------------------------------------------------
  Filename:   get_defaults.py
  Author(s):  pdm
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Functions to get the defaults for various parameters, in particular those that
depend on the host computer.

-------------------------------------------------------------------------------
"""

print "loading libs...",

from socket import gethostname
 
print "done.\n"

def getDefaultHhdfIpAddress() :
    """Get the default IP address of the HHDF, which is based on the current computer"""
    hostName  = gethostname()
    
    ipAddress = "192.168.254.1"
    if hostName == "hhdf-builder" : # Development environment VM
        ipAddress = "192.168.254.1"

    if hostName == "HP706" :        # Matahari lab machine
        ipAddress = "192.168.254.1"

    if hostName == "HP321" :        # PDM's office machine
        ipAddress = "10.10.7.100"

    if hostName == "dan235" :        # lab machine
        ipAddress = "10.10.7.102"

    if hostName == "hpsauce" :       # lab machine
        ipAddress = "10.10.7.102"

    if hostName == "HP339" :        # ABC's debug machine
        ipAddress = "10.10.7.97"
    
    print "Running on %s and target IP address is %s\n" % (hostName, ipAddress) 
    
    return ipAddress

def getDefaultHighSignalPower() :
    """Get the default high-range signal power level"""
    signal_power = (5623**2)/1.3
    
    return signal_power

def getDefaultMidSignalPower() :
    """Get the default mid-range signal power level"""
    signal_power = (2512**2)
    
    return signal_power

def getDefaultLowSignalPower() :
    """Get the default low-range signal power level"""
    signal_power = (1000**2)
    
    return signal_power

# ###################################################################
# the main program
if __name__ == '__main__':
    print "%s" % getDefaultHhdfIpAddress()

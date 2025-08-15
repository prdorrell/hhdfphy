"""
-------------------------------------------------------------------------------
Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/correlator.py $
$Revision: 4448 $
$Author: pdm $
$Date: 2011-01-27 10:55:09 +0000 (Thu, 27 Jan 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

do some correlation stuff to simulate fpga processing

-------------------------------------------------------------------------------
"""


from numpy import array, zeros, concatenate, mean, abs, argmax
from numpy.fft import fft, ifft





def fft_correlate( source, coefs ):
    """ expects coefs in format zeros|conj(fft(k))
        de and re interlaces even odd samples
        to do ? - trims result to remove end effects
    """
    
    lc = len( coefs ) # 2* block length in chips = block length in samples
    ls = len( source )
    print "fft_correlate\tcoef len",lc,"source len",ls
    ans = zeros( ls+lc ) * 1j
    
    for i in range( ls // lc ):
        even = concatenate(( source[i*lc  :(i+1)*lc  :2], zeros(lc/2) ))
        odd  = concatenate(( source[i*lc+1:(i+1)*lc+1:2], zeros(lc/2) ))
        #print i,len(even), len(odd)
        ans[i*lc  :(i+2)*lc  :2] += ifft( fft(even) * coefs )
        ans[i*lc+1:(i+2)*lc+1:2] += ifft( fft(odd)  * coefs )
    
    return (abs(ans)**2).real



class correlator:
    def __init__( self ):
        self.slots = [None]*15
        self.framebuffer = zeros( 76800 * 2 ) # hold this and next frame so end effects can be removed
        self.result = None
        self.start = 0
        self.blocks = 0
        self.threshold = 0
        self.size = 0
    
    def setup( self, mode, start=0, blocks=1, slot_av=1, frame_av=1, threshold=0 ):
        """ mode = only low size/type bits currently recognised
            start = offset in samples to start correlation
            blocks = length of correlation in num of corr blocks
            slot_av = currently ignored
            frame_av = currently ignored """
        mode = mode & 0x03
        if mode == 0:
            # 256 chip / 512 2x samples correlation block size
            size = 512
        elif mode == 1:
            # 2048 chip fixed full frame
            size = 4096
            start = 0
            blocks = 19 # 76800/4096 = 18.75 rounded up = 19
        elif mode == 2:
            # alternating antenna currently ignored same returned for both
            size = 4096
        
        if blocks == 0 : blocks = 1 # we don't like empty result arrays!
        
        self.start = start
        if( start >= 76800 or start < 0 ):
            raise Exception("start sample out of range")
        self.blocks = blocks
        if( blocks*size+start > 2*76800 ):
            raise Exception("number of blocks out of range")
        self.threshold = threshold
        self.size = size
        
        if len(self.slots[0]) != size:
            raise Exception( "size doesn't match loaded coef length %d %d"%(len(self.slots[0]), size) )
    
    def load_slot( self, slot, coefs ):
        self.slots[slot] = coefs
    
    def next_frame( self, frame ):
        # push old samples to left and append next frame
        self.framebuffer = concatenate(( self.framebuffer[76800:], frame ))
        # pre calc channel power now for future use
        self.power = mean( abs( self.framebuffer[:76800] ) ** 2 )
        self.__do_correlate()
        self.__gen_peak_map()
        self.__gen_peak_info()
    
    def __do_correlate( self ):
        # to do, accumulate multiple slots
        # correlate for one window size bigger and throw away ends to get frame without end effects
        sz = self.size
        start = self.start
        stop = start + (self.blocks + 1) * sz
        if( stop-sz > 76800 ):
            stop2 = sz + 76800 # clip the result one frame if more has been done due to corr size not evenly divisible into a frame
        else:
            stop2 = stop 
        self.result = fft_correlate( self.framebuffer[start:stop], self.slots[0] )[sz:stop2]
        
    
    def __gen_peak_map( self ):
        self.peak_map = []
        th = self.threshold * self.power
        print "cor:\t\tth fact %.3g, pwr %.2f, th %.3g, len %d"%(self.threshold, self.power, th, len(self.result))
        for i in range( len(self.result) ):
            x = self.result[i]
            if x > th:
                self.peak_map.append( (i,x) )
    
    def __gen_peak_info( self ):
        self.peak_offset = argmax( self.result )
        self.peak_info = [None]*3
        self.peak_info[0] = self.result[ self.peak_offset - 1 ]
        self.peak_info[1] = self.result[ self.peak_offset ]
        self.peak_info[2] = self.result[ (self.peak_offset + 1 )%len(self.result) ]
        
    def get_peak_info( self ):
        return (self.peak_offset, self.peak_info[0], self.peak_info[1], self.peak_info[2] )
    
    def get_power( self ):
        return self.power
    
    def get_raw( self ):
        return self.result
    


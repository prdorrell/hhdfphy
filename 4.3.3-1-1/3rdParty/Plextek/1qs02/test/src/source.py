"""
-------------------------------------------------------------------------------
Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/source.py $
$Revision: 6090 $
$Author: pdm $
$Date: 2011-06-23 11:31:17 +0100 (Thu, 23 Jun 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

source of chips 
up and down samples to create frames of samples to correlate against

-------------------------------------------------------------------------------
"""

import sys
sys.path.append( "../src" )

import gencodes
from numpy import array, repeat, tile, roll, copy, sinc, zeros, arange, cos, pi
from numpy import mean, min, max, sum, abs, concatenate, conj, flipud
from numpy.random import randint, normal
from numpy.fft import fft, ifft

from pylab      import show, plot, subplot, xlim, ylim, legend, title, suptitle


class composite_source:
    def __init__( self, T, sources, weights=None ):
        self.T = T
        self.sources = sources
        if weights == None :
            self.weights = list(1.0 for _ in range(len(sources)))
            
        else :
            self.weights = weights
    
    def length( self ):
        return 2*2*38400 # two frames
    
    def get_frame(self, num_frames, offset = 0, noise_pwr = 1, ghosts = False ):
        # offset is in fractional samples
        offset = int(round(offset))
        offset %= num_frames * 38400 * self.T
            
        # generate the chips
        chips = zeros(38400*num_frames) * 1j
        
        i = 0
        for src in self.sources:
            k = self.weights[i]
            i += 1
            chips += src.get_chips(num_frames) * k
        
        samples = zeros(self.T*num_frames*38400)*1j
        samples[::self.T] = chips
            
        samples = self.filter(samples)
        
        # apply the offset
        ans = roll(samples, offset)
        
        if ghosts:
            # add a ghost or three
            k1 = 10.0**(-3.0/20.0)
            k2 = 10.0**(-4.0/20.0)
            ans += roll(ans,20) * k1
            ans += roll(ans,-50) * k2
        
        # add unfiltered noise
        if noise_pwr > 0:
            ans += normal( 0, noise_pwr**0.5, 76800 )
        
        return ans
    
    def filter( self, samples ):
        # raise cosine filter
        # we'll do full circular fft convolution as
        # the frames wrapping back on themselves shouldn't be 
        # all that a bad thing due to chip repetition
        T = self.T # T = symbol period in samples = over sample rate
        
        # create the timebase for the filter impulse response
        t = arange(-len(samples)/2, +len(samples)/2, 1.0)
        
        # remove the group delay (works because we implement a circular filtering
        t = roll(t, len(samples)/2)
        
        b = 0.22
        rcos_kernel = sinc( t/T ) * cos( pi * b * t / T ) / ( 1.0 - (2.0*b*t/T)**2 )
        
        # normalise power
        rcos_kernel *= ( T / sum(abs(rcos_kernel)**2))**0.5      
        
        samples = ifft(fft(rcos_kernel)*fft(samples))
        
        return samples
    
    def repeat_not_filter( self ):
        self.samples = repeat( self.chips, self.T )




pilot_bits=[
[ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 ],
[ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 ],
[ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 ],
[ 0x7, 0x4, 0x6, 0x4, 0x5, 0x7, 0x7, 0x5, 0x6, 0x7, 0x6, 0x5, 0x5, 0x4, 0x4 ],
[ 0xF, 0x9, 0xD, 0x9, 0xB, 0xF, 0xF, 0xB, 0xD, 0xF, 0xD, 0xB, 0xB, 0x9, 0x9 ],
[ 0xF, 0xC, 0x16, 0x4, 0x15, 0xF, 0x7, 0x5, 0xE, 0x1F, 0x16, 0x1D, 0x5, 0x1C, 0x1C ],
[ 0x1F, 0x19, 0x2D, 0x9, 0x2B, 0x1F, 0xF, 0xB, 0x1D, 0x3F, 0x2D, 0x3B, 0xB, 0x39, 0x39 ],
[ 0x5F, 0x59, 0x6D, 0x49, 0x6B, 0x5F, 0x4F, 0x4B, 0x5D, 0x7F, 0x6D, 0x7B, 0x4B, 0x79, 0x79 ],
[ 0x7F, 0x75, 0xDD, 0x55, 0xD7, 0x7F, 0x5F, 0x57, 0x7D, 0xFF, 0xDD, 0xF7, 0x57, 0xF5, 0xF5 ] ]

class uplink_dpcch_source:
    def __init__( self, code_num, pilots_per_slot ):
        self.code            = gencodes.gen_uplink_scramble_long(code_num)
        self.pilots_per_slot = pilots_per_slot
    
    def get_chips( self, num_frames ):
        data = []
        for _ in range(num_frames) :
            for slot in range(15):
                for i in range(10):
                    if i < self.pilots_per_slot :
                        data.append( (pilot_bits[self.pilots_per_slot][slot] >> i ) & 1 )
                        
                    else :
                        # not a pilot bit, just make something up
                        #data.append( randint(0,2) )
                        data.append( 0 )
                    
        # map 0->1j, 1->-1j 
        data = array(data) * -2j + 1j
        
        # spreading code is ones(256)
        chips = repeat(data, 256)*tile(self.code, num_frames)
        
        # normalise power
        chips *= 2.0**-0.5
        
        return chips
    
    def get_fir_coefs( self, slot ):
        data = []
        for i in range(self.pilots_per_slot):
            data.append((pilot_bits[self.pilots_per_slot][slot] >> i ) & 1)
                    
        # map 0->1j, 1->-1j 
        data = array(data) * -2j + 1j
        
        # spreading code is ones(256)
        chips = repeat(data, 256)*self.code[slot*2560:slot*2560+self.pilots_per_slot*256]
    
        coefs = flipud(conj(chips))
        
        return coefs

class downlink_cpich_source:
    def __init__( self, code_num, antenna=1 ):
        self.code    = gencodes.gen_downlink_scramble(code_num)
        self.antenna = antenna

    def get_chips( self, num_frames ):
        data = []
        for frame in range(num_frames):
            for slot in range(15):
                for i in range(10):
                    if self.antenna == 1:
                        # all zeros maps to 1+1j
                        data.append( 1+1j )
                        
                    else:
                        # antenna 2
                        # 00 11 11 00 00 11 11 00 00 11 etc maps to 1+1j -1-j etc
                        if ((frame*10*15 + slot*10 + i + 1)/2) % 2 == 0:
                            data.append( 1+1j )
                        else:
                            data.append( -1-1j )
                            
        # spreading code is ones(256)
        chips = repeat(data,256) * tile(self.code, 2 )
        
        # normalise power
        chips *= 0.5
        
        return chips
    
    def get_fir_coefs( self, slot ):
        num_syms = 8
        
        data = []
        for i in range(num_syms):
            if self.antenna == 1:
                # all zeros maps to 1+1j
                data.append( 1+1j )
                
            else:
                # antenna 2
                # 00 11 11 00 00 11 11 00 00 11 etc maps to 1+1j -1-j etc
                if ((slot*10 + i + 1)/2) % 2 == 0:
                    data.append( 1+1j )
                else:
                    data.append( -1-1j )
                            
        # spreading code is ones(256)
        chips = repeat(data, 256)*self.code[slot*2560:slot*2560+num_syms*256]
    
        coefs = flipud(conj(chips))
        
        # rotate by 45 degrees and scale by 1/2 to get values of +/-1+/-j
        coefs = (1+1j)*coefs/2
        
        return coefs

class psch_source:
    def __init__( self ):
        self.code = gencodes.gen_psc()
        
    def get_chips( self, num_frames ):
        chips = concatenate((self.code, zeros(2560-len(self.code))))
        chips = tile(chips, num_frames*15)
        
        # normalise power
        chips *= 2.0**-0.5
        
        return chips
        
    def get_fir_coefs( self, slot=0 ):
        chips = self.code
    
        coefs = flipud(conj(chips))
        
        return coefs




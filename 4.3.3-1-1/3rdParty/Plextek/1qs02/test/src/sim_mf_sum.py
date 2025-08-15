"""
-------------------------------------------------------------------------------
Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/sim_mf_sum.py $
$Revision: 5515 $
$Author: pdm $
$Date: 2011-05-04 19:25:16 +0100 (Wed, 04 May 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Simulates the operation of the matched-filter followed by the summation.

-------------------------------------------------------------------------------
"""

print "loading libs...",

from numpy      import zeros, zeros_like, arange
from numpy      import minimum, maximum, argmax
from numpy      import concatenate, fliplr, flipud, reshape, roll, transpose, copy
from numpy      import real, imag, angle, conj 
from numpy      import all, equal
from numpy      import mod
from numpy      import sqrt
from numpy      import absolute
from numpy      import pi
from numpy      import array
from pylab      import show, plot, subplot, xlim, ylim, legend, title, suptitle
from scipy      import signal

import sys

import corrint
import gencodes
import memint
import phyint
import source
print "done.\n"

check_io        = False
zero_output_mem = False
power_results   = True

frames_to_average = 10
num_result_frames = 1

samplesPerChip     = 2
chipsPerSlot       = 2560
slotsPerFrame      = 15 
samplesPerSlot     = samplesPerChip*chipsPerSlot
samplesPerFrame    = samplesPerSlot*slotsPerFrame

def fir_pwr(ref_chips, inp_samples, accumulate_by_frame) :
    """The main function of the script"""
    ref_chips_ir = copy(ref_chips)
    if ref_chips_ir.ndim == 1 :
        ref_chips_ir.shape = (1, len(ref_chips_ir))
        
    num_filters = ref_chips_ir.shape[0]
    filter_len  = ref_chips_ir.shape[1]
    
    out_samples = zeros((num_filters, len(inp_samples)-2*filter_len), dtype=inp_samples.dtype)
    for filter_num in range(num_filters) :
        #
        #   Create the filter coefficients.
        #
        b = zeros(samplesPerChip*filter_len)*1j
        b[::samplesPerChip] = ref_chips_ir[filter_num, :]
        
        #
        #   Filter the samples.
        #
        z = signal.lfilter(b, 1, inp_samples)
        out_samples[filter_num, :] = roll(z[2*filter_len:], -samplesPerSlot*filter_num)
        
    #
    #   Simulate the gain of the implementation
    #
    if filter_len == 256 :
        impl_gain = 1.0/sqrt(2)/64.0
        
    else :
        impl_gain = 1.0/512.0
        
    out_samples = out_samples*impl_gain

    #
    #   Generate the power samples.
    #
    pwr_samples = abs(out_samples)**2
    
    #
    #   Accumulate the power samples for the slot averaging.
    #
    pwr_samples = pwr_samples.sum(axis=0)
    
    #
    #   Accumulate the power samples for the frame averaging.
    #
    if accumulate_by_frame:
        pwr_samples = reshape(pwr_samples, (-1, samplesPerFrame))
        
    else :
        pwr_samples = reshape(pwr_samples, (-1, samplesPerSlot))
    
    pwr_samples = pwr_samples.sum(axis=0)
    
    #
    #   Calculate the position and magnitude of the peak.
    #
    peak_pos = argmax(pwr_samples)
    peak_pwr = abs(pwr_samples[peak_pos])
    
    return pwr_samples, peak_pos, peak_pwr

# ###################################################################
# the main program
if __name__ == '__main__':
    print "Can only be called as a function"

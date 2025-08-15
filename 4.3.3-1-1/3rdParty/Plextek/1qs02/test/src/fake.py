"""
-------------------------------------------------------------------------------
Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/fake.py $
$Revision: 4852 $
$Author: pdm $
$Date: 2011-02-25 13:33:12 +0000 (Fri, 25 Feb 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

server that simulates some fpga functionality for testing

-------------------------------------------------------------------------------
"""
print "loading libs..."
import socket
import correlator
import source
import struct

from numpy import argmax, max, pi, sin, log10, roll, arange, e, zeros
from numpy.random import normal, uniform
#from pylab import plot, show, figure
print "starting..."

# -----------------------------------------------------------------------------
ul_code = 123
ul_pilots = 4
dl_code = 123
dl_ant = 1
over_sample = 16     # gets down sampled to 2x when frame requested
level_drift = 0.51   # dB per frame change in gain


dac_value = 0
gps_counter = 123456.0  #arb staring point

src = source.composite_source( over_sample,
    (source.uplink_dpcch_source( ul_code, ul_pilots ),
     source.downlink_cpich_source( dl_code, dl_ant ),
     source.psch_source() ),
     ( 1.0, 1.0, 1.0 ) ) # ratios

ghosts = True
noise_pwr = 1.0

cor = correlator.correlator()

frame_counter = 0
sample_offset = 0       # measured in 2x samples
freq_ref = 7.68e6
gain = 0                # dB range 0-127 set by user
level = 0               # dB 
level_min = 40-127
level_max = level_min+120
level_drift_up = True   # whether the level is going up or down at the moment

coef_buffer = [None]*(2**12)
fake_memory = dict()    # for storing peaks and pokes, to do - detect mappings and get raw samples

buffer_using = 0
buffer_requesting = 0

test_mem_area_address = 0xdeadbeef  # requests to this address get some sine info

# -----------------------------------------------------------------------------


def update_test_mem_area():
    global test_mem_area
    test_mem_area = roll( test_mem_area, 100 )
    

def hello():
    c.send( "hello there\r\n" )

def add( x ):
    print "add:", x[0], x[1]
    a = float(x[0]) + float(x[1])
    c.send( "%f\r\n"%(a) )

def dac_to_freq_linear( d ):
    ppm = 20e-6          # ref osc changes +- this amount over dac range
    offset = 200        # 0 Hz error is this many dac bits from dac center
    dac_size = 2**12    # 12 bit dac
    d = float(d)
    f = 7.68e6 * (1.0 - ppm + (d-offset)*ppm*2/dac_size )
    return f

def dac_to_freq_sine( d ):
    # non-linear control curve
    ppm = 20e-6 # ref osc changes +- this amount over dac range
    offset = 200 # 0 Hz error is this many dac bits from dac center
    dac_size = 2**12 # 12 bit dac
    d = float(d)
    f = 7.68e6 * ( 1.0 + ppm * sin( pi * ( d/dac_size - 0.5 ) ) - pi*ppm*offset/dac_size )
    return f

def set_dac( x ):
    global freq_ref
    dac_value = float( x[0] )
    freq_ref = dac_to_freq_linear( dac_value )
    #freq_ref = dac_to_freq_sine( dac_value )
    print "set_dac\t\t",dac_value, freq_ref
    c.send( "ok" )

def get_1pps( ):
    # 25 bit counter running at 26MHz
    
    global gps_counter, freq_ref
    noise = uniform(-1,1) 
    gps_counter += freq_ref/7.68e6*26e6 + noise # counter counts freq_ref counts per second
    gps_counter %= 2**25
    smp = int(gps_counter)
    print "get_1pps\tcnt%8d f %.2f noise %.2f"%(smp,freq_ref,noise)
    c.send( "%d"%(smp) )

def get_frame( ):
    global frame_counter
    print "get_frame\t", frame_counter
    c.send( "%d"%(frame_counter) )

def flush_result():
    # we only have one output buffer and generate 
    # the next frame when ever it is cleared
    print "flush_result\t"
    generate_new_result()
    c.send( "ok" )

def busy():
    pass
    #never used, we're block when busy

def get_channel_pwr():
    global cor
    pwr = cor.get_power()
    print "get_channel_pwr\t%.3g (%.2fdB), level %.2f, gain %d"%(pwr, 10.0*log10(pwr),level, gain)
    c.send( "%g"%(pwr) )

def setup_correlation( p ):
    global cor
    mode       = int( p[0] )
    corr_start = int( p[1] )
    num_blocks = int( p[2] )
    slot_av    = int( p[3] )
    frame_av   = int( p[4] )
    threshold_fact = float( p[5] )
    
    print "setup_corr\tmode\tstart\tblocks\tslotav\tfrmav\tthreshfact"
    print "setup_corr\t%d\t%d\t%d\t%d\t%d\t%.3g"%(mode, corr_start, num_blocks, slot_av, frame_av, threshold_fact)
    cor.setup( mode, corr_start, num_blocks, slot_av, frame_av, threshold_fact )
    c.send( "ok" )

def get_raw_data( p ):
    #print"get raw data"
    global cor
    i = int(p[0])
    c.send( "%f"%(cor.result[i]) )

def get_peaks( p ):
    #print "get peak map"
    i = int( p[0] )
    global cor
    if i >= len(cor.peak_map):
        c.send( "%d %f"%(0xffffffff, 0) )
        print "end of peaks"
    else: 
        # offset, value
        c.send( "%d, %f"%(cor.peak_map[i]) )
        print i,"peak sent:",cor.peak_map[i]

def get_peak():
    global cor
    print "get peak\t%d, %.3g, %.3g, %.3g"%cor.get_peak_info()
    c.send( "%d, %f, %f, %f"%cor.get_peak_info() )

def load_slot( p ):
    # load coeficients into correlator
    global cor, coef_buffer
    slot = int( p[0] )
    size = int( p[1] )
    print "load_slot\t",slot, size
    cor.load_slot( slot, coef_buffer[:size] )
    c.send("ok")

def download_coefs( p ):
    global coef_buffer
    i = int(p[0])
    d = []
    for k in range( (len(p)-1)/2 ):
        d.append( float(p[k*2+1]) + float(p[k*2+2])*1j )
    #print "dlc", i, d
    coef_buffer[i:i+len(d)] = d
    c.send("ok")

def set_gain( p ):
    global gain
    gain = int(p[0])
    print "set gain\t",gain,"cur level",level
    c.send("ok")

def generate_new_result():
    global src, cor, frame_counter, sample_offset, noise_pwr, ghosts, freq_ref, level_drift_up, level
    
    # we always generate 1 frame at a time
    fs = 7.68e6
    spf = 76800
    drift = (freq_ref - fs) * (spf / fs )
    sample_offset = (sample_offset+drift+spf)%(src.length())
    frame_counter += 1
    
    if level_drift_up == True:
        if level < level_max : level += level_drift
        else : 
            print "max level reached"
            level_drift_up = False
    else :
        if level > level_min : level -= level_drift
        else :
            print "min level reached"
            level_drift_up = True
    
    num_frames = 1
    fr = src.get_frame( num_frames, sample_offset, noise_pwr = noise_pwr, ghosts = ghosts )
    fr *= 10.0**((level + gain)/20.0)
    cor.next_frame( fr )
    
    print "gen_new_result\tlen %d, max %.3g, pos %d, pwr %.3g, off %.2f, drift %.3g"%(len(cor.result),max(cor.result),argmax(cor.result),cor.power,sample_offset, drift)

def raw_read( p ):
    global fake_memory
    # just a quick test for peak and poke interface
    # hopefully update all this to reflect operation and
    # memory map of real fpga at some point so can be used to get samples
    address = int(p[0])
    length = int(p[1])
    #print "raw_read\t%d, %d"%(address,length)
    ans = ""
    
    in_test_mem = address >= test_mem_area_address and address <= test_mem_area_address+76800
    
    for i in range( length ):
        if in_test_mem:
            r = test_mem_area[address-test_mem_area_address+i]
            # binary for speed
            ans += struct.pack( "<hh", r.real, r.imag ) 
            
        elif fake_memory.has_key( address+i ):
            ans += str( fake_memory[address+i] )
        else:
            r = int(uniform(-2**15,2**15-1))
            r, = struct.unpack("H",struct.pack( "h",r ) )
            ans += str( r )
            
        #ans += ' '
    c.send(ans)
    #print ans

def raw_write( p ):
    global fake_memory
    # just a quick test for peak and poke interface
    # hopefully update all this to reflect operation and
    # memory map of real fpga at some point so can be used to put samples
    address = int(p[0])
    for d in p[1:]:
        fake_memory[address] = int(d)
        address += 1
    c.send("ok")

def buffer_control( p ):
    global buffer_requesting, buffer_using
    if p[0] == 'set':
        if p[1] != 'R':
            print "trying to set something other than R"
        # to do, actually swap buffers and sync this correctly
        #print "set buf\t\t",int(p[2])
        buffer_requesting = int(p[2])
        if buffer_requesting != buffer_using:
            update_test_mem_area()
            generate_new_result()
        buffer_using = int(p[2])
        c.send("ok")
    else:
        # get
        if p[1] == 'R' : c.send( "%d"%(buffer_requesting) )
        else:            c.send( "%d"%(buffer_using) )
        #print "get buf\t\t",buffer_requesting,buffer_using,"%d,%d"%(buffer_requesting,buffer_using)



# -----------------------------------------------------------------------------
# init correlator stuff etc

# initialise the coefficents to zeros
for slot in range(15):
    cor.load_slot( slot, zeros(512)*1j )
# put the correlator in a valid mode
cor.setup( mode=0, start=0, blocks=150, slot_av=1, frame_av=1, threshold=1 )
# start the correlator so there is valid data for rssi reads etc
generate_new_result()

#test_mem_area = (2**15-1) * e**( 1j*arange(0,20*pi,20*pi/76800) )
num_frames = 1
test_mem_area = 1000.0*src.get_frame( num_frames, 0, noise_pwr = 0, ghosts = False )
# -----------------------------------------------------------------------------

cmds = {
'hello': hello,
'add': add,
'set_dac':set_dac,
'get_1pps':get_1pps,
'get_frame':get_frame,
'get_peaks':get_peaks,
'get_peak':get_peak,
'flush_result':flush_result,
'get_channel_pwr':get_channel_pwr,
'get_raw_data':get_raw_data,
'setup_correlation':setup_correlation,
'load_slot':load_slot,
'dlc':download_coefs,
'set_gain':set_gain,
'raw_read':raw_read,
'raw_write':raw_write,
'buffer':buffer_control
}

host = ""
port = 55556

s = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
s.bind(( host, port ))
s.listen(1)
print "waiting connection..."
c,a = s.accept()
print c, a


while( 1 ):
    d = c.recv(2**14)
    if not d : 
        print "received none, connection closed"
        print "waiting connection..."
        #c.close()
        c,a = s.accept()
        print c, a
        continue
    
    d = d.split()
    
    if d[0] == 'bye': break
    
    fn = cmds.get(d[0])
    if fn:
        #print d[0]
        if len(d)>1: 
            fn( d[1:] )
        else:
            fn()
    else:
        print "unknown", d
        c.send( "que?\r\n" )


c.close()


/******************************************************************************
 * Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 * All Rights Reserved
 ******************************************************************************
 * THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 * ----------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/correlationsource.cpp $
 * $Revision: 5367 $
 * $Author: pdm $
 * $Date: 2011-04-14 15:21:39 +0100 (Thu, 14 Apr 2011) $
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file correlationsource.cpp
 * \brief Correlation chip sources for DPCCH, CPICH, PSCH
 *
 * These chip sources are plugged into the correlator
 * class to define what it correlates against.
 *
 *****************************************************************************/
#include "correlationsource.hpp"
#include "3gpp.h"
#include "tables.hpp"

#include <vector>
#include <complex>
#include <iostream>
#include <fstream>
#include <cassert>

#include <boost/foreach.hpp>


//----------------------------------------------------------------------------------------------------------------------
//
// base
//
//----------------------------------------------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class.
//  @details        The only action is to store the requested debugging level.
//  @param[in]      debug               Controls the debug messages that may be generated.
//
CorrelationSource::CorrelationSource(unsigned debug) :
    blocks(),
    debugStr("Corr Src.... ", debug)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          The destructor has nothing to do.
//
CorrelationSource::~CorrelationSource()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Retrieves a block of reference chips for the specified slot.
//  @param[in]      slot_num            The number of the slot for which the reference chips are needed.
//  @return                             The block of reference chips.
//
const CorrelationSource::block_t &CorrelationSource::get_block(unsigned slot_num)
{
    return blocks[slot_num];
}

//----------------------------------------------------------------------------------------------------------------------
//
// uplink DPCCH
//
//----------------------------------------------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class.
//  @details        Generates the uplink scrambling code for a frame, reads the pilot bits from a lookup table, spreads
//                  them (the channelisation code for the uplink DPCCH is a trivial sequence of 256 '+1's), maps them
//                  onto the imaginary part of a complex chip with zero real component and multiplies the resulting
//                  chips by the scrambling code chips.
//  @param[in]      code_num            The scrambling code number.
//  @param[in]      num_pilots          The number of pilot pits to include in the reference sequence for each slot.
//  @param[in]      debug               Controls the debug messages that may be generated.
//
UplinkDPCCHSource::UplinkDPCCHSource(uint32_t code_num, uint32_t num_pilots, unsigned debug) :
    CorrelationSource(debug)
{
    // build scrambling code
    block_t sframe;
    gen_scramble( code_num, sframe );

    // build pilot bits
    blocks.resize( ThreeGPP::SLOTS_PER_FRAME );
    for( unsigned i = 0; i < ThreeGPP::SLOTS_PER_FRAME; ++i )
    {
        int s = i * ThreeGPP::CHIPS_PER_SLOT; // index into scrambling code

        for( unsigned k = 0; k < num_pilots; ++k )
        {
            // map bit 0 -> 1,  1 -> -1
            double bit = (ThreeGPP::pilotBits[num_pilots][i] >> k ) & 1;
            bit *= -2;
            bit += 1;
            // DPCCH is in Q (imaginary) path
            std::complex<double> q( 0, bit );
            // channalisation code used is C256,0 is ones(256)
            for( int j=0; j<256; ++j )
            {
                // multiply scramble and spread pilots and save chunks
                std::complex<double> v = q*sframe[ s ];
                ++s;
                blocks[i].push_back( v );
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief          The destructor has nothing to do.
///
UplinkDPCCHSource::~UplinkDPCCHSource()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Generates the long uplink scrambling code for a frame.
//  @details        The algorithm to generate the scrambling code is a shift-register implementation like that described
//                  in 25,213 ("3GPP TS 25.213 V9.1.0 (2009-12), Technical Specification, 3rd Generation Partnership
//                  Project; Technical Specification Group Radio Access Network; Spreading and modulation (FDD)
//                  (Release 9)".
//  @param[in]      code_num            The scrambling code number.
//  @param[out]     frame               The scrambling code.
//
void UplinkDPCCHSource::gen_scramble(uint32_t code_num, block_t &frame)
{
    uint32_t x = (code_num & 0x00ffffff)|0x01000000;
    uint32_t y = 0x01ffffff;
    uint32_t x0, y0, xc2, yc2, x25, y25;
    std::complex<double> clong;
    double clong1, clong2;
    double clong2prev = 0;

    for( int i = 0; i < 38400; ++i ){
        x0 = x & 1;
        xc2 = ((x>>4) ^ (x>>7) ^ (x>>18))&1;
        x25 = x0 ^ ((x>>3)&1);
        x = (x >> 1) | (x25<<24);
        y0 = y & 1;
        yc2 = ( (y>>4) ^ (y>>6) ^ (y>>17) )&1;
        y25 = (y0 ^ (y>>1) ^ (y>>2) ^ (y>>3))&1;
        y = (y >> 1) | (y25<<24);
        clong1 = static_cast<double>(x0^y0)*-2 + 1;    // 0 -> 1, 1 -> -1
        clong2 = static_cast<double>(xc2^yc2)*-2 + 1;
        if( i%2 == 0 ){
            clong = std::complex<double>( clong1, clong1*clong2 );
        }else{
            clong = std::complex<double>( clong1, -clong1*clong2prev );
        }
        clong2prev = clong2;
        frame.push_back(clong);
    }
}

//----------------------------------------------------------------------------------------------------------------------
//
// downlink CPICH
//
//----------------------------------------------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class.
//  @details        Generates the downlink scrambling code for a frame and generates the pilot symbols from the known
//                  patterns defined in 25.211 ("3GPP TS 25.211 V9.1.0 (2009-12), Technical Specification, 3rd
//                  Generation Partnership Project; Technical Specification Group Radio Access Network; Physical
//                  channels and mapping of transport channels onto physical channels (FDD) (Release 9)").  Spreads
//                  the pilot symbols (the channelisation code for the downlink PCPICH is a trivial sequence of 256
//                  '+1's) and multiplies the resulting chips by the scrambling code chips.  The resulting chips
//                  have values of +2, -2, +2j or -2j, so they are multiplied by (1+j)/2 so that they have values from
//                  the set { +/-1+/-j }, just like the other sources.
//  @param[in]      code_num            The scrambling code number.
//  @param[in]      antenna             The Node B antenna for which the reference should be generated.
//  @param[in]      syms                The number of symbols to be used.
//  @param[in]      debug               Controls the debug messages that may be generated.
//  @param[in]      debug_files         true if the scrambling code should be written to a file.
//
DownlinkPCPICHSource::DownlinkPCPICHSource(uint32_t code_num,
                                           uint32_t antenna,
                                           uint32_t syms,
                                           unsigned debug,
                                           bool debug_files) :
    CorrelationSource(debug)
{
    block_t sframe;
    gen_pri_scramble( code_num, sframe );

    if( debug_files )
    {
        std::ofstream dumpfile( "dump/scramdump.txt", std::ios::out | std::ios::trunc );
        BOOST_FOREACH( block_t::value_type x, sframe )
        {
            dumpfile<< x.real() << ' ' << x.imag() <<'\n';
        }
        dumpfile.close();
    }

    blocks.resize( ThreeGPP::SLOTS_PER_FRAME );

    for( unsigned slot = 0; slot < ThreeGPP::SLOTS_PER_FRAME; ++slot )
    {
        unsigned chip_in_frame = slot * ThreeGPP::CHIPS_PER_SLOT;

        for( unsigned chip_in_slot = 0; chip_in_slot < syms*ThreeGPP::PCPICH_SF ; ++chip_in_slot, ++chip_in_frame )
        {
            std::complex<double> sym;
            if( antenna == 1 )
            {
                sym = std::complex<double>( 1, 1 );
            }
            else
            {
                int sym_in_frame = chip_in_frame / ThreeGPP::PCPICH_SF;

                //
                //  According to the standard, symbols 0 and 3 in every group of four in the frame ar +1+j and symbols
                //  1 and 2 are -1-j.
                //
                if( ((sym_in_frame % 4) == 0) | ((sym_in_frame % 4) == 3) )
                {
                    sym = std::complex<double>( 1, 1 );
                }
                else
                {
                    sym = std::complex<double>( -1, -1 );
                }
            }

            //
            //  The channelisation code is 256 x +1 so the chip equals the symbol.
            //

            //
            //  Rotate the chips by 1+j and divide by 2 to produce values from the set +/-1+/-j so all sources have the
            //  same magnitude.
            //
            blocks[slot].push_back( std::complex<double>( 1, 1 )*sframe[chip_in_frame]*sym/2.0 );
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          The destructor has nothing to do.
//
DownlinkPCPICHSource::~DownlinkPCPICHSource()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Generates the primary downlink scrambling code for a frame.
//  @details        The algorithm to generate the scrambling code is a shift-register implementation like that
//                  described in 25,213 ("3GPP TS 25.213 V9.1.0 (2009-12), Technical Specification, 3rd Generation
//                  Partnership Project; Technical Specification Group Radio Access Network; Spreading and modulation
//                  (FDD) (Release 9)".
//                  The slight difference is that the initialisation of the "x" shift register is obtained from a
///                 lookup table that has been generated by shifting the default initialisation by the code number.
//  @param[in]      code_num            The scrambling code number.
//  @param[out]     frame               The scrambling code.
//
void DownlinkPCPICHSource::gen_pri_scramble(uint32_t code_num, block_t& frame)
{
    assert( code_num <= 0x1fff );

    // look up precalculated primary start points for a bit of speed
    uint32_t x = Tables::primary_dlscram_xinit[code_num/ThreeGPP::PRIMARY_SCRAMBLING_CODE_SEP];

    // gen code
    uint32_t y = 0x0003ffff;
    uint32_t x0,xx,x18,y0,yy,y18;

    for( int i=0; i<38400; ++i )
    {
        x0 = x & 1;
        xx = ((x>>15) ^ (x>>6) ^ (x>>4)) & 1;
        x18 = ((x) ^ (x>>7)) & 1;
        x = (x>>1) | (x18<<17);
        y0 = y & 1;
        yy = ((y>>15) ^ (y>>14) ^ (y>>13) ^ (y>>12) ^ (y>>11) ^ (y>>10) ^ (y>>9) ^ (y>>8) ^ (y>>6) ^ (y>>5)) & 1;
        y18 = ((y) ^ (y>>5) ^ (y>>7) ^ (y>>10)) & 1;
        y = (y>>1) | (y18<<17);
        double zr = double( x0^y0 ) *-2 + 1;
        double zi = double( xx^yy ) *-2 + 1;
        frame.push_back( std::complex<double>(zr, zi) );
    }
}

//----------------------------------------------------------------------------------------------------------------------
//
// downlink PSCH
//
//----------------------------------------------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class.
//  @details        The PSCH is a fixed sequence of 256 chips that appears at the start of each downlink slot.  The
//                  generation of the reference sequence simply involves copying the chips into the base class' buffer
//                  for each slot in a frame.
//  @param[in]      debug               Controls the debug messages that may be generated.
//
DownlinkPSCHSource::DownlinkPSCHSource(unsigned debug) :
    CorrelationSource(debug)
{
    using namespace std;
    blocks.resize( ThreeGPP::SLOTS_PER_FRAME );

    BOOST_FOREACH( block_t& slot, blocks )
    {
        for( size_t i=0; i<Tables::psc_len; ++i )
        {
            slot.push_back( complex<double>(Tables::psc[i*2], Tables::psc[i*2+1]) );
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          The destructor has nothing to do.
//
DownlinkPSCHSource::~DownlinkPSCHSource()
{
}

/******************************************************************************
 * Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 * All Rights Reserved
 ******************************************************************************
 * THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 * ----------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/dac.cpp $
 * $Revision: 6338 $
 * $Author: pdm $
 * $Date: 2011-07-15 14:57:08 +0100 (Fri, 15 Jul 2011) $
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file dac.cpp
 * \brief Serial bit bash for VCTCXO DAC ( AD5320 )
 *
 *****************************************************************************/
#include "dac.hpp"
#include "gpio.hpp"
#include "design.hpp"
#include "debug.hpp"
#include <stdexcept>
#include <cassert>
#include <boost/format.hpp>

/// This module provides interface to the VCTCXO DAC
namespace DAC
{

/// debug tracing output
DebugStream debugStr( "DAC......... " );

///
/// The last value written to the DAC.
///
unsigned int last_val = 0;

// ////////////////////////////////////////////////////////////////////////////
/// Perform spi write to dac
//
/// vc input of referance vctcxo is connected to a 12 bit dac
/// \param [in] v the value to set (12 bit)
//
void set( unsigned int v )
{
    if (v > Design::MAX_DAC_VAL)
    {
        debugStr(DebugStream::error) << boost::format(  "DAC control value (%u) out of range - it will be limited to %u\n")
                                                      % v
                                                      % Design::MAX_DAC_VAL;
        v = Design::MAX_DAC_VAL;
    }

    debugStr(DebugStream::info2) << "set " << v << '\n';

    // bits 13, 12 = 00 = normal operation
    // bits 15, 14 = xx = don't care

    // grab the gpio lines we need for this operation, will open appropriate files
    gpio spi_csn ( gpio::ADI5320_CS,   gpio::out_high,   debugStr.get_state() );
    gpio spi_sdio( gpio::ADI5320_MOSI, gpio::out,        debugStr.get_state() );
    gpio spi_sclk( gpio::ADI5320_CLK,  gpio::out_high,   debugStr.get_state() );

    spi_csn.set( 0 );

    int i;
    for( i = 15; i >= 0; --i )
    {
        spi_sdio.set( (v>>i)&1 );
        spi_sclk.set( 1 );
        spi_sclk.set( 0 ); // clock in on falling edge
    }

    spi_csn.set( 1 );

    last_val = v;
}

// ////////////////////////////////////////////////////////////////////////////
/// Get the last DAC value to have been set
//
/// \return the value
//
unsigned int get( void )
{
    return (last_val);
}

// /////////////////////////////////////////////////////////////////////////////
/// change debug output verbosity
/// \param [in] level verbosity bit field
//
void debug_printing ( unsigned level )
{
    debugStr.show_level( level );
}

} //namespace DAC

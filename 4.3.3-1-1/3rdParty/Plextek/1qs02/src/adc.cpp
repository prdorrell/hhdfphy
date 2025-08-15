/******************************************************************************
 * Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 * All Rights Reserved
 ******************************************************************************
 * THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 * ----------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/adc.cpp $
 * $Revision: 7469 $
 * $Author: pdm $
 * $Date: 2011-10-06 13:26:22 +0100 (Thu, 06 Oct 2011) $
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file adc.hpp
 * \brief interface to AD9863
 *
 *****************************************************************************/
#include "adc.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include <cassert>
#include <boost/format.hpp>

/// this module is the bit bashed spi driver for the AD9863
namespace ADC
{

/// debug tracing output
DebugStream debugStr("ADC......... ");


// /////////////////////////////////////////////////////////////////////////////
/// Write value to adc
//
/// Assumes default msb first mode
/// \param [in] address register address to write to
/// \param [in] data to write into register
//
void SPI_write( uint32_t address, uint32_t data )
{
    debugStr(DebugStream::info2) << boost::format("spi write address %#x, data %#x\n") % address % data;
    assert( address < 0x3f );
    assert( data <= 0xff );

    // grab the gpio lines we need for this operation, will open appropriate files
    gpio spi_csn ( gpio::ADI9863_CS,   gpio::out_high,   debugStr.get_state() );
    gpio spi_sdio( gpio::ADI9863_MOSI, gpio::out,        debugStr.get_state() );
    gpio spi_sclk( gpio::ADI9863_CLK,  gpio::out_low,    debugStr.get_state() );

    spi_csn.set( 0 );

    int i;
    // for 1byte write no mod to address needed
    for( i = 7; i >= 0; --i )
    {
        spi_sdio.set( (address>>i)&1 );
        spi_sclk.set( 1 );
        spi_sclk.set( 0 );
    }
    for( i = 7; i >= 0; --i )
    {
        spi_sdio.set( (data>>i)&1 );
        spi_sclk.set( 1 );
        spi_sclk.set( 0 );
    }

    spi_csn.set( 1 );
}

// /////////////////////////////////////////////////////////////////////////////
/// Set up the ADC in the mode we need.
//
void initialise( bool mode3G )
{
    // configure low power mode by resetting the ADC and asserting the low power while it comes out of reset.
    gpio reset   ( gpio::ADI9863_ADC_RESET_N,   gpio::out_low,  debugStr.get_state() );
    gpio lowpwr  ( gpio::ADI9863_ADC_LO_PWR,    gpio::out_high, debugStr.get_state() );
    reset.set( 1 );

    // tx off = high
    gpio txpwrdwn( gpio::ADI9863_TXPWRDWN,    gpio::out_high, debugStr.get_state() );
    // rx on = low
    gpio rxpwrdwn( gpio::ADI9863_RXPWRDWN,    gpio::out_low,  debugStr.get_state() );

    // set up for half duplex 24bit receive mode
    // see table 15 of data sheet
    SPI_write( 0x00, 0x00 );  // default msb first mode, 4 wire
    SPI_write( 0x01, 0x04 );  // standard clk_mode, enable iface2 clock out
    SPI_write( 0x02, 0xf4 );  // power down tx, pll off, but connected
    SPI_write( 0x03, 0x40 );  // don't power down rx, DC-coupled => disabled internal bias circuits
    SPI_write( 0x04, 0x40 );  // don't power down rx, DC-coupled => disabled internal bias circuits
    SPI_write( 0x05, 0x00 );  // don't power down rx
    SPI_write( 0x06, 0x20 );  // 2's complement, no DCS
    SPI_write( 0x07, 0x20 );  // 2's complement, no DCS
    SPI_write( 0x08, 0x00 );  // don't use ultra low power mode
    SPI_write( 0x09, 0x00 );  // don't use ultra low power mode
    SPI_write( 0x0a, 0x00 );  // don't use ultra low power mode
    SPI_write( 0x0b, 0x00 );  // 0x0b-0x12 = tx stuff
    SPI_write( 0x0c, 0x00 );  // 0x0b-0x12 = tx stuff
    SPI_write( 0x0d, 0x00 );  // 0x0b-0x12 = tx stuff
    SPI_write( 0x0e, 0x00 );  // 0x0b-0x12 = tx stuff
    SPI_write( 0x0f, 0x00 );  // 0x0b-0x12 = tx stuff
    SPI_write( 0x10, 0x00 );  // 0x0b-0x12 = tx stuff
    SPI_write( 0x11, 0x00 );  // 0x0b-0x12 = tx stuff
    SPI_write( 0x12, 0x00 );  // 0x0b-0x12 = tx stuff
    SPI_write( 0x13, 0x00 );  // binary (can be 2's complement)
    SPI_write( 0x14, 0x00 );  // HD 24 mode
    if (mode3G)
    {
        SPI_write( 0x15, 0x00 );  // use CLKIN1
    }
    else
    {
        SPI_write( 0x15, 0xB0 );  // use CLKIN2, bypassing the PLL but dividing the clock by 2
    }
    SPI_write( 0x16, 0x00 );  // clock stuff
}

// /////////////////////////////////////////////////////////////////////////////
/// Configure 3G mode.
//
void set_3G_mode(void)
{
    SPI_write( 0x15, 0x00 );  // use CLKIN1
}

// /////////////////////////////////////////////////////////////////////////////
/// Configure GSM mode.
//
void set_GSM_mode(void)
{
    SPI_write( 0x15, 0xB0 );  // use CLKIN2, bypassing the PLL but dividing the clock by 2
}

// /////////////////////////////////////////////////////////////////////////////
/// Enter the low-power mode.
//
void enter_low_power_mode( void )
{
    SPI_write( 0x02, 0xfe );  // power down tx and rx analog components, and the tx and rx digital components
    SPI_write( 0x03, 0xc0 );  // power down rx_A, DC-coupled => disabled internal bias circuits
    SPI_write( 0x04, 0xc0 );  // power down rx_B, DC-coupled => disabled internal bias circuits
    SPI_write( 0x05, 0xf0 );  // power down rx analog bias, references
    debugStr(DebugStream::info1) << "Entering low power mode\n";
}

// /////////////////////////////////////////////////////////////////////////////
/// Exit the low-power mode and restore normal operation.
//
void exit_low_power_mode( bool mode3G )
{
    debugStr(DebugStream::info1) << "Exiting low power mode\n";

    SPI_write( 0x02, 0xf4 );  // power down tx, pll off, but connected
    SPI_write( 0x03, 0x40 );  // don't power down rx, DC-coupled => disabled internal bias circuits
    SPI_write( 0x04, 0x40 );  // don't power down rx, DC-coupled => disabled internal bias circuits
    SPI_write( 0x05, 0x00 );  // don't power down rx
}

// /////////////////////////////////////////////////////////////////////////////
/// Set debug verbosity
/// \param [in] l verbosity level (bit field)
//
void debug_printing( unsigned l )
{
    debugStr.show_level( l );
}

}//namespace ADC

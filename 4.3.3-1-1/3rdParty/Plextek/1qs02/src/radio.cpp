/******************************************************************************
 *
 *
 ******************************************************************************
 *  Filename:   radio.cpp
 *  Author(s):  pdm
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file radio.cpp
 * \brief Interface to radio chip
 * and multi way switch
 *
 *****************************************************************************/
#include "radio.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "pcb.hpp"

#include <iostream>
#include <cassert>
#include <time.h>
#include <errno.h>
#include <stdexcept>

#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

/// Radio module consists of a set of free functions in namespace Radio
namespace Radio{

/// type relating text band name to frequency range
struct band_lookup_t
{
    char const * const str; ///< band name
    freq_t const low;       ///< low band edge
    freq_t const high;      ///< high band edge
};

/// Table to convert names to frequency ranges
band_lookup_t const band_lookup[] =
{
    { "UL800",         832000000u,  862000000u },
    { "DL800",         791000000u,  821000000u },
    { "UL850",         824000000u,  849000000u },
    { "DL850",         869000000u,  894000000u },
    { "UL900",         880000000u,  915000000u },
    { "DL900",         925000000u,  960000000u },
    { "UL1700",       1710000000u, 1755000000u },
    { "DL1700",       2110000000u, 2155000000u },
    { "UL1800",       1710000000u, 1785000000u },
    { "DL1800",       1805000000u, 1880000000u },
    { "UL1900",       1850000000u, 1910000000u },
    { "DL1900",       1930000000u, 1990000000u },
    { "UL2100",       1920000000u, 1980000000u },
    { "DL2100",       2110000000u, 2170000000u },
    { "AUTO",          824000000u, 2170000000u },
    { "LOAD",          824000000u, 2170000000u },
    { "INVALID_BAND",          1u,          0u }
};

/// type relating frequency range to band
struct freq_lookup_t
{
    freq_t const low;       ///< low band edge
    freq_t const high;      ///< high band edge
    band_t band;            ///< band id
};

/// Table to convert frequency to band
freq_lookup_t const freq_lookup[] =
{
    {  849000001u,  862000000u,  UL800 },
    {  791000000u,  821000000u,  DL800 },
    {  824000000u,  849000000u,  UL850 },
    {  869000000u,  879999999u,  DL850 },
    {  880000000u,  915000000u,  UL900 },
    {  925000000u,  960000000u,  DL900 },
    { 1710000000u, 1785000000u, UL1800 },
    { 1805000000u, 1849999999u, DL1800 },
    { 1850000000u, 1910000000u, UL1900 },
    { 1920000000u, 1980000000u, UL2100 },
    { 1980000001u, 1990000000u, DL1900 },
    { 2110000000u, 2170000000u, DL2100 }
};

/// The frequency resolution in Hz.
static const double freq_res_Hz = 50e3;

/// debug tracing output
DebugStream debugStr("Radio....... ");

/// shadow copies of some regs so they can
/// be partially updated by functions
namespace ADF4602_shadow{
    uint32_t r1;    ///< enable various blocks in chip
    uint32_t r22;   ///< gpio
}

///
/// copies of the current configuration parameters.
///
static band_t   currBand = LOAD_BAND;
static freq_t   currFreq = 1930000000;
static uint32_t currGain = 0;

namespace ADF4602_reg1{
//would there be problems with endieness if using a bitfield/int union?
enum{
    rxsynthen   = 1,
    rxlb        = 1<<1,
    rxhb1       = 2<<1,
    rxhb2       = 3<<1,
    txsynthen   = 1<<3,
    txbs        = 1<<4,
    txen        = 1<<5,
    vsup1       = 1<<6,
    vsup2       = 1<<7,
    vsup3       = 1<<8,
    vsup4       = 1<<9,
    vsup5       = 1<<10,
    vsupall     = 0x1f<<6,
    chipclken   = 1<<11,
    refclken    = 1<<12,
    rxen        = 1<<13
};
}

namespace ADF4602_reg15{
enum{
    sdmosr      = 1,
    rxbw_WCDMA5 = 0,
    rxbw_WCDMA7 = 2 << 6,
    rxbw_GSM    = 7 << 6,
    swapi       = 1 << 9,
    swapq       = 1 << 10,
    vcmsel      = 1 <<11
};

const int GAINCAL_WCDMA = 8;
const int GAINCAL_GSM   = 17;

}

// ////////////////////////////////////////////////////////////////////////////
/// Wrapper around nanosleep to do fractional sleeps
void frac_sleep( double s )
{
    timespec t;
    int sec = static_cast<int>(s);
    long ns = static_cast<int>((s-sec+0.5e-9)*1e9);
    t.tv_sec = sec;
    t.tv_nsec = ns;
    while( nanosleep( &t, &t ) == -1 && errno == EINTR );
    // to do : do something if another error occurs
}

// /////////////////////////////////////////////////////////////////////////////
/// Bit bash a word of bits
//
/// Details of controlling gpio lines to perform a spi write to the ADF4602
void ADF4602_SPI_write( uint32_t word )
{
    debugStr(DebugStream::info2)<<boost::format("SPI write ADF4602 with %#x\n") % word;

    // grab the gpio lines we need for this operation, will open appropriate files
    gpio spi_sen ( gpio::ADI4602_CS,   gpio::out_high,   debugStr.get_state() );
    gpio spi_sdio( gpio::ADI4602_MOSI, gpio::out,        debugStr.get_state() );
    gpio spi_sclk( gpio::ADI4602_CLK,  gpio::out_low,    debugStr.get_state() );

    spi_sen.set( 0 );

    for( int i = 25; i >= 0; --i )
    {
        spi_sdio.set( (word>>i)&1 );
        spi_sclk.set( 1 );  // clocked on rising edge
        spi_sclk.set( 0 );
    }

    spi_sen.set( 1 ); // transfered to reg on rise
}

// /////////////////////////////////////////////////////////////////////////////
/// Format the bits for a normal address type write
void ADF4602_SPI_write( uint32_t address, uint32_t data )
{
    uint32_t word = 0x01; //write command and cs=1 in low 5 bits
    word |= (data&0xffff)<<10;
    word |= (address&0x1f)<<5;
    ADF4602_SPI_write( word );
}

// ////////////////////////////////////////////////////////////////////////////
/// Format the bits for a 8 bit sub address type write
void ADF4602_SPI_write( uint32_t address, uint32_t sub_address, uint32_t data )
{
    assert( address == 0 );
    uint32_t word = 0x01;
    word |= (data&0xff)<<18;
    word |= (sub_address&0xff)<<10;
    ADF4602_SPI_write( word );
}

// ////////////////////////////////////////////////////////////////////////////
/// Initialse the radio module
//
/// Set up the RF1480 switch
/// and the ADF4602 radio chip
void initialise(bool mode3G)
{
    set_RF1480( LOAD50 ); // isolate to load resistor

    // set up 4602
    ADF4602_SPI_write( 2, 0x0003 );     // soft reset, 2.8V logic
    frac_sleep( 50e-6 );

    // hack to guarentee factory NVM transfer to regs
    ADF4602_SPI_write( 0, 151, 0xE0 );  // set vsup2 to 3.1V
    ADF4602_SPI_write( 31, 0x0010 );    // start NVM transfer
    frac_sleep( 200e-6 );
    ADF4602_SPI_write( 0, 0x0000 );     // reset bit
    ADF4602_SPI_write( 0, 151, 0x6F );  // return vsup2 to 2.8V
    // end hack

    using namespace ADF4602_reg1;
    using namespace ADF4602_reg15;

    if (mode3G)
    {
        ADF4602_shadow::r1 =   rxen         // receiver is needed
                             | refclken     // provides the 26 MHz clock to the ADC
                             | chipclken    // provides the 19.2 MHz clock to the ADC
                             | vsup5        // transmit baseband, modulator, DAC2, and GPOs
                             | vsup3        // receive LNAs
                             | vsup2        // receive baseband and down-converter
                             | vsup1        // receive VCO
                             | rxlb         // RX low-band enable
                             | rxsynthen;   // receiver is needed

        ADF4602_SPI_write( 1, ADF4602_shadow::r1 );
        ADF4602_SPI_write( 12, 0x0FA6 );    // datasheet default
        ADF4602_SPI_write( 13, 0x103E );    // datasheet default
        ADF4602_SPI_write( 14, 0xDD43 );    // recommended by DC offset correction application note for 3G mode
        ADF4602_SPI_write( 15, (GAINCAL_WCDMA << 1) | rxbw_WCDMA7 | vcmsel );
    }
    else
    {
        ADF4602_shadow::r1 =   rxen         // receiver is needed
                             | refclken     // provides the 26 MHz clock to the ADC
                             | vsup5        // transmit baseband, modulator, DAC2, and GPOs
                             | vsup3        // receive LNAs
                             | vsup2        // receive baseband and down-converter
                             | vsup1        // receive VCO
                             | rxlb         // RX low-band enable
                             | rxsynthen;   // receiver is needed

        ADF4602_SPI_write( 1, ADF4602_shadow::r1 );
        ADF4602_SPI_write( 12, 0x0FA6 );    // datasheet default
        ADF4602_SPI_write( 13, 0x103E );    // datasheet default
        ADF4602_SPI_write( 14, 0x8043 );    // recommended by DC offset correction application note for GSM mode
        ADF4602_SPI_write( 15, (GAINCAL_GSM   << 1) | rxbw_GSM    | vcmsel );
    }

    ADF4602_SPI_write( 21, 0x001F );    // datasheet default
    ADF4602_shadow::r22 = 0x8000;       // datasheet default dac and GPO manual control
    ADF4602_SPI_write( 22, ADF4602_shadow::r22 );
    ADF4602_SPI_write( 0, 144, 0x06 );  // datasheet default
    ADF4602_SPI_write( 0, 155, 0x78 );  // datasheet default
    ADF4602_SPI_write( 0, 153, 0x85 );  // datasheet default
    ADF4602_SPI_write( 0, 165, 0x20 );  // datasheet default
    ADF4602_SPI_write( 0, 170, 0x00 );  // disable modulator datasheet default 0xf0
    ADF4602_SPI_write( 0, 171, 0x04 );  // datasheet default tx irrelevent?
    ADF4602_SPI_write( 0, 174, 0x5f );  // datasheet default tx irrelevent?
    ADF4602_SPI_write( 0, 175, 0x14 );  // datasheet default
    ADF4602_SPI_write( 11, 0x0050 );    // datasheet default
    // ignore steps 22-25 : don't do the synths or set tx output etc

    set_gain( currGain );
}

// /////////////////////////////////////////////////////////////////////////////
/// Configure 3G mode.
//
void set_3G_mode(void)
{
    using namespace ADF4602_reg1;
    using namespace ADF4602_reg15;

    ADF4602_shadow::r1 &= (0x03<<1);
    ADF4602_shadow::r1 |=   rxen         // receiver is needed
                          | refclken     // provides the 26 MHz clock to the ADC
                          | chipclken    // provides the 19.2 MHz clock to the ADC
                          | vsup5        // transmit baseband, modulator, DAC2, and GPOs
                          | vsup3        // receive LNAs
                          | vsup2        // receive baseband and down-converter
                          | vsup1        // receive VCO
                          | rxsynthen;   // receiver is needed

    ADF4602_SPI_write( 1, ADF4602_shadow::r1 );


    ADF4602_SPI_write( 15, (GAINCAL_WCDMA << 1) | rxbw_WCDMA7 | vcmsel );
}

// /////////////////////////////////////////////////////////////////////////////
/// Configure GSM mode.
//
void set_GSM_mode(void)
{
    using namespace ADF4602_reg1;
    using namespace ADF4602_reg15;

    ADF4602_shadow::r1 &= (0x03<<1);
    ADF4602_shadow::r1 |=   rxen         // receiver is needed
                          | refclken     // provides the 26 MHz clock to the ADC
                          | vsup5        // transmit baseband, modulator, DAC2, and GPOs
                          | vsup3        // receive LNAs
                          | vsup2        // receive baseband and down-converter
                          | vsup1        // receive VCO
                          | rxsynthen;   // receiver is needed

    ADF4602_SPI_write( 1, ADF4602_shadow::r1 );



    ADF4602_SPI_write( 15, (GAINCAL_GSM   << 1) | rxbw_GSM    | vcmsel );
}

// /////////////////////////////////////////////////////////////////////////////
/// Enter the low-power mode.
//
void enter_low_power_mode( void )
{
    using namespace ADF4602_reg1;

    //
    //  Leave the clocks and band selection unchanged, and leave the GPOs
    //  enabled, but switch everything else off.
    //
    ADF4602_shadow::r1 &= 0x1806;
    ADF4602_shadow::r1 |=   vsup5        // transmit baseband, modulator, DAC2, and GPOs
                          | vsup2;       // receive baseband and down-converter

    ADF4602_SPI_write( 1, ADF4602_shadow::r1 );
    debugStr(DebugStream::info1) << "Entering low power mode\n";
}

// /////////////////////////////////////////////////////////////////////////////
/// Exit the low-power mode and restore normal operation.
//
void exit_low_power_mode(bool mode3G)
{
    using namespace ADF4602_reg1;

    debugStr(DebugStream::info1) << "Exiting low power mode\n";

    if (mode3G)
    {
        set_3G_mode();
    }
    else
    {
        set_GSM_mode();
    }

//    initialise(mode3G);
    set_frequency(currBand, currFreq);
    set_gain(currGain);
}

// /////////////////////////////////////////////////////////////////////////////
/// Change band switches
//
/// When a new band is being used call before setting frequency
static void set_band( band_t band, bool force_load )
{
    debugStr(DebugStream::info3) << "change band to " << band_to_string( band ) << '\n';
    unsigned int    gpio = 0;
    ADF4602_input_t AD_input = RXLBRF;
    RF1480_band_t   sw_band = GSM_LB_TX;

    // perhaps should make this a table lookup
    unsigned pcbRev = Pcb::getPcbRev();
    if (pcbRev == 0)
    {
        switch( band )
        {
            case UL850:
                AD_input = RXLBRF;
                gpio = 0x6;
                sw_band = GSM_LB_TX;
                break;
            case DL850:
                AD_input = RXLBRF;
                gpio = 0x0;
                sw_band = GSM_LB_TX;
                break;
            case UL900:
                AD_input = RXLBRF;
                gpio = 0xe;
                sw_band = WCDMA_1;
                break;
            case DL900:
                AD_input = RXLBRF;
                gpio = 0x8;
                sw_band = WCDMA_1;
                break;
            case UL1700:
                AD_input = RXHB1RF;
                gpio = 0x3;
                sw_band = GSM_HB_TX;
                break;
            case DL1700:
                AD_input = RXHB2RF;
                gpio = 0x8;
                sw_band = WCDMA_3;
                break;
            case UL1800:
                AD_input = RXHB1RF;
                gpio = 0x3;
                sw_band = GSM_HB_TX;
                break;
            case DL1800:
                AD_input = RXHB1RF;
                gpio = 0x0;
                sw_band = GSM_RX_2;
                break;
            case UL1900:
                AD_input = RXHB2RF;
                gpio = 0x6;
                sw_band = WCDMA_2;
                break;
            case DL1900:
                AD_input = RXHB2RF;
                gpio = 0x0;
                sw_band = WCDMA_2;
                break;
            case UL2100:
                AD_input = RXHB2RF;
                gpio = 0xe;
                sw_band = WCDMA_3;
                break;
            case DL2100:
                AD_input = RXHB2RF;
                gpio = 0x8;
                sw_band = WCDMA_3;
                break;
            case AUTO_BAND:
            case LOAD_BAND:
            case INVALID_BAND:
            default:
                assert( true );
                break;
        }
    }
    else
    if ((pcbRev == 1) || (pcbRev == 2))
    {
        switch( band )
        {
            case UL850:
                AD_input = RXLBRF;
                gpio = 0x4;
                sw_band = GSM_LB_TX;
                break;
            case DL850:
                AD_input = RXLBRF;
                gpio = 0x2;
                sw_band = GSM_LB_TX;
                break;
            case UL900:
                AD_input = RXLBRF;
                gpio = 0xC;
                sw_band = WCDMA_1;
                break;
            case DL900:
                AD_input = RXLBRF;
                gpio = 0xA;
                sw_band = WCDMA_1;
                break;
            case UL1700:
                AD_input = RXHB1RF;
                gpio = 0x1;
                sw_band = GSM_RX_1;
                break;
            case DL1700:
                AD_input = RXHB2RF;
                gpio = 0xA;
                sw_band = WCDMA_3;
                break;
            case UL1800:
                AD_input = RXHB1RF;
                gpio = 0x1;
                sw_band = GSM_RX_1;
                break;
            case DL1800:
                AD_input = RXHB1RF;
                gpio = 0x2;
                sw_band = GSM_RX_2;
                break;
            case UL1900:
                AD_input = RXHB2RF;
                gpio = 0x4;
                sw_band = WCDMA_2;
                break;
            case DL1900:
                AD_input = RXHB2RF;
                gpio = 0x2;
                sw_band = WCDMA_2;
                break;
            case UL2100:
                AD_input = RXHB2RF;
                gpio = 0xC;
                sw_band = WCDMA_3;
                break;
            case DL2100:
                AD_input = RXHB2RF;
                gpio = 0xA;
                sw_band = WCDMA_3;
                break;
            case AUTO_BAND:
            case LOAD_BAND:
            case INVALID_BAND:
            default:
                assert( true );
                break;
        }
    }
    else
    if (pcbRev == 6)
    {
        switch( band )
        {
            case UL800:
                AD_input = RXLBRF;
                gpio = 0x4;
                sw_band = GSM_LB_TX;
                break;
            case DL800:
                AD_input = RXLBRF;
                gpio = 0x2;
                sw_band = GSM_LB_TX;
                break;
            case UL900:
                AD_input = RXLBRF;
                gpio = 0xC;
                sw_band = WCDMA_1;
                break;
            case DL900:
                AD_input = RXLBRF;
                gpio = 0xA;
                sw_band = WCDMA_1;
                break;
            case UL1700:
                AD_input = RXHB1RF;
                gpio = 0x1;
                sw_band = GSM_RX_1;
                break;
            case DL1700:
                AD_input = RXHB2RF;
                gpio = 0xA;
                sw_band = WCDMA_3;
                break;
            case UL1800:
                AD_input = RXHB1RF;
                gpio = 0x1;
                sw_band = GSM_RX_1;
                break;
            case DL1800:
                AD_input = RXHB1RF;
                gpio = 0x2;
                sw_band = GSM_RX_2;
                break;
            case UL1900:
                AD_input = RXHB2RF;
                gpio = 0x4;
                sw_band = WCDMA_2;
                break;
            case DL1900:
                AD_input = RXHB2RF;
                gpio = 0x2;
                sw_band = WCDMA_2;
                break;
            case UL2100:
                AD_input = RXHB2RF;
                gpio = 0xC;
                sw_band = WCDMA_3;
                break;
            case DL2100:
                AD_input = RXHB2RF;
                gpio = 0xA;
                sw_band = WCDMA_3;
                break;
            case AUTO_BAND:
            case LOAD_BAND:
            case INVALID_BAND:
            default:
                assert( true );
                break;
        }
    }
    else
    {
        throw std::runtime_error("Unknown PCB revision");
    }

    set_ADF4602_switches( AD_input, gpio );

    if (force_load)
    {
        set_RF1480( LOAD50 );
    }
    else
    {
        set_RF1480( sw_band );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Return a string representation of the band
std::string band_to_string( band_t band )
{
    return std::string( band_lookup[band].str );
}

// /////////////////////////////////////////////////////////////////////////////
/// If possible return a band described by the string
band_t string_to_band( std::string str )
{
    boost::trim(str);
    boost::to_upper(str);

    int band;
    for( band = UL800; band < INVALID_BAND; band++ )
    {
        if( str == band_lookup[band].str ) return (band_t) band;
    }
    return INVALID_BAND;
}

// /////////////////////////////////////////////////////////////////////////////
/// Return the resolution of the frequency setup in Hz
double      get_freq_res        ( void )
{
    return freq_res_Hz;
}


// /////////////////////////////////////////////////////////////////////////////
/// Check if frequnecy is in band
//
/// \return true = freq is in band
static bool freq_in_band( freq_t f, band_t b )
{
    return  ( (band_lookup[b].low  <= f) &&
              (band_lookup[b].high >= f) );
}

// /////////////////////////////////////////////////////////////////////////////
/// Program a new frequency
//
/// Depends on band selected so change band first
/// \return true = freq is in band
bool set_frequency( band_t band, freq_t freq )
{
    debugStr(DebugStream::info3)<<"set radio freq to "<< double(freq)/1.0e6 <<"MHz\n";

    currBand = band;
    currFreq = freq;

    //
    //  Record the fact that the frontend switch should be set to the load setting.
    //
    bool force_load = (band == LOAD_BAND);

    //
    //  If the band is to be selected automatically search the lookup table to find the correct band.  Otherwise
    //  check that the band/frequency combination is valid.
    //
    if ((band == AUTO_BAND) || (band == LOAD_BAND))
    {
        band = INVALID_BAND;
        for (unsigned n = 0 ; n < sizeof(freq_lookup)/sizeof(freq_lookup[0]) ; ++n)
        {
            if ((freq_lookup[n].low <= freq) && (freq <= freq_lookup[n].high))
            {
                band = freq_lookup[n].band;
                break;
            }
        }
    }
    else
    if (!freq_in_band(freq, band))
    {
        band = INVALID_BAND;
    }

    if (band != INVALID_BAND)
    {
        //
        //  Set the band
        //
        set_band(band, force_load);

        //
        //  If bit 2 was low then low band has been selected the value written should be 2x actual frequency.
        //
        if( (ADF4602_shadow::r1 & (1<<2)) == 0 )
        {
            freq *= 2;
        }

        ADF4602_SPI_write( 10, static_cast<uint32_t>(freq/freq_res_Hz) );
        frac_sleep( 200e-6 ); //wait to lock
    }

    return (band != INVALID_BAND);
}

// /////////////////////////////////////////////////////////////////////////////
/// Program a new gain
void set_gain
(
    uint32_t a ///< Gain in dB
)
{
    assert( a >= 0 && a <= 127 );

    currGain = a;

    debugStr(DebugStream::info3) << "set radio gain to "<< a <<" dB\n";

    ADF4602_SPI_write( 11, a );
}

// /////////////////////////////////////////////////////////////////////////////
/// Get the last programmed gain value
//
/// \return Gain in dB 0-127
uint32_t get_gain( void )
{
    return currGain;
}

// /////////////////////////////////////////////////////////////////////////////
/// Set the input and switches attached to ADF4602 gpio
//
/// This is lower level functionality used for debug
/// and also called by set band
void set_ADF4602_switches
(
    ADF4602_input_t input,  ///< One of 3 input on ADF4602
    unsigned int gpio       ///< 4 bit GPIO output lines
)
{
    assert( gpio < 16 );

    // set gpio
    ADF4602_shadow::r22 |= 0x8000;      // make sure set manual gpio control
    ADF4602_shadow::r22 &= ~(0xf<<11);  // clear old gpio bits
    ADF4602_shadow::r22 |= gpio<<11;    // set gpio bits
    ADF4602_SPI_write( 22, ADF4602_shadow::r22 );

    // set input band selection
    ADF4602_shadow::r1 &= ~(0x03<<1);   // clear old bits
    // set bits
    switch( input ){
        case RXLBRF:
            ADF4602_shadow::r1 |= 0x01<<1;
            break;
        case RXHB1RF:
            ADF4602_shadow::r1 |= 0x02<<1;
            break;
        case RXHB2RF:
            ADF4602_shadow::r1 |= 0x03<<1;
            break;
    }
    ADF4602_SPI_write( 1, ADF4602_shadow::r1 );
}


///////////////////////////////////////////////////////////////////////////////
//    other radio bits that aren't the ADF4602
///////////////////////////////////////////////////////////////////////////////

// /////////////////////////////////////////////////////////////////////////////
/// Set the front end switch
void set_RF1480
(
    RF1480_band_t band  ///< RF1480 pins are labeled as bands
)
{
    uint32_t v;
    switch( band ){
        case GSM_RX_1:
            v = 0x4;
            break;
        case GSM_RX_2:
            v = 0x5;
            break;
        case LOAD50:    // Mapping valid for PCB revision 0, 1 and 2
        case GSM_RX_3:
            v = 0x6;
            break;
        case GSM_LB_TX:
            v = 0x2;
            break;
        case GSM_HB_TX:
            v = 0x1;
            break;
        case WCDMA_1:
            v = 0x8;
            break;
        case WCDMA_2:
            v = 0x9;
            break;
        case WCDMA_3:
            v = 0xa;
            break;
    }

    v <<= 11;                   // position control bits
    v = v | (1<<26);            // set bits(counting from 0)

    debugStr(DebugStream::info2) << boost::format("sending 30 bits with value %#x to RF1480 switch\n")%v;

    // cs active high (not stated in data sheet)

    // grab the gpio lines we need for this operation, will open appropriate files
    gpio spi_cs  ( gpio::RFMD1480_CS,   gpio::out_low,    debugStr.get_state() );
    gpio spi_sdio( gpio::RFMD1480_MOSI, gpio::out,        debugStr.get_state() );
    gpio spi_sclk( gpio::RFMD1480_CLK,  gpio::out_low,    debugStr.get_state() );

    spi_cs.set( 1 );

    for( int i = 29; i >= 0; --i )
    {
        spi_sdio.set( (v>>i)&1 );
        spi_sclk.set( 1 );  // don't know which edge clocked in on
        spi_sclk.set( 0 );
    }

    spi_cs.set( 0 );
}

// /////////////////////////////////////////////////////////////////////////////
/// Turn on/off debug output
//
/// \param l show level
void debug_printing( unsigned l )
{
    debugStr.show_level( l );
}

}//end namespace radio

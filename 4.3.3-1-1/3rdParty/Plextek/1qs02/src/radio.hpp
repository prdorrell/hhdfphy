/******************************************************************************
 *
 *
 ******************************************************************************
 *  Filename:   radio.hpp
 *  Author(s):  pdm
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file radio.hpp
 * \brief Interface to radio chip and multi way switch
 *
 *****************************************************************************/
#ifndef __RADIO_H__
#define __RADIO_H__

#include <string>
#include <stdint.h>
#include "types.hpp"


namespace Radio{


enum RadioGainLimits_t
{
    MIN_GAIN_DB = 0,
    MAX_GAIN_DB = 100
};

/// enum names to match RF1480 pin labels
enum RF1480_band_t
{
    GSM_RX_1,
    GSM_RX_2,
    GSM_RX_3,
    GSM_LB_TX,
    GSM_HB_TX,
    WCDMA_1,
    WCDMA_2,
    WCDMA_3,
    LOAD50
};

/// The various bands the unit can be set to
enum band_t
{
    UL800 = 0,
    DL800,
    UL850,
    DL850,
    UL900,
    DL900,
    UL1700,
    DL1700,
    UL1800,
    DL1800,
    UL1900,
    DL1900,
    UL2100,
    DL2100,
    AUTO_BAND,
    LOAD_BAND,
    INVALID_BAND
};

/// enum names to match radio chip input labels
enum ADF4602_input_t
{
    RXLBRF,
    RXHB1RF,
    RXHB2RF
};

void        initialise          (bool mode3G);

void        set_3G_mode         (void);
void        set_GSM_mode        (void);

void        enter_low_power_mode(void);
void        exit_low_power_mode (bool mode3G);

bool        set_frequency       ( band_t, freq_t );
void        set_gain            ( uint32_t );
uint32_t    get_gain            ( void );
std::string band_to_string      ( band_t band );
band_t      string_to_band      ( std::string str );
double      get_freq_res        ( void );

// these are only exported for debug commands
void ADF4602_SPI_write          ( uint32_t address, uint32_t data );
void ADF4602_SPI_write          ( uint32_t address, uint32_t sub_address, uint32_t data );
void set_RF1480                 ( RF1480_band_t );
void set_ADF4602_switches       ( ADF4602_input_t input, unsigned int gpio );

void debug_printing             ( unsigned );

}//namespace radio


#endif

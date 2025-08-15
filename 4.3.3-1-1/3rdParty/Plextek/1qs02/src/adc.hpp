/******************************************************************************
 * Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 * All Rights Reserved
 ******************************************************************************
 * THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 * ----------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/adc.hpp $
 * $Revision: 6259 $
 * $Author: pdm $
 * $Date: 2011-07-08 17:47:11 +0100 (Fri, 08 Jul 2011) $
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file adc.hpp
 * \brief interface to AD9863
 *
 *****************************************************************************/
#ifndef __ADC_HPP__
#define __ADC_HPP__

#include <stdint.h>

namespace ADC
{
    void initialise( bool mode3G );

    void set_3G_mode( void );
    void set_GSM_mode( void );

    void enter_low_power_mode( void );
    void exit_low_power_mode( bool mode3G );

    // for debug
    void SPI_write( uint32_t address, uint32_t data );
    void debug_printing( unsigned );
}

#endif

/******************************************************************************
 * Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 * All Rights Reserved
 ******************************************************************************
 * THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 * ----------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/gpio.hpp $
 * $Revision: 6981 $
 * $Author: pdm $
 * $Date: 2011-08-26 09:53:01 +0100 (Fri, 26 Aug 2011) $
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file gpio.hpp
 * \brief Declaration of gpio class. An interface to the gpio lines.
 *
 *****************************************************************************/
#ifndef __GPIO_HPP__
#define __GPIO_HPP__

#include "debug.hpp"
#include <termios.h>

/// \brief This is just more wrapping around the gpio interface.
//
/// We are using the linux userspace file system nodes
/// //sys/class/gpio/... to set/read the gpio lines.
/// A when a gpio object is created it is bound to a particular pin.
/// The set/get methods can then be used to control the pin.
class gpio{
public:
    /// \brief Direction of gpio.  out_low, out_high used for glitchless
    /// setting of output direction.
    enum dir { in, out, out_low, out_high };

    /// \brief These numbers map pin names to file numbers.
    enum ref {
        MCU1_0  =  0+ 0,
        MCU1_1  =  0+ 1,
        MCU1_2  =  0+ 2,
        MCU1_3  =  0+ 3,
        MCU1_4  =  0+ 4,
        MCU1_5  =  0+ 5,
        MCU1_6  =  0+ 6,

        MCU1_18 =  0+18, RFMD1480_MOSI  = MCU1_18,
        MCU1_19 =  0+19, RFMD1480_CLK   = MCU1_19,
        MCU1_20 =  0+20, RFMD1480_CS    = MCU1_20,

        MCU2_0  = 32+ 0,    // Green LED
        MCU2_1  = 32+ 1,    // Red LED
        MCU2_2  = 32+ 2,    // GPS on/off
        MCU2_3  = 32+ 3,    // GPS reset

        MCU2_8  = 32+ 8, ADI4602_MOSI   = MCU2_8,
        MCU2_9  = 32+ 9, ADI9863_CLK    = MCU2_9,
                         ADI5320_CLK    = MCU2_9,
        MCU2_10 = 32+10, ADI4602_CLK    = MCU2_10,
        MCU2_11 = 32+11, ADI4602_CS     = MCU2_11,
        MCU2_12 = 32+12, ADI9863_MOSI   = MCU2_12,
                         ADI5320_MOSI   = MCU2_12,
        MCU2_13 = 32+13, ADI9863_MISO   = MCU2_13,
        MCU2_14 = 32+14, ADI9863_CS     = MCU2_14,
        MCU2_15 = 32+15, ADI5320_CS     = MCU2_15,

        MCU2_16 = 32+16, ADI9863_ADC_RESET_N = MCU2_16,

        MCU2_18 = 32+18, PCB_REV0 = MCU2_18,
        MCU2_19 = 32+19, PCB_REV1 = MCU2_19,
        MCU2_20 = 32+20, PCB_REV2 = MCU2_20,
        MCU2_21 = 32+21, FPGA_SUSPEND = MCU2_21,
        MCU2_22 = 32+22,    // Power OK
        MCU2_23 = 32+23, ADI9863_TXPWRDWN   = MCU2_23,
        MCU2_24 = 32+24, ADI9863_RXPWRDWN   = MCU2_24,
        MCU2_25 = 32+25, ADI9863_ADC_LO_PWR = MCU2_25,

        MCU3_2  = 64+2,     // GPS power OK
        MCU3_3  = 64+3,     // GPS boot mode
        MCU3_4  = 64+4,     // Gyro INT
        MCU3_5  = 64+5,     // Bluetooth pair
        MCU3_9  = 64+ 9, FPGA_IMG_RST = MCU3_9,

        MCU3_10 = 64+10, FPGA_MCB_CAL_DONE = MCU3_10,
        MCU3_11 = 64+11, FPGA_192_CLK_LOCK = MCU3_11,
        MCU3_12 = 64+12, FPGA_195_CLK_LOCK = MCU3_12,
        MCU3_13 = 64+13, FPGA_LOW_PWR_MODE = MCU3_13,
        MCU3_14 = 64+14,    // spare
        MCU3_15 = 64+15,    // spare
        MCU3_16 = 64+16, FPGA_RDWR_B    = MCU3_16,
        MCU3_17 = 64+17, FPGA_DONE      = MCU3_17,
        MCU3_18 = 64+18, FPGA_INIT_B    = MCU3_18,
        MCU3_19 = 64+19, FPGA_PROGRAM_B = MCU3_19,

        MCU3_31 = 64+31,    // FPGA busy
    };
    gpio( unsigned gpionumber, dir direction, unsigned debug );
    ~gpio();
    bool get( void );
    void set( bool );
    //void change_direction( dir );


private:
    DebugStream debugStr;   ///< debug tracing output stream
    int value_file;         ///< file descriptor for value file
    int dir_file;           ///< file descriptor for direction file
    termios oldvfattr;      ///< fine tune file io behaviour
    termios olddfattr;      ///< fine tune file io behaviour
    gpio();                 ///< oh no you don't
};

#endif


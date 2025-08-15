/******************************************************************************
 * Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 * All Rights Reserved
 ******************************************************************************
 * THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 * ----------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/gpio.cpp $
 * $Revision: 6285 $
 * $Author: pdm $
 * $Date: 2011-07-12 17:45:49 +0100 (Tue, 12 Jul 2011) $
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file gpio.cpp
 * \brief definition of gpio class
 *
 *****************************************************************************/

#include "gpio.hpp"
#include <boost/format.hpp>
#include <stdexcept>
#include <string>
#include <fcntl.h>
#include <unistd.h>

/// \brief Bind this instance to the appropriate files for this gpio line.
/// \param [in] gpio a gpio file number see enum ref
/// \param [in] direction direction whether to create and input or output pin
/// \param [in] debug debug output verbosity
gpio::gpio( unsigned gpio, dir direction, unsigned debug )
    :
    debugStr( (boost::format("gpio %2d..... ") % gpio).str(), debug )
{
    using namespace std;
    string filename;

    // dir file
    filename = (boost::format( "/sys/class/gpio/gpio%d/direction" ) % gpio).str();
    if ((dir_file = open( filename.c_str(), O_RDWR)) == -1)
    {
        throw runtime_error( "failed to open "+filename );
    }

    // value file
    try
    {
        filename = (boost::format( "/sys/class/gpio/gpio%d/value" ) % gpio).str();
        if ((value_file = open( filename.c_str(), O_RDWR)) == -1)
        {
            throw runtime_error( "failed to open "+filename );
        }
    }
    catch( ... )
    {
        close( dir_file );
        throw;
    }

    termios mxc;
    tcgetattr( value_file, &oldvfattr );
    mxc = oldvfattr;
    mxc.c_lflag &= ~(ICANON | ECHO | ISIG);
    tcsetattr( value_file, TCSANOW, &mxc ); // really needed?

    tcgetattr( dir_file, &olddfattr );
    mxc = olddfattr;
    mxc.c_lflag &= ~(ICANON | ECHO | ISIG);
    tcsetattr( value_file, TCSANOW, &mxc ); // really needed?

    if      ( direction == in )         write( dir_file, "in\0", 3 );
    else if ( direction == out )        write( dir_file, "out\0", 4 );
    else if ( direction == out_low )    write( dir_file, "low\0", 4 );
    else if ( direction == out_high )   write( dir_file, "high\0", 5 );
}

/// \brief Close the files.
gpio::~gpio()
{
    // ioctl(gpio_value_file, TIOCMBIC, &line_val); // que?
    tcsetattr( value_file, TCSAFLUSH, &oldvfattr );  // really needed?
    tcsetattr( value_file, TCSAFLUSH, &olddfattr );  // really needed?

    close( value_file );
    close( dir_file );
}

/// \brief Set the pin.
/// \param v [in] value low/high to set
void gpio::set( bool v )
{
    if( v ) write( value_file, "1\0", 2 );
    else    write( value_file, "0\0", 2 );
}

/// \brief Get the pin level.
/// \return status of pin low/high (false/true)
bool gpio::get( void )
{
    using namespace std;

    char buf[2] = { '\0', '\0' };
    lseek( value_file, 0, SEEK_SET );
    read( value_file, buf, 2 );
    if( buf[0] == '1' ) return true;
    if( buf[0] == '0' ) return false;
    else throw runtime_error( "error reading io port v='" + std::string(buf) + "'" );
}

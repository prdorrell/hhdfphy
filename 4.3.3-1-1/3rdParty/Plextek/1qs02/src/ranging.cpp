/***********************************************************************************************************************
 *  Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 *  --------------------------------------------------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/ranging.cpp $
 * $Revision: 10176 $
 * $Author: pdm $
 * $Date: 2012-03-15 10:50:03 +0000 (Thu, 15 Mar 2012) $
 *******************************************************************************************************************//**
 *  @file
 *  @brief          The implementation of ::Ranging class that adjusts the RF frontend gain.
 **********************************************************************************************************************/
#include "radio.hpp"
#include "ranging.hpp"

#include <cassert>
#include <cmath>
#include <boost/format.hpp>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class.
//  @details        If members requested mode is stored and the internal mode is configured to match it.  The timeout
//                  counter is reset and the gain is set to its default.  The debug flags are stored and finally the
//                  hardware is  updated.
//  @param[in]      mode                The (auto-)ranging mode.
//  @param[in]      debug               Controls the debug messages that may be generated.
//
Ranging::Ranging( RangingMode mode, unsigned debug ) :
    requested_mode( mode ),
    internal_mode( (mode == AUTO_RANGING) ? INT_AUTO_RANGING : INT_FIXED_RANGE ),
    timeout_counter( 0 ),
    level( Radio::MIN_GAIN_DB ),
    status( RANGING_FLUSH1 ),
    debugStr( "Ranging..... ", debug )
{
    update_hardware();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          The destructor has nothing to do.
//
Ranging::~Ranging(void)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief          Update the RF hardware.
/// @details        Uses the ::Radio class's Radio::set_gain() function to update the gain.
///
void Ranging::update_hardware()
{
    assert( level >= Radio::MIN_GAIN_DB && level <= Radio::MAX_GAIN_DB );
    Radio::set_gain( level );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Update ranging based on the mean %ADC sample power.
//  @details        If auto-ranging has been enabled and the sample power is outside the range defined by
//                  Ranging::RANGING_POWER_HIGH_DB and Ranging::RANGING_POWER_LOW_DB the function adjusts the RF gain by
//                  an amount that should result in a sample power of Ranging::RANGING_POWER_MID_DB. If the gain cannot
//                  be adjusted because it is already at its upper or lower limit a warning message is sent to the debug
//                  stream and no action is taken.  If the overload count is non-zero the functions behaves as if sample
//                  power the exceeds Ranging::RANGING_POWER_HIGH_DB.
//  @param[in]      power               The mean %ADC sample power.
//  @return                             The ranging status.
//
Ranging::RangingUpdateResult Ranging::update(double power)
{
    RangingUpdateResult result = RANGE_STABLE;
    if (((internal_mode == INT_AUTO_RANGING) || (internal_mode == INT_ONLY_DECREASE_GAIN)) && (status == RANGING_IDLE))
    {
        bool update = false;

        //
        //  Convert the power to dB.  A negative value should be impossible but a value of zero might happen.  In the
        //  first case a warning is appropriate, but in both cases it is reasonable to set the dB value to 0.
        //
        if ( power < 0.0 )
        {
            debugStr(DebugStream::warn2) << "...that power shouldn't be possible!\n";
            power = 0.0;
        }
        else
        if ( power > 0.0 )
        {
            power = 10.0*log10(power);
        }

        debugStr(DebugStream::info3) << boost::format("Ranging update %2.1fdB LSB**2, old level %d, internal mode %d\n")
                                                      % power
                                                      % level
                                                      % internal_mode;
        //
        //  Check if the sample power is outside the limits, or if the overload count is too high.
        //
        if (power >= RANGING_POWER_HIGH_DB)
        {
            //
            //  Reduce the gain if is possible, otherwise issue an overload warning and leave the gain unchanged.
            //
            if( level > Radio::MIN_GAIN_DB )
            {
                int diff = static_cast<int>(power - RANGING_POWER_MID_DB);
                level -= diff;
                if( level < Radio::MIN_GAIN_DB )
                {
                    level = Radio::MIN_GAIN_DB;
                }
                update = true;

                //
                //  Increment the timeout counter and decide if its time to abandon full auto-ranging.
                //
                if (internal_mode == INT_AUTO_RANGING)
                {
                    ++timeout_counter;
                    if (timeout_counter > MAX_NUM_RANGE_ATTEMPTS)
                    {
                        internal_mode = INT_ONLY_DECREASE_GAIN;
                        debugStr(DebugStream::warn1) << "ranging unstable - switching mode\n";
                    }
                }
            }
            else
            {
                timeout_counter = 0;
                debugStr(DebugStream::warn2) << "input power overload\n";
            }
        }
        else if ((power < RANGING_POWER_LOW_DB) && (internal_mode == INT_AUTO_RANGING))
        {
            //
            //  Increase the gain if is possible, otherwise issue an underload warning and leave the gain unchanged.
            //
            if( level < Radio::MAX_GAIN_DB )
            {
                int diff = static_cast<int>(RANGING_POWER_MID_DB - power);
                level += diff;
                if( level > Radio::MAX_GAIN_DB )
                {
                    level = Radio::MAX_GAIN_DB;
                }
                update = true;
                ++timeout_counter;
                if (timeout_counter > MAX_NUM_RANGE_ATTEMPTS)
                {
                    internal_mode = INT_ONLY_DECREASE_GAIN;
                    debugStr(DebugStream::warn1) << "ranging unstable - switching mode\n";
                }
            }
            else
            {
                timeout_counter = 0;
                debugStr(DebugStream::warn2) << "input power underload\n";
            }
        }
        else
        {
            timeout_counter = 0;
        }

        //
        //  If the gain has changed update the hardware.
        //
        if( update )
        {
            update_hardware();

            result = RANGE_UNSTABLE;

            status = RANGING_FLUSH1;
            debugStr(DebugStream::info1) << "Status   RANGING_IDLE   -> RANGING_FLUSH1\n";
            debugStr(DebugStream::info4) << boost::format("Ranging update %2.1fdB LSB**2, new level %d, internal mode %d\n")
                                                          % power
                                                          % level
                                                          % internal_mode;
        }
    }
    else
    if (status == RANGING_FLUSH1)
    {
        result = RANGE_UNSTABLE;

        status = RANGING_FLUSH2;
        debugStr(DebugStream::info1) << "Status   RANGING_FLUSH1 -> RANGING_FLUSH2\n";
    }
    else
    if (status == RANGING_FLUSH2)
    {
        result = RANGE_UNSTABLE;

        status = RANGING_IDLE;
        debugStr(DebugStream::info1) << "Status   RANGING_FLUSH2 -> RANGING_IDLE\n";
    }

    return (result);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Retrieve the current gain setting as a dB value.
//  @return                             The current gain.
//
int Ranging::get_gain_dB(void) const
{
    return level;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Retrieve the current gain setting as a linear value.
//  @return                             The current gain.
//
double Ranging::get_pwr_gain_linear(void) const
{
    return pow(10.0, double(level)/10.0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Sets the gain.
//  @details        This forces the current gain to the specified value, however if auto-ranging is enabled the gain may
//                  not remain at this level for long.
//  @param[in]      gain_dB             The gain in dB.
//
void Ranging::set_gain_dB( int gain_dB )
{
    level = gain_dB;
    update_hardware();

    status = RANGING_FLUSH1;
    debugStr(DebugStream::info1) << "Status   RANGING_IDLE   -> RANGING_FLUSH1\n";
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Disables auto-ranging.
//
void Ranging::disable_auto_ranging( void )
{
    requested_mode  = FIXED_RANGE;
    internal_mode   = INT_FIXED_RANGE;
    timeout_counter = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Enables auto-ranging.
//
void Ranging::enable_auto_ranging( void )
{
    requested_mode  = AUTO_RANGING;
    internal_mode   = INT_AUTO_RANGING;
    timeout_counter = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Resets the ranging.
//  @details        Ensures that any timeouts and special conditions are cleared, and re-programs the current gain.
//
void Ranging::reset_ranging( void )
{
    if (requested_mode == AUTO_RANGING)
    {
        internal_mode = INT_AUTO_RANGING;
    }
    else
    {
        internal_mode = INT_FIXED_RANGE;
    }
    timeout_counter = 0;
    level = Radio::MIN_GAIN_DB;

    update_hardware();

    status = RANGING_FLUSH1;
    debugStr(DebugStream::info1) << "Status   ...            -> RANGING_FLUSH1\n";
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Determines if the internal mode has been forced to ::INT_ONLY_DECREASE_GAIN.
//  @details        If the requested mode was auto ranging, but a stable setting could not be found the ranging will
//                  give up after a while and force the internal mode to ::INT_ONLY_DECREASE_GAIN.  This function
//                  returns true if this has happened.
//
//  @return         true if the internal mode is ::INT_ONLY_DECREASE_GAIN, false otherwise.
//
bool Ranging::decrease_gain_forced( void )
{
    return (internal_mode == INT_ONLY_DECREASE_GAIN);
}

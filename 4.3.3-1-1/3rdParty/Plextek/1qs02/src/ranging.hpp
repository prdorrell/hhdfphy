/***********************************************************************************************************************
 *  Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 *  --------------------------------------------------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/ranging.hpp $
 * $Revision: 10176 $
 * $Author: pdm $
 * $Date: 2012-03-15 10:50:03 +0000 (Thu, 15 Mar 2012) $
 *******************************************************************************************************************//**
 *  @file
 *  @brief          The declaration of ::Ranging class that adjusts the RF frontend gain.
 **********************************************************************************************************************/
#ifndef __RANGING_HPP__
#define __RANGING_HPP__

#include "debug.hpp"

#include <boost/utility.hpp>

///
/// @brief          Controls the adjustement of the RF frontend gain.
/// @details        This class controls the RF frontend gain in order to ensure that the %ADC samples have a mean power
///                 that is best suited to the %FPGA processing.  The update() function is invoked at regular intervals
///                 and provides the class with the mean %ADC sample power and a count of the number of overloaded
///                 samples.  It uses the ::Radio class's Radio::set_gain() function to configure the frontend gain.
///
///                 An instance may be created with auto-ranging disabled in which case the gain is only configured
///                 during the construction of the instance or by using the get_gain_dB() function.  Calls to the
///                 update() function have no effect.
///
///                 Whenever the gain is changed the update() function ensures that any transients are flushed through
///                 the FPGA's pipeline by signalling that the range unstable for two invocations after the change.
///
///                 If auto-ranging has been selected but the algorithm does not manage to find an acceptable gain
///                 setting in Ranging::MAX_NUM_RANGE_ATTEMPTS consecutive attempts it will change to an internal mode
///                 in which the gain can only be decreased.  This internal mode is reset by calling any of the
///                 disable_auto_ranging(), enable_auto_ranging() or reset_ranging() functions.
///
class Ranging : private boost::noncopyable
{
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // The constants.
        //
    private:
        ///
        /// @brief          The upper limit for the mean %ADC sample power.
        /// @details        The value is 10 dB below the maximum that should occur for a full-scale DC signal.  The
        ///                 10 dB allows for the typical peak-to-average power ratio of a WCDMA signal.
        ///
#ifdef USE_CHAN_RSSI_RANGING
        static const double RANGING_POWER_HIGH_DB = 75;
#else
        static const double RANGING_POWER_HIGH_DB = 56;
#endif

        ///
        /// @brief          The lower limit for the mean %ADC sample power.
        /// @details        The value has been chosen so that the gain will reach its maximum setting if there is no
        ///                 input signal.
        ///
#ifdef USE_CHAN_RSSI_RANGING
        static double const RANGING_POWER_LOW_DB  = 60;
#else
        static double const RANGING_POWER_LOW_DB  = 41;
#endif

        ///
        /// @brief          The ideal mean %ADC sample power.
        /// @details        The value is mid-way between the upper and lower limits.
        ///
#ifdef USE_CHAN_RSSI_RANGING
        static double const RANGING_POWER_MID_DB  = 68;
#else
        static double const RANGING_POWER_MID_DB  = 49;
#endif

        ///
        /// @brief          The maximum number of auto-ranging attempts.
        /// @details        The auto-ranging algorithm will attempt to adjust the range, but if this number of
        ///                 consecutive attempts fails top yield a satisfactory result it will change mode and only
        ///                 reduce the gain until the power is below Ranging::RANGING_POWER_HIGH_DB or the gain is at
        ///                 its lower limit.
        ///
        static const unsigned   MAX_NUM_RANGE_ATTEMPTS = 10;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // The types.
        //
    public:
        ///
        /// @brief          The requested ranging mode.
        ///
        typedef enum
        {
            ///
            /// @brief          The ranging algorithm attempts to select the best range.
            ///
            AUTO_RANGING,

            ///
            /// @brief          The ranging algorithm uses the current range or the range set in subsequent calls to
            ///                 set_gain_dB().
            ///
            FIXED_RANGE
        } RangingMode;

        ///
        /// @brief          The result of a ranging update (a call to the update() function).
        ///
        typedef enum
        {
            ///
            /// @brief          The current range is acceptable and the FPGA results should be used.
            ///
            RANGE_STABLE,

            ///
            /// @brief          The range is being adjusted and the FPGA results should not be used.
            ///
            RANGE_UNSTABLE
        } RangingUpdateResult;

    private:
        ///
        /// @brief          The internal ranging mode.
        ///
        typedef enum
        {
            ///
            /// @brief          The ranging algorithm attempts to select the best range.
            ///
            INT_AUTO_RANGING,

            ///
            /// @brief          The ranging algorithm attempts to select the best range, but will only increase the
            ///                 gain.
            ///
            INT_ONLY_DECREASE_GAIN,

            ///
            /// @brief          The ranging algorithm uses the current range or the range set in subsequent calls to
            ///                 set_gain_dB().
            ///
            INT_FIXED_RANGE
        } InternalMode;

        ///
        /// @brief          The ranging status.
        /// @details        This controls the operation of the ranging algorithm.  Any change to the range must be
        ///                 flushed because the FPGA processing is pipelined.  To be safe two results must be discarded
        ///                 after a change to the range.
        ///
        typedef enum
        {
            ///
            /// @brief          The range is currently stable and FPGA results should be valid.
            ///
            RANGING_IDLE,

            ///
            /// @brief          The range has been changed and the algorithm is discarding the first result after the
            ///                 change.
            ///
            RANGING_FLUSH1,

            ///
            /// @brief          The range has been changed and the algorithm is discarding the second result after the
            ///                 change.
            ///
            RANGING_FLUSH2
        } RangingStatus;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // The values.
        //
    private:
        ///
        /// @brief          The auto-ranging mode requested by the client.
        ///
        RangingMode     requested_mode;

        ///
        /// @brief          The internal auto-ranging mode.
        ///
        InternalMode    internal_mode;

        ///
        /// @brief          The counter used in detecting an unstable ranging condition.
        ///
        unsigned        timeout_counter;

        ///
        /// @brief          The current gain setting.
        ///
        int             level;

        ///
        /// @brief          Reflects the ranging status.
        ///
        RangingStatus   status;

        ///
        /// @brief          The debug stream.
        /// @details        Debug messages are sent to this stream.  If the debugging level has been appropriately
        ///                 configured they appear on the standard output console.  See ::DebugStream.
        ///
        DebugStream     debugStr;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          The default constructor is declared but not defined in order to prevent a compiler generated
        ///                 version appearing.
        ///
    private:
        Ranging(void);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          Constructs an instance of the class.
        /// @details        If members requested mode is stored and the internal mode is configured to match it.  The
        ///                 timeout counter is reset and the gain is set to its default.  The debug flags are stored and
        ///                 finally the hardware is  updated.
        /// @param[in]      mode                The (auto-)ranging mode.
        /// @param[in]      debug               Controls the debug messages that may be generated.
        ///
    public:
        explicit Ranging( RangingMode mode, unsigned debug );

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          The destructor has nothing to do.
        ///
    public:
        ~Ranging(void);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          Update ranging based on the mean %ADC sample power.
        /// @details        If auto-ranging has been enabled and the sample power is outside the range defined by
        ///                 Ranging::RANGING_POWER_HIGH_DB and Ranging::RANGING_POWER_LOW_DB the function adjusts the RF
        ///                 gain by an amount that should result in a sample power of Ranging::RANGING_POWER_MID_DB. If
        ///                 the gain cannot be adjusted because it is already at its upper or lower limit a warning
        ///                 message is sent to the debug stream and no action is taken.  If the overload count is non-
        ///                 zero the functions behaves as if sample power the exceeds Ranging::RANGING_POWER_HIGH_DB.
        /// @param[in]      power               The mean %ADC sample power.
        /// @return                             The ranging status.
        ///
    public:
        RangingUpdateResult update(double power);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          Retrieve the current gain setting as a dB value.
        /// @return                             The current gain.
        ///
    public:
        int get_gain_dB( void ) const;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          Retrieve the current gain setting as a linear value.
        /// @return                             The current gain.
        ///
    public:
        double get_pwr_gain_linear( void ) const;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          Sets the gain.
        /// @details        This forces the current gain to the specified value, however if auto-ranging is enabled the
        ///                 gain may not remain at this level for long.
        /// @param[in]      gain_dB             The gain in dB.
        ///
    public:
        void set_gain_dB( int gain_dB );

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          Disables auto-ranging.
        ///
    public:
        void disable_auto_ranging( void );

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          Disables auto-ranging.
        ///
    public:
        void enable_auto_ranging( void );

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          Resets the ranging.
        /// @details        Ensures that any timeouts and special conditions are cleared, and re-programs the current
        ///                 gain.
        ///
    public:
        void reset_ranging( void );


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          Determines if the internal mode has been forced to ::INT_ONLY_DECREASE_GAIN.
        /// @details        If the requested mode was auto ranging, but a stable setting could not be found the ranging
        ///                 will give up after a while and force the internal mode to ::INT_ONLY_DECREASE_GAIN.  This
        ///                 function returns true if this has happened.
        ///
        /// @return         true if the internal mode is ::INT_ONLY_DECREASE_GAIN, false otherwise.
        ///
    public:
        bool decrease_gain_forced( void );

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          Set the debug verbosity level.
        /// @param[in]      level               The new verbosity level.
        ///
    public:
        void debug_printing( unsigned level )
        {
            debugStr.show_level( level );
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          Update the RF hardware.
        /// @details        Uses the ::Radio class's Radio::set_gain() function to update the gain.
        ///
    private:
        void update_hardware( void );
};

#endif


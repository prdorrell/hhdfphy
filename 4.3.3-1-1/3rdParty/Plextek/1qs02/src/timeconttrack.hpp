/***********************************************************************************************************************
 *  Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 *  --------------------------------------------------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/timeconttrack.hpp $
 * $Revision: 6380 $
 * $Author: pdm $
 * $Date: 2011-07-20 09:32:59 +0100 (Wed, 20 Jul 2011) $
 *******************************************************************************************************************//**
 *  @file
 *  @brief          The declaration of ::TimeContTrack class that controls the GSM mode time-contionuous tracking.
 **********************************************************************************************************************/
#ifndef __TIMECONTTRACK_HPP__
#define __TIMECONTTRACK_HPP__

#include <vector>

#include "gsm.hpp"
#include "gsmpwrmeter.hpp"
#include "pwrcal.hpp"
#include "ranging.hpp"

///
/// @brief          Provides an object that carries out the GSM mode time-contionus tracking.
/// @details        Makes use of an instance of the ::GsmPwrMeter class to collect the array channel power results that
///                 span a frame.  Each element is the result of averaging the power over a number of frames.
///
class TimeContTrack
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The constants.
    //
private:
    ///
    /// @brief          The number of power samples in a slot.
    /// @details        The FPGA supports 1 or 3.
    ///
    static const unsigned SAMPLES_PER_SLOT = 3;

    ///
    /// @brief          The number of frames spanned by a result.
    ///
    static const unsigned FRAMES_PER_RESULT = 7;

    ///
    /// @brief          The number of frames spanned by a result.
    ///
    static const unsigned SAMPLES_PER_RESULT = FRAMES_PER_RESULT*Gsm::SLOTS_PER_FRAME*SAMPLES_PER_SLOT;

    ///
    /// @brief          The ratio of the GSM to 3G power gain factor.
    /// @details        This is the ratio of the power gains of the channel filters.  The 3G filter has a DC gain
    ///                 amplitude gain of 9.3 and the filter bandwidth reduces the signal gain to 9.2.  The GSM filter
    ///                 has a DC gain of 15.9 and the filter bandwidth reduces the signal gain to 14.6
    ///
    static const double GSM_3G_CAL_FACTOR_RATIO = 1.0/2.53;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The types.
    //
public:
    ///
    /// @brief          The state algorithm is in.
    ///
    typedef enum
    {
        ///
        /// @brief          The algorithm has been started but no iteration has been carried out.
        ///
        TC_IDLE,

        ///
        /// @brief          The ranging is not stable.
        ///
        TC_RANGING,

        ///
        /// @brief          The algorithm is actively measuring the channel power.
        ///
        TC_ACTIVE
    } state_t;

    ///
    /// @brief          The result of the iterate() function call.
    ///
    typedef enum
    {
        ///
        /// @brief          The algorithm is waiting to produce a result.
        ///
        TC_WAITING,

        ///
        /// @brief          The algorithm has a new result.
        ///
        TC_NEW_RESULT
    } iter_res_t;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The values.
    //
private:
    ///
    /// @brief          The interface to the power meter.
    ///
    GsmPwrMeter pwrMeter;

    ///
    /// @brief          The range controller instance to be used.
    ///
    Ranging             &ranging;

    ///
    /// @brief          The power calibration instance to be used.
    ///
    PwrCal              &powerCal;

    ///
    /// @brief          The state of the algorithm.
    ///
    state_t             state;

    ///
    /// @brief          The channel power samples.
    ///
    std::vector<double> samples;

    ///
    /// @brief          The debug stream.
    /// @details        Debug messages are sent to this stream.  If the debugging level has been appropriately
    ///                 configured they appear on the standard output console.  See ::DebugStream.
    ///
    DebugStream         debugStr;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The default constructor is declared but not defined in order to prevent a compiler generated
    ///                 version appearing.
    ///
private:
    TimeContTrack(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Constructs an instance of the class.
    /// @details        Creates the interface to the power meter.
    /// @param[in]      fpga                The FPGA interface.
    /// @param[in]      rangeCtrl           The range controller.
    /// @param[in]      powerCal            The power calibration interface.
    /// @param[in]      antenna_mode        The antenna mode to be used.
    /// @param[in]      debug               Controls the debug messages that may be generated.
    ///
public:
    TimeContTrack(Fpga     &fpga,
                  Ranging  &rangeCtrl,
                  PwrCal   &powerCal,
                  unsigned antenna_mode,
                  unsigned debug);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The destructor has nothing to do.
    ///
public:
    virtual ~TimeContTrack();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Processes an iteration of the control loop.
    /// @return                             The state of the algorithm.
    ///
public:
    iter_res_t iterate(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Gets the results and resets the internal copies.
    /// @param[out]     pwr_samples         The destination for the power sample array.
    /// @return                             The average power.
    ///
public:
    double get_results(std::vector<double> &pwr_samples);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Set the debug verbosity level.
    /// @param[in]      level               The new verbosity level.
    ///
public:
    void debug_printing(unsigned level);
};

#endif

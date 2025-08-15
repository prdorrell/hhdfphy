/***********************************************************************************************************************
 *  Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 *  --------------------------------------------------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/timeconttrack.cpp $
 * $Revision: 6417 $
 * $Author: pdm $
 * $Date: 2011-07-22 11:14:36 +0100 (Fri, 22 Jul 2011) $
 *******************************************************************************************************************//**
 *  @file
 *  @brief          The definition of ::TimeContTrack class that controls the GSM mode time-contionuous tracking.
 **********************************************************************************************************************/
#include <boost/format.hpp>

#include <stdexcept>

#include "timeconttrack.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class.
//  @details        Creates the interface to the power meter.
//  @param[in]      fpga                The FPGA interface.
//  @param[in]      rangeCtrl           The range controller.
//  @param[in]      powerCal            The power calibration interface.
//  @param[in]      antenna_mode        The antenna mode to be used.
//  @param[in]      debug               Controls the debug messages that may be generated.
//
TimeContTrack::TimeContTrack(Fpga     &fpga,
                             Ranging  &rangeCtrl,
                             PwrCal   &powerCal,
                             unsigned antenna_mode,
                             unsigned debug) :
    pwrMeter(fpga, (SAMPLES_PER_SLOT == 1), antenna_mode, debug),
    ranging(rangeCtrl),
    powerCal(powerCal),
    state(TC_IDLE),
    samples(),
    debugStr("TCTrack..... ", debug)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          The destructor has nothing to do.
//
TimeContTrack::~TimeContTrack()
{
}

// /////////////////////////////////////////////////////////////////////////////
/// \brief iterate the algorithm
//
/// Call repeatedly.
/// \return mode/state to allow caller to know what is going on:
/// - IDLE                 - initial state, will just move to next state
/// - START_SEARCH         - initial setup for searching
/// - SEARCHING            - running searching algorithm
/// - START_TRACK          - started tracking mode
/// - TRACKING_WAITING     - this frame was not used, needs more calls to get results
/// - TRACKING_NEW_RESULT  - this frame was used and (possibly empty) tracking reslts list has been updated
//
TimeContTrack::iter_res_t TimeContTrack::iterate(void)
{
    iter_res_t retVal = TC_WAITING;

    //
    //  Fill the result buffer.  When ranging this allows the ADC RSSI result to be refreshed.
    //
    pwrMeter.get_results(SAMPLES_PER_RESULT-samples.size(), samples);
    if (samples.size() >= SAMPLES_PER_RESULT)
    {
        //
        //  Allow the range controller to adjust the gain if necessary.
        //
        if(ranging.update(pwrMeter.get_adc_power()) == Ranging::RANGE_UNSTABLE)
        {
            samples.clear();
            debugStr(DebugStream::info1) << "TimeContTrack   Some State -> TC_RANGING\n";
            state = TC_RANGING;
        }
        else
        {
            switch (state)
            {
                case TC_IDLE:
                {
                    //
                    //  Restart the power meter so that the frame alignment is restored.
                    //
                    pwrMeter.restart();
                    samples.clear();

                    debugStr(DebugStream::info1) << "TimeContTrack   TC_IDLE    -> TC_ACTIVE\n";
                    state = TC_ACTIVE;
                    break;
                }
                case TC_RANGING:
                {
                    //
                    //  Restart the power meter so that the frame alignment is restored.
                    //
                    pwrMeter.restart();
                    samples.clear();

                    debugStr(DebugStream::info1) << "TimeContTrack   TC_RANGING -> TC_ACTIVE\n";
                    state = TC_ACTIVE;
                    break;
                }
                case TC_ACTIVE:
                {
                    retVal = TC_NEW_RESULT;
                    break;
                }
                default :
                {
                    throw std::runtime_error( "invalid SearchAndTrack mode" );
                    break;
                }
            }
        }
    }
    return retVal;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief          Gets the results and resets the internal copies.
/// @param[out]     pwr_samples         The destination for the power sample array.
/// @return                             The average power.
///
double TimeContTrack::get_results(std::vector<double> &pwr_samples)
{
    double avg_pwr = 0.0;

    double calFactor = powerCal.getFactor(ranging.get_gain_dB())*GSM_3G_CAL_FACTOR_RATIO;

    unsigned samples_per_frame = Gsm::SLOTS_PER_FRAME*SAMPLES_PER_SLOT;
    unsigned num_frames        = samples.size()/samples_per_frame;

    pwr_samples.clear();
    pwr_samples.resize(samples_per_frame, 0.0);

    for (unsigned sample_num = 0 ; sample_num < samples_per_frame ; ++sample_num)
    {
        for (unsigned frame_num = 0 ; frame_num < num_frames ; ++frame_num)
        {
            pwr_samples[sample_num] += samples[frame_num*samples_per_frame+sample_num];
            avg_pwr                 += samples[frame_num*samples_per_frame+sample_num];
        }
        pwr_samples[sample_num] = calFactor*pwr_samples[sample_num]/static_cast<double>(num_frames);
    }
    avg_pwr = calFactor*avg_pwr/static_cast<double>(num_frames*samples_per_frame);

    samples.clear();

//    debugStr(DebugStream::info4) << boost::format("%9.1f %9.1f\n") % pwrMeter.get_adc_power() % avg_pwr;

    return (avg_pwr);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Set the debug verbosity level.
//  @param[in]      level               The new verbosity level.
//
void TimeContTrack::debug_printing(unsigned level)
{
    debugStr.show_level(level);
    pwrMeter.debug_printing(level);
}

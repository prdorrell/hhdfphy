/***********************************************************************************************************************
 *  Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 *  --------------------------------------------------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/gsmpwrmeter.cpp $
 * $Revision: 6417 $
 * $Author: pdm $
 * $Date: 2011-07-22 11:14:36 +0100 (Fri, 22 Jul 2011) $
 *******************************************************************************************************************//**
 *  @file
 *  @brief          The definition of ::GsmPwrMeter class that controls the GSM channel power measurement in the FPGA.
 *  @details        Wrap up the hardware power meter in a class with any additional processing fluff needed.  Uses the
 *                  low level FPGA interface talk to the hardware.
 **********************************************************************************************************************/
#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include "gsmpwrmeter.hpp"
#include "design.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class.
//  @details        Gets the FPGA interface to empty the result FIFO so that only new values are used.
//  @param[in]      fpga                The FPGA interface.
//  @param[in]      one_result_per_slot True if one power results per slot is required, false if three are needed.
//  @param[in]      antenna_mode        The antenna mode to be used.
//  @param[in]      debug               Controls the debug messages that may be generated.
//
GsmPwrMeter::GsmPwrMeter(Fpga     &fpga,
                         bool     one_result_per_slot,
                         unsigned antenna_mode,
                         unsigned debug) :
    fpga(fpga),
    one_result_per_slot(one_result_per_slot),
    antenna_mode(antenna_mode),
    debugStr("GsmPwrMeter. ", debug)
{
    restart();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          The destructor has nothing to do.
//
GsmPwrMeter::~GsmPwrMeter()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Restart the power meter.
//
void GsmPwrMeter::restart(void)
{
    fpga.setup_gsm_pwr_meter(one_result_per_slot, antenna_mode);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Retrieves the ADC RSSI.
//  @return                             The RSSI normalised to the LSB^2 of the ADC samples.
//
double GsmPwrMeter::get_adc_power( void )
{
    return (fpga.get_adc_power());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Discards any old results.
//  @return                             The number of discarded samples.
//
unsigned GsmPwrMeter::flush_results(void)
{
    unsigned num_flushed = fpga.empty_gsm_fifo();

    debugStr(DebugStream::info1) << boost::format("Flushed %4u results\n") % num_flushed;

    return (num_flushed);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Gets the power result samples.
//  @param[in]      num_samples         The maximum number of samples to be retrieved.
//  @param[out]     samples             The vector of result samples.
//
void GsmPwrMeter::get_results(uint32_t max_num_samples, std::vector<double> &samples)
{
    double power_scaling_factor = POWER_SUM_RESULT_SCALING_FACTOR;
    if (one_result_per_slot)
    {
        power_scaling_factor /= static_cast<double>(NUM_SAMPLES_1_RESULT_PER_SLOT);
    }
    else
    {
        power_scaling_factor /= static_cast<double>(NUM_SAMPLES_3_RESULT_PER_SLOT);
    }

    //
    //  Get the raw samples.
    //
    std::vector<uint32_t> raw_data;
    fpga.read_gsm_fifo(max_num_samples, raw_data);

    debugStr(DebugStream::info1) << boost::format("Read    %4u results\n") % raw_data.size();

    //
    //  Convert the raw samples to powers with the appropriate scaling correction.
    //
    BOOST_FOREACH(uint32_t sample, raw_data)
    {
        double pwr = static_cast<double>(sample);
        pwr *= power_scaling_factor;
        samples.push_back(pwr);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Set the debug verbosity level.
//  @param[in]      level               The new verbosity level.
//
void GsmPwrMeter::debug_printing(unsigned level)
{
    debugStr.show_level(level);
}

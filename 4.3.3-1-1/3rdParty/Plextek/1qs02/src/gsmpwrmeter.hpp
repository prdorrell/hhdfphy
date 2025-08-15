/***********************************************************************************************************************
 *  Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 *  --------------------------------------------------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/gsmpwrmeter.hpp $
 * $Revision: 7013 $
 * $Author: pdm $
 * $Date: 2011-08-30 09:55:32 +0100 (Tue, 30 Aug 2011) $
 *******************************************************************************************************************//**
 *  @file
 *  @brief          The declaration of ::GsmPwrMeter class that controls the GSM channel power measurement in the FPGA.
 *  @details        Wrap up the hardware power meter in a class with any additional processing fluff needed.  Uses the
 *                  low level FPGA interface talk to the hardware.
 **********************************************************************************************************************/
#ifndef __GSMPWRMETER_HPP__
#define __GSMPWRMETER_HPP__

#include "debug.hpp"
#include "fpga.hpp"

///
/// @brief          Provides an object returns the GSM power measurement results.
/// @details        Makes use of the ::FPGA module to talk to the low level hardware.  More than one instance shouldn't
///                 exist at once or bad things will happen as they reconfigure and talk to the single meter we have in
///                 the fpga.
///
class GsmPwrMeter
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The constants.
    //
private:
    ///
    /// @brief          The scaling factor to be applied to the power result samples.
    /// @details        The FPGA divides the power-integrator results by 128 to fit them into the 32-bit FIFO words.
    ///
    static const double POWER_SUM_RESULT_SCALING_FACTOR = 128.0;

    ///
    /// @brief          The number of power samples accumulated for one result per slot.
    ///
    static const unsigned NUM_SAMPLES_1_RESULT_PER_SLOT = 375;

    ///
    /// @brief          The number of power samples accumulated for three results per slot.
    ///
    static const unsigned NUM_SAMPLES_3_RESULT_PER_SLOT = NUM_SAMPLES_1_RESULT_PER_SLOT/3;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The types.
    //
public:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The values.
    //
private:
    ///
    /// @brief          The interface to the FPGA.
    ///
    Fpga &fpga;

    ///
    /// @brief          True if one power results per slot is required, false if three are needed.
    ///
    bool one_result_per_slot;

    ///
    /// @brief          The antenna mode to be used.
    ///
    unsigned antenna_mode;

    ///
    /// @brief          The debug stream.
    /// @details        Debug messages are sent to this stream.  If the debugging level has been appropriately
    ///                 configured they appear on the standard output console.  See ::DebugStream.
    ///
    DebugStream debugStr;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The default constructor is declared but not defined in order to prevent a compiler generated
    ///                 version appearing.
    ///
private:
    GsmPwrMeter(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Constructs an instance of the class.
    /// @details        Gets the FPGA interface to start the GSM power meter and empty the result FIFO so that only new
    ///                 values are used.
    /// @param[in]      fpga                The FPGA interface.
    /// @param[in]      one_result_per_slot True if one power results per slot is required, false if three are needed.
    /// @param[in]      antenna_mode        The antenna mode to be used.
    /// @param[in]      debug               Controls the debug messages that may be generated.
    ///
public:
    GsmPwrMeter(Fpga     &fpga,
                bool     one_result_per_slot,
                unsigned antenna_mode,
                unsigned debug);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The destructor has nothing to do.
    ///
public:
    virtual ~GsmPwrMeter();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Restart the power meter.
    ///
public:
    void restart(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Retrieves the ADC RSSI.
    /// @return                             The RSSI normalised to the LSB^2 of the ADC samples.
    ///
public:
    double get_adc_power(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Discards any old results.
    /// @return                             The number of discarded samples.
    ///
public:
    unsigned flush_results(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Gets the power result samples.
    /// @param[in]      num_samples         The maximum number of samples to be retrieved.
    /// @param[out]     samples             The vector of result samples.
    ///
public:
    void get_results(uint32_t max_num_samples, std::vector<double> &samples);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Set the debug verbosity level.
    /// @param[in]      level               The new verbosity level.
    ///
public:
    void debug_printing(unsigned level);
};

#endif

/***********************************************************************************************************************
 *  Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 *  --------------------------------------------------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/fpga.cpp $
 * $Revision: 6981 $
 * $Author: pdm $
 * $Date: 2011-08-26 09:53:01 +0100 (Fri, 26 Aug 2011) $
 *******************************************************************************************************************//**
 *  @file
 *  @brief          The implementation of ::Fpga class that controls the FPGA.
 *  @details        The fpga uses two buffers 0/1 (even/odd) for many results.  This allows the FPGA to write
 *                  correlation results, frame counters etc to one block of memory/registers while the software reads
 *                  another.  If the software is too slow the fpga can just keep overwriting the same bank.  When the
 *                  software is ready it will request the fpga use the other buffer.  After the fpga has finished
 *                  writing the current set of results it will start using the requested buffer and the software will be
 *                  able to read a completed set of results from the other buffer.  This stops partial results and
 *                  results from different frames getting confused.
 *
 **********************************************************************************************************************/
#include "3gpp.h"
#include "fpga.hpp"
#include "debug.hpp"
#include "gpio.hpp"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include <stdexcept>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

#include <sys/mman.h>
#include <sys/time.h>
#include <fcntl.h>
#include <time.h>

#ifdef SWTEST_4G_FPGA

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Print out internal floats as hex to confirm binary representation
//  @details        Includes printing out the result of converting to FPGA 16-8 format
void Fpga::checkInternalFloatRepresentation(void)
{
    const unsigned array_size = 7;
    // Manually check the float
    const float fl[array_size] = {0.0f, 1.0f, -1.0f, 3.0f, -3.0f, 5.0f, -5.0f};
    IEEE754Hex converter;

    for (unsigned index = 0; index < array_size; index++)
    {
        converter.ieee754 = (float)fl[index];
        debugStr(DebugStream::info1) << boost::format("float: %f,\thex: 0x%8x\t\n")
                                                    % converter.ieee754
                                                    % converter.hex;
        uint8_t * pui = (uint8_t *)(&converter.hex);
        uint32_t byte0, byte1, byte2, byte3;
        byte0 = (uint8_t)*pui++;
        byte1 = (uint8_t)*pui++;
        byte2 = (uint8_t)*pui++;
        byte3 = (uint8_t)*pui++;
        debugStr(DebugStream::info1) << boost::format("Byte 0 = %x, 1 = %x, 2 = %x, 3= %x\n")
                                                      % byte0
                                                      % byte1
                                                      % byte2
                                                      % byte3;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  @brief         Uses FPGA Floating point test input/output to confirm correct floating point representation
///  @details       Check the 6 test values.  Convert from float to FPGA 16-8 and give to FPGA.  Confirm integar output matches.
///  @returns       EXIT_SUCCESS if all 6 values are returned as expected.  EXIT_FAILURE otherwise.
int Fpga::checkFpgaFloatRepresentation(void)
{
    const unsigned array_size = 6;
    // 1QR003: s4.9 Floating Point Format Test Registers specifies 6 possible test values
    const float test_data[array_size] = { 1.0f, -1.0f, 3.0f, -3.0f, 5.0f, -5.0f};
    IEEE754Hex converter;

    int check_pass = EXIT_SUCCESS;
    int32_t output_int;

    checkInternalFloatRepresentation(); // output of compiler float representation in Hexadecimal

    debugStr(DebugStream::info1) << "reg_float_format_test_4g_ptr = " << (void *)(reg_float_format_test_4g_ptr) << "\n";

    for (unsigned index = 0; index < array_size; index++)
    {
        converter.ieee754 = test_data[index];
        
        *reg_float_format_test_4g_ptr = (converter.hex & BINARY_16_8_SET_ZEROES_MASK);
        usleep(10000);   // allow sufficient time for the FPGA to convert the number
        output_int = (int32_t)(*reg_float_format_test_4g_ptr);

        debugStr(DebugStream::info1) << boost::format("checkFpgaFloatRepresentation: Input = %f (as hex = 0x%8x), output = %d: ")
                                                      % converter.ieee754
                                                      % converter.hex
                                                      % output_int;
        if (output_int != (int32_t)test_data[index])
        {
            check_pass = EXIT_FAILURE;
            debugStr(DebugStream::error) << "FAIL\n";
        }
        else
        {
            debugStr(DebugStream::info1) << "Pass\n";
        }
    }
    return check_pass;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  @brief         Enable/disable the regression test interface
///  @returns       Value of regression test register EN flag
bool Fpga::regression_4g_block_enable(bool enable)
{
    if (enable)
    {
        *reg_regression_test_ctrl_4g_ptr |= REGRESSION_4G_EN;
    }
    else
    {
        *reg_regression_test_ctrl_4g_ptr &= ~(uint32_t)REGRESSION_4G_EN;
    }
    return ((*reg_regression_test_ctrl_4g_ptr & (uint32_t) REGRESSION_4G_EN) == (uint32_t) REGRESSION_4G_EN);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  @brief         Program regression test FIFO with provided samples
///  @details       Enables the test interface if required
///  @returns       EXIT_FAILURE if FIFO not currently EMPTY or if the FULL flag becomes full during programming
int Fpga::regression_4g_program_fifo(std::list <uint32_t> samples)
{
    int result = EXIT_SUCCESS;
    uint32_t count = 0;
    if ((REGRESSION_4G_EN & *reg_regression_test_ctrl_4g_ptr) != REGRESSION_4G_EN)
    {
        *reg_regression_test_ctrl_4g_ptr |= REGRESSION_4G_EN;  // avoid setting when already on
    }
    if ((REGRESSION_4G_EMPTY & *reg_regression_test_ctrl_4g_ptr) == REGRESSION_4G_EMPTY)
    {
        BOOST_FOREACH(uint32_t cur_sample, samples)
        {
            if (REGRESSION_4G_FULL & *reg_regression_test_ctrl_4g_ptr == REGRESSION_4G_FULL)
            {
                debugStr(DebugStream::error) << "regression_4g_program_fifo:: FIFO full during programming\n";
                result = EXIT_FAILURE; // FIFO full with at least one more sample to store
                break;
            }
            *reg_regression_test_value_4g_ptr = cur_sample;
            count++;
        }
        if ((REGRESSION_4G_EMPTY & *reg_regression_test_ctrl_4g_ptr) == REGRESSION_4G_EMPTY)
        {
            debugStr(DebugStream::error) << "regression_4g_program_fifo:: Empty at end!\n";
            result = EXIT_FAILURE;
        }
    }
    else
    {
        debugStr(DebugStream::error) << "regression_4g_program_fifo:: Not Empty at start\n";
        result = EXIT_FAILURE;
    }
    debugStr(DebugStream::info1) << boost::format("regression_4g_program_fifo:: Programmed %u sample pairs\n")
                                                  % count;
    return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  @brief         Set the regression control TRIG flag
///  @details       If block is set do not return until EMPTY flag is set.  Otherwise return immediately
///  @returns       EXIT_FAILURE if regression block is not enabled or the FIFO is empty
int Fpga::regression_4g_trigger(bool block)
{
    int result = EXIT_FAILURE;
    if ((REGRESSION_4G_EN != (*reg_regression_test_ctrl_4g_ptr & REGRESSION_4G_EN))
        || (REGRESSION_4G_EMPTY != (*reg_regression_test_ctrl_4g_ptr & REGRESSION_4G_EMPTY)))
    {
        *reg_regression_test_ctrl_4g_ptr =  REGRESSION_4G_TRIG | REGRESSION_4G_EN; // don't disable when we trigger!
        if (block)
        {
            while (REGRESSION_4G_EMPTY != (*reg_regression_test_ctrl_4g_ptr & REGRESSION_4G_EMPTY))
            {
                usleep(1000); // 1ms
            }
        }
        debugStr(DebugStream::info1) << "Regression_4g_trigger:: Playback triggered\n";
        result = EXIT_SUCCESS;
    }
    return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  @brief         Returns value of the regression block control register
///  @returns       Contains the 4 control flags
uint32_t Fpga::regression_4g_status(void)
{
    return *reg_regression_test_ctrl_4g_ptr;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  @brief         Check ongoing status of the 4g regression test
///  @returns       integer containing an FPGA enum value showing progress
int32_t Fpga::regression_4g_iterate(void)
{
    int result = (int)REGRESSION_4G_ST_NORESULT;
    static enum eregression_4g_state regression4g_st = REGRESSION_4G_ST_NOENABLED;
    static unsigned sleep_count;

    uint32_t regression_ctrl = regression_4g_status();

    // Monitor status of the correlator command queue
    std::list<Fpga::CorrCmdStatus_4g_t> cmd_status_list;
    get_4g_correlation_command_status(cmd_status_list);

    switch (regression4g_st)
    {
        case REGRESSION_4G_ST_NOENABLED:
            if (0 != regression_ctrl & REGRESSION_4G_EN)
            {
                regression4g_st = REGRESSION_4G_ST_RUNNING;
                result = REGRESSION_4G_ST_RUNNING;
            }
            else
            {
                result = REGRESSION_4G_ST_NOENABLED; // proc_regression_4g() will treat this as end of play
            }
            break;
        case REGRESSION_4G_ST_RUNNING:
            if (0 != (regression_ctrl & REGRESSION_4G_EMPTY))   // all ADC samples played
            {
                debugStr(DebugStream::info1) << "REGRESSION_4G_ST_EMPTY";
                regression4g_st = REGRESSION_4G_ST_EMPTY;
                sleep_count = 10; // just in case there is a brief gap and this function is called too frequently
                result = REGRESSION_4G_ST_EMPTY;
            }
            else
            {
                result = REGRESSION_4G_ST_WAITING;  // still waiting for EMPTY
            }
            break;
        case REGRESSION_4G_ST_EMPTY: // count down until we finish
            debugStr(DebugStream::info1) << boost::format("REGRESSION_4G_ST_EMPTY: sleep_count =%u\n") % sleep_count;
            if (0 != (*reg_correlation_result_info_ptr & CORR_RESULT_4G_RDY))
            {
                regression4g_st = REGRESSION_4G_ST_RESULT;
                result = REGRESSION_4G_ST_RESULT;
            }
            else if (--sleep_count == 0)
            {
                regression4g_st = REGRESSION_4G_ST_NORESULT;
                result = REGRESSION_4G_ST_NORESULT;
                break;
            }
            else
            {
                result = REGRESSION_4G_ST_WAITING; // wait for timeout
            }
            break;
        case REGRESSION_4G_ST_RESULT:
            regression4g_st = REGRESSION_4G_ST_EMPTY;   // back to check for further results
            result = REGRESSION_4G_ST_WAITING;
            break;
        case REGRESSION_4G_ST_NORESULT: // no result detected
            {
                debugStr(DebugStream::info1) << "REGRESSION_4G_ST_NORESULT\n";
                stop_correlation();
                regression_4g_block_enable(false);
                regression4g_st = REGRESSION_4G_ST_FINISHED;
                result = REGRESSION_4G_ST_FINISHED;
            }
            break;
        case REGRESSION_4G_ST_FINISHED:
            debugStr(DebugStream::info1) << "REGRESSION_4G_ST_FINISHED\n";
            regression4g_st = REGRESSION_4G_ST_NOENABLED;
            result = REGRESSION_4G_ST_NOENABLED;
            break;
        case REGRESSION_4G_ST_WAITING: // fallthrough to - valid result but not a valid state
        default:
            result = REGRESSION_4G_ST_FINISHED;
            debugStr(DebugStream::error) << boost::format("regression_4g_iterate: Statemachine error: state = %d")
                                                          % (int)regression4g_st;
            break;
    }
    return result;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Read the coefficients for channel filter 2.
//  @details        Read the input file using the format specified in 1QR005, "4G Changes to the HHDFPHY SOftware",
//                  checking the filename format and the content format.
//
//                  Error and warning conditions:
//
//                      - File format error,            DebugStream::error,     return false with no samples
//                      - Filter size even or greater than the maximum
//                                                      DebugStream::error,     return false with no samples
//                      - Coefficient out of range      DebugStream::warn2,     return true with clipped coefficient
//
//  @param[in]      filename            The name of the coefficient file.
//  @param[out]     coeffs              The ADC samples
//  @return                             EXIT_SUCCESS if the read was successful (all OK or a only warning issued,
//                                      EXIT_FAILURE otherwise.
//
int Fpga::read_regression_4g_samples(std::string filename, 
                                      std::list<uint32_t> &adc_samples)
{
    const unsigned MAX_POST_DATA_ZEROS = 5U; // allow 5 consecutive 0 samples before calling it a day
    const int PRE_DATA_ZERO_COUNT = 2;  // insert 2 i/q "zero" pairs in front of samples
    int success = EXIT_FAILURE;
    
    //
    //  Parse the filename
    //
    std::string base_filename = filename.substr(filename.find_last_of("/\\") + 1);
    debugStr(DebugStream::info1) << boost::format(  "Reading regression_4g ADC samples from %s\n" )
                                                  % filename;
    
    //
    //  Read the file contents.
    //
    std::ifstream ifs(filename.c_str());
    if (ifs.fail())
    {
        debugStr(DebugStream::error) << boost::format(  "regression_4g ADC samples file (%s) could not be opened\n" )
                                                        % filename;
    }
    else
    {
        success = EXIT_SUCCESS;
        
        //
        //  Read the sample values.
        //
        union {
            int16_t signed_sample;
            uint16_t unsigned_sample;
        } adci, adcq;
        unsigned adc_num = 0;
        bool founddatastart = false;
        unsigned post_data_zero_count = 0;
        std::string nextLine;

        while (std::getline(ifs, nextLine)
               && adc_num < Fpga::REGRESSION_4G_MAX_SAMPLES
               && post_data_zero_count < MAX_POST_DATA_ZEROS)
        {
            int adc_i, adc_q;
            int num_args_filled = sscanf(nextLine.c_str(), "%d %d", &adc_i, &adc_q);
            if (2 != num_args_filled)
            {
                debugStr(DebugStream::error) << boost::format(  "ADC samples format error (%s)\n" )
                                                                % nextLine;
                success = EXIT_FAILURE;
                break;
            }
            else
            {
                adci.signed_sample = adc_i;
                adcq.signed_sample = adc_q;
                if (Fpga::REGRESSION_4G_MAX_ADC_SAMPLE < static_cast<unsigned>(abs(adci.signed_sample)))
                {
                    //
                    //  For some reason using a class constant in the Boost formatting below produces a linker
                    //  error.
                    //
                    unsigned max_abs_adc_val = Fpga::REGRESSION_4G_MAX_ADC_SAMPLE;
                    debugStr(DebugStream::warn2) << boost::format(  "Regression test ADC i sample too large or small (abs(%d) > %u)\n" )
                                                                    % adci.signed_sample
                                                                    % max_abs_adc_val;
                    if (adci.signed_sample > 0)
                    {
                        adci.signed_sample = Fpga::REGRESSION_4G_MAX_ADC_SAMPLE;
                    }
                    else
                    {
                        adci.signed_sample = -(uint16_t)Fpga::REGRESSION_4G_MAX_ADC_SAMPLE;
                    }
                }
                if (Fpga::REGRESSION_4G_MAX_ADC_SAMPLE < static_cast<unsigned>(abs(adcq.signed_sample)))
                {
                    //
                    //  For some reason using a class constant in the Boost formatting below produces a linker
                    //  error.
                    //
                    unsigned max_abs_adc_val = Fpga::REGRESSION_4G_MAX_ADC_SAMPLE;
                    debugStr(DebugStream::warn2) << boost::format(  "Regression test ADC q sample too large or small (abs(%d) > %u)\n" )
                                                                    % adcq.signed_sample
                                                                    % max_abs_adc_val;
                    if (adcq.signed_sample > 0)
                    {
                        adcq.signed_sample = Fpga::REGRESSION_4G_MAX_ADC_SAMPLE;
                    }
                    else
                    {
                        adcq.signed_sample = -(uint16_t)Fpga::REGRESSION_4G_MAX_ADC_SAMPLE;
                    }
                }
                if (adci.signed_sample != 0 || adcq.signed_sample != 0)
                {
                    if (!founddatastart)
                    {
                        // place desired number of zero ADC samples before real data
                        for (int i = 0; i < PRE_DATA_ZERO_COUNT; i++)
                        {
                            adc_samples.push_back(( uint32_t)0u);
                            adc_num++;
                        }
                    }
                    founddatastart = true; // found first non-zero data sample
                }
                if (founddatastart)
                {
                    if (adci.signed_sample == 0 && adcq.signed_sample == 0)
                    {
                        post_data_zero_count++;
                    }
                    else
                    {
                        post_data_zero_count = 0;
                    }
                    // pack the two adc samples so the i sample is in the least signficant and q in most
                    // significant words of a 32-bit value
                    uint32_t value = adci.unsigned_sample + (adcq.unsigned_sample << 16U);
                    adc_samples.push_back(value);
                    adc_num++;
                }
            }
        }
        debugStr(DebugStream::info1) << boost::format("read_regression_4g_samples:: Found %u sample pairs in file\n")
                                                      % adc_num;

    }
    
    return success;
}

#endif // ifdef SWTEST_4G_FPGA

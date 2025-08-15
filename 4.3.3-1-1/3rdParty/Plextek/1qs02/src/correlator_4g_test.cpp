/***********************************************************************************************************************
 *  Copyright (c) 2021 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  Filename:   correlator_4g_test.cpp
 *  Author(s):  pdm
 *******************************************************************************************************************//**
 * File Description
 * ================
 *
 *  @file
 *  @brief          The implementation of functions used to test the ::Correlator_4g class and extensions to the class
 *                  for testing the FPGA.
 *  @details        The functions include test versions of the Fpga class functions used by the ::Correlator_4g class.
 *                  Many status reports are "Debug::fatal" so that it is clear that this is a debug build.
 **********************************************************************************************************************/
#include "correlator_4g.hpp"
#include "design.hpp"

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include <complex>
#include <memory>
#include <functional>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <cassert>
#include <sys/time.h>
#include <time.h>

#ifdef SWTEST_4G_CORR
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Print the correlation accumulator results
    //  @details        Only prints the results with non-zero peaks.
    //
    void Correlator_4g::test_print_acc_results(void)
    {
        bool header_printed = false;
        for (size_t peak_num = 0 ; peak_num < fpga_corr_result_accumulator.size() ; ++peak_num)
        {
            if (   (0.0 < fpga_corr_result_accumulator[peak_num].corr_out_peak_acc)
                || (0.0 < fpga_corr_result_accumulator[peak_num].corr_inp_sum_acc)
                || (0   < fpga_corr_result_accumulator[peak_num].num_peaks))
            {
                if (!header_printed)
                {
                    debugStr(DebugStream::error) <<   boost::format(  "Accumulator results:\n" );
                    header_printed = true;
                }
                debugStr(DebugStream::error) <<   boost::format(  "    %4u    %f    %f    %2u\n" )
                                                % peak_num
                                                % fpga_corr_result_accumulator[peak_num].corr_out_peak_acc
                                                % fpga_corr_result_accumulator[peak_num].corr_inp_sum_acc
                                                % fpga_corr_result_accumulator[peak_num].num_peaks;
            }
        }
        
        if (!header_printed)
        {
            debugStr(DebugStream::error) <<   boost::format(  "Accumulator results: no peaks\n" );
        }
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Print the correlation statitics results if the count is greater than zero.
    //
    void Correlator_4g::test_print_stat_results(void)
    {
        debugStr(DebugStream::error) <<   boost::format(  "Statistics results: %4u    %f    %f\n" )
                                        % corr_stat_res.count
                                        % corr_stat_res.peak_inp_sums
                                        % corr_stat_res.sum_inp_sums;
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Print the results returned by the read_results function.
    //  @details        Only prints the vector results with non-zero values.
    //  @param[out]     corr_peak_counts    The vector of counts of the number of accumulated peaks.
    //  @param[out]     corr_peak_vals      The vector of correlation peaks spanning a result interval.
    //  @param[out]     corr_peak_mean_inputs_adc_lsb_sq
    //                                      The vector of the mean of the input samples corresponding to a peak above
    //                                      the threshold. Effectively this is the mean power of the detected SRS burst
    //                                      in units of (ADC LSB)^2.
    //  @param[out]     corr_inp_max_mean_adc_lsb_sq
    //                                      The maximum of the mean of the input samples spanned by the correlation
    //                                      coefficients over the time interval spanned by the sequence. Effectively
    //                                      this is the mean power of the strongest burst in units of (ADC LSB)^2.
    //  @param[out]     corr_inp_mean_adc_lsb_sq
    //                                      The mean of the mean of the input samples spanned by the correlation
    //                                      coefficients over the time interval spanned by the sequence. Effectively
    //                                      this is the mean input power in units of (ADC LSB)^2.
    //
    void Correlator_4g::test_print_read_results_returns(std::vector<unsigned> corr_peak_counts, 
                                                        std::vector<float> corr_peak_vals, 
                                                        std::vector<float> corr_peak_mean_inputs_adc_lsb_sq, 
                                                        float corr_inp_max_mean_adc_lsb_sq,
                                                        float corr_inp_mean_adc_lsb_sq)
    {
        bool header_printed = false;
        for (size_t peak_num = 0 ; peak_num < corr_peak_counts.size() ; ++peak_num)
        {
            if (   (0   < corr_peak_counts[peak_num])
                || (0.0 < corr_peak_vals[peak_num])
                || (0.0 < corr_peak_mean_inputs_adc_lsb_sq[peak_num]))
            {
                if (!header_printed)
                {
                    debugStr(DebugStream::error) <<   boost::format(  "Averaged correlation results:\n" );
                    header_printed = true;
                }
                debugStr(DebugStream::error) <<   boost::format(  "    %4u    %4u    %f    %f\n" )
                                                % peak_num
                                                % corr_peak_counts[peak_num]
                                                % corr_peak_vals[peak_num]
                                                % corr_peak_mean_inputs_adc_lsb_sq[peak_num];
            }
        }
        
        if (!header_printed)
        {
            debugStr(DebugStream::error) <<   boost::format(  "Averaged correlation results: no non-zero results\n" );
        }

        debugStr(DebugStream::error) <<   boost::format(  "Statistics results:              %f    %f\n" )
                                    % corr_inp_max_mean_adc_lsb_sq
                                    % corr_inp_mean_adc_lsb_sq;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Wrapper for the Fpga class's send_4g_coefs function that writes the coefficients for the channel
    //                  filters and correlator for the 4G fpga.
    //  @details        If there is a fault with any of the supplied coefficients nothing is progrrammed to the fpga
    //  @param[in]      channel1    Array of int16_t with parameters for channel filter stage 1
    //  @param[in]      channel2    Array of int16_t with parameters for channel filter stage 1
    //  @param[in]      correl_taps Array of 320 correlator taps in IEEE 754 binary32 format
    //  @returns        false if there is a problem with any of the supplied parameters/coefficients
    // 
    bool Correlator_4g::fpga_send_4g_coefs(const std::vector<int16_t>&channel1,
                                           const std::vector<int16_t>&channel2,
                                           const std::vector<float>&correl_taps)
    {
        debugStr(DebugStream::error) << boost::format(  "Setting the FPGA coefficients, lengths = %u %u %u\n" )
                                                      % channel1.size()
                                                      % channel2.size()
                                                      % correl_taps.size();
                                                      
        debugStr(DebugStream::error) << "    Channel 1:  ";
        BOOST_FOREACH(int16_t val, channel1)
        {
            debugStr(DebugStream::error) << boost::format(  "%d, " ) % val;
        }                                                      
        debugStr(DebugStream::error) << "\n";

        debugStr(DebugStream::error) << "    Channel 2:  ";
        BOOST_FOREACH(int16_t val, channel2)
        {
            debugStr(DebugStream::error) << boost::format(  "%d, " ) % val;
        }                                                      
        debugStr(DebugStream::error) << "\n";        

        debugStr(DebugStream::error) << "    Correlator: ";
        BOOST_FOREACH(float val, correl_taps)
        {
            debugStr(DebugStream::error) << boost::format(  "%e, " ) % val;
        }                                                      
        debugStr(DebugStream::error) << "\n";        
        
        return (fpga.send_4g_coefs(channel1, channel2, correl_taps));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Wrapper for the Fpga class's get_4g_time function that returns the current FPGA time.
    //  @returns        Current FPGA time in a uint64_t
    // 
    uint64_t Correlator_4g::fpga_get_4g_time(void)
    {
        debugStr(DebugStream::error) << boost::format(  "Reading the FPGA time = %lu\n" )
                                                      % fpga_get_4g_time_retval;
        
        return (fpga_get_4g_time_retval);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Wrapper for the Fpga class's fpga_set_4g_threshold_sqrd that is used to set the threshold.
    //  @param[in]      threshold_sqrd   float squared threshold
    // 
    void Correlator_4g::fpga_set_4g_threshold_sqrd(float threshold_sqrd)
    {
        debugStr(DebugStream::error) << boost::format(  "Setting the FPGA threshold squared = %f\n" )
                                                      % threshold_sqrd;
        
        fpga.set_4g_threshold_sqrd(threshold_sqrd);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Fpga class's get_4g_statistics that is used to control and retrieve results from
    ///                 the statistics block.
    /// @param[in]      restart  Restart engine afterwards?
    /// @param[out]     peak     Statistics engine peak values
    /// @param[out]     sum      Statistics engine sum of values
    /// @returns        Number of values contributing to the results
    ///
    uint32_t Correlator_4g::fpga_get_4g_statistics(bool restart, float &peak, float &sum)
    {
        if (restart)
        {
            debugStr(DebugStream::error) << boost::format(  "Getting correlation statistics with restart\n" );
        }
        else
        {
            debugStr(DebugStream::error) << boost::format(  "Getting correlation statistics without restart\n" );
        }
        
        peak = fpga_get_4g_statistics_results.peak_inp_sums;
        sum = fpga_get_4g_statistics_results.sum_inp_sums;
        
        return (fpga_get_4g_statistics_results.count);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Wrapper for the Fpga class's setup_4g_correlation that is used to setup a correlation sequence.
    //  @param[in]      cmds  a list of CorrCmd_4g_t type
    //  @returns        unsigned, the number of commands that were written
    // 
    uint32_t Correlator_4g::fpga_setup_4g_correlation(std::list<Fpga::CorrCmd_4g_t> cmds)
    {
        debugStr(DebugStream::error) << boost::format(  "Setting up a correlation sequence of length %u\n" )
                                                      % cmds.size();
        
        return (fpga.setup_4g_correlation(cmds));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Wrapper for the Fpga classe's get_4g_peak_data that reads the status of recent colleration
    //                  commands.
    //  @param[in]      maxNumPeaks the maximum number of peaks to write into the destination array
    //  @param[out]     peaks array of peaks read from the peaks fifo
    //  @param[out]     numPeaks actual number of peaks placed into peaks array
    //  @returns        bool, "false" if the FIFO's overflow flag is set
    // 
    bool Correlator_4g::fpga_get_4g_peak_data(size_t maxNumPeaks, Fpga::CorrPeak_4g_t peaks[], uint32_t &numPeaks)
    {
        assert(maxNumPeaks >= fpga_get_4g_peak_data_peaks.size());  //  Would be a tester's mistake.
        
        debugStr(DebugStream::error) << boost::format(  "Reading the peak data (%u peaks) into a maximum length %u array\n" )
                                                      % fpga_get_4g_peak_data_peaks.size()
                                                      % maxNumPeaks;

        size_t peak_num = 0;
        BOOST_FOREACH(Fpga::CorrPeak_4g_t peak, fpga_get_4g_peak_data_peaks)
        {
            peaks[peak_num++] = peak;
        }
        
        numPeaks = fpga_get_4g_peak_data_peaks.size();
        fpga_get_4g_peak_data_peaks.clear();
        
        return (fpga_get_4g_peak_data_retval);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Fpga class's get_4g_correlation_command_status that teads the status of recent
    ///                 colleration commands
    /// @param[out]     cmd_status a list of CorrCmdStatus_4g_t type showing command ids and associated status
    /// @returns        bool, "false" if the FIFO's overflow flag is set
    ///
    bool Correlator_4g::fpga_get_4g_correlation_command_status(std::list<Fpga::CorrCmdStatus_4g_t> &cmd_status)
    {
        debugStr(DebugStream::error) << boost::format(  "Reading the command status (%u statuses)\n" )
                                                      % fpga_get_4g_peak_data_peaks.size();

        cmd_status = fpga_get_4g_correlation_command_status_cmd_status;
        fpga_get_4g_correlation_command_status_cmd_status.clear();
        
        return (fpga_get_4g_correlation_command_status_retval);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Test the restart_correlation() function
    //  @details        There is no automated checking of the results, which must be manually confirmed.
    // 
    void Correlator_4g::test_restart_correlation(void)
    {
        debugStr(DebugStream::info4) << boost::format(  "test_restart_correlation ----------------------------------\n" );
        
        //
        //  Should produce an error because the command queue is not empty.
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should produce an error because the command queue is not empty.\n" );
        
        current_mode = SEARCH;
        
        fpga_get_4g_time_retval = 1;
        
        fpga_corr_seq_cmds.clear();
        Fpga::CorrCmd_4g_t cmd = { Fpga::CORRELATE_FROM_TO, 33, 100, 200-1 };
        fpga_corr_seq_cmds.push_back(cmd);
        
        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output
        
        //
        //  Should start a single correlation for time 153600, to start at 153145 and finish at 1017599
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should start a single correlation for time 153600, to start at 153145 and finish at 1017599\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should empty the FPGA peak data (reporting that one was discarded).\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should restart the statistics engine.\n" );
        
        current_mode = SEARCH;
        fpga_get_4g_time_retval = 1;
        
        fpga_corr_seq_cmds.clear();
        
        fpga_get_4g_peak_data_peaks.clear();
        Fpga::CorrPeak_4g_t corr_peak_1 = { 33, 1.0, 1.0, 1.0 };
        fpga_get_4g_peak_data_peaks.push_back(corr_peak_1);
        fpga_get_4g_peak_data_retval = true;

        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output

        //
        //  Should produce an error for restarting before the last correlation is complete.
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should produce an error for restarting before the last correlation is complete.\n" );
        
        current_mode = SEARCH;
        fpga_get_4g_time_retval = 1;
        
        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output

        //
        //  Should produce an error for restarting before the last correlation is complete.
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should produce an error for restarting before the last correlation is complete.\n" );
        
        current_mode = SEARCH;
        fpga_get_4g_time_retval = 1017599;

        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output
        
        //
        //  Should produce an error for restarting before the last correlation is complete.
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should produce an error for restarting before the last correlation is complete (time OK, but command queue not yet empty).\n" );
        
        current_mode = SEARCH;
        fpga_get_4g_time_retval = 1017600;

        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output
        
        //
        //  Should start a single correlation for time 1180800, to start at 1180345 and finish at 2044799.
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should start a single correlation for time 1180800, to start at 1180345 and finish at 2044799.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should empty the FPGA peak data.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should restart the statistics engine.\n" );
        
        current_mode = SEARCH;
        fpga_get_4g_time_retval = 1027600;
        
        fpga_corr_seq_cmds.clear();

        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output
        
        //
        //  Should start 45 correlations:
        //      0:  For time 1689600, to start at 1689145 and finish at 1690044
        //      1:                    to start at 1698745 and finish at 1699644
        //
        //     44:                    to start at 2111545 and finish at 2112444
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should start 45 correlations:\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>     0:  For time 1689600, to start at 1689145 and finish at 1690044\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>     1:                    to start at 1698745 and finish at 1699644\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>     ....\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>    44:                    to start at 2111545 and finish at 2112444\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should empty the FPGA peak data.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should restart the statistics engine.\n" );
        
        current_mode = TRACK;
        track_start_offset_samples = 0;
        fpga_get_4g_time_retval = 1537600;
        
        fpga_corr_seq_cmds.clear();

        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output
        
        //
        //  Should produce an error for restarting before the last correlation is complete.
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should produce an error for restarting before the last correlation is complete.\n" );
        current_mode = TRACK;
        fpga_get_4g_time_retval = 1690045;

        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output
        
        //
        //  Should produce an error for restarting before the last correlation is complete.
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should produce an error for restarting before the last correlation is complete.\n" );
        current_mode = TRACK;
        fpga_get_4g_time_retval = 2112444;

        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output
        
        //
        //  Should start 45 correlations:
        //      0:  For time 2361700, to start at 2361245 and finish at 2362144
        //      1:                    to start at 2370845 and finish at 2371744
        //
        //     44:                    to start at 2783645 and finish at 2784544
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should start 45 correlations:\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>     0:  For time 2361700, to start at 2361245 and finish at 2362144\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>     1:                    to start at 2370845 and finish at 2371744\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>     ....\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>    44:                    to start at 2783645 and finish at 2784544\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should empty the FPGA peak data.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should restart the statistics engine.\n" );
        
        current_mode = TRACK;
        fpga_get_4g_time_retval = 2212444;
        
        fpga_corr_seq_cmds.clear();

        change_start(100);  //  Calls restart_correlation()
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output
        
        //
        //  Should start a single correlation for time 3148800, to start at 3148345 and finish at 3580799
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should start a single correlation for time 3148800, to start at 3148345 and finish at 3580799\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should empty the FPGA peak data (reporting that one was discarded).\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should restart the statistics engine.\n" );
        
        current_mode = CONT_TRACK;
        fpga_get_4g_time_retval = 3000000;
        
        fpga_corr_seq_cmds.clear();
        
        fpga_get_4g_peak_data_peaks.clear();
        Fpga::CorrPeak_4g_t corr_peak_2 = { 33, 1.0, 1.0, 1.0 };
        fpga_get_4g_peak_data_peaks.push_back(corr_peak_2);
        fpga_get_4g_peak_data_retval = true;

        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output

        //
        //  Should produce an error for restarting before the last correlation is complete.
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should produce an error for restarting before the last correlation is complete.\n" );
        
        current_mode = CONT_TRACK;
        fpga_get_4g_time_retval = 3000000;
        
        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output

        //
        //  Should produce an error for restarting before the last correlation is complete.
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should produce an error for restarting before the last correlation is complete.\n" );
        
        current_mode = CONT_TRACK;
        fpga_get_4g_time_retval = 3580799;

        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output
        
        //
        //  Should produce an error for restarting before the last correlation is complete.
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should produce an error for restarting before the last correlation is complete (time OK, but command queue not yet empty).\n" );
        
        current_mode = CONT_TRACK;
        fpga_get_4g_time_retval = 3580800;

        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output
        
        //
        //  Should start a single correlation for time 3734400, to start at 3733945 and finish at 4166399.
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should start a single correlation for time 3734400, to start at 3733945 and finish at 4166399.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should empty the FPGA peak data.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should restart the statistics engine.\n" );
        
        current_mode = CONT_TRACK;
        fpga_get_4g_time_retval = 3580800;
        
        fpga_corr_seq_cmds.clear();

        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Test the busy() function
    //  @details        There is no automated checking of the results, which must be manually confirmed.
    //
    void Correlator_4g::test_result_available(void)
    {
        debugStr(DebugStream::info4) << boost::format(  "test_result_available -------------------------------------\n" );
        
        //
        //  Test 1
        //  Clear the command sequence so that it appears that there is a result available.
        //
        fpga_corr_seq_cmds.clear();
        
        debugStr(DebugStream::info4) << boost::format(  ">>> Command sequence is empty so should produce 1 (true).\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should not read the statistics results.\n" );
        
        debugStr(DebugStream::error) << boost::format(  "Result available = %u\n" ) % result_available();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output
        
        //
        //  Test 2
        //  Start a search sequence for time 10147200 to start at 10146745 and finish at 11011199
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should start a single correlation for time 10147200 to start at 10146745 and finish at 11011199.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should empty the FPGA peak data.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should restart the statistics engine.\n" );
        
        current_mode = SEARCH;
        fpga_get_4g_time_retval = 10000000;
        
        fpga_corr_seq_cmds.clear();

        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output
        
        //
        //  No peaks and no status so no result yet available.
        //
        fpga_get_4g_time_retval = 10000000;
        
        fpga_get_4g_peak_data_peaks.clear();
        fpga_get_4g_peak_data_retval = true;
        
        fpga_get_4g_correlation_command_status_cmd_status.clear();
        fpga_get_4g_correlation_command_status_retval = true;
        
        debugStr(DebugStream::info4) << boost::format(  ">>> No peaks, no status and command queue not empty so should produce 0 (false).\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should not read the statistics results.\n" );
       
        debugStr(DebugStream::error) << boost::format(  "Result available = %u\n" ) % result_available();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output

        //
        //  Various peaks (see below), no status, peaks indicate sequence completed so result available.
        //
        fpga_get_4g_time_retval = 11013000;
        
        fpga_get_4g_peak_data_peaks.clear();
        Fpga::CorrPeak_4g_t corr_peak_A_1  = { 10140000, 1.0, 1.0, 1.0 };    //  Before the start
        Fpga::CorrPeak_4g_t corr_peak_A_2  = { 10146745, 1.0, 1.0, 1.0 };    //  At the start of the flush
        Fpga::CorrPeak_4g_t corr_peak_A_3  = { 10147199, 1.0, 1.0, 1.0 };    //  At the end of the flush
        Fpga::CorrPeak_4g_t corr_peak_A_4  = { 10147200, 1.0, 1.0, 2.0 };    //  At the start of the wanted
        Fpga::CorrPeak_4g_t corr_peak_A_5  = { 10148200, 1.0, 0.0, 1.0 };    //  In the middle of the wanted (offset 1000 in the results), but denominator zero
        Fpga::CorrPeak_4g_t corr_peak_A_6  = { 10149200, 1.0, -1.0, 1.0 };   //  In the middle of the wanted (offset 2000 in the results), but denominator negative
        Fpga::CorrPeak_4g_t corr_peak_A_7  = { 10150200, -1.0, 1.0, 1.0 };   //  In the middle of the wanted (offset 3000 in the results), but numerator negative
        Fpga::CorrPeak_4g_t corr_peak_A_8  = { 10151200, -1.0, -1.0, 1.0 };  //  In the middle of the wanted (offset 4000 in the results), but numerator and denominator negative
        Fpga::CorrPeak_4g_t corr_peak_A_9  = { 10156000, 1.0, 1.0, 1.0 };    //  In the middle of the wanted (offset 8800 in the results)
        Fpga::CorrPeak_4g_t corr_peak_A_10 = { 11011199, 1.0, 1.0, 1.0 };    //  At the end of the wanted
        Fpga::CorrPeak_4g_t corr_peak_A_11 = { 11011200, 1.0, 1.0, 1.0 };    //  After the end of the wanted (should clear the command queue)
        Fpga::CorrPeak_4g_t corr_peak_A_12 = { 11011201, 1.0, 1.0, 1.0 };    //  After the end of the wanted (queue has already been emptied)
        fpga_get_4g_peak_data_peaks.push_back(corr_peak_A_1);
        fpga_get_4g_peak_data_peaks.push_back(corr_peak_A_2);
        fpga_get_4g_peak_data_peaks.push_back(corr_peak_A_3);
        fpga_get_4g_peak_data_peaks.push_back(corr_peak_A_4);
        fpga_get_4g_peak_data_peaks.push_back(corr_peak_A_5);
        fpga_get_4g_peak_data_peaks.push_back(corr_peak_A_6);
        fpga_get_4g_peak_data_peaks.push_back(corr_peak_A_7);
        fpga_get_4g_peak_data_peaks.push_back(corr_peak_A_8);
        fpga_get_4g_peak_data_peaks.push_back(corr_peak_A_9);
        fpga_get_4g_peak_data_peaks.push_back(corr_peak_A_10);
        fpga_get_4g_peak_data_peaks.push_back(corr_peak_A_11);
        fpga_get_4g_peak_data_peaks.push_back(corr_peak_A_12);
        fpga_get_4g_peak_data_retval = true;

        fpga_get_4g_correlation_command_status_cmd_status.clear();
        fpga_get_4g_correlation_command_status_retval = true;
        
        debugStr(DebugStream::info4) << boost::format(  ">>> Various peaks, including errors and warnings so should report:\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>     Peak from before start: 10140000 <  10146745\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>     Peak with zero denominator-squared: 1 / 0 at 10148200\n" );
//  The following warning has been removed.
//        debugStr(DebugStream::info4) << boost::format(  ">>>     Peak with negative numerator or denominator-squared: 1 / -1 at 10149200\n" );
//        debugStr(DebugStream::info4) << boost::format(  ">>>     Peak with negative numerator or denominator-squared: -1 / 1 at 10150200\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>     Peak from after stop:   11011200 > 11011199\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>     Peak from after stop (queue has already been emptied): 11011201\n" );
        
        debugStr(DebugStream::info4) << boost::format(  ">>> No status, but command queue ends up empty so should produce 1 (true).\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should read the statistics results without a restart.\n" );

        fpga_get_4g_statistics_results.count = 100;
        fpga_get_4g_statistics_results.peak_inp_sums = 0.1;
        fpga_get_4g_statistics_results.sum_inp_sums  = 9.0;
        
        debugStr(DebugStream::error) << boost::format(  "Result available = %u\n" ) % result_available();
        debugStr(DebugStream::info4) << boost::format(  ">>> Valid peaks were at result interval offsets 0, 8800 and 9599 so accumulation results should be:\n" );
        
        debugStr(DebugStream::info4) << boost::format(  ">>>        0    1.500000    2.000000     1\n");
        debugStr(DebugStream::info4) << boost::format(  ">>>        1    0.500000    0.000000     0\n");
        debugStr(DebugStream::info4) << boost::format(  ">>>     8799    0.500000    0.000000     0\n");
        debugStr(DebugStream::info4) << boost::format(  ">>>     8800    1.000000    1.000000     1\n");
        debugStr(DebugStream::info4) << boost::format(  ">>>     8801    0.500000    0.000000     0\n");
        debugStr(DebugStream::info4) << boost::format(  ">>>     9598    0.500000    0.000000     0\n");
        debugStr(DebugStream::info4) << boost::format(  ">>>     9599    1.500000    1.000000     1\n");

        debugStr(DebugStream::info4) << boost::format(  ">>> Statistics should be:\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>      100    0.100000    9.000000\n");
        
        test_print_acc_results();
        test_print_stat_results();
        
        //
        //  Test 3
        //  Start a search sequence for time 20150400 to start at 20149945 and finish at 21014399
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should start a single correlation for time 20150400 to start at 20149945 and finish at 21014399.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should empty the FPGA peak data.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should restart the statistics engine.\n" );
        
        current_mode = SEARCH;
        fpga_get_4g_time_retval = 20000000;
        
        fpga_corr_seq_cmds.clear();

        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output

        //
        //  A single peak that does not complete the sequence (see below), no status, so result not available.
        //
        fpga_get_4g_time_retval = 20423999;
        
        fpga_get_4g_peak_data_peaks.clear();
        Fpga::CorrPeak_4g_t corr_peak_B_1  = { 20156000, 1.0, 1.0, 1.0 };    //  In the middle of the wanted (offset 5600 in the result interval)
        fpga_get_4g_peak_data_peaks.push_back(corr_peak_B_1);
        fpga_get_4g_peak_data_retval = true;

        fpga_get_4g_correlation_command_status_cmd_status.clear();
        fpga_get_4g_correlation_command_status_retval = true;
        
        debugStr(DebugStream::info4) << boost::format(  ">>> A single peak that does not complete the sequence (see below), no status, so result not available and so should produce 0 (false).\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should not read the statistics results.\n" );
        
        debugStr(DebugStream::error) << boost::format(  "Result available = %u\n" ) % result_available();
        debugStr(DebugStream::info4) << boost::format(  ">>> Valid peak was at result interval offset 5600 so accumulation results should be:\n" );
        
        debugStr(DebugStream::info4) << boost::format(  ">>>     5599    0.500000    0.000000     0\n");
        debugStr(DebugStream::info4) << boost::format(  ">>>     5600    1.000000    1.000000     1\n");
        debugStr(DebugStream::info4) << boost::format(  ">>>     5601    0.500000    0.000000     0\n");
        debugStr(DebugStream::info4) << boost::format(  ">>> There should be no statistics.\n" );
        
        test_print_acc_results();
        test_print_stat_results();

        //
        //  A single peak that does not complete the sequence (see below), no status, but time is after end of sequence
        //  plus one result interval so sequence completes and result is available.
        //
        fpga_get_4g_time_retval = 21024000;
        
        fpga_get_4g_peak_data_peaks.clear();
        Fpga::CorrPeak_4g_t corr_peak_C_1  = { 20252001, 1.0, 1.0, 1.0 };    //  In the middle of the wanted (offset 5601 in the result interval)
        fpga_get_4g_peak_data_peaks.push_back(corr_peak_C_1);
        fpga_get_4g_peak_data_retval = true;

        fpga_get_4g_correlation_command_status_cmd_status.clear();
        fpga_get_4g_correlation_command_status_retval = true;
        
        debugStr(DebugStream::info4) << boost::format(  ">>> A single peak that does not complete the sequence (see below), no status, but time is after end of sequence plus one result interval so sequence completes and result is available and so should produce 1 (true).\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should read the statistics results without a restart.\n" );

        fpga_get_4g_statistics_results.count = 110;
        fpga_get_4g_statistics_results.peak_inp_sums = 0.2;
        fpga_get_4g_statistics_results.sum_inp_sums  = 8.0;
        
        debugStr(DebugStream::error) << boost::format(  "Result available = %u\n" ) % result_available();
        debugStr(DebugStream::info4) << boost::format(  ">>> Valid peaks were at result buffer offsets 5600 and 5601 so accumulation results should be:\n" );
        
        debugStr(DebugStream::info4) << boost::format(  ">>>     5599    0.500000    0.000000     0\n");
        debugStr(DebugStream::info4) << boost::format(  ">>>     5600    1.500000    1.000000     1\n");
        debugStr(DebugStream::info4) << boost::format(  ">>>     5601    1.500000    1.000000     1\n");
        debugStr(DebugStream::info4) << boost::format(  ">>>     5602    0.500000    0.000000     0\n");

        debugStr(DebugStream::info4) << boost::format(  ">>> Statistics should be:\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>      110    0.200000    8.000000\n");
        
        test_print_acc_results();
        test_print_stat_results();
        
        //
        //  Test 4
        //  Start a search sequence for time 30153600 to start at 30153145 and finish at 31017599
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should start a single correlation for time 30153600 to start at 30153145 and finish at 31017599.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should empty the FPGA peak data.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should restart the statistics engine.\n" );
        
        current_mode = SEARCH;
        fpga_get_4g_time_retval = 30000000;

        fpga_corr_seq_cmds.clear();

        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output

        //
        //  A single peak that does not complete the sequence (peaks were lost), no status, but time is after end of sequence
        //  plus one result interval so sequence completes and result is available.
        //
        fpga_get_4g_time_retval = 31027200;
        
        fpga_get_4g_peak_data_peaks.clear();
        Fpga::CorrPeak_4g_t corr_peak_D_1  = { 30157600, 1.0, 1.0, 1.0 };   //  In the middle of the wanted (offset 4000 in the result interval)
        fpga_get_4g_peak_data_peaks.push_back(corr_peak_D_1);
        fpga_get_4g_peak_data_retval = false;                               //  Peaks missed

        fpga_get_4g_correlation_command_status_cmd_status.clear();
        fpga_get_4g_correlation_command_status_retval = true;
        
        debugStr(DebugStream::info4) << boost::format(  ">>> A single peak that does not complete the sequence, no status, but time is after end of sequence plus one result interval so sequence completes and result is available and so should produce 1 (true).\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should read the statistics results without a restart.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Peaks were lost so warning should be produced.\n" );

        fpga_get_4g_statistics_results.count = 120;
        fpga_get_4g_statistics_results.peak_inp_sums = 0.3;
        fpga_get_4g_statistics_results.sum_inp_sums  = 7.0;
        
        debugStr(DebugStream::error) << boost::format(  "Result available = %u\n" ) % result_available();
        debugStr(DebugStream::info4) << boost::format(  ">>> Valid peak was at result interval offset 4000 so accumulation results should be:\n" );
        
        debugStr(DebugStream::info4) << boost::format(  ">>>     3999    0.500000    0.000000     0\n");
        debugStr(DebugStream::info4) << boost::format(  ">>>     4000    1.000000    1.000000     1\n");
        debugStr(DebugStream::info4) << boost::format(  ">>>     4001    0.500000    0.000000     0\n");

        debugStr(DebugStream::info4) << boost::format(  ">>> Statistics should be:\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>      120    0.300000    7.000000\n");
        
        test_print_acc_results();
        test_print_stat_results();

        //
        //  Test 5
        //  Start a track sequence of 45 correlations:
        //      0:  For time 40147400, to start at 40146945 and finish at 40147844
        //      1:                     to start at 40156545 and finish at 40157444
        //      ...
        //     44:                     to start at 40569345 and finish at 40570244
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should start 45 correlations:\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>     0:  For time 40147400, to start at 40146945 and finish at 40147844\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>     1:                     to start at 40156545 and finish at 40157444\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>     ....\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>    44:                     to start at 40569345 and finish at 40570244\n" );
        
        current_mode = TRACK;
        fpga_get_4g_time_retval = 40000000;
        
        fpga_corr_seq_cmds.clear();

        change_start(200);
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output

        //
        //  A series of peaks in separate results that does not complete the sequence, no status, so should produce 0 (false).
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> A series of peaks in separate results that does not complete the sequence, no status, so should produce 0 (false).\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should not read the statistics results.\n" );
        for (unsigned result_num = 0 ; result_num < HS_TRACK_MODE_RAW_RESULTS_TO_AVG ; ++result_num)
        {
            uint64_t start_of_corr_window = 40147400+result_num*fpga_corr_result_accumulator.size();
            
            fpga_get_4g_time_retval = start_of_corr_window+2000;
            
            fpga_get_4g_peak_data_peaks.clear();
            Fpga::CorrPeak_4g_t corr_peak_E_1  = { start_of_corr_window+10, 1.0, 1.0, 1.0 };
            fpga_get_4g_peak_data_peaks.push_back(corr_peak_E_1);
            fpga_get_4g_peak_data_retval = true;

            fpga_get_4g_correlation_command_status_cmd_status.clear();
            fpga_get_4g_correlation_command_status_retval = true;
        
            debugStr(DebugStream::error) << boost::format(  "Result available = %u\n" ) % result_available();
            test_print_stat_results();
        }
        
        //
        //  No peaks, no status, but time is after end of sequence plus one result interval so sequence completes and result is available.
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> No peaks, no status, but time is after end of sequence plus one result interval so sequence completes and result is available, so should produce 1 (true).\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should read the statistics results without a restart.\n" );
        
        fpga_get_4g_time_retval = 40580000;

        fpga_get_4g_peak_data_peaks.clear();
        fpga_get_4g_peak_data_retval = true;

        fpga_get_4g_correlation_command_status_cmd_status.clear();
        fpga_get_4g_correlation_command_status_retval = true;
            
        fpga_get_4g_statistics_results.count = 140;
        fpga_get_4g_statistics_results.peak_inp_sums = 0.4;
        fpga_get_4g_statistics_results.sum_inp_sums  = 6.0;

        debugStr(DebugStream::error) << boost::format(  "Result available = %u\n" ) % result_available();
        
        debugStr(DebugStream::info4) << boost::format(  ">>> Valid peak was at result interval offset 210 so accumulation results should be:\n" );
        
        debugStr(DebugStream::info4) << boost::format(  ">>>      209   22.500000     0.000000     0\n");
        debugStr(DebugStream::info4) << boost::format(  ">>>      210   45.000000    45.000000    45\n");
        debugStr(DebugStream::info4) << boost::format(  ">>>      211   22.500000     0.000000     0\n");

        debugStr(DebugStream::info4) << boost::format(  ">>> Statistics should be:\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>      140    0.400000    6.000000\n");
        
        test_print_acc_results();
        test_print_stat_results();
        
        //
        //  Test 6
        //  Start a search sequence for time 40147200 to start at 40146745 and finish at 41011199
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should start a single correlation for time 40147200 to start at 40146745 and finish at 41011199.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should empty the FPGA peak data.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should restart the statistics engine.\n" );
        
        current_mode = SEARCH;
        fpga_get_4g_time_retval = 40000000;
        
        fpga_corr_seq_cmds.clear();

        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output

        //
        //  No peaks, but a status indicating the sequence was rejected. This leaves the command in the SW's list so no
        //  result is available.
        //  Also, status lost flag set.
        //
        fpga_get_4g_time_retval = 40146800;
        
        fpga_get_4g_peak_data_peaks.clear();
        fpga_get_4g_peak_data_retval = true;

        fpga_get_4g_correlation_command_status_cmd_status.clear();
        Fpga::CorrCmdStatus_4g_t corr_cmd_status_F_1 = { Fpga::REJECTED, fpga_next_cmd_id-1 };
        fpga_get_4g_correlation_command_status_cmd_status.push_back(corr_cmd_status_F_1);
        fpga_get_4g_correlation_command_status_retval = false;  //  status lost
        
        debugStr(DebugStream::info4) << boost::format(  ">>> No peaks, but a status indicating the sequence was rejected, which leaves the command in the SW's list so no result is available and should produce 0 (false).\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Sequence was rejected so error should be produced: ID = %u\n" ) % static_cast<unsigned>(fpga_next_cmd_id-1);
        debugStr(DebugStream::info4) << boost::format(  ">>> Status also lost producing a second error\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should not read the statistics results.\n" );
        debugStr(DebugStream::error) << boost::format(  "Result available = %u\n" ) % result_available();
        
        test_print_acc_results();
        test_print_stat_results();

        //
        //  Test 7
        //  Start a search sequence for time 50150400 to start at 50149945 and finish at 51014399
        //
        debugStr(DebugStream::info4) << boost::format(  ">>> Should start a single correlation for time 50150400 to start at 50149945 and finish at 51014399.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should empty the FPGA peak data.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should restart the statistics engine.\n" );
        
        current_mode = SEARCH;
        fpga_get_4g_time_retval = 50000000;

        fpga_corr_seq_cmds.clear();

        restart_correlation();
        test_print_acc_results();   //  No output
        test_print_stat_results();  //  No output

        //
        //  No peaks, but two statuses, both indicating acceptance, but with the second having an ID after the last in the command sequence.
        //
        fpga_get_4g_time_retval = 51024000;
        
        fpga_get_4g_peak_data_peaks.clear();
        fpga_get_4g_peak_data_retval = true;

        fpga_get_4g_correlation_command_status_cmd_status.clear();
        Fpga::CorrCmdStatus_4g_t corr_cmd_status_G_1 = { Fpga::FETCHED, fpga_next_cmd_id-1 };
        Fpga::CorrCmdStatus_4g_t corr_cmd_status_G_2 = { Fpga::FETCHED, fpga_next_cmd_id };
        fpga_get_4g_correlation_command_status_cmd_status.push_back(corr_cmd_status_G_1);
        fpga_get_4g_correlation_command_status_cmd_status.push_back(corr_cmd_status_G_2);
        fpga_get_4g_correlation_command_status_retval = true;
        
        debugStr(DebugStream::info4) << boost::format(  ">>> No peaks, but two statuses, both indicating acceptance, but with the second having an ID after the last in the command sequence; time is after end of sequence plus margin so result is available and should produce 1 (true).\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Should read the statistics results without a restart.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Error should be produced: ID = %u\n" ) % static_cast<unsigned>(fpga_next_cmd_id);

        fpga_get_4g_statistics_results.count = 150;
        fpga_get_4g_statistics_results.peak_inp_sums = 0.5;
        fpga_get_4g_statistics_results.sum_inp_sums  = 5.0;
        
        debugStr(DebugStream::error) << boost::format(  "Result available = %u\n" ) % result_available();
        debugStr(DebugStream::info4) << boost::format(  ">>> No valid peaks expected.\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Statistics should be:\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>      150    0.500000    5.000000\n");
        
        test_print_acc_results();
        test_print_stat_results();
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Test the read_results() function
    /// @details        There is no automated checking of the results, which must be manually confirmed.
    ///
    void Correlator_4g::test_read_results(void)
    {
        debugStr(DebugStream::info4) << boost::format(  "test_read_results -----------------------------------------\n" );

        sum_corr_inp_acc_t acc_reset_entry = { 0.0, 0.0, 0 };
        sum_corr_inp_acc_t acc_entry = { 0.0, 0.0, 0 };

        //
        //  Test 1
        //  All zeros.
        //
        std::fill(fpga_corr_result_accumulator.begin(), fpga_corr_result_accumulator.end(), acc_reset_entry);
        
        corr_stat_res.count = 0;
        corr_stat_res.peak_inp_sums = 0.0;
        corr_stat_res.sum_inp_sums = 0.0;
        
        low_power = false;
        current_mode = SEARCH;
        
        search_mode_raw_results_to_avg = HS_SEARCH_MODE_RAW_RESULTS_TO_AVG;
        track_mode_raw_results_to_avg =  HS_TRACK_MODE_RAW_RESULTS_TO_AVG;
        
        search_avg_threshold = 0.1;
        track_avg_threshold = 0.1;
        cont_track_avg_threshold = 0.1;
        
        std::vector<unsigned> corr_peak_counts; 
        std::vector<float> corr_peak_vals; 
        std::vector<float> corr_peak_mean_inputs_adc_lsb_sq;
        float corr_inp_max_mean_adc_lsb_sq;
        float corr_inp_mean_adc_lsb_sq;
        debugStr(DebugStream::info4) << boost::format(  ">>> Should produce an error because the statistics count is zero.\n" );
        Correlator_4g::read_results(corr_peak_counts,
                                    corr_peak_vals, 
                                    corr_peak_mean_inputs_adc_lsb_sq, 
                                    corr_inp_max_mean_adc_lsb_sq,
                                    corr_inp_mean_adc_lsb_sq);

        debugStr(DebugStream::info4) << boost::format(  ">>> All results zero.\n" );
        test_print_read_results_returns(corr_peak_counts,
                                        corr_peak_vals, 
                                        corr_peak_mean_inputs_adc_lsb_sq, 
                                        corr_inp_max_mean_adc_lsb_sq,
                                        corr_inp_mean_adc_lsb_sq);
        
        //
        //  Test 2
        //  All zeros except for statistics count.
        //
        std::fill(fpga_corr_result_accumulator.begin(), fpga_corr_result_accumulator.end(), acc_reset_entry);
        
        corr_stat_res.count = 1;
        corr_stat_res.peak_inp_sums = 0.0;
        corr_stat_res.sum_inp_sums = 0.0;
        
        low_power = false;
        current_mode = SEARCH;
        
        search_mode_raw_results_to_avg = HS_SEARCH_MODE_RAW_RESULTS_TO_AVG;
        track_mode_raw_results_to_avg =  HS_TRACK_MODE_RAW_RESULTS_TO_AVG;
        
        search_avg_threshold = 0.1;
        track_avg_threshold = 0.1;
        cont_track_avg_threshold = 0.1;

        Correlator_4g::read_results(corr_peak_counts,
                                    corr_peak_vals, 
                                    corr_peak_mean_inputs_adc_lsb_sq, 
                                    corr_inp_max_mean_adc_lsb_sq,
                                    corr_inp_mean_adc_lsb_sq);

        debugStr(DebugStream::info4) << boost::format(  ">>> All results zero.\n" );
        test_print_read_results_returns(corr_peak_counts,
                                        corr_peak_vals, 
                                        corr_peak_mean_inputs_adc_lsb_sq, 
                                        corr_inp_max_mean_adc_lsb_sq,
                                        corr_inp_mean_adc_lsb_sq);
        
        //
        //  Test 3
        //  High sensitivity search with three peaks, a maximum, one above the threshold and one below.
        //
        std::fill(fpga_corr_result_accumulator.begin(), fpga_corr_result_accumulator.end(), acc_reset_entry);
        acc_entry.corr_out_peak_acc = static_cast<float>(HS_SEARCH_MODE_RAW_RESULTS_TO_AVG);
        acc_entry.corr_inp_sum_acc  = static_cast<float>(HS_SEARCH_MODE_RAW_RESULTS_TO_AVG*NUM_CORR_COEFFS);
        acc_entry.num_peaks         = HS_SEARCH_MODE_RAW_RESULTS_TO_AVG;
        fpga_corr_result_accumulator[0] = acc_entry;

        acc_entry.corr_out_peak_acc = 0.100001*static_cast<float>(HS_SEARCH_MODE_RAW_RESULTS_TO_AVG);
        acc_entry.corr_inp_sum_acc  = 0.050000*static_cast<float>(HS_SEARCH_MODE_RAW_RESULTS_TO_AVG*NUM_CORR_COEFFS);
        acc_entry.num_peaks         = HS_SEARCH_MODE_RAW_RESULTS_TO_AVG;
        fpga_corr_result_accumulator[fpga_corr_result_accumulator.size()/2] = acc_entry;

        acc_entry.corr_out_peak_acc = 0.099999*static_cast<float>(HS_SEARCH_MODE_RAW_RESULTS_TO_AVG);
        acc_entry.corr_inp_sum_acc  = 0.050000*static_cast<float>(HS_SEARCH_MODE_RAW_RESULTS_TO_AVG*NUM_CORR_COEFFS);
        acc_entry.num_peaks         = HS_SEARCH_MODE_RAW_RESULTS_TO_AVG;
        fpga_corr_result_accumulator[fpga_corr_result_accumulator.size()-1] = acc_entry;
        
        corr_stat_res.count = 10;
        corr_stat_res.peak_inp_sums = 1.0*static_cast<float>(NUM_CORR_COEFFS);
        corr_stat_res.sum_inp_sums = 10.0*static_cast<float>(NUM_CORR_COEFFS);
        
        low_power = false;
        current_mode = SEARCH;
        
        search_mode_raw_results_to_avg = HS_SEARCH_MODE_RAW_RESULTS_TO_AVG;
        track_mode_raw_results_to_avg =  HS_TRACK_MODE_RAW_RESULTS_TO_AVG;
        
        search_avg_threshold = 0.1;
        track_avg_threshold = 0.0;      //  Shouldn't be used so set to 0.0 to make all peaks pass if used.
        cont_track_avg_threshold = 0.0; //  Shouldn't be used so set to 0.0 to make all peaks pass if used.

        Correlator_4g::read_results(corr_peak_counts,
                                    corr_peak_vals, 
                                    corr_peak_mean_inputs_adc_lsb_sq, 
                                    corr_inp_max_mean_adc_lsb_sq,
                                    corr_inp_mean_adc_lsb_sq);

        debugStr(DebugStream::info4) << boost::format(  ">>> HS Search with three peaks, a maximum, one above the threshold and one below, so:\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>         0       90    1.000000    1.000000\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>      4800       90    0.100001    0.050000\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>      9599       90    0.000000    0.000000\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Statistics:  1.000000    1.000000\n" );
        test_print_read_results_returns(corr_peak_counts,
                                        corr_peak_vals, 
                                        corr_peak_mean_inputs_adc_lsb_sq, 
                                        corr_inp_max_mean_adc_lsb_sq,
                                        corr_inp_mean_adc_lsb_sq);
        
        //
        //  Test 4
        //  Low power search with three peaks, a maximum, one above the threshold and one below.
        //
        std::fill(fpga_corr_result_accumulator.begin(), fpga_corr_result_accumulator.end(), acc_reset_entry);
        acc_entry.corr_out_peak_acc = static_cast<float>(LP_SEARCH_MODE_RAW_RESULTS_TO_AVG);
        acc_entry.corr_inp_sum_acc  = static_cast<float>(LP_SEARCH_MODE_RAW_RESULTS_TO_AVG*NUM_CORR_COEFFS);
        acc_entry.num_peaks         = LP_SEARCH_MODE_RAW_RESULTS_TO_AVG;
        fpga_corr_result_accumulator[0] = acc_entry;

        acc_entry.corr_out_peak_acc = 0.100001*static_cast<float>(LP_SEARCH_MODE_RAW_RESULTS_TO_AVG);
        acc_entry.corr_inp_sum_acc  = 0.050000*static_cast<float>(LP_SEARCH_MODE_RAW_RESULTS_TO_AVG*NUM_CORR_COEFFS);
        acc_entry.num_peaks         = LP_SEARCH_MODE_RAW_RESULTS_TO_AVG;
        fpga_corr_result_accumulator[fpga_corr_result_accumulator.size()/2] = acc_entry;

        acc_entry.corr_out_peak_acc = 0.099999*static_cast<float>(LP_SEARCH_MODE_RAW_RESULTS_TO_AVG);
        acc_entry.corr_inp_sum_acc  = 0.050000*static_cast<float>(LP_SEARCH_MODE_RAW_RESULTS_TO_AVG*NUM_CORR_COEFFS);
        acc_entry.num_peaks         = LP_SEARCH_MODE_RAW_RESULTS_TO_AVG;
        fpga_corr_result_accumulator[fpga_corr_result_accumulator.size()-1] = acc_entry;
        
        corr_stat_res.count = 10;
        corr_stat_res.peak_inp_sums = 1.0*static_cast<float>(NUM_CORR_COEFFS);
        corr_stat_res.sum_inp_sums = 10.0*static_cast<float>(NUM_CORR_COEFFS);
        
        low_power = true;
        current_mode = SEARCH;
        
        search_mode_raw_results_to_avg = LP_SEARCH_MODE_RAW_RESULTS_TO_AVG;
        track_mode_raw_results_to_avg =  LP_TRACK_MODE_RAW_RESULTS_TO_AVG;
        
        search_avg_threshold = 0.1;
        track_avg_threshold = 0.0;      //  Shouldn't be used so set to 0.0 to make all peaks pass if used.
        cont_track_avg_threshold = 0.0; //  Shouldn't be used so set to 0.0 to make all peaks pass if used.

        Correlator_4g::read_results(corr_peak_counts,
                                    corr_peak_vals, 
                                    corr_peak_mean_inputs_adc_lsb_sq, 
                                    corr_inp_max_mean_adc_lsb_sq,
                                    corr_inp_mean_adc_lsb_sq);

        debugStr(DebugStream::info4) << boost::format(  ">>> LP Search with three peaks, a maximum, one above the threshold and one below, so:\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>         0        1    1.000000    1.000000\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>      4800        1    0.100001    0.050000\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>      9599        1    0.000000    0.000000\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Statistics:  1.000000    1.000000\n" );
        test_print_read_results_returns(corr_peak_counts,
                                        corr_peak_vals, 
                                        corr_peak_mean_inputs_adc_lsb_sq, 
                                        corr_inp_max_mean_adc_lsb_sq,
                                        corr_inp_mean_adc_lsb_sq);
        
        //
        //  Test 5
        //  High sensitivity track with three peaks, a maximum, one above the threshold and one below.
        //
        std::fill(fpga_corr_result_accumulator.begin(), fpga_corr_result_accumulator.end(), acc_reset_entry);
        acc_entry.corr_out_peak_acc = static_cast<float>(HS_TRACK_MODE_RAW_RESULTS_TO_AVG);
        acc_entry.corr_inp_sum_acc  = static_cast<float>(HS_TRACK_MODE_RAW_RESULTS_TO_AVG*NUM_CORR_COEFFS);
        acc_entry.num_peaks         = HS_TRACK_MODE_RAW_RESULTS_TO_AVG;
        fpga_corr_result_accumulator[0] = acc_entry;

        acc_entry.corr_out_peak_acc = 0.200001*static_cast<float>(HS_TRACK_MODE_RAW_RESULTS_TO_AVG);
        acc_entry.corr_inp_sum_acc  = 0.100000*static_cast<float>(HS_TRACK_MODE_RAW_RESULTS_TO_AVG*NUM_CORR_COEFFS);
        acc_entry.num_peaks         = HS_TRACK_MODE_RAW_RESULTS_TO_AVG;
        fpga_corr_result_accumulator[fpga_corr_result_accumulator.size()/2] = acc_entry;

        acc_entry.corr_out_peak_acc = 0.199999*static_cast<float>(HS_TRACK_MODE_RAW_RESULTS_TO_AVG);
        acc_entry.corr_inp_sum_acc  = 0.100000*static_cast<float>(HS_TRACK_MODE_RAW_RESULTS_TO_AVG*NUM_CORR_COEFFS);
        acc_entry.num_peaks         = HS_TRACK_MODE_RAW_RESULTS_TO_AVG;
        fpga_corr_result_accumulator[fpga_corr_result_accumulator.size()-1] = acc_entry;
        
        corr_stat_res.count = 10;
        corr_stat_res.peak_inp_sums = 1.0*static_cast<float>(NUM_CORR_COEFFS);
        corr_stat_res.sum_inp_sums = 10.0*static_cast<float>(NUM_CORR_COEFFS);
        
        low_power = false;
        current_mode = TRACK;
        
        search_mode_raw_results_to_avg = HS_SEARCH_MODE_RAW_RESULTS_TO_AVG;
        track_mode_raw_results_to_avg =  HS_TRACK_MODE_RAW_RESULTS_TO_AVG;
        
        search_avg_threshold = 0.0;     //  Shouldn't be used so set to 0.0 to make all peaks pass if used.
        track_avg_threshold = 0.2; 
        cont_track_avg_threshold = 0.0; //  Shouldn't be used so set to 0.0 to make all peaks pass if used.

        Correlator_4g::read_results(corr_peak_counts,
                                    corr_peak_vals, 
                                    corr_peak_mean_inputs_adc_lsb_sq, 
                                    corr_inp_max_mean_adc_lsb_sq,
                                    corr_inp_mean_adc_lsb_sq);

        debugStr(DebugStream::info4) << boost::format(  ">>> HS Track with three peaks, a maximum, one above the threshold and one below, so:\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>         0       45    1.000000    1.000000\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>      4800       45    0.200001    0.100000\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>      9599       45    0.000000    0.000000\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Statistics:  1.000000    1.000000\n" );
        test_print_read_results_returns(corr_peak_counts,
                                        corr_peak_vals, 
                                        corr_peak_mean_inputs_adc_lsb_sq, 
                                        corr_inp_max_mean_adc_lsb_sq,
                                        corr_inp_mean_adc_lsb_sq);
        
        //
        //  Test 6
        //  Low power track with three peaks, a maximum, one above the threshold and one below.
        //
        std::fill(fpga_corr_result_accumulator.begin(), fpga_corr_result_accumulator.end(), acc_reset_entry);
        acc_entry.corr_out_peak_acc = static_cast<float>(LP_TRACK_MODE_RAW_RESULTS_TO_AVG);
        acc_entry.corr_inp_sum_acc  = static_cast<float>(LP_TRACK_MODE_RAW_RESULTS_TO_AVG*NUM_CORR_COEFFS);
        acc_entry.num_peaks         = LP_TRACK_MODE_RAW_RESULTS_TO_AVG;
        fpga_corr_result_accumulator[0] = acc_entry;

        acc_entry.corr_out_peak_acc = 0.200001*static_cast<float>(LP_TRACK_MODE_RAW_RESULTS_TO_AVG);
        acc_entry.corr_inp_sum_acc  = 0.100000*static_cast<float>(LP_TRACK_MODE_RAW_RESULTS_TO_AVG*NUM_CORR_COEFFS);
        acc_entry.num_peaks         = LP_TRACK_MODE_RAW_RESULTS_TO_AVG;
        fpga_corr_result_accumulator[fpga_corr_result_accumulator.size()/2] = acc_entry;

        acc_entry.corr_out_peak_acc = 0.199999*static_cast<float>(LP_TRACK_MODE_RAW_RESULTS_TO_AVG);
        acc_entry.corr_inp_sum_acc  = 0.100000*static_cast<float>(LP_TRACK_MODE_RAW_RESULTS_TO_AVG*NUM_CORR_COEFFS);
        acc_entry.num_peaks         = LP_TRACK_MODE_RAW_RESULTS_TO_AVG;
        fpga_corr_result_accumulator[fpga_corr_result_accumulator.size()-1] = acc_entry;
        
        corr_stat_res.count = 10;
        corr_stat_res.peak_inp_sums = 1.0*static_cast<float>(NUM_CORR_COEFFS);
        corr_stat_res.sum_inp_sums = 10.0*static_cast<float>(NUM_CORR_COEFFS);
        
        low_power = true;
        current_mode = TRACK;
        
        search_mode_raw_results_to_avg = LP_SEARCH_MODE_RAW_RESULTS_TO_AVG;
        track_mode_raw_results_to_avg =  LP_TRACK_MODE_RAW_RESULTS_TO_AVG;
        
        search_avg_threshold = 0.0;     //  Shouldn't be used so set to 0.0 to make all peaks pass if used.
        track_avg_threshold = 0.2; 
        cont_track_avg_threshold = 0.0; //  Shouldn't be used so set to 0.0 to make all peaks pass if used.

        Correlator_4g::read_results(corr_peak_counts,
                                    corr_peak_vals, 
                                    corr_peak_mean_inputs_adc_lsb_sq, 
                                    corr_inp_max_mean_adc_lsb_sq,
                                    corr_inp_mean_adc_lsb_sq);

        debugStr(DebugStream::info4) << boost::format(  ">>> LP Track with three peaks, a maximum, one above the threshold and one below, so:\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>         0        1    1.000000    1.000000\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>      4800        1    0.200001    0.100000\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>      9599        1    0.000000    0.000000\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Statistics:  1.000000    1.000000\n" );
        test_print_read_results_returns(corr_peak_counts,
                                        corr_peak_vals, 
                                        corr_peak_mean_inputs_adc_lsb_sq, 
                                        corr_inp_max_mean_adc_lsb_sq,
                                        corr_inp_mean_adc_lsb_sq);
          
        //
        //  Test 7
        //  High sensitivity continuous track with three peaks, a maximum, one above the threshold and one below.
        //
        std::fill(fpga_corr_result_accumulator.begin(), fpga_corr_result_accumulator.end(), acc_reset_entry);
        acc_entry.corr_out_peak_acc = static_cast<float>(HS_TRACK_MODE_RAW_RESULTS_TO_AVG);
        acc_entry.corr_inp_sum_acc  = static_cast<float>(HS_TRACK_MODE_RAW_RESULTS_TO_AVG*NUM_CORR_COEFFS);
        acc_entry.num_peaks         = HS_TRACK_MODE_RAW_RESULTS_TO_AVG;
        fpga_corr_result_accumulator[0] = acc_entry;

        acc_entry.corr_out_peak_acc = 0.200001*static_cast<float>(HS_TRACK_MODE_RAW_RESULTS_TO_AVG);
        acc_entry.corr_inp_sum_acc  = 0.100000*static_cast<float>(HS_TRACK_MODE_RAW_RESULTS_TO_AVG*NUM_CORR_COEFFS);
        acc_entry.num_peaks         = HS_TRACK_MODE_RAW_RESULTS_TO_AVG;
        fpga_corr_result_accumulator[fpga_corr_result_accumulator.size()/2] = acc_entry;

        acc_entry.corr_out_peak_acc = 0.199999*static_cast<float>(HS_TRACK_MODE_RAW_RESULTS_TO_AVG);
        acc_entry.corr_inp_sum_acc  = 0.100000*static_cast<float>(HS_TRACK_MODE_RAW_RESULTS_TO_AVG*NUM_CORR_COEFFS);
        acc_entry.num_peaks         = HS_TRACK_MODE_RAW_RESULTS_TO_AVG;
        fpga_corr_result_accumulator[fpga_corr_result_accumulator.size()-1] = acc_entry;
        
        corr_stat_res.count = 10;
        corr_stat_res.peak_inp_sums = 1.0*static_cast<float>(NUM_CORR_COEFFS);
        corr_stat_res.sum_inp_sums = 10.0*static_cast<float>(NUM_CORR_COEFFS);
        
        low_power = false;
        current_mode = CONT_TRACK;
        
        search_mode_raw_results_to_avg = HS_SEARCH_MODE_RAW_RESULTS_TO_AVG;
        track_mode_raw_results_to_avg =  HS_TRACK_MODE_RAW_RESULTS_TO_AVG;
        
        search_avg_threshold = 0.0;     //  Shouldn't be used so set to 0.0 to make all peaks pass if used.
        track_avg_threshold = 0.0;      //  Shouldn't be used so set to 0.0 to make all peaks pass if used. 
        cont_track_avg_threshold = 0.2; 

        Correlator_4g::read_results(corr_peak_counts,
                                    corr_peak_vals, 
                                    corr_peak_mean_inputs_adc_lsb_sq, 
                                    corr_inp_max_mean_adc_lsb_sq,
                                    corr_inp_mean_adc_lsb_sq);

        debugStr(DebugStream::info4) << boost::format(  ">>> HS Continuous Track with three peaks, a maximum, one above the threshold and one below, so:\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>         0       45    1.000000    1.000000\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>      4800       45    0.200001    0.100000\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>      9599       45    0.000000    0.000000\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Statistics:  1.000000    1.000000\n" );
        test_print_read_results_returns(corr_peak_counts,
                                        corr_peak_vals, 
                                        corr_peak_mean_inputs_adc_lsb_sq, 
                                        corr_inp_max_mean_adc_lsb_sq,
                                        corr_inp_mean_adc_lsb_sq);
        
        //
        //  Test 8
        //  Low power continuous track with three peaks, a maximum, one above the threshold and one below.
        //
        std::fill(fpga_corr_result_accumulator.begin(), fpga_corr_result_accumulator.end(), acc_reset_entry);
        acc_entry.corr_out_peak_acc = static_cast<float>(LP_TRACK_MODE_RAW_RESULTS_TO_AVG);
        acc_entry.corr_inp_sum_acc  = static_cast<float>(LP_TRACK_MODE_RAW_RESULTS_TO_AVG*NUM_CORR_COEFFS);
        acc_entry.num_peaks         = LP_TRACK_MODE_RAW_RESULTS_TO_AVG;
        fpga_corr_result_accumulator[0] = acc_entry;

        acc_entry.corr_out_peak_acc = 0.200001*static_cast<float>(LP_TRACK_MODE_RAW_RESULTS_TO_AVG);
        acc_entry.corr_inp_sum_acc  = 0.100000*static_cast<float>(LP_TRACK_MODE_RAW_RESULTS_TO_AVG*NUM_CORR_COEFFS);
        acc_entry.num_peaks         = LP_TRACK_MODE_RAW_RESULTS_TO_AVG;
        fpga_corr_result_accumulator[fpga_corr_result_accumulator.size()/2] = acc_entry;

        acc_entry.corr_out_peak_acc = 0.199999*static_cast<float>(LP_TRACK_MODE_RAW_RESULTS_TO_AVG);
        acc_entry.corr_inp_sum_acc  = 0.100000*static_cast<float>(LP_TRACK_MODE_RAW_RESULTS_TO_AVG*NUM_CORR_COEFFS);
        acc_entry.num_peaks         = LP_TRACK_MODE_RAW_RESULTS_TO_AVG;
        fpga_corr_result_accumulator[fpga_corr_result_accumulator.size()-1] = acc_entry;
        
        corr_stat_res.count = 10;
        corr_stat_res.peak_inp_sums = 1.0*static_cast<float>(NUM_CORR_COEFFS);
        corr_stat_res.sum_inp_sums = 10.0*static_cast<float>(NUM_CORR_COEFFS);
        
        low_power = true;
        current_mode = CONT_TRACK;
        
        search_mode_raw_results_to_avg = LP_SEARCH_MODE_RAW_RESULTS_TO_AVG;
        track_mode_raw_results_to_avg =  LP_TRACK_MODE_RAW_RESULTS_TO_AVG;
        
        search_avg_threshold = 0.0;     //  Shouldn't be used so set to 0.0 to make all peaks pass if used.
        track_avg_threshold = 0.0;      //  Shouldn't be used so set to 0.0 to make all peaks pass if used. 
        cont_track_avg_threshold = 0.2; 

        Correlator_4g::read_results(corr_peak_counts,
                                    corr_peak_vals, 
                                    corr_peak_mean_inputs_adc_lsb_sq, 
                                    corr_inp_max_mean_adc_lsb_sq,
                                    corr_inp_mean_adc_lsb_sq);

        debugStr(DebugStream::info4) << boost::format(  ">>> LP Continuous Track with three peaks, a maximum, one above the threshold and one below, so:\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>         0        1    1.000000    1.000000\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>      4800        1    0.200001    0.100000\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>>      9599        1    0.000000    0.000000\n" );
        debugStr(DebugStream::info4) << boost::format(  ">>> Statistics:  1.000000    1.000000\n" );
        test_print_read_results_returns(corr_peak_counts,
                                        corr_peak_vals, 
                                        corr_peak_mean_inputs_adc_lsb_sq, 
                                        corr_inp_max_mean_adc_lsb_sq,
                                        corr_inp_mean_adc_lsb_sq);
    }
#endif //   SWTEST_4G_CORR

#ifdef SWTEST_4G_FPGA_TIMING
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Test the FPGA timing
    //  @details        Tests the FPGA timing in a tight loop that reads the timing value twice and then sleeps for
    //                  a period of time. The results are stored in arrays and then printed as debug strings after
    //                  the loop is completed. The analysis of the results is manual.
    //
    //                  Caveat the default number of loop iterations and sleep time means that the test takes 34 minutes
    //                  to complete as this ensures that the upper 32 bits of the FPGA counter is also tested (at least
    //                  its bit 0).
    //
    void Correlator_4g::test_fpga_timing(void)
    {
        size_t num_iter = 20000;
        unsigned sleep_time_us = 100000;
        uint64_t times1[num_iter];
        uint64_t times2[num_iter];


        struct timeval tic;
        gettimeofday(&tic, NULL);
        for (size_t n = 0 ; n < num_iter ; ++n)
        {
            times1[n] = fpga_get_4g_time();
            times2[n] = fpga_get_4g_time();
            usleep(sleep_time_us);
        }
        struct timeval toc;
        gettimeofday(&toc, NULL);

        double total_time_ms  =   double(toc.tv_sec -tic.tv_sec )*1e3
                                + double(toc.tv_usec-tic.tv_usec)/1e3;

        debugStr(DebugStream::info1) <<   boost::format(  "Loop time = %.1f ms\n" )
                                        % (total_time_ms/double(num_iter)) ;

        for (size_t n = 0 ; n < num_iter ; ++n)
        {
            debugStr(DebugStream::error) <<   boost::format(  "%5u    %20lu    %016lx    %20.9f    %20lu    %016lx    %20.9f\n" )
                                            % n
                                            % times1[n]
                                            % times1[n]
                                            % (  static_cast<double>(times1[n])
                                               / static_cast<double>(Design::SAMPLES_PER_SEC_4G))
                                            % times2[n]
                                            % times2[n]
                                            % (  static_cast<double>(times2[n])
                                               / static_cast<double>(Design::SAMPLES_PER_SEC_4G));
        }
    }
#endif  //  SWTEST_4G_FPGA_TIMING



#ifdef SWTEST_4G_FPGA
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief         Start correlating continuously
    //
    void Correlator_4g::start_cont_correlation( void )
    {
        //
        //  Clear the accumulator and statistics.
        //
        sum_corr_inp_acc_t acc_reset_entry = { 0.0, 0.0, 0 };
        std::fill(fpga_corr_result_accumulator.begin(), fpga_corr_result_accumulator.end(), acc_reset_entry);
        
        corr_stat_res.count = 0;
        corr_stat_res.peak_inp_sums = 0.0;
        corr_stat_res.sum_inp_sums = 0.0;
        
        //
        //  Check that the last correlation sequence should have been completed.
        //
        uint64_t current_time_samples = fpga_get_4g_time();
        if (!fpga_corr_seq_cmds.empty())
        {
            debugStr(DebugStream::error) << "Request for new correlation before last one is complete: "
                                        << "current time is "               << current_time_samples << ", "
                                        << "last correlation completes at " << fpga_corr_seq_cmds.back().stop << "\n";
            return;
        }
        
        //
        //  Set the threshold to be used, noting that the FPGA uses the square of the value.
        //
        double raw_threshold = search_raw_threshold;
        if (current_mode == TRACK)
        {
            raw_threshold = track_raw_threshold;
        }
        fpga_set_4g_threshold_sqrd(raw_threshold*raw_threshold);
        
        //
        //  Calculate the start time for the first FPGA correlation in the sequence.
        //
        uint64_t current_time_in_result_buffers = current_time_samples/fpga_corr_result_accumulator.size();
        uint64_t start_of_next_result_buffer =   (current_time_in_result_buffers+1+CORR_START_TIME_OFFSET_RESULT_BUFFERS)
                                               * fpga_corr_result_accumulator.size();
        
        //
        //  The FPGA start time must allow for the time required to flush the FPGA's processing chain.
        //
        Fpga::CorrCmd_4g_t cmd =
        {
            Fpga::CORRELATE_CONTINUOUSLY,
            fpga_next_cmd_id++,
            start_of_next_result_buffer-num_samples_to_flush_fpga_sig_proc_chain,
            0
        };
        fpga_corr_seq_cmds.push_back(cmd);
        
        debugStr(DebugStream::info1) << "CONTINUOUS: "
                                    << "current time = "    << current_time_samples << ", "
                                    << "low power mode = "  << low_power << ", "
                                    << "raw threshold = "   << raw_threshold << ", "
                                    << "FPGA cmd ID = "     << static_cast<unsigned>(cmd.id) << ", "
                                    << "FPGA start time = " << cmd.start << ", "
                                    << "FPGA stop time = "  << cmd.stop << "\n";
        
        //  Restart the statistics, discarding old results.
        bool restart_corr_stats = true;
        float peak_inp_sums;
        float sum_inp_sums;
        (void)fpga_get_4g_statistics(restart_corr_stats, peak_inp_sums, sum_inp_sums);    

        //  Start the correlation.
        fpga_setup_4g_correlation(fpga_corr_seq_cmds);
    }
#endif // SWTEST_4G_FPGA

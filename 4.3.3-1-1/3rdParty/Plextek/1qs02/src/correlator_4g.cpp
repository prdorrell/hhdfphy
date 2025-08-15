/***********************************************************************************************************************
 *  Copyright (c) 2021 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  Filename:   correlator_4g.cpp
 *  Author(s):  pdm
 *******************************************************************************************************************//**
 * File Description
 * ================
 *
 *  @file
 *  @brief          The implementation of ::Correlator_4g class that controls the 4G correlator in the FPGA.
 *  @details        Wrap up the hardware 4G correlator in a class that extends the processing with the averaging of the
 *                  peaks.
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
#include <time.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class.
//  @details        Read the channel filter and correlator coefficients and pass them to the Fpga class.
//                  Calculate the number of samples required to flush the FPGA processing chain.
//                  Calculate the correlation thresholds.
//                  Determine the power correction factor from the channel bandwidth in the file names.
//                  Get the FPGA interface to setup the correlation for search mode.
//  @param[in]      fpga                The FPGA interface.
//  @param[in]      chan_filt_1_filename
//                                      The name of the file containing the channel filter 1 coefficients.
//  @param[in]      chan_filt_2_filename
//                                      The name of the file containing the channel filter 2 coefficients.
//  @param[in]      corr_filename       The name of the file containing the correlator coefficients.
//  @param[in]      client_search_averages
//                                      The client-defined number of raw correlation results to average in search
//                                      mode (0 forces the use of the defaults that depend on the low_power
//                                      setting).
//  @param[in]      client_track_averages
//                                      The client-defined number of raw correlation results to average in track
//                                      mode (0 forces the use of the defaults that depend on the low_power
//                                      setting).
//  @param[in]      search_threshold_factor_dB
//                                      The client-defined factor applied to the averaging threshold in search mode.
//  @param[in]      track_threshold_factor_dB
//                                      The client-defined factor applied to the averaging threshold in track mode.
//  @param[in]      low_power           True if the low-power settings should be used, false if high-sensitivity is
//                                      required (partially or wholly over-ridden by num_search_averages or
//                                      num_track_averages being greater than zero).
//  @param[in]      debug               Controls the debug messages that may be generated.
//  @param[in]      restart             true means restart_correlator at end of constructor.  Optional parameter, default true
//  @throw          std::invalid_argument
//                                      If the coefficient file parsing produced an error.
//
Correlator_4g::Correlator_4g(Fpga        &fpga,
                             std::string chan_filt_1_filename,
                             std::string chan_filt_2_filename,
                             std::string corr_filename,
                             unsigned    client_search_averages,
                             unsigned    client_track_averages,
                             double      search_threshold_factor_dB,
                             double      track_threshold_factor_dB,
                             bool        low_power,
                             unsigned    debug,
                             bool        restart) :
    fpga(fpga),
    chan_filt_1_coeffs(),
    chan_filt_2_coeffs(),
    corr_coeffs(),
    tuning_offset_Hz(0),
    cal_factor_ratio(1.0),
    fpga_corr_result_accumulator(Design::SAMPLES_PER_4G_SRS_PERIOD),
    corr_stat_res(),
    num_samples_to_flush_fpga_sig_proc_chain(0),
    fpga_next_cmd_id(0),
    low_power(low_power),
    current_mode(SEARCH),
    track_start_offset_samples(0),
    fpga_corr_seq_cmds(),
    search_mode_raw_results_to_avg(HS_SEARCH_MODE_RAW_RESULTS_TO_AVG),
    track_mode_raw_results_to_avg(HS_TRACK_MODE_RAW_RESULTS_TO_AVG),
    search_raw_threshold(1.0),
    track_raw_threshold(1.0),
    cont_track_raw_threshold(1.0),
    search_avg_threshold(1.0),
    track_avg_threshold(1.0),
    cont_track_avg_threshold(1.0),
    total_num_peaks_retrieved(0),
#ifndef SWTEST_4G_CORR
    debugStr("Correlator_4g.. ", debug)
#else
    debugStr("Correlator_4g.. ", debug),
    fpga_get_4g_time_retval(0),
    fpga_get_4g_statistics_results(),
    fpga_get_4g_peak_data_peaks(),
    fpga_get_4g_peak_data_retval(true),
    fpga_get_4g_correlation_command_status_cmd_status(),
    fpga_get_4g_correlation_command_status_retval(true)
#endif
{
    double chan_filt_1_chan_bw_MHz;
    bool chan_filt_1_coeffs_ok = read_chan_filt_1_coeffs(chan_filt_1_filename,
                                                         chan_filt_1_chan_bw_MHz,
                                                         tuning_offset_Hz,
                                                         chan_filt_1_coeffs);

    double chan_filt_2_chan_bw_MHz;
    bool chan_filt_2_coeffs_ok = read_chan_filt_2_coeffs(chan_filt_2_filename,
                                                         chan_filt_2_chan_bw_MHz,
                                                         chan_filt_2_coeffs);

    double corr_chan_bw_MHz;
    bool corr_coeffs_ok = read_corr_coeffs(corr_filename, corr_chan_bw_MHz, corr_coeffs);

    bool any_coeff_error = !(chan_filt_1_coeffs_ok && chan_filt_2_coeffs_ok && corr_coeffs_ok);
    if (!any_coeff_error)
    {
        if ((chan_filt_1_chan_bw_MHz != chan_filt_2_chan_bw_MHz) || (chan_filt_1_chan_bw_MHz != corr_chan_bw_MHz))
        {
            any_coeff_error = true;
            debugStr(DebugStream::error) << boost::format(  "Channel bandwidths not the same (%e, %e, %e)\n" )
                                                          % chan_filt_1_chan_bw_MHz
                                                          % chan_filt_2_chan_bw_MHz
                                                          % corr_chan_bw_MHz;
        }
    }
    
    if (any_coeff_error)
    {
        throw std::invalid_argument("Coefficient file error");
    }
    
    
    // send the coefficients to the FPGA engines. The filter coefficients need to be reversed because the file format
    // puts the central tap first and the FPGA interface requires it last. They may also need to have zeros inserted
    // because the FPGA interface always requires Fpga::NUM_CHANNEL_FILTER_TAP_VALS_4G values. (The FPGA interface
    // requires a fixed-length symmetric filter with an odd number of taps and only the values up to the central tap are
    // programmed.)
    std::reverse(chan_filt_1_coeffs.begin(), chan_filt_1_coeffs.end());
    chan_filt_1_coeffs.insert(chan_filt_1_coeffs.begin(), 
                              Fpga::NUM_CHANNEL_FILTER_TAP_VALS_4G-chan_filt_1_coeffs.size(), 
                              0);
    std::reverse(chan_filt_2_coeffs.begin(), chan_filt_2_coeffs.end());
    chan_filt_2_coeffs.insert(chan_filt_2_coeffs.begin(), 
                              Fpga::NUM_CHANNEL_FILTER_TAP_VALS_4G-chan_filt_2_coeffs.size(), 
                              0);
    
    if (!fpga_send_4g_coefs(chan_filt_1_coeffs, chan_filt_2_coeffs, corr_coeffs))
    {
        throw std::invalid_argument("Failed to write coefficients");
    }
    
    // see 1QS003 for the justification of this calculation, noting that the case of filter coefficients the vectors
    // hold (N+1)/2 values where N is the filter length because the interface takes advantage of their symmetry.
    unsigned num_chan_filt_1_coeffs = 2*chan_filt_1_coeffs.size()-1;
    unsigned num_chan_filt_2_coeffs = 2*chan_filt_2_coeffs.size()-1;
    unsigned num_corr_coeffs = corr_coeffs.size();
    num_samples_to_flush_fpga_sig_proc_chain =   FPGA_CORR_STARTUP_DELAY_SAMPLES
                                               + num_chan_filt_1_coeffs
                                               + num_chan_filt_2_coeffs
                                               + num_corr_coeffs
                                               - 2;
    
    // calculate the the number of raw results used for averaging and then the raw and average thresholds for the search
    // and track modes.
    calc_averages(client_search_averages, client_track_averages);
    calc_thresholds(search_threshold_factor_dB, track_threshold_factor_dB);

    // determine the factor to apply when calibrating power results.
    if (5 == chan_filt_1_chan_bw_MHz)
    {
        cal_factor_ratio = Design::CAL_FACTOR_RATIO_4G_5MHZ_CHAN_BW;
    }
    else
    if (10 == chan_filt_1_chan_bw_MHz)
    {
        cal_factor_ratio = Design::CAL_FACTOR_RATIO_4G_10MHZ_CHAN_BW;
    }
    else
    {
        throw std::invalid_argument("Unsupported channel bandwidth");
    }

#ifdef SWTEST_4G_FPGA_TIMING
    test_fpga_timing();
#endif

    if (restart)    // default of optional parameter is true
    {
        //  Read and discard any results left over from an earlier sequence (an abort can leave the FPGA running).
        Fpga::CorrPeak_4g_t fpga_peaks_to_discard[MAX_PEAKS_FROM_FPGA];
        uint32_t num_peaks_retrieved = 0;
        (void)fpga_get_4g_peak_data(MAX_PEAKS_FROM_FPGA, fpga_peaks_to_discard, num_peaks_retrieved); 
        debugStr(DebugStream::info1) << "Peaks discarded in constructor: " << num_peaks_retrieved << "\n";
    
        // start the correlation
        restart_correlation();
    }    

    // confirm the conditional compilation effects.
    debugStr(DebugStream::info4) << boost::format(  "Tracking window size = %u\n" )
                                                  % Design::TRACK_WINDOW_SIZE_4G_SAMPLES;
#ifdef DISCARD_LOW_PWR_PEAKS_4G        
    debugStr(DebugStream::info4) << boost::format(  "Discarding raw peaks with burst-power peaks less than 1/10 of maximum burst power\n" );
#else
    debugStr(DebugStream::info4) << boost::format(  "Retaining raw peaks with burst-power peaks less than 1/10 of maximum burst power\n" );
#endif

#ifdef DISCARD_HIGH_CORR_PEAKS_4G
    debugStr(DebugStream::info4) << boost::format(  "Discarding raw peaks correlation > 1.1\n" );
#else
    debugStr(DebugStream::info4) << boost::format(  "Retaining raw peaks correlation > 1.1\n" );
#endif
    
    // run tests
 #ifdef SWTEST_4G_CORR
    test_restart_correlation();
    test_result_available();
    test_read_results();
#endif   
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          The destructor has nothing to do.
//
Correlator_4g::~Correlator_4g()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Retrieves the tuning offset to be used.
//  @return                             The tuning offset in Hz.
// 
freq_offset_t Correlator_4g::get_tuning_offset(void)
{
    return tuning_offset_Hz;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Retrieves the ratio of the 4G calibration factors to the 3G ones.
//  @return                             The ratio to be applied to the 3G calibration factors when correcting the 
//                                      4G results returned by read_results().
//
double Correlator_4g::get_cal_factor_ratio(void)
{
    return (cal_factor_ratio);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Enter low power mode.
//
void Correlator_4g::enter_low_power_mode(void)
{
    fpga.enter_low_power_mode();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Exit low power mode.
//
void Correlator_4g::exit_low_power_mode(void)
{
    fpga.exit_low_power_mode();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Restart the current correlation with the same settings.
//
void Correlator_4g::restart_correlation( void )
{
    //
    //  Select the averaging according to the mode.
    //
    unsigned raw_result_averages = get_num_raw_results_for_averaging();
    
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
    double avg_threshold = search_avg_threshold;
    if (current_mode == TRACK)
    {
        raw_threshold = track_raw_threshold;
        avg_threshold = track_avg_threshold;
    }
    else
    if (current_mode == CONT_TRACK)
    {
        raw_threshold = cont_track_raw_threshold;
        avg_threshold = cont_track_avg_threshold;
    }
    fpga_set_4g_threshold_sqrd(raw_threshold*raw_threshold);
    
    //
    //  Calculate the start time for the first FPGA correlation in the sequence.
    //
    uint64_t current_time_in_result_buffers = current_time_samples/fpga_corr_result_accumulator.size();
    uint64_t start_of_next_result_buffer =   (current_time_in_result_buffers+1+CORR_START_TIME_OFFSET_RESULT_BUFFERS)
                                           * fpga_corr_result_accumulator.size();
    
    if (current_mode == SEARCH)
    {
        //
        //  The FPGA start time must allow for the time required to flush the FPGA's processing chain. The stop time is
        //  an inclusive value, i.e. the last reported peak may appear at the stop time.
        //
        Fpga::CorrCmd_4g_t cmd =
        {
            Fpga::CORRELATE_FROM_TO,
            fpga_next_cmd_id++,
            start_of_next_result_buffer-num_samples_to_flush_fpga_sig_proc_chain,
            start_of_next_result_buffer+raw_result_averages*fpga_corr_result_accumulator.size()-1
        };
        fpga_corr_seq_cmds.push_back(cmd);
        
        debugStr(DebugStream::info1) << "Search: "
                                     << "current time = "    << current_time_samples << ", "
                                     << "low power mode = "  << low_power << ", "
                                     << "averaging = "       << raw_result_averages << ", "
                                     << "raw threshold = "   << raw_threshold << ", "
                                     << "avg threshold = "   << avg_threshold << ", "
                                     << "FPGA cmd ID = "     << static_cast<unsigned>(cmd.id) << ", "
                                     << "FPGA start time = " << cmd.start << ", "
                                     << "FPGA stop time = "  << cmd.stop << "\n";
    }
    else
    if (current_mode == TRACK)
    {
        uint64_t start_of_corr_samples = start_of_next_result_buffer+track_start_offset_samples;
        for (unsigned corr_num = 0 ; corr_num < raw_result_averages ; ++corr_num)
        {
            //
            //  The FPGA start time must allow for the time required to flush the FPGA's processing chain. The stop time
            //  is an inclusive value, i.e. the last reported peak may appear at the stop time.
            //
            Fpga::CorrCmd_4g_t cmd =
            {
                Fpga::CORRELATE_FROM_TO,
                fpga_next_cmd_id++,
                start_of_corr_samples-num_samples_to_flush_fpga_sig_proc_chain,
                start_of_corr_samples+Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1
            };
            fpga_corr_seq_cmds.push_back(cmd);
            
            start_of_corr_samples += fpga_corr_result_accumulator.size();
        
            debugStr(DebugStream::info1) << "Track: "
                                         << "current time = "    << current_time_samples << ", "
                                         << "low power mode = "  << low_power << ", "
                                         << "averaging = "       << raw_result_averages << ", "
                                         << "raw threshold = "   << raw_threshold << ", "
                                         << "avg threshold = "   << avg_threshold << ", "
                                         << "FPGA cmd ID = "     << static_cast<unsigned>(cmd.id) << ", "
                                         << "FPGA start time = " << cmd.start << ", "
                                         << "FPGA stop time = "  << cmd.stop << "\n";
        }
    }
    else
    {
        //
        //  The FPGA start time must allow for the time required to flush the FPGA's processing chain. The stop time is
        //  an inclusive value, i.e. the last reported peak may appear at the stop time.
        //
        Fpga::CorrCmd_4g_t cmd =
        {
            Fpga::CORRELATE_FROM_TO,
            fpga_next_cmd_id++,
            start_of_next_result_buffer-num_samples_to_flush_fpga_sig_proc_chain,
            start_of_next_result_buffer+raw_result_averages*fpga_corr_result_accumulator.size()-1
        };
        fpga_corr_seq_cmds.push_back(cmd);
        
        debugStr(DebugStream::info1) << "Continuous track: "
                                     << "current time = "    << current_time_samples << ", "
                                     << "low power mode = "  << low_power << ", "
                                     << "averaging = "       << raw_result_averages << ", "
                                     << "raw threshold = "   << raw_threshold << ", "
                                     << "avg threshold = "   << avg_threshold << ", "
                                     << "FPGA cmd ID = "     << static_cast<unsigned>(cmd.id) << ", "
                                     << "FPGA start time = " << cmd.start << ", "
                                     << "FPGA stop time = "  << cmd.stop << "\n";
    }

    //  Read and discard any results left over from an earlier sequence (an abort can leave the FPGA running).
    Fpga::CorrPeak_4g_t fpga_peaks_to_discard[MAX_PEAKS_FROM_FPGA];
    uint32_t num_peaks_retrieved = 0;
    (void)fpga_get_4g_peak_data(MAX_PEAKS_FROM_FPGA, fpga_peaks_to_discard, num_peaks_retrieved); 
    if (0 < num_peaks_retrieved)
    {
        debugStr(DebugStream::info4) << "Peaks discarded in restart_correlation: " << num_peaks_retrieved << "\n";
    }
    
    //  Restart the statistics, discarding old results.
    bool restart_corr_stats = true;
    float peak_inp_sums;
    float sum_inp_sums;
    (void)fpga_get_4g_statistics(restart_corr_stats, peak_inp_sums, sum_inp_sums);    

    //  Start the correlation.
    total_num_peaks_retrieved = 0;
    fpga_setup_4g_correlation(fpga_corr_seq_cmds);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Change the correlation mode (and optionally the start offset) and restart it.
//  @param[in]      m                   The new mode.
//  @param[in]      start_offset_samples
//                                      Sample offset in a result interval to begin correlation (in track mode).
//
void Correlator_4g::change_mode( mode_t m, unsigned start_offset_samples )
{
    current_mode = m;
    track_start_offset_samples = start_offset_samples;
    restart_correlation();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Change the correlation start and length settings and restart it.
//  @param[in]      start_offset_samples               
//                                      Sample offset in a result interval to begin correlation (in track mode).
//
void Correlator_4g::change_start( unsigned start_offset_samples )
{
    track_start_offset_samples = start_offset_samples;
    restart_correlation();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Determines if there is a new result available.
//  @details        In the process of determining this the function retrieves the FPGA results and implements the
//                  accumulation of detected peaks. If a result has become available the function also reads the
//                  statistics.
//
//                  The function also monitors the correlation status, but this does not affect whether or not results
//                  are available. It only produces warning and error messages indicating a possible implementation
//                  problem.
//  @return                             true if a result is available.
//
bool Correlator_4g::result_available(void)
{
    if (!fpga_corr_seq_cmds.empty())
    {
        read_and_accumulate_fpga_results();
        
        //
        //  Retrieve the command status results.
        //
        bool some_status_read = false;
        bool no_status_lost = true;
        uint8_t last_cmd_id_read = 0;
        bool last_cmd_processed_by_fpga = false;
        uint64_t current_time_samples = fpga_get_4g_time();
        while (true)
        {
            current_time_samples = fpga_get_4g_time();
            std::list<Fpga::CorrCmdStatus_4g_t> cmd_status_list;
            bool no_more_status_lost = fpga_get_4g_correlation_command_status(cmd_status_list);
            no_status_lost = no_status_lost && no_more_status_lost;
            if (cmd_status_list.empty())
            {
                break;
            }

            some_status_read = true;
            BOOST_FOREACH( Fpga::CorrCmdStatus_4g_t cmd_status, cmd_status_list )
            {
                switch (cmd_status.status)
                {
                case Fpga::REJECTED:
                    //  Consideration was given to removing the rejected command from the SW's list, but that will
                    //  happen anyway when a later call is for a time after the command's stop time (plus margin).
                    //  Recovery takes longer, but the code is simpler.
                    debugStr(DebugStream::warn2) << "Time = "
                                                 << current_time_samples
                                                 << " : Correlation command rejected: ID = " 
                                                 << static_cast<unsigned>(cmd_status.id) << "\n";
                    break;
                    
                case Fpga::FETCHED:
                    debugStr(DebugStream::info1) << "Time = "
                                                 << current_time_samples
                                                 << " : Correlation command fetched for execution: ID = "
                                                 << static_cast<unsigned>(cmd_status.id) << "\n";
                    break;
                    
                default:
                    debugStr(DebugStream::fatal) << "Time = "
                                                 << current_time_samples
                                                 << " : Unknown correlation command status: ID = "
                                                 << static_cast<unsigned>(cmd_status.id) << "\n";
                }
                
                if (cmd_status.id == (fpga_next_cmd_id-1))
                {
                    last_cmd_processed_by_fpga = true;
                }
                
                last_cmd_id_read = cmd_status.id; 
            }
        }    
        
        //
        //  The loss of status implies the FIFO is too small or the SW generated an excessively long sequence of
        //  commands, either of which would be an implementation error. It is possibly survivable.
        //
        if (!no_status_lost)
        {
            debugStr(DebugStream::fatal) << "Time = "
                                         << current_time_samples
                                         << " : FPGA correlation status was lost" << "\n";
        }
        
        //
        //  The presence of a status for a command after the last command that was programmed into the FPGA implies the
        //  SW started a sequence without waiting for the last one to complete. It is possibly survivable.
        //
        if (last_cmd_processed_by_fpga && (last_cmd_id_read != (fpga_next_cmd_id-1)))
        {
            debugStr(DebugStream::fatal) << "Time = "
                                         << current_time_samples
                                         << " : A command status was read after the status of the last expected command: "
                                         << " Last ID expected = " << static_cast<unsigned>(fpga_next_cmd_id-1) << ", "
                                         << " Last ID read = " << static_cast<unsigned>(last_cmd_id_read) << "\n";
        }

        //
        //  Determine if the last correlation sequence should have been completed, allowing a result buffer margin that
        //  ensures all peaks and status should have been reported. If all is done, make sure that the results have all
        //  been retrieved (processing may have been interrupted since this was last done at the start of the function).
        //  and empty the command queue.
        //
        if (!fpga_corr_seq_cmds.empty())
        {
            uint64_t current_time_samples = fpga_get_4g_time();
            if (current_time_samples >= (fpga_corr_seq_cmds.back().stop+fpga_corr_result_accumulator.size()))
            {
                read_and_accumulate_fpga_results();
                fpga_corr_seq_cmds.clear();

                debugStr(DebugStream::info1) << "Time = "
                                             << current_time_samples
                                             << " : Timeout termination of the correlation sequence ("
                                             << total_num_peaks_retrieved
                                             << " peaks retrieved from the FPGA)\n";
                                
                //  The correlation sequence is complete so get the statistics, storing the results.
                bool restart_corr_stats = false;
                corr_stat_res.count = fpga_get_4g_statistics(restart_corr_stats, 
                                                             corr_stat_res.peak_inp_sums,
                                                             corr_stat_res.sum_inp_sums);
            }
        }
    }
    
    return (fpga_corr_seq_cmds.empty());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Reads the most recent results.
//  @details        Divides the accumulated peaks by the number of result intervals spanned and applies the averaging
//                  threshold so that peaks below it are set to 0.0.
//
//                  Divides the sum of the correlator input sums by the number of results accumulated and the number of
//                  samples spanned by the correlator coefficients to give the average value of the input samples that 
//                  controbuted to the peak.
//
//                  Divides the maximum mean input samples by the number of samples spanned by the correlator
//                  coefficients to give the maximum average input, which since the SRS burst should be the strongest is
//                  a measure of their power even if they are not being detected by the correlator.
//
//                  Divides the sum of the mean input sample by the number of samples spanned by the correlator
//                  coefficients and the number of means accumulated to give the long-term average input.
//
//                  As a result of these scalings all the signal-level results are measures of the average correlator-
//                  input samples. These samples are the instaneous powers of the samples output be the channel filter.
//                  The channel filter and the deimation filter that precedes it have unity DC gain and so the signal-
//                  level results have units of (ADC LSB)^2.
//  @param[out]     corr_peak_counts    The vector of counts of the number of accumulated peaks.
//  @param[out]     corr_peak_vals      The vector of correlation peaks spanning a result interval.
//  @param[out]     corr_peak_mean_inputs_adc_lsb_sq
//                                      The vector of the mean of the input samples corresponding to a peak above the
//                                      threshold. Effectively this is the mean power of the detected SRS burst in units
//                                      of (ADC LSB)^2.
//  @param[out]     corr_inp_max_mean_adc_lsb_sq
//                                      The maximum of the mean of the input samples spanned by the correlation
//                                      coefficients over the time interval spanned by the sequence. Effectively this is
//                                      the mean power of the strongest burst in units of (ADC LSB)^2.
//  @param[out]     corr_inp_mean_adc_lsb_sq
//                                      The mean of the mean of the input samples spanned by the correlation
//                                      coefficients over the time interval spanned by the sequence. Effectively this is
//                                      the mean input power in units of (ADC LSB)^2.
// 
void Correlator_4g::read_results(std::vector<unsigned> &corr_peak_counts, 
                                 std::vector<float> &corr_peak_vals, 
                                 std::vector<float> &corr_peak_mean_inputs_adc_lsb_sq, 
                                 float &corr_inp_max_mean_adc_lsb_sq,
                                 float &corr_inp_mean_adc_lsb_sq)
{
    if (0 < corr_stat_res.count)
    {
        corr_inp_max_mean_adc_lsb_sq = corr_stat_res.peak_inp_sums/NUM_CORR_COEFFS;
        corr_inp_mean_adc_lsb_sq = corr_stat_res.sum_inp_sums/(NUM_CORR_COEFFS*corr_stat_res.count);
    }
    else
    {
        debugStr(DebugStream::error) << boost::format(  "read_results called when statistics count is zero\n" );
        corr_inp_max_mean_adc_lsb_sq = 0.0;
        corr_inp_mean_adc_lsb_sq = 0.0;
    }
    
    unsigned raw_results_averaged = get_num_raw_results_for_averaging();
    float threshold = search_avg_threshold;
    if (current_mode == TRACK)
    {
        threshold = track_avg_threshold;
    }
    else
    if (current_mode == CONT_TRACK)
    {
        threshold = cont_track_avg_threshold;
    }
    
    corr_peak_counts.clear();
    corr_peak_vals.clear();
    corr_peak_mean_inputs_adc_lsb_sq.clear();
    BOOST_FOREACH(sum_corr_inp_acc_t sum_corr_inp_acc, fpga_corr_result_accumulator)
    {
        float corr_out_peak_avg = 0.0;
        float corr_inp_mean_adc_lsb_sq_avg = 0.0;
        if (sum_corr_inp_acc.corr_out_peak_acc >= (threshold*raw_results_averaged))
        {
            corr_out_peak_avg = sum_corr_inp_acc.corr_out_peak_acc/raw_results_averaged;
            corr_inp_mean_adc_lsb_sq_avg =   sum_corr_inp_acc.corr_inp_sum_acc
                                           / static_cast<float>(NUM_CORR_COEFFS*raw_results_averaged);        // This gives more accurate results at low powers        
//                                           / static_cast<float>(NUM_CORR_COEFFS*sum_corr_inp_acc.num_peaks);    // This gives more accurate results at high powers
        }
        
#ifdef DISCARD_LOW_PWR_PEAKS_4G        
        if (corr_inp_mean_adc_lsb_sq_avg > (corr_inp_max_mean_adc_lsb_sq/10.0))
        {
#endif            
            corr_peak_counts.push_back(sum_corr_inp_acc.num_peaks);
            corr_peak_vals.push_back(corr_out_peak_avg);
            corr_peak_mean_inputs_adc_lsb_sq.push_back(corr_inp_mean_adc_lsb_sq_avg);
#ifdef DISCARD_LOW_PWR_PEAKS_4G        
        }
        else
        {
            corr_peak_counts.push_back(0);
            corr_peak_vals.push_back(0.0);
            corr_peak_mean_inputs_adc_lsb_sq.push_back(0.0);
        }
#endif        
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief          Calculate the number of raw results to be used for averaging based on the current mode and power
///                 setting.
/// @return                             The number of raw correlation results
///
unsigned Correlator_4g::get_num_raw_results_for_averaging(void)
{
    unsigned raw_results_to_average = search_mode_raw_results_to_avg;
    if ((current_mode == TRACK) || (current_mode == CONT_TRACK))
    {
        raw_results_to_average = track_mode_raw_results_to_avg;
    }
    
    return (raw_results_to_average);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Calculate the number of raw results to average.
//  @details        The default values depend on whether low power consumption or high sensitivity is required, but
//                  these are over-ridden if the client-supplied values are greater than zero.
//  @param[in]      client_search_averages
//                                      The client-defined number of raw correlation results to average in search
//                                      mode (0 forces the use of the defaults that depend on the low_power
//                                      setting).
//  @param[in]      client_track_averages
//                                      The client-defined number of raw correlation results to average in track
//                                      mode (0 forces the use of the defaults that depend on the low_power
//                                      setting).
//
void Correlator_4g::calc_averages(unsigned client_search_averages, unsigned client_track_averages)
{
    search_mode_raw_results_to_avg = client_search_averages;
    if (0 == search_mode_raw_results_to_avg)
    {
        if (low_power)
        {
            search_mode_raw_results_to_avg = LP_SEARCH_MODE_RAW_RESULTS_TO_AVG;
        }
        else
        {
            search_mode_raw_results_to_avg = HS_SEARCH_MODE_RAW_RESULTS_TO_AVG;
        }
    }
    
    track_mode_raw_results_to_avg = client_track_averages;
    if (0 == track_mode_raw_results_to_avg)
    {
        if (low_power)
        {
            track_mode_raw_results_to_avg = LP_TRACK_MODE_RAW_RESULTS_TO_AVG;
        }
        else
        {
            track_mode_raw_results_to_avg = HS_TRACK_MODE_RAW_RESULTS_TO_AVG;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Calculate the thresholds based on the current mode.
//  @details        The raw threshold is used by the FPGA to limit the number of correlation peaks that it reports. The
//                  averaging threshold is used by the Correlation class after it has performed the averaging and it
//                  determines the final false positive rate and sensitivity.
//
//                  The raw thresholds are based on the results reported in 1QR006, "SRS Performance Comparison" and
//                  they should allow the high sensitivity target to be met after the averaging is applied. The
//                  averaging thresholds are calculated from the raw thresholds by allowing for the processing gain of
//                  the averaging, i.e. sqrt(number of values averaged).
//
//                  The threshold factor is ony applied to the averaging threshold because applying it to the raw
//                  threshold risks too many FPGA peaks being produced so that some may have to be discarded.
//  @param[in]      search_threshold_factor_dB
//                                      The client-defined factor applied to the averaging threshold in search mode.
//  @param[in]      track_threshold_factor_dB
//                                      The client-defined factor applied to the averaging threshold in track mode.
//
void Correlator_4g::calc_thresholds(double search_threshold_factor_dB, double track_threshold_factor_dB)
{
    //
    //  The raw thresholds only depend on the mode (low power or high sensitivity) and any offsets. 
    //
    if (low_power)
    {
        search_raw_threshold =   LP_RAW_THRESHOLD
                               * pow(10.0, search_threshold_factor_dB/10.0);

        track_raw_threshold =   LP_RAW_THRESHOLD
                              * pow(10.0, (track_threshold_factor_dB+TRACK_THRESHOLD_FACTOR_DB)/10.0);

        cont_track_raw_threshold = search_raw_threshold;
    }
    else
    {
        search_raw_threshold =   HS_RAW_THRESHOLD
                               * pow(10.0, search_threshold_factor_dB/10.0);

        track_raw_threshold =   HS_RAW_THRESHOLD
                              * pow(10.0, (track_threshold_factor_dB+TRACK_THRESHOLD_FACTOR_DB)/10.0);
                              
        cont_track_raw_threshold = search_raw_threshold;
    }

    //
    //  The thresholds used after the averaging depend on the number of SRS intervals spanned by the sequence.
    //
    search_avg_threshold =   search_raw_threshold
                           / sqrt(search_mode_raw_results_to_avg);

    track_avg_threshold =   track_raw_threshold
                          / sqrt(track_mode_raw_results_to_avg);

    cont_track_avg_threshold =   cont_track_raw_threshold
                               / sqrt(track_mode_raw_results_to_avg);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Reads the correlator's result FIFO until it is empty.
//  @details        Retrieve the peaks from the FPGA in a loop and accumulate the results Each iteration should
//                  empty the FPGA FIFO and while those peaks are processed the FPGA may generate a few more peaks.
//                  Therefore continue to iterate until no more peaks are reported.
// 
void Correlator_4g::read_and_accumulate_fpga_results(void)
{
    Fpga::CorrPeak_4g_t fpga_peaks[MAX_PEAKS_FROM_FPGA];
    uint32_t num_peaks_retrieved = 0;
    uint32_t num_peaks_retrieved_in_this_call = 0;
    bool no_peaks_lost = true;
    while (true)
    {
        uint64_t current_time_samples = fpga_get_4g_time();
        bool no_more_peaks_lost = fpga_get_4g_peak_data(MAX_PEAKS_FROM_FPGA, fpga_peaks, num_peaks_retrieved);
        no_peaks_lost = no_peaks_lost && no_more_peaks_lost;
        total_num_peaks_retrieved = total_num_peaks_retrieved+num_peaks_retrieved;
        num_peaks_retrieved_in_this_call = num_peaks_retrieved_in_this_call+num_peaks_retrieved;
        if (0 == num_peaks_retrieved)
        {
            break;
        }
        
        //
        //  For each peak determine if it lies within the limits of one of the sequences. If there are no sequences
        //  left then the peak is outside the last sequence that was removed from the list in an earlier iteration.
        //  Otherwise discard and report any peak that is outside a sequence still in the list. If the peak is in the
        //  flush interval of a sequence simply discard it. Remove all sequences whose end is before the peak (they
        //  must be complete).
        //
        for (size_t peak_num = 0 ; peak_num < num_peaks_retrieved ; ++peak_num)
        {
            bool discard = false;
            if (fpga_corr_seq_cmds.empty())
            {
                debugStr(DebugStream::fatal) << "Time = "
                                             << current_time_samples
                                             << " : Peak from after stop (queue has already been emptied): " 
                                             << fpga_peaks[peak_num].time << "\n";
                discard = true;
            }
            else
            {
                while (!fpga_corr_seq_cmds.empty())
                {
                    if (fpga_peaks[peak_num].time < fpga_corr_seq_cmds.front().start)
                    {
                        debugStr(DebugStream::fatal) << "Time = "
                                                     << current_time_samples
                                                     << " : Peak from before start: " 
                                                     << fpga_peaks[peak_num].time
                                                     << " < "
                                                     << fpga_corr_seq_cmds.front().start << "\n";
                        discard = true;
                        break;
                    }
                    else
                    if (  fpga_peaks[peak_num].time
                        < (fpga_corr_seq_cmds.front().start+num_samples_to_flush_fpga_sig_proc_chain))
                    {
                        discard = true;
                        break;
                    }
                    else
                    if (fpga_peaks[peak_num].time <= fpga_corr_seq_cmds.front().stop)
                    {
                        break;
                    }
                    else
                    {
                        uint64_t last_stop = fpga_corr_seq_cmds.front().stop;
                        fpga_corr_seq_cmds.erase(fpga_corr_seq_cmds.begin());
                        if (fpga_corr_seq_cmds.empty())
                        {
                            discard = true;
                            debugStr(DebugStream::fatal) << "Time = "
                                                         << current_time_samples
                                                         << " : Peak from after stop: " 
                                                         << fpga_peaks[peak_num].time
                                                         << " > "
                                                         << last_stop
                                                         << " so terminating ("
                                                         << total_num_peaks_retrieved
                                                         << " peaks retrieved from the FPGA)\n";
                            
                            //  The correlation sequence is complete so get the statistics, storing the results.
                            bool restart_corr_stats = false;
                            corr_stat_res.count = fpga_get_4g_statistics(restart_corr_stats, 
                                                                         corr_stat_res.peak_inp_sums,
                                                                         corr_stat_res.sum_inp_sums);
                            
                            break;
                        }
                    }
                }
            }

            //
            //  Only consider peaks from within a sequence for inclusion in the accumulation.
            //
            //  Discard any peaks with negative numerators or denominators. A negative denominator may result from
            //  the finite precision and implementation of the accumulator used in the FPGA. If this is the case
            //  then the numerator will also be negative and the peak is discarded without a warning. If only one
            //  of the two values is negative, or if the denominator-squared is zero then issue a warning.
            //
            //  Accumulate remaining peaks into the buffer and allow for timing jitter by accumulating half the peak
            //  into the adjacent offsets. (Note that the power sum is accumulated in a similar way, but without the 
            //  scaling.)
            //
            if (!discard)
            {
                if ((0.0 >= fpga_peaks[peak_num].CorrPeakDenSqrd) || (0.0 > fpga_peaks[peak_num].CorrPeakNum))
                {
                    if (0.0 == fpga_peaks[peak_num].CorrPeakDenSqrd)
                    {
                        debugStr(DebugStream::warn2) << "Peak with zero denominator-squared: " 
                                                     << fpga_peaks[peak_num].CorrPeakNum << " / " 
                                                     << fpga_peaks[peak_num].CorrPeakDenSqrd << " at " 
                                                     << fpga_peaks[peak_num].time << "\n";
                    }
                    else
                    if (!(   (0.0 > fpga_peaks[peak_num].CorrPeakDenSqrd) 
                          && (0.0 > fpga_peaks[peak_num].CorrPeakNum)))
                    {
//  Warning removed because the FPGA implementation method produces such results quite often as a result of the
//  relatively small mantissa used in its floating-point processing.
//                        debugStr(DebugStream::warn2) << "Peak with negative numerator or denominator-squared: " 
//                                                     << fpga_peaks[peak_num].CorrPeakNum << " / " 
//                                                     << fpga_peaks[peak_num].CorrPeakDenSqrd << " at " 
//                                                     << fpga_peaks[peak_num].time << "\n";
                    }
                }
                else
                {
                    size_t result_interval_offset_samples =   fpga_peaks[peak_num].time 
                                                            % fpga_corr_result_accumulator.size();
                    float corr_peak = fpga_peaks[peak_num].CorrPeakNum/sqrt(fpga_peaks[peak_num].CorrPeakDenSqrd);
#ifdef DISCARD_HIGH_CORR_PEAKS_4G                    
                    if (corr_peak > 1.1)
                    {
                        debugStr(DebugStream::warn2) << "Large peak: " 
                                                     << corr_peak << "\n";
                    }
                    else
                    {
#endif
                        float sum_corr_inp = fpga_peaks[peak_num].SumCorrInpAtPeak;
                        fpga_corr_result_accumulator[result_interval_offset_samples].corr_out_peak_acc += corr_peak;
                        fpga_corr_result_accumulator[result_interval_offset_samples].corr_inp_sum_acc += sum_corr_inp;
                        fpga_corr_result_accumulator[result_interval_offset_samples].num_peaks += 1;
                        
                        size_t lower_index = result_interval_offset_samples-1;
                        size_t upper_index = result_interval_offset_samples+1;
                        if (0 == result_interval_offset_samples)
                        {
                            lower_index = fpga_corr_result_accumulator.size()-1;
                        }
                        else
                        if ((fpga_corr_result_accumulator.size()-1) == result_interval_offset_samples)
                        {
                            upper_index = 0;
                        }
                        
                        fpga_corr_result_accumulator[lower_index].corr_out_peak_acc += corr_peak/2.0;

                        fpga_corr_result_accumulator[upper_index].corr_out_peak_acc += corr_peak/2.0;
#ifdef DISCARD_HIGH_CORR_PEAKS_4G                    
                    }
#endif
                }
            }
        }
    }
    
    //
    //  Peak loss is an indication that the software is not keeping up. Performance may be affected, but unless
    //  there are other timing problems it is survivable.
    //
    if (!no_peaks_lost)
    {
        debugStr(DebugStream::warn2) << "FPGA correlation peaks were lost (retrieved this call: "
                                     << num_peaks_retrieved_in_this_call
                                     << ", total:"
                                     << total_num_peaks_retrieved
                                     << ")\n";
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Set the FPGA's test register
//  @param[in]      val                 The value to write to the register.
//
void Correlator_4g::set_test_register(uint32_t val)
{
    fpga.set_test_register(val);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Set the debug verbosity level.
//  @param[in]      level               The new verbosity level.
//
void Correlator_4g::debug_printing(unsigned level)
{
    debugStr.show_level(level);
}

/***********************************************************************************************************************
 *  Copyright (c) 2021 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  Filename:   searchandtrack_4g_test.cpp
 *  Author(s):  pdm
 *******************************************************************************************************************//**
 * File Description
 * ================
 *
 *  @file
 *  @brief          The implementation of functions used to test the ::SearchAndTrack_4g class.
 *  @details        The functions include test versions of the Correlator class functions used by the
 *                  ::SearchAndTrack_4g class.
 *                  Many status reports are "Debug::fatal" so that it is clear that this is a debug build.
 **********************************************************************************************************************/
#include "searchandtrack_4g.hpp"
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

#ifdef SWTEST_4G_SEARCH_TRACK
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Print paths
    //
    void SearchAndTrack_4g::test_print_paths(void)
    {
        if (path_list.size() > 0)
        {
            debugStr(DebugStream::error) <<   boost::format(  "        Paths:\n" );
            BOOST_FOREACH( PathList_t::value_type& path, path_list )
            {
                debugStr(DebugStream::error) <<   boost::format(  "        %4u    %+2d    %4u    %f\n" )
                                                % path.id
                                                % path.valid_count
                                                % path.offset
                                                % path.power;
            }
        }
        else
        {
            debugStr(DebugStream::error) <<   boost::format(  "        No paths\n" );
        }
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Print correlation peaks and powers
    //  @details        Only prints the results with non-zero peaks.
    //  @param[in]      corr_peak_vals          The correlation peak data.
    //  @param[in]      corr_peak_burst_powers  The correlation burst power data.
    //
    void SearchAndTrack_4g::test_print_corr_peaks_and_pwrs(const std::vector<float> &corr_peak_vals, 
                                                           const std::vector<float> &corr_peak_burst_powers)
    {
        bool header_printed = false;
        for (size_t offset = 0 ; offset < corr_peak_vals.size() ; ++offset)
        {
            if ((0.0 < corr_peak_vals[offset]) || (0.0 < corr_peak_burst_powers[offset]))
            {
                if (!header_printed)
                {
                    debugStr(DebugStream::error) <<   boost::format(  "    Correlation peaks and burst powers:\n" );
                    header_printed = true;
                }
                debugStr(DebugStream::error) <<   boost::format(  "    %4u    %f    %f\n" )
                                                % offset
                                                % corr_peak_vals[offset]
                                                % corr_peak_burst_powers[offset];
            }
        }
        
        if (!header_printed)
        {
            debugStr(DebugStream::error) <<   boost::format(  "    No correlation peaks\n" );
        }
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Print the annihilated peaks
    //  @details        Only prints the results with zero peaks.
    //  @param[in]      corr_peak_vals          The correlation peak data.
    //  @param[in]      det_burst_powers_mW     The correlation burst power data.
    //
    void SearchAndTrack_4g::test_print_annihilated_peaks(const std::vector<float> &corr_peak_vals, 
                                                         const std::vector<float> &det_burst_powers_mW)
    {
        bool header_printed = false;
        for (size_t offset = 0 ; offset < corr_peak_vals.size() ; ++offset)
        {
            if ((0.0 == corr_peak_vals[offset]) || (0.0 == det_burst_powers_mW[offset]))
            {
                if (!header_printed)
                {
                    debugStr(DebugStream::error) <<   boost::format(  "    Annihilated offsets:\n" );
                    header_printed = true;
                }
                debugStr(DebugStream::error) <<   boost::format(  "    %4u    %f    %f\n" )
                                                % offset
                                                % corr_peak_vals[offset]
                                                % det_burst_powers_mW[offset];
            }
        }
        
        if (!header_printed)
        {
            debugStr(DebugStream::error) <<   boost::format(  "    No annihilated offsets\n" );
        }
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Print the instance variables after a call to iterate() in the testIterate() function.
    //  @param[in]      iterate_retval          The value returned by the iterate function.
    //
    void SearchAndTrack_4g::test_print_instance_state(enum iter_res_t iterate_retval)
    {
        debugStr(DebugStream::error) <<   boost::format(  "        The mode is                     \"%s\"\n" )
                                        % test_get_mode_name(mode);
        debugStr(DebugStream::error) <<   boost::format(  "        The path list size is           %u\n" )
                                        % path_list.size();
        if (0 < path_list.size())
        {
            test_print_paths();
        }
        debugStr(DebugStream::error) <<   boost::format(  "        The correlation start is        %u\n" )
                                        % corr_start;
        debugStr(DebugStream::error) <<   boost::format(  "        The path ID counter is          %u\n" )
                                        % path_id_counter;
        debugStr(DebugStream::error) <<   boost::format(  "        The input channel power is      %f mW\n" )
                                        % input_channel_power_mW;
        debugStr(DebugStream::error) <<   boost::format(  "        The searching pause is          %u\n" )
                                        % searching_pause;
        debugStr(DebugStream::error) <<   boost::format(  "        The tracking pause is           %u\n" )
                                        % tracking_pause;
        debugStr(DebugStream::error) <<   boost::format(  "        The tracking timeout timer is   %u\n" )
                                        % no_path_timer;
        if (ranging_update_called)
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The ranging_update() function was called\n" );
            debugStr(DebugStream::error) <<   boost::format(  "            The ranging power was           %f\n" )
                                            % ranging_update_power;
        }
        else
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The ranging_update() function was not called\n" );
        }
        if (corr_restart_correlation_called)
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The corr_restart_correlation() function was called\n" );
        }
        else
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The corr_restart_correlation() function was not called\n" );
        }
        if (corr_change_mode_called)
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The corr_change_mode() function was called\n" );
            debugStr(DebugStream::error) <<   boost::format(  "            The correlator mode was         \"%s\"\n" )
                                            % test_get_corr_mode_name(corr_change_mode_m);
            debugStr(DebugStream::error) <<   boost::format(  "            The correlator start offset was %u\n" )
                                            % corr_change_mode_start_offset_samples;
        }
        else
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The corr_change_mode() function was not called\n" );
        }
        if (corr_change_start_called)
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The corr_change_start() function was called\n" );
            debugStr(DebugStream::error) <<   boost::format(  "            The correlator start offset was %u\n" )
                                            % corr_change_start_start_offset_samples;
        }
        else
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The corr_change_start() function was not called\n" );
        }
        if (corr_enter_low_power_mode_called)
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The corr_enter_low_power_mode() function was called\n" );
        }
        else
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The corr_enter_low_power_mode() function was not called\n" );
        }
        if (corr_exit_low_power_mode_called)
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The corr_exit_low_power_mode() function was called\n" );
        }
        else
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The corr_exit_low_power_mode() function was not called\n" );
        }
        if (adc_enter_low_power_mode_called)
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The adc_enter_low_power_mode() function was called\n" );
        }
        else
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The adc_enter_low_power_mode() function was not called\n" );
        }
        if (adc_exit_low_power_mode_called)
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The adc_exit_low_power_mode() function was called\n" );
        }
        else
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The adc_exit_low_power_mode() function was not called\n" );
        }
        
        if (radio_enter_low_power_mode_called)
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The radio_enter_low_power_mode() function was called\n" );
        }
        else
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The radio_enter_low_power_mode() function was not called\n" );
        }
        if (radio_exit_low_power_mode_called)
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The radio_exit_low_power_mode() function was called\n" );
        }
        else
        {
            debugStr(DebugStream::error) <<   boost::format(  "        The radio_exit_low_power_mode() function was not called\n" );
        }
        debugStr(DebugStream::error) <<   boost::format(  "        The iterate() return value was  \"%s\"\n" )
                                        % test_get_iterate_retval_name(iterate_retval);
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Return the mode name.
    //  @param[in]      mode                    The mode.
    //  @return                                 The mode name.
    //
    std::string SearchAndTrack_4g::test_get_mode_name(enum mode_t mode)
    {
        std::string name;
        switch (mode)
        {
            case IDLE:
                name = "IDLE";
                break;
        
            case SEARCHING:
                name = "SEARCHING";
                break;
        
            case TRACKING:
                name = "TRACKING";
                break;
        
            case PRE_TRACKING_RANGING:
                name = "PRE_TRACKING_RANGING";
                break;
        
            case TRACKING_RANGING:
                name = "TRACKING_RANGING";
                break;
                
            default:
                name = "UNKNOWN";
                break;
        }
        return (name);
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Return the name of the iterate() function's return value.
    //  @param[in]      iterate_retval          The return value.
    //  @return                                 The return value's name.
    //
    std::string SearchAndTrack_4g::test_get_iterate_retval_name(enum iter_res_t iterate_retval)
    {
        std::string name;
        switch (iterate_retval)
        {
            case WAITING:
                name = "WAITING";
                break;
        
            case START_SEARCH:
                name = "START_SEARCH";
                break;
        
            case START_TRACK:
                name = "START_TRACK";
                break;
        
            case NEW_RESULT:
                name = "NEW_RESULT";
                break;
        
            case NEW_RESULT_RANGING:
                name = "NEW_RESULT_RANGING";
                break;
                
            default:
                name = "UNKNOWN";
                break;
        }
        return (name);
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Return the name of the correlator mode.
    //  @param[in]      mode                    The mode.
    //  @return                                 The mode name.
    //
    std::string SearchAndTrack_4g::test_get_corr_mode_name(Correlator_4g::mode_t mode)
    {
        std::string name;
        switch (mode)
        {
            case Correlator_4g::SEARCH:
                name = "SEARCH";
                break;
        
            case Correlator_4g::TRACK:
                name = "TRACK";
                break;
        
            case Correlator_4g::CONT_TRACK:
                name = "CONT_TRACK";
                break;
                
            default:
                name = "UNKNOWN";
                break;
        }
        return (name);
    
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Wrapper for the Correlator_4g class's get_cal_factor_ratio function that retrieves the ratio of
    //                  the 4G calibration factors to the 3G ones from the Correlator class.
    //  @return                             The ratio to be applied to the 3G calibration factors when correcting the 
    //                                      4G results returned by read_results().
    // 
    double SearchAndTrack_4g::corr_get_cal_factor_ratio(void)
    {
        return (corr_get_cal_factor_ratio_retval);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Wrapper for the Correlator_4g class's result_available function that determines if a new result
    //                  is avaliable.
    //  @return                             true if a result is available.
    // 
    bool SearchAndTrack_4g::corr_result_available(void)
    {
        return (corr_result_available_retval);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Wrapper for the Correlator_4g class's read_results function that retrieves any results that may
    //                  be available.
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
    void SearchAndTrack_4g::corr_read_results(std::vector<unsigned> &corr_peak_counts, 
                                              std::vector<float> &corr_peak_vals, 
                                              std::vector<float> &corr_peak_mean_inputs_adc_lsb_sq, 
                                              float &corr_inp_max_mean_adc_lsb_sq,
                                              float &corr_inp_mean_adc_lsb_sq)
    {
        corr_peak_counts                 = corr_read_results_peak_counts;
        corr_peak_vals                   = corr_read_results_peak_vals;
        corr_peak_mean_inputs_adc_lsb_sq = corr_read_results_peak_mean_inputs_adc_lsb_sq;
        corr_inp_max_mean_adc_lsb_sq     = corr_read_results_inp_max_mean_adc_lsb_sq;
        corr_inp_mean_adc_lsb_sq         = corr_read_results_inp_mean_adc_lsb_sq;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Wrapper for the Correlator_4g class's restart_correlation function that restarts the correlation
    //                  with the same settings.
    // 
    void SearchAndTrack_4g::corr_restart_correlation(void)
    {
        corr_restart_correlation_called = true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Wrapper for the Correlator_4g class's change_mode function that changes the mode and offset and
    //                  restarts the correlation with the new settings.
    //  @param[in]      m                   The new mode.
    //  @param[in]      start_offset_samples
    //                                      Sample offset in result interval to begin correlation (in track mode).
    // 
    void SearchAndTrack_4g::corr_change_mode(Correlator_4g::mode_t m, unsigned start_offset_samples)
    {
        corr_change_mode_called = true;
        corr_change_mode_m = m;
        corr_change_mode_start_offset_samples = start_offset_samples;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Wrapper for the Correlator_4g class's change_start function that changes the offset and restarts
    //                  the correlation with the new settings.
    //  @param[in]      start_offset_samples
    //                                      Sample offset in result interval to begin correlation (in track mode).
    // 
    void SearchAndTrack_4g::corr_change_start(unsigned start_offset_samples)
    {
        corr_change_start_called = true;
        corr_change_start_start_offset_samples = start_offset_samples;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Wrapper for the Correlator_4g class's getFactor function that retrieves the linear power
    //                  calibration factor.
    //  @details        The factor should be applied to linear power samples normalised to the channel filter
    //                  output.  It includes a component that corresponds to the frontend gain.
    //  @param[in]      gain_dB             The frontend gain in dB.
    //  @return                             The calibration factor.
    //
    double SearchAndTrack_4g::pwrcal_getFactor(uint16_t gain_dB)
    {
        pwrcal_getFactor_gain_dB = gain_dB;
        return (pwrcal_getFactor_retval);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Wrapper for the Ranging class's get_gain_dB function that retrieve the current gain setting as a
    //                  dB value.
    //  @return                             The current gain.
    // 
    int SearchAndTrack_4g::ranging_get_gain_dB( void )
    {
        return (ranging_get_gain_dB_retval);
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Wrapper for the Ranging class's update function that is used to update the ranging.
    //  @param[in]      power               The mean %ADC sample power.
    //  @return                             The ranging status.
    //
    Ranging::RangingUpdateResult SearchAndTrack_4g::ranging_update(double power)
    {
        ranging_update_called = true;
        ranging_update_power = power;
        return (ranging_update_retval);
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Test the correct_powers_and_discard_weak_peaks() function
    //  @details        There is no automated checking of the results, which must be manually confirmed.
    //
    void SearchAndTrack_4g::test_correct_powers_and_discard_weak_peaks(void)
    {
        debugStr(DebugStream::info4) << boost::format(  "test_correct_powers_and_discard_weak_peaks ----------------\n" );
        
        double corr_inp_mean;
        std::vector<float> corr_peak_vals;
        std::vector<float> corr_peak_mean_inputs;
        
        //
        //  Check the power correction.
        //
        corr_inp_mean = 1.0;
        
        corr_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
        corr_peak_vals[corr_peak_vals.size()/2] = 2.0;

        corr_peak_mean_inputs.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
        corr_peak_mean_inputs[corr_peak_vals.size()/2] = 3.0;
        
        double cal_factor = 70.0;
        
        correct_powers_and_discard_weak_peaks(corr_inp_mean, corr_peak_vals, corr_peak_mean_inputs, cal_factor);

        debugStr(DebugStream::info4) <<   boost::format(  ">>> Should convert the Correlator mean input power to 70.0.\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>> Should produce:\n" );
        debugStr(DebugStream::info4) <<   boost::format(  "     4800    2.000000    210.000000\n" );
        
        debugStr(DebugStream::error) <<   boost::format(  "    Correlator mean input power became %.1f.\n" )
                                        % corr_inp_mean;
        test_print_corr_peaks_and_pwrs(corr_peak_vals, corr_peak_mean_inputs);
        
        //
        //  Check that weak peak is discarded.
        //
        corr_inp_mean = 1.0;
        
        corr_peak_vals.clear();
        corr_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
        corr_peak_vals[0]                         = 3.0*REL_THRESHOLD_FACTOR-0.0001;
        corr_peak_vals[1]                         = 3.0*REL_THRESHOLD_FACTOR/2.0;
        corr_peak_vals[corr_peak_vals.size()/2-1] = 3.0;
        corr_peak_vals[corr_peak_vals.size()/2]   = 2.0;
        corr_peak_vals[corr_peak_vals.size()/2+1] = 1.0;
        corr_peak_vals[corr_peak_vals.size()-2]   = 3.0*REL_THRESHOLD_FACTOR+0.0001;
        corr_peak_vals[corr_peak_vals.size()-1]   = 3.0*REL_THRESHOLD_FACTOR;

        corr_peak_mean_inputs.clear();
        corr_peak_mean_inputs.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
        corr_peak_mean_inputs[0]                         = 1.0;
        corr_peak_mean_inputs[1]                         = 2.0;
        corr_peak_mean_inputs[corr_peak_vals.size()/2-1] = 0.1;
        corr_peak_mean_inputs[corr_peak_vals.size()/2]   = 0.2;
        corr_peak_mean_inputs[corr_peak_vals.size()/2+1] = 0.3;
        corr_peak_mean_inputs[corr_peak_vals.size()-2]   = 0.0001;
        corr_peak_mean_inputs[corr_peak_vals.size()-1]   = 0.0002;
        
        cal_factor = 1.0;
        
        correct_powers_and_discard_weak_peaks(corr_inp_mean, corr_peak_vals, corr_peak_mean_inputs, cal_factor);

        debugStr(DebugStream::info4) <<   boost::format(  ">>> Should convert the Correlator mean input power to 1.0.\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>> Should produce:\n" );
        debugStr(DebugStream::info4) <<   boost::format(  "     4799    3.000000    0.100000\n" );
        debugStr(DebugStream::info4) <<   boost::format(  "     4800    2.000000    0.200000\n" );
        debugStr(DebugStream::info4) <<   boost::format(  "     4801    1.000000    0.300000\n" );
        debugStr(DebugStream::info4) <<   boost::format(  "     9598    0.300100    0.000100\n" );
        debugStr(DebugStream::info4) <<   boost::format(  "     9599    0.300000    0.000200\n" );
        
        debugStr(DebugStream::error) <<   boost::format(  "    Correlator mean input power became %.1f.\n" )
                                        % corr_inp_mean;
        test_print_corr_peaks_and_pwrs(corr_peak_vals, corr_peak_mean_inputs);
        
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Test the find_new_peak_nearby() function
    //  @details        There is no automated checking of the results, which must be manually confirmed.
    // 
    void SearchAndTrack_4g::test_find_new_peak_nearby(void)
    {
        debugStr(DebugStream::info4) << boost::format(  "test_find_new_peak_nearby ---------------------------------\n" );

        unsigned offset = Design::SAMPLES_PER_4G_SRS_PERIOD;
        std::vector<float> corr_peak_vals(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
        unsigned offset_of_strongest;
        
        //
        //  Should produce an error because the offset is outside the vector and the result should then be false as
        //  there are no peaks near the wrapped offset.
        //
        offset = corr_peak_vals.size();
        
        debugStr(DebugStream::info4) << boost::format(  ">>> Should produce an error because the offset is outside the vector and the result should then be false as there are no peaks near the wrapped offset.\n" );
        debugStr(DebugStream::error) <<   boost::format(  "    Peak found = %u\n" )
                                        % find_new_peak_nearby(offset, corr_peak_vals, offset_of_strongest);
        
        //
        //  The result should be false as there are no peaks near the offset at the start of the vector.
        //
        offset = 0;
        corr_peak_vals.clear();
        corr_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 1.0);
        std::fill_n(corr_peak_vals.begin(), MAX_OFFSET_DIFF+1, 0.0);
        std::fill_n(corr_peak_vals.end()-MAX_OFFSET_DIFF, MAX_OFFSET_DIFF, 0.0);
        
        debugStr(DebugStream::info4) << boost::format(  ">>> The result should be false as there are no peaks near the offset at the start of the vector.\n" );
        debugStr(DebugStream::error) <<   boost::format(  "    Peak found = %u\n" )
                                        % find_new_peak_nearby(offset, corr_peak_vals, offset_of_strongest);
        
        //
        //  The result should be true as there is one peak near the offset at the start of the vector.
        //
        offset = 0;
        corr_peak_vals.clear();
        corr_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 1.0);
        std::fill_n(corr_peak_vals.begin(), MAX_OFFSET_DIFF+1, 0.0);
        std::fill_n(corr_peak_vals.end()-(MAX_OFFSET_DIFF-1), (MAX_OFFSET_DIFF-1), 0.0);
        
        debugStr(DebugStream::info4) << boost::format(  ">>> The result should be true as there is one peak near the offset at the start of the vector.\n" );
        debugStr(DebugStream::error) <<   boost::format(  "    Peak found = %u\n" )
                                        % find_new_peak_nearby(offset, corr_peak_vals, offset_of_strongest);
        debugStr(DebugStream::error) <<   boost::format(  "    Offset of peak = %u (should be %u)\n" )
                                        % offset_of_strongest
                                        % (corr_peak_vals.size()-MAX_OFFSET_DIFF);
        
        //
        //  The result should be true as there is one peak near the offset at the start of the vector.
        //
        offset = 0;
        corr_peak_vals.clear();
        corr_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 1.0);
        std::fill_n(corr_peak_vals.begin(), MAX_OFFSET_DIFF, 0.0);
        std::fill_n(corr_peak_vals.end()-MAX_OFFSET_DIFF, MAX_OFFSET_DIFF, 0.0);
        
        debugStr(DebugStream::info4) << boost::format(  ">>> The result should be true as there is one peak near the offset at the start of the vector.\n" );
        debugStr(DebugStream::error) <<   boost::format(  "    Peak found = %u\n" )
                                        % find_new_peak_nearby(offset, corr_peak_vals, offset_of_strongest);
        debugStr(DebugStream::error) <<   boost::format(  "    Offset of peak = %u (should be %u)\n" )
                                        % offset_of_strongest
                                        % static_cast<unsigned>(MAX_OFFSET_DIFF);
        
        //
        //  The result should be false as there are no peaks near the offset at the end of the vector.
        //
        offset = corr_peak_vals.size()-1;
        corr_peak_vals.clear();
        corr_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 1.0);
        std::fill_n(corr_peak_vals.begin(), MAX_OFFSET_DIFF, 0.0);
        std::fill_n(corr_peak_vals.end()-(MAX_OFFSET_DIFF+1), MAX_OFFSET_DIFF+1, 0.0);
        
        debugStr(DebugStream::info4) << boost::format(  ">>> The result should be false as there are no peaks near the offset at the end of the vector.\n" );
        debugStr(DebugStream::error) <<   boost::format(  "    Peak found = %u\n" )
                                        % find_new_peak_nearby(offset, corr_peak_vals, offset_of_strongest);
        
        //
        //  The result should be true as there is one peak near the offset at the end of the vector.
        //
        offset = corr_peak_vals.size()-1;
        corr_peak_vals.clear();
        corr_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 1.0);
        std::fill_n(corr_peak_vals.begin(), MAX_OFFSET_DIFF, 0.0);
        std::fill_n(corr_peak_vals.end()-MAX_OFFSET_DIFF, MAX_OFFSET_DIFF, 0.0);
        
        debugStr(DebugStream::info4) << boost::format(  ">>> The result should be true as there is one peak near the offset at the end of the vector.\n" );
        debugStr(DebugStream::error) <<   boost::format(  "    Peak found = %u\n" )
                                        % find_new_peak_nearby(offset, corr_peak_vals, offset_of_strongest);
        debugStr(DebugStream::error) <<   boost::format(  "    Offset of peak = %u (should be %u)\n" )
                                        % offset_of_strongest
                                        % (corr_peak_vals.size()-(MAX_OFFSET_DIFF+1));
        
        //
        //  The result should be true as there is one peak near the offset at the end of the vector.
        //
        offset = corr_peak_vals.size()-1;
        corr_peak_vals.clear();
        corr_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 1.0);
        std::fill_n(corr_peak_vals.begin(), MAX_OFFSET_DIFF-1, 0.0);
        std::fill_n(corr_peak_vals.end()-(MAX_OFFSET_DIFF+1), MAX_OFFSET_DIFF+1, 0.0);
        
        debugStr(DebugStream::info4) << boost::format(  ">>> The result should be true as there is one peak near the offset at the end of the vector.\n" );
        debugStr(DebugStream::error) <<   boost::format(  "    Peak found = %u\n" )
                                        % find_new_peak_nearby(offset, corr_peak_vals, offset_of_strongest);
        debugStr(DebugStream::error) <<   boost::format(  "    Offset of peak = %u (should be %u)\n" )
                                        % offset_of_strongest
                                        % static_cast<unsigned>(MAX_OFFSET_DIFF-1);
        
        //
        //  The result should be false as there are no peaks near the offset in the middle of the vector.
        //
        offset = Design::SAMPLES_PER_4G_SRS_PERIOD/2;
        corr_peak_vals.clear();
        corr_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 1.0);
        std::fill_n(corr_peak_vals.begin()+Design::SAMPLES_PER_4G_SRS_PERIOD/2-MAX_OFFSET_DIFF, 
                    2*MAX_OFFSET_DIFF+1,
                    0.0);
        
        debugStr(DebugStream::info4) << boost::format(  ">>> The result should be false as there are no peaks near the offset in the middle of the vector.\n" );
        debugStr(DebugStream::error) <<   boost::format(  "    Peak found = %u\n" )
                                        % find_new_peak_nearby(offset, corr_peak_vals, offset_of_strongest);
        
        //
        //  The result should be true as there is one peak near the offset in the middle of the vector.
        //
        offset = Design::SAMPLES_PER_4G_SRS_PERIOD/2;
        corr_peak_vals.clear();
        corr_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 1.0);
        std::fill_n(corr_peak_vals.begin()+Design::SAMPLES_PER_4G_SRS_PERIOD/2-MAX_OFFSET_DIFF, 
                    2*MAX_OFFSET_DIFF,
                    0.0);
        
        debugStr(DebugStream::info4) << boost::format(  ">>> The result should be true as there is one peak near the offset in the middle of the vector.\n" );
        debugStr(DebugStream::error) <<   boost::format(  "    Peak found = %u\n" )
                                        % find_new_peak_nearby(offset, corr_peak_vals, offset_of_strongest);
        debugStr(DebugStream::error) <<   boost::format(  "    Offset of peak = %u (should be %u)\n" )
                                        % offset_of_strongest
                                        % (Design::SAMPLES_PER_4G_SRS_PERIOD/2+MAX_OFFSET_DIFF);
        
        //
        //  The result should be true as there is one peak near the offset in the middle of the vector.
        //
        offset = Design::SAMPLES_PER_4G_SRS_PERIOD/2;
        corr_peak_vals.clear();
        corr_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 1.0);
        std::fill_n(corr_peak_vals.begin()+Design::SAMPLES_PER_4G_SRS_PERIOD/2-MAX_OFFSET_DIFF+1, 
                    2*MAX_OFFSET_DIFF,
                    0.0);
        
        debugStr(DebugStream::info4) << boost::format(  ">>> The result should be true as there is one peak near the offset in the middle of the vector.\n" );
        debugStr(DebugStream::error) <<   boost::format(  "    Peak found = %u\n" )
                                        % find_new_peak_nearby(offset, corr_peak_vals, offset_of_strongest);
        debugStr(DebugStream::error) <<   boost::format(  "    Offset of peak = %u (should be %u)\n" )
                                        % offset_of_strongest
                                        % (Design::SAMPLES_PER_4G_SRS_PERIOD/2-MAX_OFFSET_DIFF);
        
        //
        //  The result should be true as there are two peaks near the offset in the middle of the vector - the stronger should be identified.
        //
        offset = Design::SAMPLES_PER_4G_SRS_PERIOD/2;
        corr_peak_vals.clear();
        corr_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 1.0);
        std::fill_n(corr_peak_vals.begin()+Design::SAMPLES_PER_4G_SRS_PERIOD/2-MAX_OFFSET_DIFF, 
                    2*MAX_OFFSET_DIFF+1,
                    0.0);
        corr_peak_vals[Design::SAMPLES_PER_4G_SRS_PERIOD/2-1] = 0.2;
        corr_peak_vals[Design::SAMPLES_PER_4G_SRS_PERIOD/2+1] = 0.200001;
        
        debugStr(DebugStream::info4) << boost::format(  ">>> The result should be true as there are two peaks near the offset in the middle of the vector - the stronger should be identified.\n" );
        debugStr(DebugStream::error) <<   boost::format(  "    Peak found = %u\n" )
                                        % find_new_peak_nearby(offset, corr_peak_vals, offset_of_strongest);
        debugStr(DebugStream::error) <<   boost::format(  "    Offset of peak = %u (should be %u)\n" )
                                        % offset_of_strongest
                                        % (Design::SAMPLES_PER_4G_SRS_PERIOD/2+1);
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Test the annihilate() function
    //  @details        There is no automated checking of the results, which must be manually confirmed.
    // 
    void SearchAndTrack_4g::test_annihilate(void)
    {
        debugStr(DebugStream::info4) << boost::format(  "test_annihilate -------------------------------------------\n" );

        std::vector<float> corr_peak_vals(Design::SAMPLES_PER_4G_SRS_PERIOD);
        std::vector<float> det_burst_powers_mW(Design::SAMPLES_PER_4G_SRS_PERIOD);
        
        //
        //  Should remove all peaks around the start of the vectors.
        //
        corr_peak_vals.clear();
        corr_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 1.0);
        
        det_burst_powers_mW.clear();
        det_burst_powers_mW.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 1.0);
        
        annihilate(0, corr_peak_vals, det_burst_powers_mW);

        debugStr(DebugStream::info4) <<   boost::format(  ">>> Should annihilate offsets: %u to %u and %u to %u\n" )
                                        % static_cast<unsigned>(0)
                                        % static_cast<unsigned>(MAX_ANNIHILATION_DIFF)
                                        % (corr_peak_vals.size()-MAX_ANNIHILATION_DIFF)
                                        % (corr_peak_vals.size()-1);
                                        
        test_print_annihilated_peaks(corr_peak_vals, det_burst_powers_mW);
        
        //
        //  Should remove all peaks around the end of the vectors.
        //
        corr_peak_vals.clear();
        corr_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 1.0);
        
        det_burst_powers_mW.clear();
        det_burst_powers_mW.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 1.0);
        
        annihilate(corr_peak_vals.size()-1, corr_peak_vals, det_burst_powers_mW);

        debugStr(DebugStream::info4) <<   boost::format(  ">>> Should annihilate offsets: %u to %u and %u to %u\n" )
                                        % static_cast<unsigned>(0)
                                        % static_cast<unsigned>(MAX_ANNIHILATION_DIFF-1)
                                        % (corr_peak_vals.size()-MAX_ANNIHILATION_DIFF-1)
                                        % (corr_peak_vals.size()-1);
                                        
        test_print_annihilated_peaks(corr_peak_vals, det_burst_powers_mW);
        
        //
        //  Should remove all peaks around the middle of the vectors.
        //
        corr_peak_vals.clear();
        corr_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 1.0);
        
        det_burst_powers_mW.clear();
        det_burst_powers_mW.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 1.0);
        
        annihilate(corr_peak_vals.size()/2, corr_peak_vals, det_burst_powers_mW);

        debugStr(DebugStream::info4) <<   boost::format(  ">>> Should annihilate offsets: %u to %u\n" )
                                        % (corr_peak_vals.size()/2-MAX_ANNIHILATION_DIFF)
                                        % (corr_peak_vals.size()/2+MAX_ANNIHILATION_DIFF);
                                        
        test_print_annihilated_peaks(corr_peak_vals, det_burst_powers_mW);
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Test the update_paths() function
    //  @details        There is no automated checking of the results, which must be manually confirmed.
    //
    void SearchAndTrack_4g::test_update_paths(void)
    {
        debugStr(DebugStream::info4) << boost::format(  "test_update_paths -----------------------------------------\n" );

        std::vector<float> corr_peak_vals;
        std::vector<float> det_burst_powers_mW;

        path_list.clear();
        
        corr_peak_vals.clear();
        corr_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
        
        det_burst_powers_mW.clear();
        det_burst_powers_mW.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
        
        path_id_counter = 10;
        
        //
        //  This path is exactly matched by a new peak of equal power. The valid count can increment.
        //
        unsigned offset1 = 10;
        struct Path_t path1 = { 0, VALID_COUNT_INIT_VAL, offset1, 0.001 };
        path_list.push_back(path1);
        
        corr_peak_vals[offset1]      = 1.0;
        det_burst_powers_mW[offset1] = 0.001;

        //
        //  This path is exactly matched by a new peak of twice the power at an offset. The valid count can increment.
        //  There is a second weaker peak that is ignored as being too close.
        //
        unsigned offset2 = 20;
        struct Path_t path2 = { 1, VALID_COUNT_MAX_VAL, offset2, 0.0005 };
        path_list.push_back(path2);
        
        corr_peak_vals[offset2+1]      = 1.0;
        det_burst_powers_mW[offset2+1] = 0.001;
        
        corr_peak_vals[offset2-1]      = 0.9999;
        det_burst_powers_mW[offset2-1] = 0.001;

        //
        //  This path has no matching peak so the valid count should decrement.
        //
        unsigned offset3 = 30;
        struct Path_t path3 = { 2, VALID_COUNT_INIT_VAL, offset3, 0.0001 };
        path_list.push_back(path3);

        //
        //  This peak does not match a path so it should be added to the path list.
        //
        unsigned offset4 = 40;
        corr_peak_vals[offset4]      = 0.5;
        det_burst_powers_mW[offset4] = 0.002;

        //
        //  This pair of peak do not match a path anly the strongest should be added to the path list.
        //
        unsigned offset5 = 50;
        corr_peak_vals[offset5+1]      = 0.1;
        det_burst_powers_mW[offset5+1] = 0.003;
        corr_peak_vals[offset5-1]      = 0.09;
        det_burst_powers_mW[offset5-1] = 0.004;

        //
        //  Update the paths.
        //
        debugStr(DebugStream::info4) <<   boost::format(  ">>> Initially paths at:\n" );
        test_print_paths();
        debugStr(DebugStream::info4) <<   boost::format(  ">>> And peaks at:\n" );
        test_print_corr_peaks_and_pwrs(corr_peak_vals, det_burst_powers_mW);

        update_paths(corr_peak_vals, det_burst_powers_mW);

        debugStr(DebugStream::info4) <<   boost::format(  ">>> Should leave paths at:\n" );
        debugStr(DebugStream::info4) <<   boost::format(  "       0    -3      10    0.001000\n" );
        debugStr(DebugStream::info4) <<   boost::format(  "       1     0      21    0.000750\n" );
        debugStr(DebugStream::info4) <<   boost::format(  "       2    -6      30    0.000100\n" );
        debugStr(DebugStream::info4) <<   boost::format(  "      10    -4      40    0.002000\n" );
        debugStr(DebugStream::info4) <<   boost::format(  "      11    -4      51    0.003000\n" );

        debugStr(DebugStream::info4) <<   boost::format(  ">>> Should leave no peaks.\n" );

        test_print_paths();
        test_print_corr_peaks_and_pwrs(corr_peak_vals, det_burst_powers_mW);
    }

    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Test the calc_new_track_offset() function
    //  @details        There is no automated checking of the results, which must be manually confirmed.
    //
    void SearchAndTrack_4g::test_calc_new_track_offset(void)
    {
        debugStr(DebugStream::info4) << boost::format(  "test_calc_new_track_offset --------------------------------\n" );

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //  A single path gives a result centred at its offset, i.e. starting at its offset minus
        //  (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2.
        //
        //  Offset result = 0 when the old start was 0
        //
        {
            corr_start = 0;
            path_list.clear();
            
            unsigned offset = (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2;
            struct Path_t path = { 0, VALID_COUNT_CONFIRMED, offset, 0.001 };
            path_list.push_back(path);

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Case  1)\n" );
            test_print_paths();

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Should give an offset at 0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  "    Offset = %u\n" )
                                            % calc_new_track_offset();
        }
        
        //
        //  Offset result = 9599 when the old start was 0
        //
        {
            corr_start = 0;
            path_list.clear();
            
            unsigned offset =   (  Design::SAMPLES_PER_4G_SRS_PERIOD
                                 - 1
                                 + (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2)
                              % Design::SAMPLES_PER_4G_SRS_PERIOD;
            struct Path_t path = { 0, VALID_COUNT_CONFIRMED, offset, 0.001 };
            path_list.push_back(path);

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Case  2)\n" );
            test_print_paths();

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Should give an offset at 9599\n" );
            debugStr(DebugStream::info4) <<   boost::format(  "    Offset = %u\n" )
                                            % calc_new_track_offset();
        }
        
        //
        //  Offset result = 9378 when the old start was 0
        //
        {
            corr_start = 0;
            path_list.clear();
            
            unsigned offset = 0;
            struct Path_t path = { 0, VALID_COUNT_CONFIRMED, offset, 0.001 };
            path_list.push_back(path);

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Case  3)\n" );
            test_print_paths();

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Should give an offset at 9378\n" );
            debugStr(DebugStream::info4) <<   boost::format(  "    Offset = %u\n" )
                                            % calc_new_track_offset();
        }
        
        //
        //  Offset result = 9377 when the old start was 0
        //
        {
            corr_start = 0;
            path_list.clear();
            
            unsigned offset = Design::SAMPLES_PER_4G_SRS_PERIOD-1;
            struct Path_t path = { 0, VALID_COUNT_CONFIRMED, offset, 0.001 };
            path_list.push_back(path);

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Case  4)\n" );
            test_print_paths();

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Should give an offset at 9377\n" );
            debugStr(DebugStream::info4) <<   boost::format(  "    Offset = %u\n" )
                                            % calc_new_track_offset();
        }

        //
        //  Offset result = 0 when the old start was 9377
        //
        {
            corr_start = Design::SAMPLES_PER_4G_SRS_PERIOD-(Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2-1;
            path_list.clear();
            
            unsigned offset = (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2;
            struct Path_t path = { 0, VALID_COUNT_CONFIRMED, offset, 0.001 };
            path_list.push_back(path);

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Case  5)\n" );
            test_print_paths();

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Should give an offset at 0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  "    Offset = %u\n" )
                                            % calc_new_track_offset();
        }

        //
        //  Offset result = 0 when the old start was 9378
        //
        {
            corr_start = Design::SAMPLES_PER_4G_SRS_PERIOD-(Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2;
            path_list.clear();
            
            unsigned offset = (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2;
            struct Path_t path = { 0, VALID_COUNT_CONFIRMED, offset, 0.001 };
            path_list.push_back(path);

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Case  6)\n" );
            test_print_paths();

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Should give an offset at 0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  "    Offset = %u\n" )
                                            % calc_new_track_offset();
        }

        //
        //  Offset result = 0 when the old start was 9599
        //
        {
            corr_start = Design::SAMPLES_PER_4G_SRS_PERIOD-1;
            path_list.clear();
            
            unsigned offset = (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2;
            struct Path_t path = { 0, VALID_COUNT_CONFIRMED, offset, 0.001 };
            path_list.push_back(path);

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Case  7)\n" );
            test_print_paths();

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Should give an offset at 0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  "    Offset = %u\n" )
                                            % calc_new_track_offset();
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //  Multiple paths.
        //
        //  Offset result = 0 when the old start was 0, with two paths of equal weight
        //
        {
            corr_start = 0;
            path_list.clear();
            
            unsigned offset1 = (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2+1;
            struct Path_t path1 = { 0, VALID_COUNT_CONFIRMED, offset1, 0.001 };
            path_list.push_back(path1);
            
            unsigned offset2 = (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2-1;
            struct Path_t path2 = { 1, VALID_COUNT_CONFIRMED, offset2, 0.001 };
            path_list.push_back(path2);

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Case  8)\n" );
            test_print_paths();

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Should give an offset at 0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  "    Offset = %u\n" )
                                            % calc_new_track_offset();
        }
        
        //
        //  Offset result = 0 when the old start was 0, with two paths of unequal weight
        //
        {
            corr_start = 0;
            path_list.clear();
            
            unsigned offset1 = (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2+2;
            struct Path_t path1 = { 0, VALID_COUNT_CONFIRMED, offset1, 0.001 };
            path_list.push_back(path1);
            
            unsigned offset2 = (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2-1;
            struct Path_t path2 = { 1, VALID_COUNT_CONFIRMED, offset2, 0.002 };
            path_list.push_back(path2);

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Case  9)\n" );
            test_print_paths();

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Should give an offset at 0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  "    Offset = %u\n" )
                                            % calc_new_track_offset();
        }
        
        //
        //  Offset result = 4799 when the old start was 0, with three paths of unequal weight
        //
        {
            corr_start = 0;
            path_list.clear();
            
            unsigned offset1 = (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2+2;
            struct Path_t path1 = { 0, VALID_COUNT_CONFIRMED, offset1, 0.0010 };
            path_list.push_back(path1);
            
            unsigned offset2 = (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2-1;
            struct Path_t path2 = { 1, VALID_COUNT_CONFIRMED, offset2, 0.0020 };
            path_list.push_back(path2);
            
            unsigned offset3 = (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2-6;
            struct Path_t path3 = { 2, VALID_COUNT_CONFIRMED, offset3, 0.0006 };
            path_list.push_back(path3);

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Case 10)\n" );
            test_print_paths();

            debugStr(DebugStream::info4) <<   boost::format(  ">>> Should give an offset at 9599\n" );
            debugStr(DebugStream::info4) <<   boost::format(  "    Offset = %u\n" )
                                            % calc_new_track_offset();
        }
    }
        
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Test the track() function
    //  @details        There is no automated checking of the results, which must be manually confirmed.
    //
    void SearchAndTrack_4g::test_track(void)
    {
        debugStr(DebugStream::info4) << boost::format(  "test_track ------------------------------------------------\n" );

        //
        //  A relatively simple test as the sub-functions are tested in detail elsewhere.
        //
        input_channel_power_mW = 0.0012;
        corr_start = 0;
        path_list.clear();
        
        std::vector<float> corr_peak_vals(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
        std::vector<float> corr_peak_mean_inputs(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);

        //
        //  Three paths, a peak that gets reinforced, and two that don't one of which is then removed.
        //
        unsigned path_offset1 = (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2+10;
        struct Path_t path1 = { 3, VALID_COUNT_CONFIRMED, path_offset1, 0.0010 };
        path_list.push_back(path1);
            
        unsigned path_offset2 = (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2-20;
        struct Path_t path2 = { 7, VALID_COUNT_CONFIRMED, path_offset2, 0.0008 };
        path_list.push_back(path2);
            
        unsigned path_offset3 = (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2+30;
        struct Path_t path3 = { 9, VALID_COUNT_IS_INVALID-VALID_COUNT_PATH_NOT_FOUND, path_offset3, 0.002 };
        path_list.push_back(path3);

        path_id_counter = 11;
        
        debugStr(DebugStream::info4) <<   boost::format(  "    Current input channel power = %f mW\n" )
                                        % input_channel_power_mW;
        debugStr(DebugStream::info4) <<   boost::format(  "    Current correlation start   = %d\n" )
                                        % corr_start;
        test_print_paths();
        debugStr(DebugStream::info4) <<   boost::format(  "    Current next path ID = %u\n" )
                                        % path_id_counter;
        
        //
        //  Three peaks, one that reinforces a path and makes it the strongest, one that matches a path's timing but is
        //  discarded because its correlatioon is too weak, and one that does not match a peak.
        //
        double corr_inp_mean = 1.1;

        unsigned peak_offset1 = (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2-20;
        corr_peak_vals[peak_offset1]      = 0.5;
        corr_peak_mean_inputs[peak_offset1] = 2.2;

        unsigned peak_offset2 = (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2-30;
        corr_peak_vals[peak_offset2]      = 0.2;
        corr_peak_mean_inputs[peak_offset2] = 1.1;

        unsigned peak_offset3 = (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2+30;
        corr_peak_vals[peak_offset3]      = 0.01;
        corr_peak_mean_inputs[peak_offset3] = 0.1;

        debugStr(DebugStream::info4) <<   boost::format(  "    New input channel power = %f (ADC LSB)^2\n" )
                                        % corr_inp_mean;
        test_print_corr_peaks_and_pwrs(corr_peak_vals, corr_peak_mean_inputs);

        double cal_factor = 0.001;

        unsigned new_corr_start = track(corr_inp_mean, corr_peak_vals, corr_peak_mean_inputs, cal_factor);

        debugStr(DebugStream::info4) <<   boost::format(  ">>> The new input channel power should be 0.0011 mW\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>> The new correlation start should be 9580\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>> The new paths should be:\n" );
        debugStr(DebugStream::info4) <<   boost::format(  "        7     0     202    0.001500\n" );
        debugStr(DebugStream::info4) <<   boost::format(  "       11    -4     192    0.001100\n" );
        debugStr(DebugStream::info4) <<   boost::format(  "        3    -2     232    0.001000\n" );
        
        debugStr(DebugStream::info4) <<   boost::format(  ">>> The new next path ID should be 12\n" );
        
        debugStr(DebugStream::info4) <<   boost::format(  "    New input channel power = %f mW\n" )
                                        % input_channel_power_mW;
        debugStr(DebugStream::info4) <<   boost::format(  "    New correlation start = %d\n" )
                                        % new_corr_start;
        test_print_paths();
        debugStr(DebugStream::info4) <<   boost::format(  "    New next path ID = %u\n" )
                                        % path_id_counter;

    }

        
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Test the iterate() function
    //  @details        There is no automated checking of the results, which must be manually confirmed.
    //
    void SearchAndTrack_4g::test_iterate(void)
    {
        debugStr(DebugStream::info4) << boost::format(  "test_iterate ----------------------------------------------\n" );
        
        ranging_get_gain_dB_retval       = 33;
        pwrcal_getFactor_retval          = 0.01;
        corr_get_cal_factor_ratio_retval = 0.1;
        
        //
        //  No correlation result available.
        //
        debugStr(DebugStream::info4) <<   boost::format(  ">>> Case  1) No correlator result available.\n" );

        enum mode_t all_modes[]  = {  IDLE,   SEARCHING,   TRACKING,   PRE_TRACKING_RANGING,    TRACKING_RANGING };
        for (size_t mode_num = 0 ; mode_num < sizeof(all_modes)/sizeof(all_modes[0]) ; ++mode_num)
        {
            mode = all_modes[mode_num];
            path_list.clear();
            corr_start = 0;
            path_id_counter = 1;
            input_channel_power_mW = 0.0;
            searching_input_channel_power_mW = 0.0;
            searching_pause = 0;
            tracking_pause = 0;
            no_path_timer = 0;

            corr_result_available_retval = false;

            ranging_update_called             = false;
            corr_restart_correlation_called   = false;
            corr_change_mode_called           = false;
            corr_change_start_called          = false;
            corr_enter_low_power_mode_called  = false;
            corr_exit_low_power_mode_called   = false;
            adc_enter_low_power_mode_called   = false;
            adc_exit_low_power_mode_called    = false;
            radio_enter_low_power_mode_called = false;
            radio_exit_low_power_mode_called  = false;
            
            corr_change_mode_m = static_cast<Correlator_4g::mode_t>(-1);
            corr_change_mode_start_offset_samples = Design::SAMPLES_PER_4G_SRS_PERIOD;

            iter_res_t iterate_retval = iterate();
            
            debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                   \"%s\"\n" )
                                            % test_get_mode_name(all_modes[mode_num]);
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be         0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be      0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be        1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be    0.000000 mW\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be        0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be         0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be 0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be \"WAITING\"\n" );

            test_print_instance_state(iterate_retval);
        }

        //
        //  A correlation result with no peaks is available, ranging is required.
        //
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>> Case  2) A correlation result with no peaks is available, ranging is required.\n" );

        for (size_t mode_num = 0 ; mode_num < sizeof(all_modes)/sizeof(all_modes[0]) ; ++mode_num)
        {
            mode = all_modes[mode_num];
            path_list.clear();
            if ((IDLE == all_modes[mode_num]) || (SEARCHING == all_modes[mode_num]))
            {
                corr_start = 0;
            }
            else
            {
                corr_start = 100;
            }
            path_id_counter = 1;
            input_channel_power_mW = 0.0;
            searching_input_channel_power_mW = 0.01;
            searching_pause = 0;
            tracking_pause = 0;
            no_path_timer = 0;

            corr_result_available_retval = true;
                    
            corr_read_results_peak_counts.clear();
            corr_read_results_peak_counts.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0);
            corr_read_results_peak_vals.clear();
            corr_read_results_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            corr_read_results_peak_mean_inputs_adc_lsb_sq.clear();
            corr_read_results_peak_mean_inputs_adc_lsb_sq.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            
            corr_read_results_inp_max_mean_adc_lsb_sq = 10.0*pow(10.0, +5.0/10.0);
            corr_read_results_inp_mean_adc_lsb_sq = 1.0;
            
            ranging_update_called             = false;
            corr_restart_correlation_called   = false;
            corr_change_mode_called           = false;
            corr_change_start_called          = false;
            corr_enter_low_power_mode_called  = false;
            corr_exit_low_power_mode_called   = false;
            adc_enter_low_power_mode_called   = false;
            adc_exit_low_power_mode_called    = false;
            radio_enter_low_power_mode_called = false;
            radio_exit_low_power_mode_called  = false;
            
            corr_change_mode_m = static_cast<Correlator_4g::mode_t>(-1);
            corr_change_mode_start_offset_samples = Design::SAMPLES_PER_4G_SRS_PERIOD;
            
            ranging_update_retval = Ranging::RANGE_UNSTABLE;
            
            iter_res_t iterate_retval = iterate();
            
            debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
            if (TRACKING == all_modes[mode_num])
            {
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                   \"%s\"\n" )
                                                % test_get_mode_name(PRE_TRACKING_RANGING);
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be         0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be      100\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be        1\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be    0.001000 mW\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be        0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be         0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be 0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be          10.000000\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should not be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator mode should be         \"CONT_TRACK\"\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator start offset should be 0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should not be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be \"NEW_RESULT_RANGING\"\n" );
            } else
            if (PRE_TRACKING_RANGING == all_modes[mode_num])
            {
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                   \"%s\"\n" )
                                                % test_get_mode_name(TRACKING_RANGING);
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be         0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be      100\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be        1\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be    0.001000 mW\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be        0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be         0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be 0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be          10.000000\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should not be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator mode should be         \"TRACK\"\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator start offset should be 100\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should not be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be \"NEW_RESULT_RANGING\"\n" );
            } else
            if (TRACKING_RANGING == all_modes[mode_num])
            {
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                   \"%s\"\n" )
                                                % test_get_mode_name(TRACKING_RANGING);
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be         0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be      100\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be        1\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be    0.001000 mW\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be        0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be         0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be 0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be          10.000000\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should not be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator mode should be         \"TRACK\"\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator start offset should be 100\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should not be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be \"NEW_RESULT_RANGING\"\n" );
            }
            else
            {
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                   \"%s\"\n" )
                                                % test_get_mode_name(all_modes[mode_num]);
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be         0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be      0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be        1\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be    0.000000 mW\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be        0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be         0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be 0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be          10.000000\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should not be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should not be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be \"WAITING\"\n" );
            }
            
            test_print_instance_state(iterate_retval);
        }

        //
        //  A correlation result with two peaks is available, ranging is required.
        //
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>> Case  3) A correlation result with two peaks is available, ranging is required.\n" );

        for (size_t mode_num = 0 ; mode_num < sizeof(all_modes)/sizeof(all_modes[0]) ; ++mode_num)
        {
            mode = all_modes[mode_num];
            path_list.clear();
            if ((IDLE == all_modes[mode_num]) || (SEARCHING == all_modes[mode_num]))
            {
                corr_start = 0;
            }
            else
            {
                corr_start = 100;
            }
            path_id_counter = 1;
            input_channel_power_mW = 0.0;
            searching_input_channel_power_mW = 0.0;
            searching_pause = 0;
            tracking_pause = 0;
            no_path_timer = 0;

            corr_result_available_retval = true;
                    
            corr_read_results_peak_counts.clear();
            corr_read_results_peak_counts.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0);
            corr_read_results_peak_counts[100] = 1;
            corr_read_results_peak_counts[105] = 1;
            corr_read_results_peak_vals.clear();
            corr_read_results_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            corr_read_results_peak_vals[100] = 1.0;
            corr_read_results_peak_vals[105] = 0.7;
            corr_read_results_peak_mean_inputs_adc_lsb_sq.clear();
            corr_read_results_peak_mean_inputs_adc_lsb_sq.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            corr_read_results_peak_mean_inputs_adc_lsb_sq[100] =  9.0;
            corr_read_results_peak_mean_inputs_adc_lsb_sq[105] = 11.0;
            corr_read_results_inp_max_mean_adc_lsb_sq = 10.0*pow(10.0, +5.0/10.0);
            corr_read_results_inp_mean_adc_lsb_sq = 2.0;
            
            ranging_update_called             = false;
            corr_restart_correlation_called   = false;
            corr_change_mode_called           = false;
            corr_change_start_called          = false;
            corr_enter_low_power_mode_called  = false;
            corr_exit_low_power_mode_called   = false;
            adc_enter_low_power_mode_called   = false;
            adc_exit_low_power_mode_called    = false;
            radio_enter_low_power_mode_called = false;
            radio_exit_low_power_mode_called  = false;
            
            corr_change_mode_m = static_cast<Correlator_4g::mode_t>(-1);
            corr_change_mode_start_offset_samples = Design::SAMPLES_PER_4G_SRS_PERIOD;
            
            ranging_update_retval = Ranging::RANGE_UNSTABLE;
            
            iter_res_t iterate_retval = iterate();

            debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
            if (TRACKING == all_modes[mode_num])
            {
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                   \"%s\"\n" )
                                                % test_get_mode_name(PRE_TRACKING_RANGING);
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be         2\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> Paths:\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>    2    -4     105    0.011000\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>    1    -4     100    0.009000\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be      100\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be        3\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be    0.002000 mW\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be        0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be         0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be 0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be          10.000000\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should not be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator mode should be         \"CONT_TRACK\"\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator start offset should be 0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should not be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be \"NEW_RESULT_RANGING\"\n" );
            } else
            if (PRE_TRACKING_RANGING == all_modes[mode_num])
            {
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                   \"%s\"\n" )
                                                % test_get_mode_name(TRACKING_RANGING);
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be         2\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> Paths:\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>    2    -4     105    0.009000\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>    1    -4     100    0.007000\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be      100\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be        3\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be    0.002000 mW\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be        0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be         0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be 0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be          10.000000\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should not be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator mode should be         \"TRACK\"\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator start offset should be 100\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should not be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be \"NEW_RESULT_RANGING\"\n" );
            } else
            if (TRACKING_RANGING == all_modes[mode_num])
            {
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                   \"%s\"\n" )
                                                % test_get_mode_name(TRACKING_RANGING);
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be         2\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> Paths:\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>    2    -4     105    0.011000\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>    1    -4     100    0.009000\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be      100\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be        3\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be    0.002000 mW\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be        0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be         0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be 0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be          10.000000\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should not be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator mode should be         \"TRACK\"\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator start offset should be 100\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should not be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be \"NEW_RESULT_RANGING\"\n" );
            }
            else
            {
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                   \"%s\"\n" )
                                                % test_get_mode_name(all_modes[mode_num]);
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be         0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be      0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be        1\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be    0.000000 mW\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be        0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be         0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be 0\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be          10.000000\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should not be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should not be called\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
                debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be \"WAITING\"\n" );
            }
            
            test_print_instance_state(iterate_retval);
        }

        //
        //  A correlation result is available, ranging is stable, IDLE -> SEARCHING.
        //
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>> Case  4) A correlation result is available, ranging is stable, IDLE -> SEARCHING.\n" );

        {
            mode = IDLE;
            path_list.clear();
            corr_start = 0;
            path_id_counter = 1;
            input_channel_power_mW = 0.0;
            searching_input_channel_power_mW = 0.0;
            searching_pause = 0;
            tracking_pause = 0;
            no_path_timer = 0;

            corr_result_available_retval = true;
                    
            corr_read_results_peak_counts.clear();
            corr_read_results_peak_counts.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0);
            corr_read_results_peak_vals.clear();
            corr_read_results_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            corr_read_results_peak_mean_inputs_adc_lsb_sq.clear();
            corr_read_results_peak_mean_inputs_adc_lsb_sq.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            corr_read_results_inp_max_mean_adc_lsb_sq = 10.0*pow(10.0, +5.0/10.0);
            corr_read_results_inp_mean_adc_lsb_sq = 1.0;
            
            ranging_update_called             = false;
            corr_restart_correlation_called   = false;
            corr_change_mode_called           = false;
            corr_change_start_called          = false;
            corr_enter_low_power_mode_called  = false;
            corr_exit_low_power_mode_called   = false;
            adc_enter_low_power_mode_called   = false;
            adc_exit_low_power_mode_called    = false;
            radio_enter_low_power_mode_called = false;
            radio_exit_low_power_mode_called  = false;
            
            corr_change_mode_m = static_cast<Correlator_4g::mode_t>(-1);
            corr_change_mode_start_offset_samples = Design::SAMPLES_PER_4G_SRS_PERIOD;
            
            ranging_update_retval = Ranging::RANGE_STABLE;
            
            iter_res_t iterate_retval = iterate();
            
            debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                    \"SEARCHING\"\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be          0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be       0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be         1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be     0.000000 mW\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be         0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be          0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be  0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be           10.000000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator mode should be         \"SEARCH\"\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator start offset should be 0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be  \"START_SEARCH\"\n" );
            
            test_print_instance_state(iterate_retval);
        }

        //
        //  A correlation result is available without peaks, ranging is stable, SEARCHING -> SEARCHING without a pause.
        //
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>> Case  5) A correlation result is available without peaks, ranging is stable, SEARCHING -> SEARCHING without a pause.\n" );

        {
            mode = SEARCHING;
            path_list.clear();
            corr_start = 0;
            path_id_counter = 1;
            input_channel_power_mW = 0.0;
            searching_input_channel_power_mW = 0.0;
            searching_pause = 0;
            tracking_pause = 0;
            no_path_timer = 0;

            corr_result_available_retval = true;
                    
            corr_read_results_peak_counts.clear();
            corr_read_results_peak_counts.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0);
            corr_read_results_peak_vals.clear();
            corr_read_results_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            corr_read_results_peak_mean_inputs_adc_lsb_sq.clear();
            corr_read_results_peak_mean_inputs_adc_lsb_sq.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            corr_read_results_inp_max_mean_adc_lsb_sq = 10.0*pow(10.0, +5.0/10.0);
            corr_read_results_inp_mean_adc_lsb_sq = 1.0;
            
            ranging_update_called             = false;
            corr_restart_correlation_called   = false;
            corr_change_mode_called           = false;
            corr_change_start_called          = false;
            corr_enter_low_power_mode_called  = false;
            corr_exit_low_power_mode_called   = false;
            adc_enter_low_power_mode_called   = false;
            adc_exit_low_power_mode_called    = false;
            radio_enter_low_power_mode_called = false;
            radio_exit_low_power_mode_called  = false;
            
            corr_change_mode_m = static_cast<Correlator_4g::mode_t>(-1);
            corr_change_mode_start_offset_samples = Design::SAMPLES_PER_4G_SRS_PERIOD;
            
            ranging_update_retval = Ranging::RANGE_STABLE;
            
            iter_res_t iterate_retval = iterate();
            
            debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                    \"SEARCHING\"\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be          0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be       0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be         1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be     0.000000 mW\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be         0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be          0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be  0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be           10.000000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be  \"WAITING\"\n" );
            
            test_print_instance_state(iterate_retval);
        }

        //
        //  A correlation result is available without peaks, ranging is stable, SEARCHING -> SEARCHING with a 100 ms pause.
        //
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>> Case  6) A correlation result is available without peaks, ranging is stable, SEARCHING -> SEARCHING with a 100 ms pause.\n" );

        {
            mode = SEARCHING;
            path_list.clear();
            corr_start = 0;
            path_id_counter = 1;
            input_channel_power_mW = 0.0;
            searching_input_channel_power_mW = 0.0;
            searching_pause = 10;
            tracking_pause = 0;
            no_path_timer = 0;

            corr_result_available_retval = true;
                    
            corr_read_results_peak_counts.clear();
            corr_read_results_peak_counts.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0);
            corr_read_results_peak_vals.clear();
            corr_read_results_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            corr_read_results_peak_mean_inputs_adc_lsb_sq.clear();
            corr_read_results_peak_mean_inputs_adc_lsb_sq.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            corr_read_results_inp_max_mean_adc_lsb_sq = 10.0*pow(10.0, +5.0/10.0);
            corr_read_results_inp_mean_adc_lsb_sq = 1.0;
            
            ranging_update_called             = false;
            corr_restart_correlation_called   = false;
            corr_change_mode_called           = false;
            corr_change_start_called          = false;
            corr_enter_low_power_mode_called  = false;
            corr_exit_low_power_mode_called   = false;
            adc_enter_low_power_mode_called   = false;
            adc_exit_low_power_mode_called    = false;
            radio_enter_low_power_mode_called = false;
            radio_exit_low_power_mode_called  = false;
            
            corr_change_mode_m = static_cast<Correlator_4g::mode_t>(-1);
            corr_change_mode_start_offset_samples = Design::SAMPLES_PER_4G_SRS_PERIOD;
            
            ranging_update_retval = Ranging::RANGE_STABLE;
            
            iter_res_t iterate_retval = iterate();
            
            debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                    \"SEARCHING\"\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be          0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be       0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be         1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be     0.000000 mW\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be         10\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be          0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be  0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be           10.000000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should be put into and then taken out of low-power mode\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be  \"WAITING\"\n" );
            
            test_print_instance_state(iterate_retval);
        }

        //
        //  A correlation result is available with a single peak at the start of the correlation window, ranging is stable, SEARCHING -> TRACKING.
        //
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>> Case  7) A correlation result is available with a single peak at the start of the correlation window, ranging is stable, SEARCHING -> TRACKING.\n" );

        {
            mode = SEARCHING;
            path_list.clear();
            corr_start = 0;
            path_id_counter = 1;
            input_channel_power_mW = 0.0;
            searching_input_channel_power_mW = 0.0;
            searching_pause = 0;
            tracking_pause = 0;
            no_path_timer = 500;

            corr_result_available_retval = true;
                    
            corr_read_results_peak_counts.clear();
            corr_read_results_peak_counts.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0);
            corr_read_results_peak_vals.clear();
            corr_read_results_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            corr_read_results_peak_mean_inputs_adc_lsb_sq.clear();
            corr_read_results_peak_mean_inputs_adc_lsb_sq.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            
            size_t peak_offset = 0;
            corr_read_results_peak_counts[peak_offset] = 1;
            corr_read_results_peak_vals[peak_offset] = 1.0;
            corr_read_results_peak_mean_inputs_adc_lsb_sq[peak_offset] = 9.0;
            
            corr_read_results_inp_max_mean_adc_lsb_sq = 10.0*pow(10.0, +5.0/10.0);
            corr_read_results_inp_mean_adc_lsb_sq = 1.0;
            
            ranging_update_called             = false;
            corr_restart_correlation_called   = false;
            corr_change_mode_called           = false;
            corr_change_start_called          = false;
            corr_enter_low_power_mode_called  = false;
            corr_exit_low_power_mode_called   = false;
            adc_enter_low_power_mode_called   = false;
            adc_exit_low_power_mode_called    = false;
            radio_enter_low_power_mode_called = false;
            radio_exit_low_power_mode_called  = false;
            
            corr_change_mode_m = static_cast<Correlator_4g::mode_t>(-1);
            corr_change_mode_start_offset_samples = Design::SAMPLES_PER_4G_SRS_PERIOD;
            
            ranging_update_retval = Ranging::RANGE_STABLE;
            
            iter_res_t iterate_retval = iterate();
            
            debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                    \"TRACKING\"\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be          0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be       9378\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be         1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be     0.000000 mW\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be         0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be          0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be  0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be          10.000000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator mode should be         \"TRACK\"\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator start offset should be 9378\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be  \"START_TRACK\"\n" );
            
            test_print_instance_state(iterate_retval);
        }

        //
        //  A correlation result is available with a single peak in the middle of the correlation window, ranging is stable, SEARCHING -> TRACKING.
        //
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>> Case  8) A correlation result is available with a single peak in the middle of the correlation window, ranging is stable, SEARCHING -> TRACKING.\n" );

        {
            mode = SEARCHING;
            path_list.clear();
            corr_start = 0;
            path_id_counter = 1;
            input_channel_power_mW = 0.0;
            searching_input_channel_power_mW = 0.0;
            searching_pause = 0;
            tracking_pause = 0;
            no_path_timer = 500;

            corr_result_available_retval = true;
                    
            corr_read_results_peak_counts.clear();
            corr_read_results_peak_counts.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0);
            corr_read_results_peak_vals.clear();
            corr_read_results_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            corr_read_results_peak_mean_inputs_adc_lsb_sq.clear();
            corr_read_results_peak_mean_inputs_adc_lsb_sq.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            
            size_t peak_offset = 1222;
            corr_read_results_peak_counts[peak_offset] = 1;
            corr_read_results_peak_vals[peak_offset] = 1.0;
            corr_read_results_peak_mean_inputs_adc_lsb_sq[peak_offset] = 8.0;

            corr_read_results_inp_max_mean_adc_lsb_sq = 10.0*pow(10.0, +5.0/10.0);
            corr_read_results_inp_mean_adc_lsb_sq = 1.0;
            
            ranging_update_called             = false;
            corr_restart_correlation_called   = false;
            corr_change_mode_called           = false;
            corr_change_start_called          = false;
            corr_enter_low_power_mode_called  = false;
            corr_exit_low_power_mode_called   = false;
            adc_enter_low_power_mode_called   = false;
            adc_exit_low_power_mode_called    = false;
            radio_enter_low_power_mode_called = false;
            radio_exit_low_power_mode_called  = false;
            
            corr_change_mode_m = static_cast<Correlator_4g::mode_t>(-1);
            corr_change_mode_start_offset_samples = Design::SAMPLES_PER_4G_SRS_PERIOD;
            
            ranging_update_retval = Ranging::RANGE_STABLE;
            
            iter_res_t iterate_retval = iterate();
            
            debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                    \"TRACKING\"\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be          0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be       1000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be         1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be     0.000000 mW\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be         0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be          0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be  0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be          10.000000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator mode should be         \"TRACK\"\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator start offset should be 1000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be  \"START_TRACK\"\n" );
            
            test_print_instance_state(iterate_retval);
        }

        //
        //  A correlation result is available with no peaks and the path list is empty, ranging is stable, the timeout timer is low, TRACKING -> TRACKING.
        //
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>> Case  9) A correlation result is available with no peaks and the path list is empty, ranging is stable, the timeout timer is low, TRACKING -> TRACKING.\n" );

        {
            mode = TRACKING;
            path_list.clear();
            corr_start = 1000;
            path_id_counter = 1;
            input_channel_power_mW = 0.0;
            searching_input_channel_power_mW = 0.0;
            searching_pause = 0;
            tracking_pause = 0;
            no_path_timer = 108;

            corr_result_available_retval = true;
                    
            corr_read_results_peak_counts.clear();
            corr_read_results_peak_counts.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0);
            corr_read_results_peak_vals.clear();
            corr_read_results_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            corr_read_results_peak_mean_inputs_adc_lsb_sq.clear();
            corr_read_results_peak_mean_inputs_adc_lsb_sq.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);

            corr_read_results_inp_max_mean_adc_lsb_sq = 4.0*pow(10.0, +5.0/10.0);
            corr_read_results_inp_mean_adc_lsb_sq = 2.0;
            
            ranging_update_called             = false;
            corr_restart_correlation_called   = false;
            corr_change_mode_called           = false;
            corr_change_start_called          = false;
            corr_enter_low_power_mode_called  = false;
            corr_exit_low_power_mode_called   = false;
            adc_enter_low_power_mode_called   = false;
            adc_exit_low_power_mode_called    = false;
            radio_enter_low_power_mode_called = false;
            radio_exit_low_power_mode_called  = false;
            
            corr_change_mode_m = static_cast<Correlator_4g::mode_t>(-1);
            corr_change_mode_start_offset_samples = Design::SAMPLES_PER_4G_SRS_PERIOD;
            
            ranging_update_retval = Ranging::RANGE_STABLE;
            
            iter_res_t iterate_retval = iterate();
            
            debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                    \"TRACKING\"\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be          0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be       1000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be         1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be     0.002000 mW\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be         0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be          0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be  109\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be           4.000000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator start offset should be 1000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be  \"NEW_RESULT\"\n" );
            
            test_print_instance_state(iterate_retval);
        }

        //
        //  A correlation result is available with no peaks and the path list is empty, ranging is stable, the timeout timer is at the limit, TRACKING -> SEARCHING.
        //
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>> Case 10) A correlation result is available with no peaks and the path list is empty, ranging is stable, the timeout timer is low, TRACKING -> TRACKING.\n" );

        {
            mode = TRACKING;
            path_list.clear();
            corr_start = 1000;
            path_id_counter = 1;
            input_channel_power_mW = 0.0;
            searching_input_channel_power_mW = 0.0;
            searching_pause = 0;
            tracking_pause = 0;
            no_path_timer = 109;

            corr_result_available_retval = true;
                    
            corr_read_results_peak_counts.clear();
            corr_read_results_peak_counts.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0);
            corr_read_results_peak_vals.clear();
            corr_read_results_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            corr_read_results_peak_mean_inputs_adc_lsb_sq.clear();
            corr_read_results_peak_mean_inputs_adc_lsb_sq.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);

            corr_read_results_inp_max_mean_adc_lsb_sq = 4.0*pow(10.0, +5.0/10.0);
            corr_read_results_inp_mean_adc_lsb_sq = 2.0;
            
            ranging_update_called             = false;
            corr_restart_correlation_called   = false;
            corr_change_mode_called           = false;
            corr_change_start_called          = false;
            corr_enter_low_power_mode_called  = false;
            corr_exit_low_power_mode_called   = false;
            adc_enter_low_power_mode_called   = false;
            adc_exit_low_power_mode_called    = false;
            radio_enter_low_power_mode_called = false;
            radio_exit_low_power_mode_called  = false;
            
            corr_change_mode_m = static_cast<Correlator_4g::mode_t>(-1);
            corr_change_mode_start_offset_samples = Design::SAMPLES_PER_4G_SRS_PERIOD;
            
            ranging_update_retval = Ranging::RANGE_STABLE;
            
            iter_res_t iterate_retval = iterate();
            
            debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                    \"SEARCHING\"\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be          0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be       1000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be         1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be     0.002000 mW\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be         0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be          0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be  110\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be           4.000000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator mode should be         \"SEARCH\"\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator start offset should be 0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be  \"START_SEARCH\"\n" );
            
            test_print_instance_state(iterate_retval);
        }

        //
        //  A correlation result is available with one peak and the path list is empty, ranging is stable, the timeout timer is zero, TRACKING -> TRACKING.
        //
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>> Case 11) A correlation result is available with one peak and the path list is empty, ranging is stable, the timeout timer is at the limit, TRACKING -> TRACKING.\n" );

        {
            mode = TRACKING;
            path_list.clear();
            corr_start = 1000;
            path_id_counter = 1;
            input_channel_power_mW = 0.0;
            searching_input_channel_power_mW = 0.0;
            searching_pause = 0;
            tracking_pause = 0;
            no_path_timer = 0;

            corr_result_available_retval = true;
                    
            corr_read_results_peak_counts.clear();
            corr_read_results_peak_counts.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0);
            corr_read_results_peak_vals.clear();
            corr_read_results_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            corr_read_results_peak_mean_inputs_adc_lsb_sq.clear();
            corr_read_results_peak_mean_inputs_adc_lsb_sq.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            
            size_t peak_offset = 1063;
            corr_read_results_peak_counts[peak_offset] = 1;
            corr_read_results_peak_vals[peak_offset] = 1.0;
            corr_read_results_peak_mean_inputs_adc_lsb_sq[peak_offset] = 13.0;

            corr_read_results_inp_max_mean_adc_lsb_sq = 4.0*pow(10.0, +5.0/10.0);
            corr_read_results_inp_mean_adc_lsb_sq = 2.2;
            
            ranging_update_called             = false;
            corr_restart_correlation_called   = false;
            corr_change_mode_called           = false;
            corr_change_start_called          = false;
            corr_enter_low_power_mode_called  = false;
            corr_exit_low_power_mode_called   = false;
            adc_enter_low_power_mode_called   = false;
            adc_exit_low_power_mode_called    = false;
            radio_enter_low_power_mode_called = false;
            radio_exit_low_power_mode_called  = false;
            
            corr_change_mode_m = static_cast<Correlator_4g::mode_t>(-1);
            corr_change_mode_start_offset_samples = Design::SAMPLES_PER_4G_SRS_PERIOD;
            
            ranging_update_retval = Ranging::RANGE_STABLE;
            
            iter_res_t iterate_retval = iterate();
            
            debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                    \"TRACKING\"\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be          1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> Paths:\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>    1    -4    1063    0.013000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be       1000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be         2\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be     0.002200 mW\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be         0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be          0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be  1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be            4.000000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator start offset should be 1000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be  \"NEW_RESULT\"\n" );
            
            test_print_instance_state(iterate_retval);
        }

        //
        //  A correlation result is available with one peak and the path list has a similar detection, ranging is stable, the timeout timer is low, TRACKING -> TRACKING (the timeout counter is reset).
        //
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>> Case 12) A correlation result is available with one peak and the path list has a similar detection, ranging is stable, the timeout timer is low, TRACKING -> TRACKING (the timeout counter is reset).\n" );

        {
            mode = TRACKING;
            
            path_list.clear();
            unsigned path_offset = 1062;
            struct Path_t path = { 0, VALID_COUNT_CONFIRMED-VALID_COUNT_PATH_FOUND, path_offset, 0.001 };
            path_list.push_back(path);
            
            corr_start = 1000;
            path_id_counter = 1;
            input_channel_power_mW = 0.0;
            searching_input_channel_power_mW = 0.0;
            searching_pause = 0;
            tracking_pause = 0;
            no_path_timer = 4;

            corr_result_available_retval = true;
                    
            corr_read_results_peak_counts.clear();
            corr_read_results_peak_counts.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0);
            corr_read_results_peak_vals.clear();
            corr_read_results_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            corr_read_results_peak_mean_inputs_adc_lsb_sq.clear();
            corr_read_results_peak_mean_inputs_adc_lsb_sq.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            
            size_t peak_offset = 1064;
            corr_read_results_peak_counts[peak_offset] = 1;
            corr_read_results_peak_vals[peak_offset] = 1.0;
            corr_read_results_peak_mean_inputs_adc_lsb_sq[peak_offset] = 13.0;

            corr_read_results_inp_max_mean_adc_lsb_sq = 4.0*pow(10.0, +5.0/10.0);
            corr_read_results_inp_mean_adc_lsb_sq = 2.3;
            
            ranging_update_called             = false;
            corr_restart_correlation_called   = false;
            corr_change_mode_called           = false;
            corr_change_start_called          = false;
            corr_enter_low_power_mode_called  = false;
            corr_exit_low_power_mode_called   = false;
            adc_enter_low_power_mode_called   = false;
            adc_exit_low_power_mode_called    = false;
            radio_enter_low_power_mode_called = false;
            radio_exit_low_power_mode_called  = false;
            
            corr_change_mode_m = static_cast<Correlator_4g::mode_t>(-1);
            corr_change_mode_start_offset_samples = Design::SAMPLES_PER_4G_SRS_PERIOD;
            
            ranging_update_retval = Ranging::RANGE_STABLE;
            
            iter_res_t iterate_retval = iterate();
            
            debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                    \"TRACKING\"\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be          1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> Paths:\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>    0     0    1064    0.007000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be       842\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be         1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be     0.002300 mW\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be         0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be          0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be  0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be            4.000000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator start offset should be 842\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be  \"NEW_RESULT\"\n" );
            
            test_print_instance_state(iterate_retval);
        }

        //
        //  A correlation result is available with one peak and the path list has a similar detection, ranging is stable, the timeout timer is low, TRACKING -> TRACKING (the timeout counter is decremented).
        //
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>> Case 13) A correlation result is available with one peak and the path list has a similar detection, ranging is stable, the timeout timer is low, TRACKING -> TRACKING (the timeout counter is decremented).\n" );

        {
            mode = TRACKING;
            
            path_list.clear();
            unsigned path_offset = 1062;
            struct Path_t path = { 0, VALID_COUNT_CONFIRMED-VALID_COUNT_PATH_FOUND, path_offset, 0.001 };
            path_list.push_back(path);
            
            corr_start = 1000;
            path_id_counter = 1;
            input_channel_power_mW = 0.0;
            searching_input_channel_power_mW = 0.0;
            searching_pause = 0;
            tracking_pause = 0;
            no_path_timer = 6;

            corr_result_available_retval = true;
                    
            corr_read_results_peak_counts.clear();
            corr_read_results_peak_counts.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0);
            corr_read_results_peak_vals.clear();
            corr_read_results_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            corr_read_results_peak_mean_inputs_adc_lsb_sq.clear();
            corr_read_results_peak_mean_inputs_adc_lsb_sq.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            
            size_t peak_offset = 1064;
            corr_read_results_peak_counts[peak_offset] = 1;
            corr_read_results_peak_vals[peak_offset] = 1.0;
            corr_read_results_peak_mean_inputs_adc_lsb_sq[peak_offset] = 13.0;

            corr_read_results_inp_max_mean_adc_lsb_sq = 4.0*pow(10.0, +5.0/10.0);
            corr_read_results_inp_mean_adc_lsb_sq = 2.3;
            
            ranging_update_called             = false;
            corr_restart_correlation_called   = false;
            corr_change_mode_called           = false;
            corr_change_start_called          = false;
            corr_enter_low_power_mode_called  = false;
            corr_exit_low_power_mode_called   = false;
            adc_enter_low_power_mode_called   = false;
            adc_exit_low_power_mode_called    = false;
            radio_enter_low_power_mode_called = false;
            radio_exit_low_power_mode_called  = false;
            
            corr_change_mode_m = static_cast<Correlator_4g::mode_t>(-1);
            corr_change_mode_start_offset_samples = Design::SAMPLES_PER_4G_SRS_PERIOD;
            
            ranging_update_retval = Ranging::RANGE_STABLE;
            
            iter_res_t iterate_retval = iterate();
            
            debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                    \"TRACKING\"\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be          1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> Paths:\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>    0     0    1064    0.007000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be       842\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be         1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be     0.002300 mW\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be         0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be          0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be  1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be            4.000000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator start offset should be 842\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be  \"NEW_RESULT\"\n" );
            
            test_print_instance_state(iterate_retval);
        }

        //
        //  A correlation result is available with one peak and the path list has a similar detection, ranging is stable, the timeout timer is low, TRACKING -> TRACKING_PAUSE.
        //
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>> Case 14) A correlation result is available with one peak and the path list has a similar detection, ranging is stable, the timeout timer is low, TRACKING -> TRACKING_PAUSE.\n" );

        {
            mode = TRACKING;
            
            path_list.clear();
            unsigned path_offset = 1062;
            struct Path_t path = { 0, VALID_COUNT_CONFIRMED-VALID_COUNT_PATH_FOUND, path_offset, 0.001 };
            path_list.push_back(path);
            
            corr_start = 1000;
            path_id_counter = 1;
            input_channel_power_mW = 0.0;
            searching_input_channel_power_mW = 0.0;
            searching_pause = 0;
            tracking_pause = 20;
            no_path_timer = 5;

            corr_result_available_retval = true;
                    
            corr_read_results_peak_counts.clear();
            corr_read_results_peak_counts.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0);
            corr_read_results_peak_vals.clear();
            corr_read_results_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            corr_read_results_peak_mean_inputs_adc_lsb_sq.clear();
            corr_read_results_peak_mean_inputs_adc_lsb_sq.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            
            size_t peak_offset = 1064;
            corr_read_results_peak_counts[peak_offset] = 1;
            corr_read_results_peak_vals[peak_offset] = 1.0;
            corr_read_results_peak_mean_inputs_adc_lsb_sq[peak_offset] = 13.0;

            corr_read_results_inp_max_mean_adc_lsb_sq = 4.0*pow(10.0, +5.0/10.0);
            corr_read_results_inp_mean_adc_lsb_sq = 2.3;
            
            ranging_update_called             = false;
            corr_restart_correlation_called   = false;
            corr_change_mode_called           = false;
            corr_change_start_called          = false;
            corr_enter_low_power_mode_called  = false;
            corr_exit_low_power_mode_called   = false;
            adc_enter_low_power_mode_called   = false;
            adc_exit_low_power_mode_called    = false;
            radio_enter_low_power_mode_called = false;
            radio_exit_low_power_mode_called  = false;
            
            corr_change_mode_m = static_cast<Correlator_4g::mode_t>(-1);
            corr_change_mode_start_offset_samples = Design::SAMPLES_PER_4G_SRS_PERIOD;
            
            ranging_update_retval = Ranging::RANGE_STABLE;
            
            iter_res_t iterate_retval = iterate();
            
            debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                    \"TRACKING\"\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be          1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> Paths:\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>    0     0    1064    0.007000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be       842\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be         1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be     0.002300 mW\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be         0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be          20\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be  0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be            4.000000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator start offset should be 842\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be  \"NEW_RESULT\"\n" );
            
            test_print_instance_state(iterate_retval);
        }

        //
        //  A correlation result is available with one peak and the path list has a similar detection, ranging is stable, the timeout timer is zero, TRACKING_RANGING -> TRACKING.
        //
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
        debugStr(DebugStream::info4) <<   boost::format(  ">>> Case 15) A correlation result is available with one peak and the path list has a similar detection, ranging is stable, the timeout timer is zero, TRACKING_RANGING -> TRACKING.\n" );

        {
            mode = TRACKING_RANGING;
            
            path_list.clear();
            unsigned path_offset = 1062;
            struct Path_t path = { 0, VALID_COUNT_CONFIRMED-VALID_COUNT_PATH_FOUND, path_offset, 0.001 };
            path_list.push_back(path);
            
            corr_start = 1000;
            path_id_counter = 1;
            input_channel_power_mW = 0.0;
            searching_input_channel_power_mW = 0.0;
            searching_pause = 0;
            tracking_pause = 20;
            no_path_timer = 0;

            corr_result_available_retval = true;
                    
            corr_read_results_peak_counts.clear();
            corr_read_results_peak_counts.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0);
            corr_read_results_peak_vals.clear();
            corr_read_results_peak_vals.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            corr_read_results_peak_mean_inputs_adc_lsb_sq.clear();
            corr_read_results_peak_mean_inputs_adc_lsb_sq.resize(Design::SAMPLES_PER_4G_SRS_PERIOD, 0.0);
            
            size_t peak_offset = 1064;
            corr_read_results_peak_counts[peak_offset] = 1;
            corr_read_results_peak_vals[peak_offset] = 1.0;
            corr_read_results_peak_mean_inputs_adc_lsb_sq[peak_offset] = 13.0;

            corr_read_results_inp_max_mean_adc_lsb_sq = 4.0*pow(10.0, +5.0/10.0);
            corr_read_results_inp_mean_adc_lsb_sq = 2.3;
            
            ranging_update_called             = false;
            corr_restart_correlation_called   = false;
            corr_change_mode_called           = false;
            corr_change_start_called          = false;
            corr_enter_low_power_mode_called  = false;
            corr_exit_low_power_mode_called   = false;
            adc_enter_low_power_mode_called   = false;
            adc_exit_low_power_mode_called    = false;
            radio_enter_low_power_mode_called = false;
            radio_exit_low_power_mode_called  = false;
            
            corr_change_mode_m = static_cast<Correlator_4g::mode_t>(-1);
            corr_change_mode_start_offset_samples = Design::SAMPLES_PER_4G_SRS_PERIOD;
            
            ranging_update_retval = Ranging::RANGE_STABLE;
            
            iter_res_t iterate_retval = iterate();
            
            debugStr(DebugStream::info4) <<   boost::format(  ">>>\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The mode should be                    \"TRACKING\"\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path list size should be          1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> Paths:\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>    0     0    1064    0.007000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The correlation start should be       842\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The path ID counter should be         1\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The input channel power should be     0.002300 mW\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The searching pause should be         0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking pause should be          20\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The tracking timeout timer should be  0\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The ranging_update() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The ranging power should be            4.000000\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_restart_correlation() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_mode() function should be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator mode should be         \"TRACK\"\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>>     The correlator start offset should be 842\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The corr_change_start() function should not be called\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The Correlator, ADC and Radio should not be put into or taken out of low-power mode\n" );
            debugStr(DebugStream::info4) <<   boost::format(  ">>> The iterate() return value should be  \"NEW_RESULT\"\n" );
            
            test_print_instance_state(iterate_retval);
        }
    }
#endif //   SWTEST_4G_SEARCH_TRACK

/***********************************************************************************************************************
 *
 *
 ***********************************************************************************************************************
 *  Filename:   searchandtrack_4g.cpp
 *  Author(s):  pdm
 *******************************************************************************************************************//**
 * File Description
 * ================
 *
 *  @file
 *  @brief          The implementation of ::SearchAndTrack_4g that controls 4G searching and tracking.
 **********************************************************************************************************************/

#include "adc.hpp"
#include "searchandtrack_4g.hpp"
#include "radio.hpp"
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class.
//  @details        Initialises the member variables, setting the mode to IDLE.
//  @param[in]      rangeCtrl           The range control object to use.
//  @param[in]      pwrCal              The power calibration object to use.
//  @param[in]      c                   The correlator object to use.
//  @param[in]      searching_pause     The number of idle frames between searching iterations
//  @param[in]      tracking_pause      The number of idle frames between tracking iterations
//  @param[in]      debug               Controls the debug messages that may be generated.
// 
SearchAndTrack_4g::SearchAndTrack_4g(Ranging                        &rangeCtrl,
                                     PwrCal                         &pwrCal,
                                     std::auto_ptr< Correlator_4g > c,
                                     unsigned                       searching_pause,
                                     unsigned                       tracking_pause,
                                     unsigned                       debug) :
    path_list(),
    the_correlator( c ),
    ranging( rangeCtrl ),
    power_cal( pwrCal ),
    mode( IDLE ),
    corr_start( 0 ),
    path_id_counter( 1 ),
    searching_input_channel_power_mW( 0.0 ),
    input_channel_power_mW( 0.0 ),
    searching_pause( searching_pause ),
    tracking_pause( tracking_pause ),
    no_path_timer( 0 ),
#ifndef SWTEST_4G_SEARCH_TRACK
    debugStr("SearchAndTrack_4g.. ", debug)
#else
    debugStr("SearchAndTrack_4g.. ", debug),
    corr_get_cal_factor_ratio_retval(0.0),
    corr_result_available_retval(false),
    corr_read_results_peak_vals(),
    corr_read_results_peak_mean_inputs_adc_lsb_sq(),
    corr_read_results_inp_max_mean_adc_lsb_sq(0.0),
    corr_read_results_inp_mean_adc_lsb_sq(0.0),
    corr_restart_correlation_called(false),
    corr_change_mode_called(false),
    corr_change_mode_m(Correlator_4g::SEARCH),
    corr_change_mode_start_offset_samples(0),
    corr_change_start_called(false),
    corr_change_start_start_offset_samples(0),
    corr_enter_low_power_mode_called(false),
    corr_exit_low_power_mode_called(false),
    adc_enter_low_power_mode_called(false),
    adc_exit_low_power_mode_called(false),
    radio_enter_low_power_mode_called(false),
    radio_exit_low_power_mode_called(false),
    pwrcal_getFactor_gain_dB(0),
    pwrcal_getFactor_retval(0.0),
    ranging_get_gain_dB_retval(0),
    ranging_update_called(false),
    ranging_update_retval(Ranging::RANGE_UNSTABLE),
    ranging_update_power(0.0)
#endif
    
{
#ifdef SWTEST_4G_SEARCH_TRACK
    test_correct_powers_and_discard_weak_peaks();
    test_find_new_peak_nearby();
    test_annihilate();
    test_update_paths();
    test_calc_new_track_offset();
    test_track();
    test_iterate();
    while (true)
    {
    }
#endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Track peaks as they drift
//  @details        Identify the strongest new peak and calculate a threshold that is used to limit the peaks to be
//                  to the ones most likely to be useful.
// 
//                  Retrieve the power correction factor from the PwrCal class instance and apply it (together with the 
//                  3G to 4G adjustment) to all the power results, eliminating the peaks that fall below the threshold. 
// 
//                  Iterate through the current path list and find the strongest new detection within MAX_OFFSET_DIFF
//                  samples of each path. If there is one use it to update the path timing (adopt the new detection's
//                  timing), power (the average of old and new) and adjust its validity (count incremented by
//                  VALID_COUNT_PATH_FOUND up to VALID_COUNT_MAX_VAL). Finally remove the peak that has been used from
//                  the detection list, and any other peak within MAX_ANNIHILATION_DIFF of it.
// 
//                  If no new detection matches the existing path list entry then adjust its validity (count
//                  incremented by VALID_COUNT_PATH_NOT_FOUND).
// 
//                  If any new detection peaks are left, add them to the path list. Take the strongest first and as each
//                  one is added discard any new peak that is within MAX_ANNIHILATION_DIFF of it.
// 
//                  Remove all paths from the path list that are no longer valid.
// 
//                  Reorder the path list so that the strongest peak is first.
// 
//                  Use the positions in each path of the list to calculate the new correlation start offset. Use a
//                  centre of mass algorithm with the path's power as the weight. (Experiments in 1QS06 indicate that 
//                  has less variance than choosing the strongest. Using the correlation peak value as the weight made
//                  no difference and an unweighted average of the positions seemed slightly poorer at higher SNR.)
// 
//  @param[in]      corr_inp_mean           The mean power of the channel samples over the interval spanned by the
//                                          correlation.
//  @param[in]      corr_peak_vals          The normalised correlation peaks in a result interval.
//  @param[in]      corr_peak_mean_inputs_adc_lsb_sq
//                                          The mean power of the channel samples contributing to peak detections
//                                          in a result interval.
//  @param[in]      cal_factor              The calibration factor to use.
//  @return                                 The new offset for the start of correlation window.
// 
int SearchAndTrack_4g::track(double corr_inp_mean, 
                             std::vector<float> corr_peak_vals, 
                             std::vector<float> corr_peak_mean_inputs,
                             double cal_factor)
{
    //
    //  Correct the powers to produce results for the antenna input, zeroing all those whose correlation peak value is
    //  below the relative threshold.
    //
    correct_powers_and_discard_weak_peaks(corr_inp_mean, 
                                          corr_peak_vals, 
                                          corr_peak_mean_inputs,
                                          cal_factor);
                                          
    input_channel_power_mW = corr_inp_mean;
    
    std::vector<float> &det_burst_powers_mW = corr_peak_mean_inputs;    //  After the correction the values are in mW.

    //
    //  Update the paths, identifying the strongest peak that is close in offset and using them to update the entries,
    //  adjusting the valid count as appropriate (incrementing if a peaks is found, decremented otherwise). Peaks that
    //  have been matched are removed, as are any that are close, but weaker than the one that was used.
    //
    update_paths(corr_peak_vals, det_burst_powers_mW);

    //
    //  Clean up the path list by removing detections that are no longer considered to be valid.
    //
    path_list.remove_if( is_invalid );

    //
    //  Sort the list in order of descending power.
    //
    path_list.sort( descending_power );
    
    //
    //  Calculate the position of the next correlation.
    //
    unsigned new_corr_start = calc_new_track_offset();
    
    return new_corr_start;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Helper function to apply the power correction to produce results for the antenna input and remove
//                  peaks below REL_THRESHOLD_FACTOR relative to the strongest peak.
//  @param[in,out]  corr_inp_mean           The mean power of the channel samples over the interval spanned by the
//                                          correlation. The input power is in units of (ADC LSB)^2 and the output is in
//                                          mW.
//  @param[in,out]  corr_peak_vals          The normalised correlation peaks in a result interval.
//  @param[in]      corr_peak_mean_inputs   The mean power of the channel samples contributing to peak detections
//                                          in a result interval. The input power is in units of (ADC LSB)^2 and the
//                                          output is in mW.
//  @param[in]      cal_factor              The calibration factor to use.
// 
void SearchAndTrack_4g::correct_powers_and_discard_weak_peaks(double &corr_inp_mean, 
                                                              std::vector<float> &corr_peak_vals, 
                                                              std::vector<float> &corr_peak_mean_inputs,
                                                              double cal_factor)
{
    //
    //  Identify the strongest peak and calculate a relative threshold.
    //
    double max_peak_val = *std::max_element(corr_peak_vals.begin(), corr_peak_vals.end());
    double rel_threshold = REL_THRESHOLD_FACTOR*max_peak_val;
    
    //
    //  Correct the powers to produce results for the antenna input, zeroing all those whose correlation peak value is
    //  below the relative threshold.
    //
    corr_inp_mean = cal_factor*corr_inp_mean;
    std::vector<float> det_burst_powers_mW;
    
    unsigned num_peaks = 0;
    unsigned num_peaks_above_rel_threshold = 0;
    for (size_t sample_num = 0 ; sample_num < corr_peak_vals.size() ; ++sample_num)
    {
        if (0.0 < corr_peak_vals[sample_num])
        {
            ++num_peaks;
        }
        
        if (rel_threshold < corr_peak_vals[sample_num])
        {
            //
            //  Subtract the mean search-mode channel power as a reasonable estimate of the noise power, but never allow
            //  the result to be negative. A value of 0.0 might cause problems so make it 1e-18 (-180 dBm).
            //
            double corr_peak_mean_inputs_mW = cal_factor*corr_peak_mean_inputs[sample_num];
            if (corr_peak_mean_inputs_mW > searching_input_channel_power_mW)
            {
                corr_peak_mean_inputs_mW = corr_peak_mean_inputs_mW-searching_input_channel_power_mW;
            }
            else
            {
                corr_peak_mean_inputs_mW = 1e-18;
            }
            
            ++num_peaks_above_rel_threshold;
            corr_peak_mean_inputs[sample_num] = corr_peak_mean_inputs_mW;
        }
        else
        {
            corr_peak_vals[sample_num] = 0.0;
            corr_peak_mean_inputs[sample_num] = 0.0;
        }
    }

    debugStr(DebugStream::info1) <<   boost::format("Original peaks = %u, maximum = %.2f, peaks above relative threshold = %3u\n")
                                    % num_peaks
                                    % max_peak_val
                                    % num_peaks_above_rel_threshold;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Helper function to update the path list, identifying new peaks that match existing paths and adding
//                  any unmatched peaks to the list.
//  @param[in,out]  corr_peak_vals          The normalised correlation peaks in a result interval, updated to remove
//                                          peaks that match existing paths. 
//  @param[in,out]  det_burst_powers_mW     The mean power of the channel samples contributing to peak detections
//                                          in a result interval, updated to remove values for peaks thatmatch existing
//                                          paths.
// 
void SearchAndTrack_4g::update_paths(std::vector<float> &corr_peak_vals, 
                                     std::vector<float> &det_burst_powers_mW)
{
    //
    //  Iterate through the current path list and identify the strongest (in correlation peak terms) new peak within
    //  MAX_OFFSET_DIFF of its position. If at least one non-zero new peak exists update the path list entry.
    //
    BOOST_FOREACH( PathList_t::value_type& path, path_list )
    {
        //
        //  Find the strongest correlation peak close to the existing path position.
        //
        unsigned offset_of_strongest;
        bool peak_found = find_new_peak_nearby(path.offset,
                                               corr_peak_vals,
                                               offset_of_strongest);
        
        //
        //  If the peak has been re-discovered update its entry in the path list and remove it from the samples so that
        //  it does not appear in the search for new peaks.
        //
        if (peak_found)
        {
            debugStr(DebugStream::info1) <<   boost::format("Old peak found: offset = %5u, corr = %e, power = %e\n")
                                            % offset_of_strongest
                                            % corr_peak_vals[offset_of_strongest]
                                            % det_burst_powers_mW[offset_of_strongest];

            path.power       = (path.power+det_burst_powers_mW[offset_of_strongest])/2.0;     // As used in 3G
            path.offset      = offset_of_strongest;
            path.valid_count = path.valid_count+VALID_COUNT_PATH_FOUND;
            if( path.valid_count >= VALID_COUNT_MAX_VAL )
            {
                path.valid_count = VALID_COUNT_MAX_VAL;
            }
            annihilate(offset_of_strongest, corr_peak_vals, det_burst_powers_mW);
        }
        else
        {
            debugStr(DebugStream::info1) <<   boost::format("Old peak not found: path.valid_count = %d\n")
                                            % path.valid_count;

            //
            //  Not found this time, so  reduce the valid_count so eventually path is removed.
            //
            path.valid_count = path.valid_count+VALID_COUNT_PATH_NOT_FOUND;
        }
    }
    
    //
    //  Identify the strongest peak left, and while there is one add it to the path list and remove it from the samples
    //  so that it does not appear again.
    //
    bool found_peak;
    do
    {
        std::vector<float>::iterator iter_at_peak = std::max_element(corr_peak_vals.begin(), corr_peak_vals.end());
        
        found_peak = false;
        if (0.0 < *iter_at_peak)
        {
            found_peak = true;
            
            size_t offset_of_strongest = std::distance(corr_peak_vals.begin(), iter_at_peak);

            debugStr(DebugStream::info1) <<   boost::format("New peak found: offset = %5u, corr = %e, power = %e\n")
                                            % offset_of_strongest
                                            % corr_peak_vals[offset_of_strongest]
                                            % det_burst_powers_mW[offset_of_strongest];

            Path_t new_path;
            new_path.id          = path_id_counter++;
            new_path.valid_count = VALID_COUNT_INIT_VAL;
            new_path.offset      = offset_of_strongest;
            new_path.power       = det_burst_powers_mW[offset_of_strongest];

            path_list.push_back( new_path );

            annihilate(offset_of_strongest, corr_peak_vals, det_burst_powers_mW);
        }
    } while( found_peak );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Helper function to find a new detection peak close to the given offset.
//  @param[in]      offset                  Center point of the search
//  @param[in]      corr_peak_vals          The correlation peak data to search
//  @param[out]     offset_of_strongest     The offset in the input vector of the strongest peak in the range
//  @return                                 true of a peak was found
// 
bool SearchAndTrack_4g::find_new_peak_nearby(unsigned offset, 
                                             const std::vector<float>& corr_peak_vals,
                                             unsigned &offset_of_strongest)
{
    bool peak_found = false;
    offset_of_strongest = 0;
    
    size_t peak_vector_len = corr_peak_vals.size();
    if (offset >= peak_vector_len)
    {
        debugStr(DebugStream::error) <<   boost::format("find_new_peak_nearby : offset outside vector (%u >= %u)\n")
                                        % offset
                                        % peak_vector_len;
        offset = offset % peak_vector_len;
    }

    unsigned new_peak_offset = offset;
    float corr_peak_of_strongest = 0.0;
    if (MAX_OFFSET_DIFF <= new_peak_offset)
    {
        new_peak_offset -= MAX_OFFSET_DIFF;
    }
    else
    {
        new_peak_offset += (peak_vector_len-MAX_OFFSET_DIFF);
    }
    
    for(unsigned offset_num = 0 ; offset_num < (2*MAX_OFFSET_DIFF+1) ; ++offset_num)
    {
        if (corr_peak_of_strongest < corr_peak_vals[new_peak_offset])
        {
            peak_found = true;
            corr_peak_of_strongest = corr_peak_vals[new_peak_offset];
            offset_of_strongest = new_peak_offset;
        }
        ++new_peak_offset;
        if (peak_vector_len <= new_peak_offset)
        {
            new_peak_offset -= peak_vector_len;
        }
    }
        
    return (peak_found);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Helper function to zero results around a point in the correlation result buffers.
//  @param[in]      offset                  Center point of zeroing.
//  @param[in,out]  corr_peak_vals          The correlation peak data to modify.
//  @param[in,out]  det_burst_powers_mW     The correlation burst power data to modify.
// 
void SearchAndTrack_4g::annihilate(unsigned offset, 
                                   std::vector<float>& corr_peak_vals,
                                   std::vector<float>& det_burst_powers_mW )
{
    assert(corr_peak_vals.size() == det_burst_powers_mW.size());

    size_t vector_size = corr_peak_vals.size();

    unsigned next_offset = offset;
    if (MAX_ANNIHILATION_DIFF <= next_offset)
    {
        next_offset -= MAX_ANNIHILATION_DIFF;
    }
    else
    {
        next_offset += (vector_size-MAX_ANNIHILATION_DIFF);
    }
        
    for(unsigned offset_num = 0 ; offset_num < (2*MAX_ANNIHILATION_DIFF+1) ; ++offset_num)
    {
        corr_peak_vals[next_offset] = 0.0;
        det_burst_powers_mW[next_offset] = 0.0;
        
        ++next_offset;
        if (vector_size <= next_offset)
        {
            next_offset -= vector_size;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Helper function to calculate the offset at which to centre the tracking window
//  @details        Uses a centre-of-mass algorithm applied to the paths in the path list, with the SRS burst power
//                  estimate as the weight.
//  @return                                 The new offset to use.
// 
unsigned SearchAndTrack_4g::calc_new_track_offset(void)
{
    //
    //  Make sure that the midpoint offset lies between 0 and +Design::SAMPLES_PER_4G_SRS_PERIOD-1
    //
    int curr_track_win_midpoint = corr_start+(Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2;
    if (static_cast<int>(Design::SAMPLES_PER_4G_SRS_PERIOD) <= curr_track_win_midpoint)
    {
        curr_track_win_midpoint = curr_track_win_midpoint-Design::SAMPLES_PER_4G_SRS_PERIOD;
    }
                                  
    float sum_rel_offset_power = 0.0;
    float sum_power = 0.0;
    BOOST_FOREACH( PathList_t::value_type& path, path_list )
    {
        if( path.valid_count >= SearchAndTrack_4g::VALID_COUNT_CONFIRMED )
        {
            //
            //  Make sure that the relative offset lies between -Design::SAMPLES_PER_4G_SRS_PERIOD)/2 and
            //  +Design::SAMPLES_PER_4G_SRS_PERIOD)/2-1
            //
            int peak_rel_offset = path.offset-curr_track_win_midpoint;
            if (peak_rel_offset < -static_cast<int>(Design::SAMPLES_PER_4G_SRS_PERIOD)/2)
            {
                peak_rel_offset += Design::SAMPLES_PER_4G_SRS_PERIOD;
            }
            else
            if (peak_rel_offset > +static_cast<int>(Design::SAMPLES_PER_4G_SRS_PERIOD)/2)
            {
                peak_rel_offset -= Design::SAMPLES_PER_4G_SRS_PERIOD;
            }

            sum_rel_offset_power += path.power*peak_rel_offset;
            sum_power += path.power;
        }
    }
    
    //
    //  The path relative offsets lie between -Design::SAMPLES_PER_4G_SRS_PERIOD)/2 and
    //  +Design::SAMPLES_PER_4G_SRS_PERIOD)/2-1, so the CoM relative offset also lies in that range.
    //
    int new_rel_offset = static_cast<int>(round(sum_rel_offset_power/sum_power));
    
    //
    //  The new midpoint can lie between -Design::SAMPLES_PER_4G_SRS_PERIOD)/2 and
    //  +3*Design::SAMPLES_PER_4G_SRS_PERIOD)/2-2.
    //
    int new_track_win_midpoint = curr_track_win_midpoint+new_rel_offset;

    int new_corr_start = new_track_win_midpoint-(Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2;
    if (0 > new_corr_start)
    {
        new_corr_start = new_corr_start+Design::SAMPLES_PER_4G_SRS_PERIOD;
    }
    else
    if (static_cast<int>(Design::SAMPLES_PER_4G_SRS_PERIOD) <= new_corr_start)
    {
        new_corr_start = new_corr_start-Design::SAMPLES_PER_4G_SRS_PERIOD;
    }
                              
    return (static_cast<unsigned>(new_corr_start));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief      Get the list of paths being tracked.
// 
SearchAndTrack_4g::PathList_t const& SearchAndTrack_4g::get_paths(void)
{
    return path_list;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief      Get the last measured channel power corrected to the input of receiver
//  @return                         The power in mW.
//
double SearchAndTrack_4g::get_power(void)
{
    return searching_input_channel_power_mW;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief      Iterate the algorithm.
//  @details    The function should be called repeatedly as it manages the state machine, the processing of the 
//              correlation results to control the analogue range and produce the list of detected paths.
//
//              Determines if the Correlator instance has a result available and if not return WAITING.
//
//              If a correlator result is available update the Ranging instance and if it signals that the range is
//              unstable restart the Correlator instance with the settings unchanged and return WAITING.
//
//              Otherwise update the instance's state machine. If the current state is:
// 
//                  - IDLE          If the range is unstable restart the Correlator instance with the settings
//                                  unchanged and return WAITING.
// 
//                                  Otherwise, restart the Correlator instance in search mode, change the state to 
//                                  SEARCHING and return START_SEARCH.
// 
//                  - SEARCHING     If the range is unstable restart the Correlator instance with the settings
//                                  unchanged and return WAITING.
// 
//                                  Otherwise, if the correlator result included at least one detection peak restart
//                                  the Correlator instance in track mode with the start of the tracking window such
//                                  that the strongest peak detected is in its centre. Change the state to TRACKING
//                                  and return START_TRACK.
// 
//                                  Otherwise, if inter-search pauses are configured put the Correlator, ADC and
//                                  Radio classes into low-power mode while sleeping for the programmed delay. After
//                                  any pause restart the Correlator instance with no change to its settings and
//                                  return START_SEARCH.
// 
//                  - TRACKING      If the range is unstable, pause for RANGING_PAUSE_MS milliseconds, update the
//                                  path list with any new detections and calculate a new start offset for
//                                  the tracking window to be used. Restart the Correlator instance in continuous
//                                  tracking mode, change the state to PRE_TRACKING_RANGING and return
//                                  NEW_RESULT_RANGING.
// 
//                                  Otherwise update the path list with any new detections and calculate a new start
//                                  offset for the tracking window to be used.
// 
//                                  If there is at least one valid path in the updated path list, decrement the
//                                  tracking-timeout counter, otherwise increment it.
// 
//                                  If the tracking-timeout limit has been reached, restart the Correlator instance in
//                                  search mode, change the state to SEARCHING and  return START_SEARCH.
// 
//                                  Otherwise, if a tracking pause has been configured sleep for the pause and then
//                                  restart the Correlator instance with the updated start offset and return
//                                  NEW_RESULT.
// 
//                  - PRE_TRACKING_RANGING
//                                  If the range is unstable, update the path list with any new detections and
//                                  calculate a new start offset for the tracking window to be used. Restart the
//                                  Correlator instance in tracking mode with the calculated start offset, change
//                                  the state to TRACKING_RANGING and return NEW_RESULT_RANGING.
// 
//                                  Otherwise should never happen so throw an exception.
// 
//                  - TRACKING_RANGING
//                                  If the range is unstable, update the path list with any new detections and
//                                  calculate a new start offset for the tracking window to be used. Restart the
//                                  correlator instance in tracking mode with the calculated start offset and
//                                  return NEW_RESULT_RANGING.
// 
//                                  Otherwise update the path list with any new detections and calculate a new start
//                                  offset for the tracking window to be used, change the state to TRACKING and 
//                                  restart the Correlator instance instance in tracking mode with the updated start
//                                  offset, and return NEW_RESULT.
//
//  @return                         The visible mode/state of the instance to allow caller to know what is going on.
// 
SearchAndTrack_4g::iter_res_t SearchAndTrack_4g::iterate( void )
{
    SearchAndTrack_4g::iter_res_t retVal = WAITING;

    if (corr_result_available())
    {
        //
        //  Retrieve the correlation results and identify the maximum peak.
        //
        std::vector<unsigned> corr_peak_counts; 
        std::vector<float> corr_peak_vals; 
        std::vector<float> corr_peak_mean_inputs_adc_lsb_sq;
        float corr_inp_max_mean_adc_lsb_sq;
        float corr_inp_mean_adc_lsb_sq;
        corr_read_results(corr_peak_counts,
                          corr_peak_vals, 
                          corr_peak_mean_inputs_adc_lsb_sq, 
                          corr_inp_max_mean_adc_lsb_sq,
                          corr_inp_mean_adc_lsb_sq);

        float  max_peak_val = 0.0;
        size_t max_peak_index = corr_peak_vals.size();
        unsigned num_peaks = 0;
        for (size_t peak_num = 0 ; peak_num < corr_peak_vals.size() ; ++peak_num)
        {
            if (corr_peak_vals[peak_num] > max_peak_val)
            {
                max_peak_val = corr_peak_vals[peak_num];
                max_peak_index = peak_num;
            }
            
            if (0 < corr_peak_vals[peak_num])
            {
                num_peaks = num_peaks+1;
            }
        }
        
        bool corr_peak_found = (corr_peak_vals.size() != max_peak_index);
        
        debugStr(DebugStream::info1) << "Correlation peaks retrieved: " << num_peaks << "\n";
        
        //
        //  The USE_CHAN_RSSI_RANGING variant was developed in the 3G and GSM implementations, but is not used in the
        //  production build of the the software, presumably because the ranging based on the ADC RSSI was more
        //  reliable. In 4G mode there is no ADC RSSI measurement, but unlike the 3G and GSM cases the power results are
        //  relative to the ADC's LSB squared.
        //
        //  Therefore in 4G mode the values are taken as being measures of the ADC RSSI. This is appropriate even in the
        //  10 MHz bandwidth case when the SRS Bandwidth is 8.64 MHz and about half the signal is discarded by the
        //  filtering that has a 3.84 MHz bandwidth. This is because the analogue filter discards most of the signal so
        //  that the ADC still samples a signal with a bandwidth limited to 3.84 MHz.
        //
#ifdef USE_CHAN_RSSI_RANGING
#error "Not supported for 4G builds"
#else
        //
        //  Retrieve the calibration factor before adjusting the range otherwise the value for the new range is
        //  returned.
        //
        double cal_factor = corr_get_cal_factor_ratio()*pwrcal_getFactor(ranging_get_gain_dB());

        //
        //  Use the peak from the statistics for ranging.
        //
        //  Allow for the fact that the SRS has a lower peak-to-average power ratio (PAPR) than 3G by reducing the
        //  power. The SRS PAPR is less than 6 dB whereas the ranging class allows 10 dB of headroom for the 3G case, so
        //  a 4 dB reduction in the power should result in a better performance at low signal levels.
        //
        int old_gain_dB = ranging_get_gain_dB();
        double channel_power_adc_lsb_sq = corr_inp_max_mean_adc_lsb_sq;
        double ranging_factor = pow(10.0, -5.0/10.0);
        Ranging::RangingUpdateResult rangingUpdateResult = ranging_update(channel_power_adc_lsb_sq*ranging_factor);
        
        if (num_peaks > 0)
        {
            debugStr(DebugStream::info4) << boost::format("%1u, %1u (%3d / %3d) : %4d : %4u peaks ; stats : %7.1f, %7.1f ; peak : %4u, %2u, %5.3f, %7.1f\n")
                                                          % static_cast<unsigned>(mode)
                                                          % static_cast<unsigned>(rangingUpdateResult)
                                                          % old_gain_dB
                                                          % ranging_get_gain_dB()
                                                          % corr_start
                                                          % num_peaks
                                                          % sqrt(corr_inp_max_mean_adc_lsb_sq)
                                                          % sqrt(corr_inp_mean_adc_lsb_sq)
                                                          % max_peak_index
                                                          % corr_peak_counts[max_peak_index]
                                                          % corr_peak_vals[max_peak_index]
                                                          % sqrt(corr_peak_mean_inputs_adc_lsb_sq[max_peak_index]);
        }
        else
        {
            debugStr(DebugStream::info4) << boost::format("%1u, %1u (%3d / %3d) : %4d : %4u peaks ; stats : %7.1f, %7.1f ; peak : ----, --, -----, -------\n")
                                                          % static_cast<unsigned>(mode)
                                                          % static_cast<unsigned>(rangingUpdateResult)
                                                          % old_gain_dB
                                                          % ranging_get_gain_dB()
                                                          % corr_start
                                                          % num_peaks
                                                          % sqrt(corr_inp_max_mean_adc_lsb_sq)
                                                          % sqrt(corr_inp_mean_adc_lsb_sq);
        }
        
/*        
        static int debug_message_timer = 11;
        if (10 < debug_message_timer)
        {
            debugStr(DebugStream::info4) << boost::format("%3u peaks, mean pwr = %e, peak val = %.3f, peak pwr = %e\n")
                                                          % num_peaks
                                                          % corr_inp_max_mean_adc_lsb_sq
                                                          % corr_peak_vals[max_peak_index]
                                                          % corr_peak_mean_inputs_adc_lsb_sq[max_peak_index];
            debug_message_timer = 0;
        }
        ++debug_message_timer;
*/
#endif

        //
        //  If the range is stable the results can be used for the search and track processing.
        //
        if( rangingUpdateResult == Ranging::RANGE_STABLE )
        {
            switch ( mode )
            {
                case IDLE:
                {
                    corr_change_mode( Correlator_4g::SEARCH );
                    debugStr(DebugStream::info4) << boost::format("IDLE                  -> SEARCHING\n");
                    mode = SEARCHING;
                    retVal = START_SEARCH;
                    break;
                }       
                case SEARCHING:
                {
                    //
                    //  Calibrate and store the mean channel power.
                    //
                    searching_input_channel_power_mW = cal_factor*corr_inp_mean_adc_lsb_sq;
                    
                    if( corr_peak_found )
                    {
                        //
                        //  Found a path.  Reset the timer that triggers the return to search mode if there are no
                        //  paths and calculate the offset to be used in tracking.
                        //
                        no_path_timer = 0;

                        if (max_peak_index >= (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2)
                        {
                            corr_start = max_peak_index-(Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2;
                        }
                        else
                        {
                            corr_start =   max_peak_index
                                         + (  Design::SAMPLES_PER_4G_SRS_PERIOD
                                         - (Design::TRACK_WINDOW_SIZE_4G_SAMPLES-1)/2);
                        }

                        corr_change_mode( Correlator_4g::TRACK, corr_start );
                        debugStr(DebugStream::info4) << boost::format("SEARCHING             -> TRACKING\n");
                        mode = TRACKING;
                        retVal = START_TRACK;
                    }
                    else
                    {
                        //
                        //  No peak found so perform another search after any pause that may be required.
                        //
                        if (0 < searching_pause)
                        {
                            unsigned pause_us = (searching_pause*1000000)/FourG::FRAMES_PER_SEC;

                            corr_enter_low_power_mode();
                            adc_enter_low_power_mode();
                            radio_enter_low_power_mode();

                            debugStr(DebugStream::info1) << boost::format("pause for %u us\n") % pause_us;
                            usleep(pause_us);

                            radio_exit_low_power_mode();
                            adc_exit_low_power_mode();
                            corr_exit_low_power_mode();
                        }
                        
                        corr_restart_correlation();
                    }
                    break;
                }
                case TRACKING:
                {
                    //
                    //  Perform the basic path identification and tracking.
                    //
                    int new_start = track(corr_inp_mean_adc_lsb_sq,
                                          corr_peak_vals,
                                          corr_peak_mean_inputs_adc_lsb_sq,
                                          cal_factor);

                    //
                    //  Determine if there is at least one valid path.
                    //
                    bool at_least_one_valid_path = false;
                    BOOST_FOREACH( PathList_t::value_type& path, path_list )
                    {
                        if( path.valid_count >= VALID_COUNT_CONFIRMED )
                        {
                            at_least_one_valid_path = true;
                            break;
                        }
                    }

                    if (at_least_one_valid_path)
                    {
                        //
                        //  Found a path.  Decrement the timer that triggers the return to search mode if there are no paths.
                        //
                        if (no_path_timer > TRACKING_PATH_FOUND_DEC)
                        {
                            no_path_timer = no_path_timer-TRACKING_PATH_FOUND_DEC;
                        }
                        else
                        {
                            no_path_timer = 0;
                        }
                    }
                    else
                    {
                        //
                        //  If there are no valid paths increment the timeout timer.
                        //
                        no_path_timer = no_path_timer+TRACKING_NO_PATH_FOUND_INC;
                    }

                    //
                    //  If there have been no valid paths for too long switch back to search mode.
                    //
                    if (no_path_timer >= TRACKING_TIMEOUT)
                    {
                        corr_change_mode( Correlator_4g::SEARCH );
                        debugStr(DebugStream::info4) << boost::format("TRACKING              -> SEARCHING\n");
                        mode = SEARCHING;
                        retVal = START_SEARCH;
                    }
                    else
                    {
                        corr_start = new_start;
                        if(0 < tracking_pause)
                        {
                            unsigned pause_us = (tracking_pause*1000000)/FourG::FRAMES_PER_SEC;

                            debugStr(DebugStream::info1) << boost::format("pause for %u us\n") % pause_us;
                            usleep(pause_us);
                        }
                        
                        corr_change_start( corr_start );
                        retVal = NEW_RESULT;
                    }

                    break;
                }
                case TRACKING_RANGING:
                {
                    //
                    //  Perform the basic path identification and tracking.
                    //
                    int new_start = track(corr_inp_mean_adc_lsb_sq,
                                          corr_peak_vals,
                                          corr_peak_mean_inputs_adc_lsb_sq,
                                          cal_factor);

                    corr_start = new_start;

                    //
                    //  Return to tracking at the same offset as before the ranging began.
                    //
                    corr_change_mode( Correlator_4g::TRACK, corr_start );
                    debugStr(DebugStream::info4) << boost::format("TRACKING_RANGING      -> TRACKING\n");
                    mode = TRACKING;
                    retVal = NEW_RESULT;
                    break;
                }
                case PRE_TRACKING_RANGING:
                default :
                {
                    throw std::runtime_error( "invalid SearchAndTrack_4g mode" );
                    break;
                }
            }
        }
        else
        if (   (SEARCHING == mode)
            || (IDLE == mode))
        {
            //
            //  The range is unstable so restart the correlation with the same settings.
            //
            corr_restart_correlation();
        }
        else
        {
#ifdef RANGING_FORCES_SEARCH_4G
            //
            //  The range is unstable so switch back to search mode.
            //
            corr_change_mode( Correlator_4g::SEARCH );
            debugStr(DebugStream::info4) << boost::format("TRACKING              -> SEARCHING\n");
            mode = SEARCHING;
            retVal = START_SEARCH;
#else
            if (TRACKING == mode)
            {
                //
                //  The gain has been adjusted, but the data was acquired using the old gain. Pause for a sleep to let
                //  the gain settle, then process the results and switch to PRE_TRACKING_RANGING mode, using the
                //  correlator's SEARCH mode.
                //
                unsigned pause_us = RANGING_PAUSE_MS*1000;
                usleep(pause_us);
                
                //
                //  Perform the basic path identification and tracking.
                //
                int new_start = track(corr_inp_mean_adc_lsb_sq,
                                      corr_peak_vals,
                                      corr_peak_mean_inputs_adc_lsb_sq,
                                      cal_factor);

                corr_start = new_start;

                //
                //  The range is unstable so switch back to correlator search mode and restart the correlation, but
                //  remember to return to tracking afterwards.
                //
                corr_change_mode( Correlator_4g::CONT_TRACK );
                debugStr(DebugStream::info4) << boost::format("TRACKING              -> PRE_TRACKING_RANGING\n");
                mode = PRE_TRACKING_RANGING;
                retVal = NEW_RESULT_RANGING;
            }
            else
            if (PRE_TRACKING_RANGING == mode)
            {
                //
                //  The first full pass after a gain adjustment, which has been carried out with the correlator's SEARCH
                //  mode so calibrate and store the mean channel power.
                //
                searching_input_channel_power_mW = cal_factor*corr_inp_mean_adc_lsb_sq;

                //
                //  Perform the basic path identification and tracking.
                //
                int new_start = track(corr_inp_mean_adc_lsb_sq,
                                      corr_peak_vals,
                                      corr_peak_mean_inputs_adc_lsb_sq,
                                      cal_factor);

                corr_start = new_start;

                //
                //  Switch back to correlator tracking mode and restart the correlation.
                //
                corr_change_mode( Correlator_4g::TRACK, corr_start );
                debugStr(DebugStream::info4) << boost::format("PRE_TRACKING_RANGING  -> TRACKING_RANGING\n");
                mode = TRACKING_RANGING;
                retVal = NEW_RESULT_RANGING;
            }
            else
            {
                //
                //  Perform the basic path identification and tracking.
                //
                int new_start = track(corr_inp_mean_adc_lsb_sq,
                                      corr_peak_vals,
                                      corr_peak_mean_inputs_adc_lsb_sq,
                                      cal_factor);

                corr_start = new_start;

                //
                //  restart the correlator.
                //
                corr_change_mode( Correlator_4g::TRACK, corr_start );
                retVal = NEW_RESULT_RANGING;
            }
#endif    
        }
    }

    return retVal;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Reset the state machine so it starts a full search again.
//
void SearchAndTrack_4g::restart( void )
{
    mode = IDLE;
    path_list.clear();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Set the debug verbosity level.
//  @param[in]      level               The new verbosity level.
//
void SearchAndTrack_4g::debug_printing( unsigned l )
{
    debugStr.show_level( l );
    the_correlator->debug_printing( l );
}


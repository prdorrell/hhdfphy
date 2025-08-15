/******************************************************************************
 * Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 * All Rights Reserved
 ******************************************************************************
 * THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 * ----------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/searchandtrack.cpp $
 * $Revision: 7469 $
 * $Author: pdm $
 * $Date: 2011-10-06 13:26:22 +0100 (Thu, 06 Oct 2011) $
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file searchandtrack.cpp
 * \brief Definition of SearchAndTrack class that implements algorithm to
 * find and track the correlation paths.
 *
 *****************************************************************************/

#include "adc.hpp"
#include "searchandtrack.hpp"
#include "radio.hpp"
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

// /////////////////////////////////////////////////////////////////////////////
/// \brief set up new search and track object
/// \param c [in] correlator object to use
/// \param am [im] antenna mode
/// \param search_th [in] search threshold factor
/// \param track_th [in] track threshold factor
/// \param r [in] use ranging or not
/// \param searching_pause [in] number of idle frames between searching iterations
/// \param tracking_pause [in] number of idle frames between tracking iterations
/// \param debug [in] debug output verbosity
//
SearchAndTrack::SearchAndTrack( Ranging                     &rangeCtrl,
                                PwrCal                      &pwrCal,
                                std::auto_ptr< Correlator > c,
                                unsigned                    am,
                                double                      search_th,
                                double                      track_th,
                                unsigned                    searching_pause,
                                unsigned                    tracking_pause,
                                unsigned                    debug) :
    antenna_mode( am ),
    ranging( rangeCtrl ),
    power_cal( pwrCal ),
    search_threshold_factor_dB( search_th ),
    track_threshold_factor_dB( track_th ),
    searching_pause( searching_pause ),
    tracking_pause( tracking_pause ),
    peak_annihilation_span( 3 ),
    peak_search_span( 2 ),
    tracking_window_size( 200 ),
    no_path_timer( 0 ),
    debugStr( "search-trk.. ", debug )
{
    the_correlator = c;     // take ownership
    path_id_counter = 1;
    mode = IDLE;
}

// /////////////////////////////////////////////////////////////////////////////
/// \brief Search for a peak in frame
//
/// \return Correlation window start position that centers window on peak.
///         -1 = no detection found.
//
int SearchAndTrack::search( void )
{
    int retVal = -1;


    int offset_diff = abs(  static_cast<int>(search_peak2.sample_offset)
                          - static_cast<int>(search_peak1.sample_offset));

    if (   (search_peak1.exceeds_threshold && search_peak2.exceeds_threshold)
        && (offset_diff <= 1))
    {
        int peak_pos = search_peak1.sample_offset;

        debugStr(DebugStream::info3) << "search peak at " << peak_pos;

#ifdef CHECK_FPGA_PEAK_DETECTION
        //
        //  Write the position into the FPGA's test register and set the LED bit (debugging peak detector problems).
        //
        the_correlator->set_test_register((peak_pos << 1) | 1);
#endif

        peak_pos = peak_pos-Design::TRACK_MODE_SKEW;
        peak_pos =   Design::TRACK_MODE_OFFSET_RES
                   * (  (peak_pos+Design::TRACK_MODE_OFFSET_RES/2)
                      / Design::TRACK_MODE_OFFSET_RES);

        if( peak_pos < 0 )
        {
            peak_pos += Design::samples_per_frame;
        }
        else
        if( peak_pos > (int)Design::samples_per_frame )
        {
            peak_pos -= Design::samples_per_frame;
        }

        debugStr(DebugStream::info3) << ", return window pos " << peak_pos <<'\n';

        retVal = peak_pos;

        //
        //  Remove any paths left over frmo a previous tracking state.
        //
        path_list.clear();
    }

    return retVal;
}

// /////////////////////////////////////////////////////////////////////////////
/// \brief Track peaks as they drift
//
/// Algorithm used:
/// - get a chunk of correlation data around peak
/// - check list of peaks to see if they exist close to last time
/// - remove non-existant peaks after a few iterations
/// - add peaks and validate after a few iterations
///
/// \return new offset for start of correlation window
///         -1 = waiting for time to pass, call again
//
int SearchAndTrack::track( void )
{
    int64_t frame = the_correlator->get_frame();
    double pwr = the_correlator->get_channel_power();

    // divide samples by radio gain to take this into account
    // so channel power and correlation results are referenced to input at antenna

    input_channel_power = pwr*power_cal.getFactor(ranging.get_gain_dB());

    //
    //  Get the samples - all values below the thyreshold are returned as zero.
    //
    std::vector< double > data;
    std::vector< double > data_alt;

    uint32_t raw_data_start_offset = Design::TRACK_MODE_SKEW-tracking_window_size/2;
    the_correlator->get_raw_corr_data( raw_data_start_offset,
                                       tracking_window_size,
                                       false,
                                       data );
    the_correlator->get_raw_corr_data( raw_data_start_offset,
                                       tracking_window_size,
                                       true,
                                       data_alt );

//#ifdef CHECK_FPGA_PEAK_DETECTION
    uint32_t fpga_peak_offset = the_correlator->get_peak_sample_offset();

    std::vector< double > temp_data;
    bool use_threshold = false;
    the_correlator->get_raw_corr_data( fpga_peak_offset, 1, false, temp_data, use_threshold );

    debugStr(DebugStream::info1) <<   boost::format("Checking FPGA peak: offset = %5u, power = %f\n")
                                    % fpga_peak_offset
                                    % temp_data[0];
//#endif

    //
    //  Let the FPGA start filling the result buffer again.
    //
    the_correlator->pop_result();

    //
    //  Identify the true peaks, i.e. samples that have smaller values either side.
    //
    std::vector< double > dataPeaks(data.size(), 0.0);

    unsigned numNonZeroSamples = 0;
    double nonZeroSamplePowerSum = 0.0;

    unsigned numPeaks = 0;
    double truePeakVal = 0.0;
    double lastPeakVal = 0.0;

    for(int i = 1 ; i < tracking_window_size-1 ; ++i)
    {
        if(data[i] > 0.0)
        {
            ++numNonZeroSamples;
            nonZeroSamplePowerSum += numNonZeroSamples;
            if((data[i] > data[i-1]) && (data[i] > data[i+1]))
            {
                dataPeaks[i] = data[i];

                ++numPeaks;
                if (data[i] > truePeakVal)
                {
                    lastPeakVal = truePeakVal;
                    truePeakVal = data[i];
                }
            }
        }
    }
    data = dataPeaks;

    double nonZeroSamplePowerMean = 0.0;
    if (numNonZeroSamples > 0)
    {
        nonZeroSamplePowerMean = nonZeroSamplePowerSum/numNonZeroSamples;
    }

    debugStr(DebugStream::info1) <<   boost::format("Samples above threshold = %3u, true peaks = %3u, mean = %.2f peaks : %.1f (%.1f dB), %.1f (%.1f dB)\n")
                                    % numNonZeroSamples
                                    % numPeaks
                                    % nonZeroSamplePowerMean
                                    % truePeakVal
                                    % (10.0*log10(truePeakVal/nonZeroSamplePowerMean))
                                    % lastPeakVal
                                    % (10.0*log10(lastPeakVal/nonZeroSamplePowerMean));

    //
    //  Only allow peaks that are above a threshold set 10 dB below the strongest peak.
    //
    for(int i = 0 ; i < tracking_window_size ; ++i)
    {
        if(data[i] < (truePeakVal*REL_THRESHOLD_FACTOR))
        {
            data[i] = 0.0;
        }
    }

    //
    //  Calculate the time in samples of the start of this block of data.  Note that the 64-bit value will never wrap
    //  (2**63/7.68e6/60/60/24/356 = 39044 years!) as long as the frame counter in the fpga does not wrap.  This is
    //  handled in the FPGA interface.
    //
    int64_t data_start = frame*Design::samples_per_frame + corr_start + raw_data_start_offset;

    //
    //  Look for a peak close to each current entry in the path list
    //
    int64_t min_pos = std::numeric_limits<int64_t>::max();
    int64_t max_pos = std::numeric_limits<int64_t>::min();
    BOOST_FOREACH( PathList_t::value_type& path, path_list )
    {
        //
        //  Estimate the number of frames elapsed between the stored path time and the start of the new data, rounded
        //  to the nearest frame.
        //
        int64_t frames_elapsed = ( data_start - path.pos + Design::samples_per_frame/2 ) /
                                 Design::samples_per_frame;

        //
        //  Estimate the position in the new sample array at which the peak should be found.
        //
        int64_t est_pos = path.pos + frames_elapsed * Design::samples_per_frame;

        //
        //  Look for a new peak near the estimated position.
        //
        bool    peak_found   = false;
        int64_t new_peak_pos = 0;
        double new_peak_power = 0.0;
        for( int i = -peak_search_span; i <= peak_search_span; ++i )
        {
            int64_t pos = est_pos + i;
            if( pos < data_start || pos >= data_start+tracking_window_size )
            {
                // out of range, nothing more to do
                continue;
            }

            double sample = data[ pos-data_start ];

            if( sample > new_peak_power )
            {
                // found it but there may be a bigger peak
                new_peak_power = sample;
                new_peak_pos   = pos;
                peak_found     = true;
            }
        }

        //
        //  Apply the calibration factor.
        //
        new_peak_power = new_peak_power*power_cal.getFactor(ranging.get_gain_dB());

        //
        //  If the peak has been re-discovered update its entry in the path list and remove it from the samples so that
        //  it does not appear in the search for new peaks.
        //
        if( peak_found )
        {
            debugStr(DebugStream::info1) <<   boost::format("Old peak found: offset = %5u, power = %e\n")
                                            % (new_peak_pos % Design::samples_per_frame)
                                            % new_peak_power;

#ifdef CHECK_FPGA_PEAK_DETECTION
            if (fpga_peak_offset != (new_peak_pos % Design::samples_per_frame))
            {
                debugStr(DebugStream::error) <<   boost::format("FPGA peak detection offset disagrees with software result\n");
            }

            //
            //  Write the position into the FPGA's test register and set the LED bit (debugging peak detector problems).
            //
            the_correlator->set_test_register(((new_peak_pos % Design::samples_per_frame) << 1) | 1);
#endif

            //
            //  Retrieve the power of the corresponding sample in the alternate antenna data.
            //
            double new_peak_power_alt = data_alt[new_peak_pos-data_start]*power_cal.getFactor(ranging.get_gain_dB());

            // update peak info
            path.power     = (path.power     + new_peak_power     )/2.0;     // is this filtering ok?
            path.power_alt = (path.power_alt + new_peak_power_alt )/2.0;

            path.pos       = new_peak_pos;

            path.valid_count = path.valid_count+VALID_COUNT_PATH_FOUND;
            if( path.valid_count >= VALID_COUNT_MAX_VAL )
            {
                path.valid_count = VALID_COUNT_MAX_VAL;
            }

            if( path.valid_count >= VALID_COUNT_CONFIRMED )
            {
                if( new_peak_pos < min_pos ) min_pos = new_peak_pos;
                if( new_peak_pos > max_pos ) max_pos = new_peak_pos;
            }

            annihilate( new_peak_pos-data_start, data );
        }
        else
        {
            debugStr(DebugStream::info1) <<   boost::format("Old peak not found: path.valid_count = %d\n")
                                            % path.valid_count;

            // not found this time
            // reduce the valid_count so eventually path is removed
            path.pos = est_pos;
            path.valid_count = path.valid_count+VALID_COUNT_PATH_NOT_FOUND;
        }

    } // foreach in list

    //
    //  Look for new peaks.
    //
    bool found_peak;
    do
    {
        //
        //  Find the strongest peak left in the samples.
        //
        found_peak = false;

        double  new_peak_power  = 0.0;
        int     new_peak_offset = 0;
        for( int i = 0; i < tracking_window_size; ++i )
        {
            if( data[i] > new_peak_power )
            {
                new_peak_power  = data[i];
                new_peak_offset = i;
                found_peak      = true;
            }
        }

        //
        //  If a peak has been found add it to the path list and remove it from the samples so that it does not appear
        //  again.
        //
        if( found_peak )
        {
            Path_t new_path;
            new_path.id        = path_id_counter++;
            new_path.pos       = data_start + new_peak_offset;
            new_path.power     = new_peak_power*power_cal.getFactor(ranging.get_gain_dB());
            new_path.power_alt = data_alt[new_peak_offset]*power_cal.getFactor(ranging.get_gain_dB());

            new_path.valid_count = VALID_COUNT_INIT_VAL;
            path_list.push_back( new_path );

            debugStr(DebugStream::info1) <<   boost::format("New peak found: offset = %5u, power = %e\n")
                                            % (new_path.pos % Design::samples_per_frame)
                                            % new_path.power;

#ifdef CHECK_FPGA_PEAK_DETECTION
            if (fpga_peak_offset != (new_path.pos % Design::samples_per_frame))
            {
                debugStr(DebugStream::warn1) <<   boost::format("FPGA peak detection offset disagrees with software result\n");
            }

            //
            //  Write the position into the FPGA's test register and set the LED bit (debugging peak detector problems).
            //
            the_correlator->set_test_register(((new_path.pos % Design::samples_per_frame) << 1) | 1);
#endif

            annihilate( new_peak_offset, data );
        }
    }while( found_peak );

    //
    //  Clean up the path list by removing detections that are no longer considered to be valid.
    //
    path_list.remove_if( is_invalid );

    //
    //  Sort the list in order of descending power.
    //
    path_list.sort( descending_power );

    //
    //  Calculate the new tracking window position so that the peaks that have been found are centred.
    //
    int new_corr_start = corr_start;
    if( min_pos != std::numeric_limits<int64_t>::max() )
    {
        new_corr_start = ((min_pos+max_pos)/2) % Design::samples_per_frame;
        new_corr_start = new_corr_start-Design::TRACK_MODE_SKEW;
        new_corr_start =   Design::TRACK_MODE_OFFSET_RES
                         * (  (new_corr_start+Design::TRACK_MODE_OFFSET_RES/2)
                            / Design::TRACK_MODE_OFFSET_RES);

        if( new_corr_start < 0 )
        {
            new_corr_start += Design::samples_per_frame;
        }
        else
        if( new_corr_start > (int)Design::samples_per_frame )
        {
            new_corr_start -= Design::samples_per_frame;
        }

        debugStr(DebugStream::info1) <<   boost::format("Old start offset = %5d, New start offset = %5d\n")
                                        % corr_start
                                        % new_corr_start;
    }

    return  new_corr_start;
}

// /////////////////////////////////////////////////////////////////////////////
/// \brief helper function to zero results around a point in the buffer
/// \param pos [in] center point of zeroing
/// \param data [in out] the data to modify
//
void SearchAndTrack::annihilate( int pos, std::vector<double>& data )
{
    // remove from data so not found again
    int start = pos - peak_annihilation_span;
    if( start < 0 )
    {
        start = 0;
    }
    int stop  = pos + peak_annihilation_span;
    if( stop > (static_cast<int>(data.size())-1) )
    {
        stop = data.size()-1;
    }
    for( int i = start; i <= stop; ++i )
    {
        data[ i ] = 0;
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// \brief get the list of paths being tracked
//
SearchAndTrack::PathList_t const& SearchAndTrack::get_paths( void )
{
    return path_list;
}

// /////////////////////////////////////////////////////////////////////////////
/// \brief last measured channel power
/// \return measured channel power corrected to input of receiver
/// (effect of radio gain settings taken into account)
//
double SearchAndTrack::get_power( void )
{
    return input_channel_power;
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
SearchAndTrack::iter_res_t SearchAndTrack::iterate( void )
{
    SearchAndTrack::iter_res_t retVal = WAITING;

    Correlator::busy_result_t busy_result = the_correlator->busy();
    if (busy_result == Correlator::RESULT_AVAILABLE)
    {
        //
        //  Allow the range controller to adjust the gain if necessary, but not while flushing the effects of a
        //  correlator restart.
        //
        double channel_power = the_correlator->get_channel_power();
        if (   (mode == IDLE)
            || (mode == START_SEARCH_FLUSH1)
            || (mode == START_SEARCH_FLUSH2)
            || (mode == SEARCHING_PART1)
            || (mode == SEARCHING_PART2))
        {
            the_correlator->set_threshold(channel_power, search_threshold_factor_dB);
        }
        else
        {
            the_correlator->set_threshold(channel_power, track_threshold_factor_dB);
        }

        Ranging::RangingUpdateResult rangingUpdateResult = Ranging::RANGE_STABLE;

        if (   (mode != START_SEARCH_FLUSH1) && (mode != START_SEARCH_FLUSH1)
            && (mode != START_TRACK_FLUSH1 ) && (mode != START_TRACK_FLUSH2 ) )
        {
#ifdef USE_CHAN_RSSI_RANGING
            rangingUpdateResult = ranging.update(channel_power);
#else
            rangingUpdateResult = ranging.update(the_correlator->get_adc_power());
#endif
        }

        if( rangingUpdateResult == Ranging::RANGE_UNSTABLE )
        {
            the_correlator->pop_result();
        }
        else
        {
            switch ( mode )
            {
                case IDLE:
                {
                    //  select the antenna - use antenna 1 if it is selected or if the swapping mode is active, and
                    //  only use antenna 2 if explicitly required to do so.
                    if( antenna_mode == 2 )
                    {
                        the_correlator->change_mode( Correlator::SEARCH_ANT2 );
                    }
                    else
                    {
                        the_correlator->change_mode( Correlator::SEARCH_ANT1 );
                    }
                    debugStr(DebugStream::info1) << "SearchAndTrack   IDLE                  -> START_SEARCH_FLUSH1\n";
                    mode = START_SEARCH_FLUSH1;
                    retVal = START_SEARCH;
                    break;
                }
                case START_SEARCH_FLUSH1:
                {
                    // discard this result - tha antenna mode may have just changed.
                    the_correlator->pop_result();
                    debugStr(DebugStream::info1) << "SearchAndTrack   START_SEARCH_FLUSH1   -> START_SEARCH_FLUSH2\n";
                    mode = START_SEARCH_FLUSH2;
                    break;
                }
                case START_SEARCH_FLUSH2:
                {
                    // discard this result - tha antenna mode may have just changed.
                    the_correlator->pop_result();
                    debugStr(DebugStream::info1) << "SearchAndTrack   START_SEARCH_FLUSH2   -> SEARCHING_PART1\n";
                    mode = SEARCHING_PART1;
                    break;
                }
                case SEARCHING_PART1:
                {
                    // collect and store the peaks map for the first part of the search
                    the_correlator->get_peak_data( search_peak1 );
                    the_correlator->pop_result();
                    debugStr(DebugStream::info1) << "SearchAndTrack   SEARCHING_PART1       -> SEARCHING_PART2\n";
                    mode = SEARCHING_PART2;
                    break;
                }
                case SEARCHING_PART2:
                {
                    // collect and store the peaks map for the second part of the search and perform the search
                    the_correlator->get_peak_data( search_peak2 );
                    the_correlator->pop_result();
                    corr_start = search();
                    if( corr_start == -1 )
                    {
                        if (searching_pause == 0)
                        {
                            debugStr(DebugStream::info1) << "SearchAndTrack   SEARCHING_PART2       -> SEARCHING_PART1\n";
                            mode = SEARCHING_PART1;
                        }
                        else
                        {
                            unsigned pause_us = (searching_pause*1000000)/ThreeGPP::FRAMES_PER_SEC;

                            the_correlator->enter_low_power_mode();
                            ADC::enter_low_power_mode();
                            Radio::enter_low_power_mode();

                            debugStr(DebugStream::info1) << boost::format("pause for %u us\n") % pause_us;
                            usleep(pause_us);

                            Radio::exit_low_power_mode(true);
                            ADC::exit_low_power_mode(true);
                            the_correlator->exit_low_power_mode();

                            the_correlator->restart_correlation();

                            debugStr(DebugStream::info1) << "SearchAndTrack   SEARCHING_PART2       -> START_SEARCH_FLUSH1\n";
                            mode = START_SEARCH_FLUSH1;
                            retVal = START_SEARCH;
                        }
                    }
                    else
                    {
                        //
                        //  Found a path.  Reset the timer that triggers the return to search mode if there are no paths.
                        //
                        no_path_timer = 0;

                        //
                        //  Select the final antenna mode
                        //
                        if ( antenna_mode == 1 )
                        {
                            the_correlator->change_mode( Correlator::TRACK_ANT1, corr_start );
                        }
                        else
                        {
                            the_correlator->change_mode( Correlator::TRACK_ANT2, corr_start );
                        }

                        debugStr(DebugStream::info1) << "SearchAndTrack   SEARCHING_PART2       -> START_TRACK_FLUSH1\n";
                        mode = START_TRACK_FLUSH1;
                        retVal = START_TRACK;
                    }
                    break;
                }
                case START_TRACK_FLUSH1:
                {
                    the_correlator->pop_result();
                    debugStr(DebugStream::info1) << "SearchAndTrack   START_TRACK_FLUSH1    -> START_TRACK_FLUSH2\n";
                    mode = START_TRACK_FLUSH2;
                    break;
                }
                case START_TRACK_FLUSH2:
                {
                    the_correlator->pop_result();
                    debugStr(DebugStream::info1) << "SearchAndTrack   START_TRACK_FLUSH2    -> TRACKING\n";
                    mode = TRACKING;
                    break;
                }
                case TRACKING:
                {
                    //
                    //  Perform the basic path identification and tracking.
                    //
                    int new_start = track();

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
                        //  select the antenna - use antenna 1 if it is selected or if the swapping mode is active, and
                        //  only use antenna 2 if explicitly required to do so.
                        if( antenna_mode == 2 )
                        {
                            the_correlator->change_mode( Correlator::SEARCH_ANT2 );
                        }
                        else
                        {
                            the_correlator->change_mode( Correlator::SEARCH_ANT1 );
                        }
                        debugStr(DebugStream::info1) << "SearchAndTrack   TRACKING              -> START_SEARCH_FLUSH1\n";
                        mode = START_SEARCH_FLUSH1;
                        retVal = START_SEARCH;
                    }
                    else
                    if( new_start != -1 )
                    {
                        if( new_start != corr_start )
                        {
                            corr_start = new_start;
                            the_correlator->change_start( corr_start );
                            if( tracking_pause > 0 )
                            {
                                debugStr(DebugStream::info1) << "SearchAndTrack   TRACKING              -> TRACKING_PAUSE\n";
                                mode = TRACKING_PAUSE;
                            }
                            else
                            {
                                debugStr(DebugStream::info1) << "SearchAndTrack   TRACKING              -> START_TRACK_FLUSH1\n";
                                mode = START_TRACK_FLUSH1;
                            }
                        }
                        else
                        if( tracking_pause > 0 )
                        {
                            debugStr(DebugStream::info1) << "SearchAndTrack   TRACKING              -> TRACKING_PAUSE\n";
                            mode = TRACKING_PAUSE;
                        }
                        retVal = NEW_RESULT;
                    }

                    break;
                }
                case TRACKING_PAUSE:
                {
                    unsigned pause_us = (tracking_pause*1000000)/ThreeGPP::FRAMES_PER_SEC;
                    debugStr(DebugStream::info1) << boost::format("pause for %u us\n") % pause_us;

//                    ADC::enter_low_power_mode(); Oddly increases current if pause is low
                    usleep(pause_us);
//                    ADC::exit_low_power_mode(true);

                    the_correlator->restart_correlation();

                    debugStr(DebugStream::info1) << "SearchAndTrack   TRACKING_PAUSE         -> START_TRACK_FLUSH1\n";
                    mode = START_TRACK_FLUSH1;
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
    else
    if (busy_result == Correlator::RESULT_TIMEOUT)
    {
#ifdef LOCKUP_RECOVERY
        switch ( mode )
        {
            //
            //  The recovery in the IDLE state is to simply restart the correlator.
            //
            case IDLE:
            {
                the_correlator->restart_correlation();
                break;
            }

            //
            //  The recovery in any of the SEARCH states is to restart the correlator and return to the
            //  START_SEARCH_FLUSH1 state.
            //
            case START_SEARCH_FLUSH1:
            case START_SEARCH_FLUSH2:
            case SEARCHING_PART1:
            case SEARCHING_PART2:
            {
                the_correlator->restart_correlation();
                debugStr(DebugStream::info1) << "SearchAndTrack   SEARCHING STATE       -> START_SEARCH_FLUSH1 (lockup recovery)\n";
                mode = START_SEARCH_FLUSH1;
                retVal = START_SEARCH;
                break;
            }

            //
            //  The recovery in any of the TRACKING states is to reselect the antenna mode (which restarts the
            //  correlator) and return to the START_SEARCH_FLUSH1 state.
            //
            case START_TRACK_FLUSH1:
            case START_TRACK_FLUSH2:
            case TRACKING:
            case TRACKING_DELAY:
            {
                //  select the antenna - use antenna 1 if it is selected or if the swapping mode is active, and
                //  only use antenna 2 if explicitly required to do so.
                if( antenna_mode == 2 )
                {
                    the_correlator->change_mode( Correlator::SEARCH_ANT2 );
                }
                else
                {
                    the_correlator->change_mode( Correlator::SEARCH_ANT1 );
                }

                debugStr(DebugStream::info1) << "SearchAndTrack   TRACKING STATE        -> START_SEARCH_FLUSH1 (lockup recovery)\n";
                mode = START_SEARCH_FLUSH1;
                retVal = START_SEARCH;
                break;
            }
            default :
            {
                throw std::runtime_error( "invalid SearchAndTrack mode" );
                break;
            }
        }
#else
        debugStr(DebugStream::info1) << "SearchAndTrack   Some State            -> ABORT_TIMEOUT\n";
        mode = IDLE;
        retVal = ABORT_TIMEOUT;
#endif
    }

    return retVal;
}

// /////////////////////////////////////////////////////////////////////////////
/// \brief Reset state machine so it starts a full search again.
//
void SearchAndTrack::restart( void )
{
    mode = IDLE;
    path_list.clear();
}

// /////////////////////////////////////////////////////////////////////////////
/// \param l [in] debug output verbosity level
//
void SearchAndTrack::debug_printing( unsigned l )
{
    debugStr.show_level( l );
    the_correlator->debug_printing( l );
}


/***********************************************************************************************************************
 *
 *
 ***********************************************************************************************************************
 *  Filename:   searchandtrack_4g.hpp
 *  Author(s):  pdm
 *******************************************************************************************************************//**
 * File Description
 * ================
 *
 *  @file
 *  @brief          The declaration of ::SearchAndTrack_4g class that controls 4G searching and tracking.
 *  @details        Use state machine to control a Correlator_4g instance performing search- and tracking-mode
 *                  correlations.  Use a Ranging class instance to control the analogue gain and a PwrCal instance to
 *                  correct the measured power levels.
 **********************************************************************************************************************/
#ifndef __SEARCHANDTRACK_4G_HPP__
#define __SEARCHANDTRACK_4G_HPP__

#include "adc.hpp"
#include "correlator_4g.hpp"
#include "debug.hpp"
#include "pwrcal.hpp"
#include "ranging.hpp"

#include <list>
#include <memory>

///
/// @brief          Controls the search and tracking process, using a Correlator_4g instance to perform the correlations
///                 and Ranging and PwrCal instances to control the analogue gain and correct power results.
/// @details        An instance begins by searching for detections using a continuous-time correlation spanning one or
///                 more subfarames. When a detection occurs the mode is switched to tracking in which the correlation
///                 is restricted to a small window with each result interval of the correlation duration. The position
///                 of the window in the result intervals tracks the detections and if there are no detections for a
///                 period of time the state machine reverts to the search-mode.
///
///                 Pauses can be inserted between correlation operations to reduce power consumption.
///
///                 After each coorrelation operation the signal level is calculated and the Ranging class instance
///                 is provided with the result. If it decides to adjust the analogue gain no further calculations are
///                 made with those results or any subsequent results until the ranging is stable.
///
class SearchAndTrack_4g
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  The constants.
    //
private:
    //------------------------------------------------------------------------------------------------------------------
    //  The constants controlling the matching of new detections to existing paths.
    
    ///
    /// @brief      The maximum difference between the timing of an existing path and a new detection of that path.
    /// @details    If the tracking is configured to produce a result every 100 ms (high-sensitivity 57 ms detection
    ///             with a 43 ms pause) then this difference would allow for a timing drift of +/-4 ppm.
    ///
    static const unsigned MAX_OFFSET_DIFF = 2;
    
    ///
    /// @brief      The number of new detections removed either side of one that matches an existing path.
    /// @details    Raw 4G detection peaks are isolated samples but timing jitter and the allowance made in the
    ///             averaging can result in associated peaks apperaing either side of the strongest. The same +/-4 ppm
    ///             drift allowed for in MAX_OFFSET_DIFF would result in a single sample effect over the time span of
    ///             high-sensitivity tracking.
    ///
    static const unsigned MAX_ANNIHILATION_DIFF = 2;

    //------------------------------------------------------------------------------------------------------------------
    //  The constants controlling the life of a path in terms of the valid_count values.
    //
    
    ///
    /// @brief      The value used when a path is first identified.
    ///
    static const int32_t VALID_COUNT_INIT_VAL       = -4;
    
    ///
    /// @brief      The threshold used to confirm the path as valid.
    /// @details    This is public as it is used by the Phy class to determine if a path should included in the results.
    ///             A low-priority improvement woould hide this behind a function.
    ///
public:    
    static const int32_t VALID_COUNT_CONFIRMED      = +0;
    
    ///
    /// @brief      The maximum value taken by the valid_count.
    ///
private:    
    static const int32_t VALID_COUNT_MAX_VAL        = +0;
    
    ///
    /// @brief      The value at which the path is declared invalid and is removed from the list.
    ///
    static const int32_t VALID_COUNT_IS_INVALID     = -6;
    
    ///
    /// @brief      The increment used when a new peak matches an existing peak.
    ///
    static const int32_t VALID_COUNT_PATH_FOUND     = +1;
    
    ///
    /// @brief      The increment used when no new peak matches an existing peak.
    ///
    static const int32_t VALID_COUNT_PATH_NOT_FOUND = -2;


    //------------------------------------------------------------------------------------------------------------------
    //  The constants controlling the tracking mode timeout.
    //

    ///
    /// @brief      The timeout that, when reached, forces a transition to searching.
    ///
    static const unsigned   TRACKING_TIMEOUT           = 110;

    ///
    /// @brief      The amount by which the timer is decremented each time a path is found.
    ///
    static const unsigned   TRACKING_PATH_FOUND_DEC    = 5;
    
    ///
    /// @brief      The amount by which the timer is incremented each time a path is found.
    ///
    static const unsigned   TRACKING_NO_PATH_FOUND_INC = 1;

    //------------------------------------------------------------------------------------------------------------------
    //  Other constants.
    //

    ///
    /// @brief      The threshold relative to the strongest peak in the tracking window that defines significant
    ///             multipath components.
    ///
    static const double REL_THRESHOLD_FACTOR = 0.1;

    ///
    /// @brief      The pause in milliseconds to be introduced after a gain change to allow its effects to settle.
    ///
    static const unsigned RANGING_PAUSE_MS = 50;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The types.
    //

    ///
    /// @brief      Structure to group useful correlation path information.
    ///
public:    
    struct Path_t
    {
        uint32_t        id;             ///< GUID (increments from 1)
        int32_t         valid_count;    ///< For temporal filtering + old path destruction
        unsigned        offset;         ///< Offset of the path in a result interval
        double          power;          ///< Path power (corr peak value divided by radio power gain, linear)
    };
    
    ///
    /// @brief  List of paths found
    ///
public:    
    typedef std::list< Path_t >   PathList_t;

    ///
    /// @brief      Mode/state algorithm is in.
    ///
private:    
    enum mode_t
    {
        ///
        /// @brief      The initial state before the first call to iterate() or after a call to restart().
        ///
        IDLE,

        ///
        /// @brief      The instance is searching across all possible offsets in a result interval for a detection.
        ///
        SEARCHING,

        ///
        /// @brief      The instance is tracking one or more detections using a small window of offsets in each
        ///             result interval.
        ///
        TRACKING,

        ///
        /// @brief      The instance is ranging during tracking and this is the first iteration after the gain change.
        ///
        PRE_TRACKING_RANGING,

        ///
        /// @brief      The instance is ranging during tracking and when the ranging is complete tracking should
        ///             continue.
        ///
        TRACKING_RANGING
    };

    ///
    /// @brief      The results of the iterate() function.
    ///
public:    
    enum iter_res_t
    {
        ///
        /// @brief      The instance is waiting for the correlator to produce a result or for the ranging to settle.
        ///
        WAITING,
        
        ///
        /// @brief      The instance has received a correlator result with a stable range and is transitioning to search
        ///             mode.
        ///
        START_SEARCH,
        
        ///
        /// @brief      The instance has received a correlator result with a stable range and is transitioning to track
        ///             mode.
        ///
        START_TRACK,

        ///
        /// @brief      The instance has received a correlator result with a stable range and there is a valid result
        ///             available.
        ///
        NEW_RESULT,

        ///
        /// @brief      The instance has received a correlator result and there is a valid result available, but ranging
        ///             is in progress.
        ///
        NEW_RESULT_RANGING,
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The values.
    //
private:
    ///
    /// @brief      The list of paths that have been found.
    ///
    PathList_t                     path_list;
    
    ///
    /// @brief      The Correlator object to use to perform the correlations and interact with the FPGA.
    ///
    std::auto_ptr< Correlator_4g > the_correlator;
    
    ///
    /// @brief      The Ranging object to use for the analogue gain control.
    ///
    Ranging                        &ranging;
    
    ///
    /// @brief      The PwrCal object to use to provide the power correction values.
    ///
    PwrCal                         &power_cal;                 ///< power calibration object
    
    ///
    /// @brief      The current search or track mode (state)..
    ///
    mode_t                         mode;
    
    ///
    /// @brief      The offset in samples from the start of a result interval for the current correlation process.
    ///
    int                            corr_start;
    
    ///
    /// @brief      A GUID for paths (used in identifying them in debug printing.
    ///
    uint32_t                       path_id_counter;            ///< GUID for paths
    
    ///
    /// @brief      The average channel power in mW at the antenna input during the last correlation process to have
    ///             been completed while searching.
    /// @details    In the searching mode the accumulation of statistics is continuous and this is a better estimate of
    ///             the channel power than the value obtained during the tracking mode, which is dominated by the SRS
    ///             burst. However its age may be an issue if tracking mode has lasted too long.
    ///
    double                         searching_input_channel_power_mW;
    
    ///
    /// @brief      The average channel power in mW at the antenna input during the last correlation process to have
    ///             been completed.
    ///
    double                         input_channel_power_mW;
    
    ///
    /// @brief      The number of frames to wait between searching iterations.
    ///
    unsigned                       searching_pause;
    
    ///
    /// @brief      The number of frames to wait between tracking iterations.
    ///
    unsigned                       tracking_pause;

    ///
    /// @brief      The time controlling the transition from tracking to searching when no paths are being detected.
    ///
    unsigned                no_path_timer;

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
    SearchAndTrack_4g(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Constructs an instance of the class.
    /// @details        Initialises the member variables, setting the mode to IDLE.
    /// @param[in]      rangeCtrl           The range control object to use.
    /// @param[in]      pwrCal              The power calibration object to use.
    /// @param[in]      c                   The correlator object to use.
    /// @param[in]      searching_pause     The number of idle frames between searching iterations
    /// @param[in]      tracking_pause      The number of idle frames between tracking iterations
    /// @param[in]      debug               Controls the debug messages that may be generated.
    ///
public:    
    SearchAndTrack_4g(Ranging &rangeCtrl,
                      PwrCal &pwrCal,
                      std::auto_ptr< Correlator_4g >,
                      unsigned searching_pause,
                      unsigned tracking_pause,
                      unsigned debug);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief      Iterate the algorithm.
    /// @details    The function should be called repeatedly as it manages the state machine, the processing of the 
    ///             correlation results to control the analogue range and produce the list of detected paths.
    ///
    ///             Determines if the Correlator instance has a result available and if not return WAITING.
    ///
    ///             If a correlator result is available update the Ranging instance and if it signals that the range is
    ///             unstable restart the Correlator instance with the settings unchanged and return WAITING.
    ///
    ///             Otherwise update the instance's state machine. If the current state is:
    ///
    ///                 - IDLE          If the range is unstable restart the Correlator instance with the settings
    ///                                 unchanged and return WAITING.
    ///
    ///                                 Otherwise, restart the Correlator instance in search mode, change the state to 
    ///                                 SEARCHING and return START_SEARCH.
    ///
    ///                 - SEARCHING     If the range is unstable restart the Correlator instance with the settings
    ///                                 unchanged and return WAITING.
    ///
    ///                                 Otherwise, if the correlator result included at least one detection peak restart
    ///                                 the Correlator instance in track mode with the start of the tracking window such
    ///                                 that the strongest peak detected is in its centre. Change the state to TRACKING
    ///                                 and return START_TRACK.
    ///
    ///                                 Otherwise, if inter-search pauses are configured put the Correlator, ADC and
    ///                                 Radio classes into low-power mode while sleeping for the programmed delay. After
    ///                                 any pause restart the Correlator instance with no change to its settings and
    ///                                 return START_SEARCH.
    ///
    ///                 - TRACKING      If the range is unstable, pause for RANGING_PAUSE_MS milliseconds, update the
    ///                                 path list with any new detections and calculate a new start offset for
    ///                                 the tracking window to be used. Restart the Correlator instance in continuous
    ///                                 tracking mode, change the state to PRE_TRACKING_RANGING and return
    ///                                 NEW_RESULT_RANGING.
    ///
    ///                                 Otherwise update the path list with any new detections and calculate a new start
    ///                                 offset for the tracking window to be used.
    ///
    ///                                 If there is at least one valid path in the updated path list, decrement the
    ///                                 tracking-timeout counter, otherwise increment it.
    ///
    ///                                 If the tracking-timeout limit has been reached, restart the Correlator instance in
    ///                                 search mode, change the state to SEARCHING and  return START_SEARCH.
    ///
    ///                                 Otherwise, if a tracking pause has been configured sleep for the pause and then
    ///                                 restart the Correlator instance with the updated start offset and return
    ///                                 NEW_RESULT.
    ///
    ///                 - PRE_TRACKING_RANGING
    ///                                 If the range is unstable, update the path list with any new detections and
    ///                                 calculate a new start offset for the tracking window to be used. Restart the
    ///                                 Correlator instance in tracking mode with the calculated start offset, change
    ///                                 the state to TRACKING_RANGING and return NEW_RESULT_RANGING.
    ///
    ///                                 Otherwise should never happen so throw an exception.
    ///
    ///                 - TRACKING_RANGING
    ///                                 If the range is unstable, update the path list with any new detections and
    ///                                 calculate a new start offset for the tracking window to be used. Restart the
    ///                                 correlator instance in tracking mode with the calculated start offset and
    ///                                 return NEW_RESULT_RANGING.
    ///
    ///                                 Otherwise update the path list with any new detections and calculate a new start
    ///                                 offset for the tracking window to be used, change the state to TRACKING and 
    ///                                 restart the Correlator instance instance in tracking mode with the updated start
    ///                                 offset, and return NEW_RESULT.
    ///
    /// @return                         The visible mode/state of the instance to allow caller to know what is going on.
    ///
public:    
    iter_res_t iterate(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief      Get the list of paths being tracked.
    ///
public:    
    PathList_t const&   get_paths(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief      Get the last measured channel power corrected to the input of receiver
    //  @return                         The power in mW.
    //
public:    
    double              get_power(void);
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Reset the state machine so it starts a full search again.
    ///
public:    
    void                restart( void );

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Set the debug verbosity level.
    /// @param[in]      level               The new verbosity level.
    ///
public:
    void debug_printing(unsigned level);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Track peaks as they drift
    /// @details        Identify the strongest new peak and calculate a threshold that is used to limit the peaks to be
    ///                 to the ones most likely to be useful.
    ///
    ///                 Retrieve the power correction factor from the PwrCal class instance and apply it (together with 
    ///                 the 3G to 4G adjustment) to all the power results, eliminating the peaks that fall below the
    ///                 threshold. 
    ///
    ///                 Iterate through the current path list and find the strongest new detection within
    ///                 MAX_OFFSET_DIFF samples of each path. If there is one use it to update the path timing (adopt
    ///                 the new detection's timing), power (the average of old and new) and adjust its validity (count
    ///                 incremented by VALID_COUNT_PATH_FOUND up to VALID_COUNT_MAX_VAL). Finally remove the peak that
    ///                 has been used from the detection list, and any other peak within MAX_ANNIHILATION_DIFF of it.
    ///
    ///                 If no new detection matches the existing path list entry then adjust its validity (count
    ///                 incremented by VALID_COUNT_PATH_NOT_FOUND).
    ///
    ///                 If any new detection peaks are left, add them to the path list. Take the strongest first and as
    ///                 each one is added discard any new peak that is within MAX_ANNIHILATION_DIFF of it.
    ///
    ///                 Remove all paths from the path list that are no longer valid.
    ///
    ///                 Reorder the path list so that the strongest peak is first.
    ///
    ///                 Use the positions in each path of the list to calculate the new correlation start offset. Use a
    ///                 centre of mass algorithm with the path's power as the weight. (Experiments in 1QS06 indicate 
    ///                 that has less variance than choosing the strongest. Using the correlation peak value as the
    ///                 weight made no difference and an unweighted average of the positions seemed slightly poorer at
    ///                 higher SNR.)
    ///
    /// @param[in]      corr_inp_mean           The mean power of the channel samples over the interval spanned by the
    ///                                         correlation in units of (ADC LSB)^2).
    /// @param[in]      corr_peak_vals          The normalised correlation peaks in a result interval.
    /// @param[in]      corr_peak_mean_inputs   The mean power of the channel samples contributing to peak detections
    ///                                         in a result interval in units of (ADC LSB)^2).
    //  @param[in]      cal_factor              The calibration factor to use.
    /// @return                                 The new offset for the start of correlation window.
    ///
private:
    int track( double corr_inp_mean, 
               std::vector<float> corr_peak_vals, 
               std::vector<float> corr_peak_mean_inputs,
               double cal_factor);
                               
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Helper function to apply the power correction to produce results for the antenna input and
    ///                 remove peaks below REL_THRESHOLD_FACTOR relative to the strongest peak.
    /// @param[in,out]  corr_inp_mean           The mean power of the channel samples over the interval spanned by the
    ///                                         correlation. The input power is in units of (ADC LSB)^2 and the output
    ///                                         is in mW.
    /// @param[in,out]  corr_peak_vals          The normalised correlation peaks in a result interval.
    /// @param[in]      corr_peak_mean_inputs   The mean power of the channel samples contributing to peak detections
    ///                                         in a result interval. The input power is in units of (ADC LSB)^2 and the
    ///                                         output is in mW.
    //  @param[in]      cal_factor              The calibration factor to use.
    ///
private:
    void correct_powers_and_discard_weak_peaks(double &corr_inp_mean, 
                                               std::vector<float> &corr_peak_vals, 
                                               std::vector<float> &corr_peak_mean_inputs,
                                               double cal_factor);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Helper function to update the path list, identifying new peaks that match existing paths and
    ///                 adding any unmatched peaks to the list.
    /// @param[in,out]  corr_peak_vals          The normalised correlation peaks in a result interval, updated to remove
    ///                                         peaks that match existing paths. 
    /// @param[in,out]  det_burst_powers_mW     The mean power of the channel samples contributing to peak detections
    ///                                         in a result interval, updated to remove values for peaks thatmatch
    ///                                         existing paths.
    /// 
private:
    void update_paths(std::vector<float> &corr_peak_vals, 
                      std::vector<float> &det_burst_powers_mW);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          helper function to find a new detection peak close to the given offset
    /// @param[in]      offset                  Center point of the search
    /// @param[in]      corr_peak_vals          The correlation peak data to search
    /// @param[out]     offset_of_strongest     The offset in the input vector of the strongest peak in the range
    /// @return                                 true of a peak was found
    ///
private:
    bool find_new_peak_nearby(unsigned offset, 
                              const std::vector<float>& corr_peak_vals,
                              unsigned &offset_of_strongest);
                                             
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Helper function to zero results around a point in the correlation result buffers.
    /// @param[in]      offset                  Center point of zeroing.
    /// @param[in,out]  corr_peak_vals          The correlation peak data to modify.
    /// @param[in,out]  det_burst_powers_mW     The correlation burst power data to modify.
    ///
private:
    void annihilate( unsigned, std::vector<float>&, std::vector<float>& );

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Helper function to calculate the offset at which to centre the tracking window
    /// @details        Uses a centre-of-mass algorithm applied to the paths in the path list, with the SRS burst power
    ///                 estimate as the weight.
    /// @return                                 The new offset to use.
    ///
private:
    unsigned calc_new_track_offset(void);
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Algorithm predicate to signal if a path is no longer valid.
    /// @param[in]      a                       The path.
    /// @return                                 true if it is invalid.
    ///
private:
    static bool is_invalid( Path_t const & a ) { return a.valid_count <= VALID_COUNT_IS_INVALID; }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Algorithm predicate to signal if one path has a greater power than another.
    /// @param[in]      a                       One path.
    /// @param[in]      b                       Another path.
    /// @return                                 true if the power of a is greater than that of b.
    ///
private:
    static bool descending_power( Path_t const & a, Path_t const & b ) { return a.power > b.power; }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The functions used to interact with other classes that are replaced by calls to simmulators in sw test mode.
    //
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Correlator_4g class's get_cal_factor_ratio function that retrieves the ratio of
    ///                 the 4G calibration factors to the 3G ones from the Correlator class.
    /// @return                             The ratio to be applied to the 3G calibration factors when correcting the 
    ///                                     4G results returned by read_results().
    ///
private:    
#ifndef SWTEST_4G_SEARCH_TRACK
    double corr_get_cal_factor_ratio(void)
    {
        return (the_correlator->get_cal_factor_ratio());
    }
#else
    double corr_get_cal_factor_ratio_retval;
    double corr_get_cal_factor_ratio(void);
#endif
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Correlator_4g class's result_available function that determines if a new result
    ///                 is avaliable.
    /// @return                             true if a result is available.
    ///
private:    
#ifndef SWTEST_4G_SEARCH_TRACK
    bool corr_result_available(void)
    {
        return (the_correlator->result_available());
    }
#else
    bool corr_result_available_retval;
    bool corr_result_available(void);
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Correlator_4g class's read_results function that retrieves any results that may
    ///                 be available.
    /// @param[out]     corr_peak_counts    The vector of counts of the number of accumulated peaks.
    /// @param[out]     corr_peak_vals      The vector of correlation peaks spanning a result interval.
    /// @param[out]     corr_peak_mean_inputs_adc_lsb_sq
    ///                                     The vector of the mean of the input samples corresponding to a peak above
    ///                                     the threshold. Effectively this is the mean power of the detected SRS burst
    ///                                     in units of (ADC LSB)^2.
    /// @param[out]     corr_inp_max_mean_adc_lsb_sq
    ///                                     The maximum of the mean of the input samples spanned by the correlation
    ///                                     coefficients over the time interval spanned by the sequence. Effectively
    ///                                     this is the mean power of the strongest burst in units of (ADC LSB)^2.
    /// @param[out]     corr_inp_mean_adc_lsb_sq
    ///                                     The mean of the mean of the input samples spanned by the correlation
    ///                                     coefficients over the time interval spanned by the sequence. Effectively
    ///                                     this is the mean input power in units of (ADC LSB)^2.
    ///
private:    
#ifndef SWTEST_4G_SEARCH_TRACK
    void corr_read_results(std::vector<unsigned> &corr_peak_counts, 
                           std::vector<float> &corr_peak_vals, 
                           std::vector<float> &corr_peak_mean_inputs_adc_lsb_sq, 
                           float &corr_inp_max_mean_adc_lsb_sq,
                           float &corr_inp_mean_adc_lsb_sq)
    {
        return (the_correlator->read_results(corr_peak_counts, 
                                             corr_peak_vals, 
                                             corr_peak_mean_inputs_adc_lsb_sq, 
                                             corr_inp_max_mean_adc_lsb_sq,
                                             corr_inp_mean_adc_lsb_sq));
    }
#else
    std::vector<unsigned> corr_read_results_peak_counts;
    std::vector<float> corr_read_results_peak_vals;
    std::vector<float> corr_read_results_peak_mean_inputs_adc_lsb_sq;
    float corr_read_results_inp_max_mean_adc_lsb_sq;
    float corr_read_results_inp_mean_adc_lsb_sq;
    void corr_read_results(std::vector<unsigned> &corr_peak_counts, 
                           std::vector<float> &corr_peak_vals, 
                           std::vector<float> &corr_peak_mean_inputs_adc_lsb_sq, 
                           float &corr_inp_max_mean_adc_lsb_sq,
                           float &corr_inp_mean_adc_lsb_sq);
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Correlator_4g class's restart_correlation function that restarts the correlation
    ///                 with the same settings.
    ///
private:    
#ifndef SWTEST_4G_SEARCH_TRACK
    void corr_restart_correlation(void)
    {
        the_correlator->restart_correlation();
    }
#else
    bool corr_restart_correlation_called;
    void corr_restart_correlation(void);
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Correlator_4g class's change_mode function that changes the mode and offset and
    ///                 restarts the correlation with the new settings.
    /// @param[in]      m                   The new mode.
    /// @param[in]      start_offset_samples
    ///                                     Sample offset in result interval to begin correlation (in track mode).
    ///
private:    
#ifndef SWTEST_4G_SEARCH_TRACK
    void corr_change_mode(Correlator_4g::mode_t m, unsigned start_offset_samples = 0)
    {
        the_correlator->change_mode(m, start_offset_samples);
    }
#else
    bool                   corr_change_mode_called;
    Correlator_4g::mode_t  corr_change_mode_m;
    unsigned               corr_change_mode_start_offset_samples;
    void corr_change_mode(Correlator_4g::mode_t m, unsigned start_offset_samples = 0);
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Correlator_4g class's change_start function that changes the offset and restarts
    ///                 the correlation with the new settings.
    /// @param[in]      start_offset_samples
    ///                                     Sample offset in result interval to begin correlation (in track mode).
    ///
private:    
#ifndef SWTEST_4G_SEARCH_TRACK
    void corr_change_start(unsigned start_offset_samples = 0)
    {
        the_correlator->change_start(start_offset_samples);
    }
#else
    bool                   corr_change_start_called;
    unsigned               corr_change_start_start_offset_samples;
    void corr_change_start(unsigned start_offset_samples = 0);
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Correlator_4g class's enter_low_power_mode function that puts the correlator
    ///                 into low power mode.
    ///
private:    
#ifndef SWTEST_4G_SEARCH_TRACK
    void corr_enter_low_power_mode(void)
    {
        the_correlator->enter_low_power_mode();
    }
#else
    bool corr_enter_low_power_mode_called;
    void corr_enter_low_power_mode(void)
    {
        corr_enter_low_power_mode_called = true;
    }
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Correlator_4g class's exit_low_power_mode function that takes the correlator
    ///                 out of low power mode.
    ///
private:    
#ifndef SWTEST_4G_SEARCH_TRACK
    void corr_exit_low_power_mode(void)
    {
        the_correlator->exit_low_power_mode();
    }
#else
    bool corr_exit_low_power_mode_called;
    void corr_exit_low_power_mode(void)
    {
        corr_exit_low_power_mode_called = true;
    }
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the ADC's enter_low_power_mode function that puts the ADC into low power mode.
    ///
private:    
#ifndef SWTEST_4G_SEARCH_TRACK
    void adc_enter_low_power_mode(void)
    {
        ADC::enter_low_power_mode();
    }
#else
    bool adc_enter_low_power_mode_called;
    void adc_enter_low_power_mode(void)
    {
        adc_enter_low_power_mode_called = true;
    }
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the ADC's exit_low_power_mode function that takes the ADC out of low power mode.
    ///
private:    
#ifndef SWTEST_4G_SEARCH_TRACK
    void adc_exit_low_power_mode(void)
    {
        ADC::exit_low_power_mode(true);     //  4G uses the 3G settings
    }
#else
    bool adc_exit_low_power_mode_called;
    void adc_exit_low_power_mode(void)
    {
        adc_exit_low_power_mode_called = true;
    }
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Radio's enter_low_power_mode function that puts the Radio into low power mode.
    ///
private:    
#ifndef SWTEST_4G_SEARCH_TRACK
    void radio_enter_low_power_mode(void)
    {
        Radio::enter_low_power_mode();
    }
#else
    bool radio_enter_low_power_mode_called;
    void radio_enter_low_power_mode(void)
    {
        radio_enter_low_power_mode_called = true;
    }
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Radio's exit_low_power_mode function that takes the Radio out of low power mode.
    ///
private:    
#ifndef SWTEST_4G_SEARCH_TRACK
    void radio_exit_low_power_mode(void)
    {
        Radio::exit_low_power_mode(true);   //  4G uses the 3G settings
    }
#else
    bool radio_exit_low_power_mode_called;
    void radio_exit_low_power_mode(void)
    {
        radio_exit_low_power_mode_called = true;
    }
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Correlator_4g class's getFactor function that retrieves the linear power
    ///                 calibration factor.
    /// @details        The factor should be applied to linear power samples normalised to the channel filter
    ///                 output.  It includes a component that corresponds to the frontend gain.
    /// @param[in]      gain_dB             The frontend gain in dB.
    /// @return                             The calibration factor.
    ///
private:    
#ifndef SWTEST_4G_SEARCH_TRACK
    double pwrcal_getFactor(uint16_t gain_dB)
    {
        return (power_cal.getFactor(gain_dB));
    }
#else
    uint16_t pwrcal_getFactor_gain_dB;
    double pwrcal_getFactor_retval;
    double pwrcal_getFactor(uint16_t gain_dB);
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Ranging class's get_gain_dB function that retrieve the current gain setting as a
    ///                 dB value.
    /// @return                             The current gain.
    ///
private:    
#ifndef SWTEST_4G_SEARCH_TRACK
    int ranging_get_gain_dB( void )
    {
        return (ranging.get_gain_dB());
    }
#else
    int ranging_get_gain_dB_retval;
    int ranging_get_gain_dB( void );
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Ranging class's update function that is used to update the ranging.
    /// @param[in]      power               The mean %ADC sample power.
    /// @return                             The ranging status.
    ///
private:    
#ifndef SWTEST_4G_SEARCH_TRACK
    Ranging::RangingUpdateResult ranging_update(double power)
    {
        return (ranging.update(power));
    }
#else
    bool                         ranging_update_called;
    Ranging::RangingUpdateResult ranging_update_retval;
    double                       ranging_update_power;
    Ranging::RangingUpdateResult ranging_update(double power);
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The functions used to test the implementations of the principal functions.
    //

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Print paths
    //
private:    
#ifdef SWTEST_4G_SEARCH_TRACK
    void test_print_paths(void);
#endif    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Print correlation peaks and powers
    /// @details        Only prints the results with non-zero peaks.
    /// @param[in]      corr_peak_vals          The correlation peak data.
    /// @param[in]      corr_peak_burst_powers  The correlation burst power data.
    ///
private:    
#ifdef SWTEST_4G_SEARCH_TRACK
    void test_print_corr_peaks_and_pwrs(const std::vector<float> &corr_peak_vals, 
                                        const std::vector<float> &corr_peak_burst_powers);
#endif    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Print the annihilated peaks
    /// @details        Only prints the results with zero peaks.
    /// @param[in]      corr_peak_vals          The correlation peak data.
    /// @param[in]      det_burst_powers_mW     The correlation burst power data.
    ///
private:    
#ifdef SWTEST_4G_SEARCH_TRACK
    void test_print_annihilated_peaks(const std::vector<float> &corr_peak_vals, 
                                      const std::vector<float> &det_burst_powers_mW);
#endif    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Print the instance variables after a call to iterate() in the testIterate() function.
    /// @param[in]      iterate_retval          The value returned by the iterate function.
    ///
private:    
#ifdef SWTEST_4G_SEARCH_TRACK
    void test_print_instance_state(enum iter_res_t iterate_retval);
#endif    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Return the mode name.
    /// @param[in]      mode                    The mode.
    /// @return                                 The mode name.
    ///
private:    
#ifdef SWTEST_4G_SEARCH_TRACK
    std::string test_get_mode_name(enum mode_t mode);
#endif    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Return the name of the iterate() function's return value.
    /// @param[in]      iterate_retval          The return value.
    /// @return                                 The return value's name.
    ///
private:    
#ifdef SWTEST_4G_SEARCH_TRACK
    std::string test_get_iterate_retval_name(enum iter_res_t iterate_retval);
#endif
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Return the name of the correlator mode.
    /// @param[in]      mode                    The mode.
    /// @return                                 The mode name.
    ///
private:    
#ifdef SWTEST_4G_SEARCH_TRACK
    std::string test_get_corr_mode_name(Correlator_4g::mode_t mode);
#endif
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Test the correct_powers_and_discard_weak_peaks() function
    /// @details        There is no automated checking of the results, which must be manually confirmed.
    ///
private:    
#ifdef SWTEST_4G_SEARCH_TRACK
    void test_correct_powers_and_discard_weak_peaks(void);
#endif    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Test the find_new_peak_nearby() function
    /// @details        There is no automated checking of the results, which must be manually confirmed.
    ///
private:    
#ifdef SWTEST_4G_SEARCH_TRACK
    void test_find_new_peak_nearby(void);
#endif    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Test the annihilate() function
    /// @details        There is no automated checking of the results, which must be manually confirmed.
    ///
private:    
#ifdef SWTEST_4G_SEARCH_TRACK
    void test_annihilate(void);
#endif    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Test the update_paths() function
    /// @details        There is no automated checking of the results, which must be manually confirmed.
    ///
private:    
#ifdef SWTEST_4G_SEARCH_TRACK
    void test_update_paths(void);
#endif    

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Test the calc_new_track_offset() function
    /// @details        There is no automated checking of the results, which must be manually confirmed.
    ///
private:    
#ifdef SWTEST_4G_SEARCH_TRACK
    void test_calc_new_track_offset(void);
#endif    

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Test the track() function
    /// @details        There is no automated checking of the results, which must be manually confirmed.
    ///
private:    
#ifdef SWTEST_4G_SEARCH_TRACK
    void test_track(void);
#endif 
   
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Test the iterate() function
    /// @details        There is no automated checking of the results, which must be manually confirmed.
    ///
private:    
#ifdef SWTEST_4G_SEARCH_TRACK
    void test_iterate(void);
#endif
};

#endif

/***********************************************************************************************************************
 *  Copyright (c) 2021 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  Filename:   correlator_4g.hpp
 *  Author(s):  pdm
 *******************************************************************************************************************//**
 * File Description
 * ================
 *
 *  @file
 *  @brief          The declaration of ::Correlator_4g class that controls the 4G correlator in the FPGA.
 *  @details        Wrap up the hardware 4G correlator in a class that extends the processing with the averaging of the
 *                  peaks.
 **********************************************************************************************************************/
#ifndef __CORRELATOR_4G_HPP__
#define __CORRELATOR_4G_HPP__

#include "debug.hpp"
#include "design.hpp"
#include "fpga.hpp"
#include "types.hpp"
#include <map>
#include <memory>
#include <list>
#include <vector>

///
/// @brief          Provides an object that takes channel filter and correlation coefficients, controls the FPGA'safe
///                 correlation engine through an Fpga class instance, implements the averaging of the raw results and
///                 makes averaged correlation results available trhough its interface.
/// @details        Makes use of the ::Fpga module to talk to the low level hardware.  The channel filter and
///                 correlation coefficients are defined in files at construction time.  More than one correlator object
///                 shouldn't exist at once or bad things will happen as they reconfigure and talk to the single
///                 correlator we have in the FPGA.
///
class Correlator_4g
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The constants.
    //
private:
    //------------------------------------------------------------------------------------------------------------------
    //  Channel filter and correlator coefficient files.
    //
    
    ///
    /// @brief          The supported file format version.
    ///
    static const unsigned COEFF_FILE_FORMAT_VERSION = 1;
    
    ///
    /// @brief          The scaling factor of the decimal portion of the bandwidth (1 digit).
    ///
    static const double CHAN_BW_SF = 10.0;

    //------------------------------------------------------------------------------------------------------------------
    //  Channel filter and correlator coefficients.
    //
    
    ///
    /// @brief          The supported channel filter scaling factor.
    ///
    static const unsigned CHAN_FILT_COEFF_SCALING_FACTOR = 32768;
    
    ///
    /// @brief          The supported channel filter scaling factor.
    ///
    static const unsigned CHAN_FILT_MAX_ABS_COEFF_VAL = CHAN_FILT_COEFF_SCALING_FACTOR-1;
    
    ///
    /// @brief          The maximum channel filter length.
    /// @details        The filters are required to be have an odd number of taps and to be symmetric and so the length
    ///                 is defined in terms of this requirement.
    ///
    static const unsigned CHAN_FILT_MAX_LENGTH = 2*Fpga::NUM_CHANNEL_FILTER_TAP_VALS_4G-1;
    
    ///
    /// @brief          The largest Physical Channel ID.
    ///
    static const unsigned MAX_PHY_CHAN_ID = 29;
    
    ///
    /// @brief          The number of correlator coefficients.
    ///
    static const unsigned NUM_CORR_COEFFS = Design::NUM_CORRELATOR_TAPS_4G;

    //------------------------------------------------------------------------------------------------------------------
    //  Control of the FPGA correlator.
    //
    
    ///
    /// @brief          The delay between the programmed FPGA correlator's start time and the first sample to be passed
    ///                 to the channel filter.
    /// @details        This should be added to the delays arising in the filtering and correlation when calculating the
    ///                 FPGA correlator's start time from the time of the first sample in the result window (see 1QR003).
    ///
    static const unsigned FPGA_CORR_STARTUP_DELAY_SAMPLES = 11;
    
    ///
    /// @brief          The offset to be added to the current time when starting a correlation sequence.
    /// @details        Allows for the software processing delay and possible interruptions by the OS. A value of 15
    ///                 result buffers (with SRS periodicity of 2 ms) corresponds to 30 ms, which should be safe
    ///                 (occasional errors arose with a value of 10).
    ///
    static const unsigned CORR_START_TIME_OFFSET_RESULT_BUFFERS = 15;

    ///
    /// @brief          The raw threshold applied to the correlation peaks in high sensitivity mode.
    /// @details        This is the value used in the simulations reported in 1QR006.
    ///
    static const float HS_RAW_THRESHOLD = 0.15;

    ///
    /// @brief          The factor applied to the threshold for tracking.
    /// @details        This is an experimental value that is justified by the narrow tracking window being less likely
    ///                 to produce false positives.
    ///
    static const float TRACK_THRESHOLD_FACTOR_DB = -1.0;

    ///
    /// @brief          The raw threshold applied to the correlation peaks in low power mode.
    /// @details        This is the value determined by using the simulation scripts that generated the 1QR006 results.
    ///                 It is appropriate when no averaging is applied and produces an acceptably low false positive
    ///                 rate (around 0.1 %). The sensitivity appears to be about 8 dB worse.
    ///
    static const float LP_RAW_THRESHOLD = 0.3;
    ///
    /// @brief          Maximum number of results to retrieve in a call to the Fpga::get_4g_peak_data.
    /// @details        This is set to the number of results that can be held by the FPGA's result FIFO
    ///
    static const unsigned MAX_PEAKS_FROM_FPGA = 2048;

    //------------------------------------------------------------------------------------------------------------------
    //  Averaging.
    //

    ///
    /// @brief          The raw results to be non-coherently combined in search mode when low power consumption is the
    ///                 priority.
    ///
    static const unsigned LP_SEARCH_MODE_RAW_RESULTS_TO_AVG       = 1;

    ///
    /// @brief          The raw results to be non-coherently combined in track mode when low power consumption is the
    ///                 priority.
    ///
    static const unsigned LP_TRACK_MODE_RAW_RESULTS_TO_AVG        = 1;

    ///
    /// @brief          The raw results to be non-coherently combined in search mode when high sensitivity is the priority.
    ///
    static const unsigned HS_SEARCH_MODE_RAW_RESULTS_TO_AVG       = 2*45;

    ///
    /// @brief          The raw results to be non-coherently combined in track mode when high sensitivity is the priority.
    ///
    static const unsigned HS_TRACK_MODE_RAW_RESULTS_TO_AVG        = 45;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The types.
    //

    ///
    /// @brief          The accumulator for the correlation results.
    ///
private:
    typedef struct
    {
        ///
        /// @brief          The accumulated correlation output peak value.
        ///
        float corr_out_peak_acc;

        ///
        /// @brief          The accumulated correlation input sum at the peak.
        ///
        float corr_inp_sum_acc;

        ///
        /// @brief          The number of peak results accumulated.
        /// @details        This is only used to calculate the average of the correlation input sums. The "average" of
        ///                 the correlation output peak value uses the expected number of SRS bursts in the time
        ///                 interval.
        ///
        unsigned num_peaks;
    } sum_corr_inp_acc_t;

    ///
    /// @brief          The results from for the correlation statistics block.
    /// @details        The block accumulates statistics for the sums of the samples in every (overlapping) block
    ///                 entering the correlator.
    ///
private:
    typedef struct
    {
        ///
        /// @brief          The count of results included in the statistics.
        ///
        unsigned count;

        ///
        /// @brief          The peak of the correlation input sums.
        ///
        float peak_inp_sums;

        ///
        /// @brief          The peak of the correlation input sums.
        ///
        float sum_inp_sums;
    } corr_stat_res_t;

    ///
    /// @brief          The valid 4G correlator mode combinations.
    ///
public:
    typedef enum
    {
        ///
        /// @brief          Searching.
        /// @details        The correlation is continuous over the SRS periods defined by
        ///                 LP_SEARCH_MODE_RAW_RESULTS_TO_AVG or HS_SEARCH_MODE_RAW_RESULTS_TO_AVG
        ///
        SEARCH = Fpga::OP_MODE_SEARCH,

        ///
        /// @brief          Tracking.
        /// @details        The correlation is only covers a small window in each SRS period in an interval defined by
        ///                 LP_TRACK_MODE_RAW_RESULTS_TO_AVG or HS_TRACK_MODE_RAW_RESULTS_TO_AVG
        ///
        TRACK  = Fpga::OP_MODE_TRACK,

        ///
        /// @brief          Continuous tracking.
        /// @details        The correlation is continuous over the SRS periods defined by
        ///                 LP_CONT_TRACK_MODE_RAW_RESULTS_TO_AVG or HS_CONT_TRACK_MODE_RAW_RESULTS_TO_AVG
        ///
        CONT_TRACK
    } mode_t;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The values.
    //
private:
    ///
    /// @brief          The interface to the FPGA.
    ///
    Fpga &fpga;

    ///
    /// @brief          The channel filter 1 coefficients.
    ///
    std::vector<int16_t> chan_filt_1_coeffs;

    ///
    /// @brief          The channel filter 2 coefficients.
    ///
    std::vector<int16_t> chan_filt_2_coeffs;

    ///
    /// @brief          The correlator coefficients.
    ///
    std::vector<float> corr_coeffs;

    ///
    /// @brief          The tuning offset in Hz.
    ///
    freq_offset_t tuning_offset_Hz;
    
    ///
    /// @brief          The ratio to be applied to the 3G calibration factors when correcting the 4G results returned by
    ///                 read_results().
    ///
    double cal_factor_ratio;

    ///
    /// @brief          The accumulator for the FPGA correlation peak results.
    /// @details        Spans a frame, but only partially used when tracking.
    ///
    std::vector<sum_corr_inp_acc_t> fpga_corr_result_accumulator;

    ///
    /// @brief          The statistics of the correlator input sums.
    ///
    corr_stat_res_t corr_stat_res;

    ///
    /// @brief          The number of initial samples to flush the FPGA's signal processing chain.
    /// @details        This should be the sum of the channel filter lengths, plus the length of the correlation filter,
    ///                 minus 2 (see 1QS003).
    ///
    unsigned num_samples_to_flush_fpga_sig_proc_chain;

    ///
    /// @brief          The next ID to be used with an FPGA command.
    /// @details        The value is arbitrary and wraps, its purpose being to support debugging.
    ///
    uint8_t fpga_next_cmd_id;

    ///
    /// @brief          Signals that the low-power settings for the averaging should be used.
    ///
    bool low_power;

    ///
    /// @brief          The current mode combination.
    ///
    mode_t current_mode;

    ///
    /// @brief          The start of the tracking correlation in a result interval.
    ///
    unsigned track_start_offset_samples;

    ///
    /// @brief          The last correlation sequence that was requested of the Fpga class.
    /// @details        A new sequence cannot be requested until this has completed.
    ///
    std::list<Fpga::CorrCmd_4g_t> fpga_corr_seq_cmds;

    ///
    /// @brief          The raw results to be non-coherently combined in search mode.
    ///
    unsigned search_mode_raw_results_to_avg;

    ///
    /// @brief          The raw results to be non-coherently combined in track mode.
    ///
    unsigned track_mode_raw_results_to_avg;

    ///
    /// @brief          The threshold for the FPGA peak detector in search mode.
    ///
    double search_raw_threshold;

    ///
    /// @brief          The threshold for the FPGA peak detector in track mode.
    ///
    double track_raw_threshold;

    ///
    /// @brief          The threshold for the FPGA peak detector in continuous track mode.
    ///
    double cont_track_raw_threshold;

    ///
    /// @brief          The threshold for the averaging results in search mode.
    ///
    double search_avg_threshold;

    ///
    /// @brief          The threshold for the averaging results in track mode.
    ///
    double track_avg_threshold;

    ///
    /// @brief          The threshold for the averaging results in continuous track mode.
    ///
    double cont_track_avg_threshold;

    ///
    /// @brief          The total number of peaks retrieved from the FPGA during a correlation sequence (used for
    ///                 debugging).
    ///
    uint32_t total_num_peaks_retrieved;
    

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
    Correlator_4g(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Constructs an instance of the class.
    /// @details        Read the channel filter and correlator coefficients and pass them to the Fpga class.
    ///                 Calculate the number of samples required to flush the FPGA processing chain.
    ///                 Calculate the correlation thresholds.
    ///                 Determine the power correction factor from the channel bandwidth in the file names.
    ///                 Get the FPGA interface to setup the correlation for search mode.
    /// @param[in]      fpga                The FPGA interface.
    /// @param[in]      chan_filt_1_filename
    ///                                     The name of the file containing the channel filter 1 coefficients.
    /// @param[in]      chan_filt_2_filename
    ///                                     The name of the file containing the channel filter 2 coefficients.
    /// @param[in]      corr_filename       The name of the file containing the correlator coefficients.
    /// @param[in]      client_search_averages
    ///                                     The client-defined number of raw correlation results to average in search
    ///                                     mode (0 forces the use of the defaults that depend on the low_power
    ///                                     setting).
    /// @param[in]      client_track_averages
    ///                                     The client-defined number of raw correlation results to average in track
    ///                                     mode (0 forces the use of the defaults that depend on the low_power
    ///                                     setting).
    /// @param[in]      search_threshold_factor_dB
    ///                                     The client-defined factor applied to the averaging threshold in search mode.
    /// @param[in]      track_threshold_factor_dB
    ///                                     The client-defined factor applied to the averaging threshold in track mode.
    /// @param[in]      low_power           True if the low-power settings should be used, false if high-sensitivity is
    ///                                     required (partially or wholly over-ridden by num_search_averages or
    ///                                     num_track_averages being greater than zero).
    /// @param[in]      debug               Controls the debug messages that may be generated.
    /// @param[in]      restart             true means restart_correlator at end of constructor.  Optional parameter, default true
    ///
public:
    Correlator_4g(Fpga        &fpga,
                  std::string chan_filt_1_filename,
                  std::string chan_filt_2_filename,
                  std::string corr_filename,
                  unsigned    client_search_averages,
                  unsigned    client_track_averages,
                  double      search_threshold_factor_dB,
                  double      track_threshold_factor_dB,
                  bool        low_power,
                  unsigned    debug,
                  bool        restart = true);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The destructor has nothing to do.
    ///
public:
    virtual ~Correlator_4g();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Retrieves the tuning offset to be used.
    /// @return                             The tuning offset in Hz.
    ///
public:
    freq_offset_t get_tuning_offset(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Retrieves the ratio of the 4G calibration factors to the 3G ones.
    /// @return                             The ratio to be applied to the 3G calibration factors when correcting the 
    ///                                     4G results returned by read_results().
    ///
public:
    double get_cal_factor_ratio(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Enter low power mode.
    ///
public:
    void enter_low_power_mode(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Exit low power mode.
    ///
public:
    void exit_low_power_mode(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Determines if there is a new result available.
    /// @details        In the process of determining this the function retrieves the FPGA results and implements the
    ///                 accumulation of detected peaks. If a result has become available the function also reads the
    ///                 statistics.
    ///
    ///                 The function also monitors the correlation status, but this does not affect whether or not
    ///                 results are available. It only produces warning and error messages indicating a possible
    ///                 implementation problem.
    /// @return                             true if a result is available.
    ///
public:
    bool result_available(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Reads the most recent results.
    /// @details        Divides the accumulated peaks by the number of result intervals spanned and applies the
    ///                 averaging threshold so that peaks below it are set to 0.0.
    ///
    ///                 Divides the sum of the correlator input sums by the number of results accumulated and the number
    ///                 of samples spanned by the correlator coefficients to give the average value of the input samples 
    ///                 that controbuted to the peak.
    ///
    ///                 Divides the maximum mean input samples by the number of samples spanned by the correlator
    ///                 coefficients to give the maximum average input, which since the SRS burst should be the
    ///                 strongest is a measure of their power even if they are not being detected by the correlator.
    ///
    ///                 Divides the sum of the mean input sample by the number of samples spanned by the correlator
    ///                 coefficients and the number of means accumulated to give the long-term average input.
    ///
    ///                 As a result of these scalings all the signal-level results are measures of the average
    ///                 correlator- input samples. These samples are the instaneous powers of the samples output be the
    ///                 channel filter. The channel filter and the deimation filter that precedes it have unity DC gain
    ///                 and so the signal- level results have units of (ADC LSB)^2.
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
public:
    void read_results(std::vector<unsigned> &corr_peak_counts, 
                      std::vector<float> &corr_peak_vals, 
                      std::vector<float> &corr_peak_mean_inputs_adc_lsb_sq, 
                      float &corr_inp_max_mean_adc_lsb_sq,
                      float &corr_inp_mean_adc_lsb_sq);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Restart the current correlation with the same settings.
    ///
public:
    void restart_correlation(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Change the correlation mode (and optionally the start offset) and restart it.
    /// @param[in]      m                   The new mode.
    /// @param[in]      start_offset_samples
    ///                                     Sample offset in a result interval to begin correlation (in track mode).
    ///
public:
    void change_mode(mode_t m, unsigned start_offset_samples = 0);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Change the correlation start setting and restart it.
    /// @param[in]      start_offset_samples               
    ///                                     Sample offset in a result interval to begin correlation (in track mode).
    ///
public:
    void change_start(unsigned start_offset_samples);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Read the coefficients for channel filter 1.
    /// @details        Read the input file using the format specified in 1QR005, "4G Changes to the HHDFPHY SOftware",
    ///                 checking the filename format and the content format.
    ///
    ///                 Error and warning conditions:
    ///
    ///                     - Filename format error,        DebugStream::error,     return false with all-pass filter
    ///                     - Filter number not 1,          DebugStream::error,     return false with all-pass filter
    ///                     - File format version wrong,    DebugStream::error,     return false with all-pass filter
    ///                     - File format error,            DebugStream::error,     return false with all-pass filter
    ///                     - Scaling factor wrong,         DebugStream::error,     return false with all-pass filter
    ///                     - Filter size even or greater than the maximum
    ///                                                     DebugStream::error,     return false with all-pass filter
    ///                     - Tuning offset not multiple of resolution
    ///                                                     DebugStream::warn2,     return true with rounded offset
    ///                     - Coefficient out of range      DebugStream::warn2,     return true with clipped coefficient
    ///
    ///
    /// @param[in]      filename            The name of the coefficient file.
    /// @param[out]     chan_bw_MHz         The channel bandwidth in MHz.
    /// @param[out]     tuning_offset_Hz    The tuning offset in Hz.
    /// @param[out]     coeffs              The filter coefficients.
    /// @return                             true if the read was successful (all OK or a only warning issued, false
    ///                                     otherwise.
    ///
private:
    bool read_chan_filt_1_coeffs(std::string filename, 
                                 double &chan_bw_MHz, 
                                 freq_offset_t &tuning_offset_Hz,
                                 std::vector<int16_t> &coeffs);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Read the coefficients for channel filter 2.
    /// @details        Read the input file using the format specified in 1QR005, "4G Changes to the HHDFPHY SOftware",
    ///                 checking the filename format and the content format.
    ///
    ///                 Error and warning conditions:
    ///
    ///                     - Filename format error,        DebugStream::error,     return false with all-pass filter
    ///                     - Filter number not 1,          DebugStream::error,     return false with all-pass filter
    ///                     - File format version wrong,    DebugStream::error,     return false with all-pass filter
    ///                     - File format error,            DebugStream::error,     return false with all-pass filter
    ///                     - Scaling factor wrong,         DebugStream::error,     return false with all-pass filter
    ///                     - Filter size even or greater than the maximum
    ///                                                     DebugStream::error,     return false with all-pass filter
    ///                     - Tuning offset not multiple of resolution
    ///                                                     DebugStream::warn2,     return true with rounded offset
    ///                     - Coefficient out of range      DebugStream::warn2,     return true with clipped coefficient
    ///
    /// @param[in]      filename            The name of the coefficient file.
    /// @param[out]     chan_bw_MHz         The channel bandwidth in MHz.
    /// @param[out]     coeffs              The filter coefficients.
    /// @return                             true if the read was successful (all OK or a only warning issued, false
    ///                                     otherwise.
    ///
private:
    bool read_chan_filt_2_coeffs(std::string filename, 
                                 double &chan_bw_MHz, 
                                 std::vector<int16_t> &coeffs);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Read the coefficients for the correlator.
    //  @details        Read the input file using the format specified in 1QR005, "4G Changes to the HHDFPHY SOftware",
    ///                 checking the filename format and the content format.
    ///
    ///                 Error and warning conditions:
    ///
    ///                     - Filename format error,        DebugStream::error,     return false with all-zero coefficients
    ///                     - File format version wrong,    DebugStream::error,     return false with all-zero coefficients
    ///                     - File format error,            DebugStream::error,     return false with all-zero coefficients
    ///                     - Wrong number of coefficients  DebugStream::error,     return false with all-zero coefficients
    ///                     - Physical Channel ID out of range
    ///                                                     DebugStream::warn2,     return true
    ///                     - Coefficient mean not 0.0 or variance not 1.0 (error > 1e-3)
    ///                                                     DebugStream::warn2,     return true
    ///                     - Coefficient mean not 0.0 or variance not 1.0 (error > float epsilon)
    ///                                                     DebugStream::infor3,    return true
    ///
    /// @param[in]      filename            The name of the coefficient file.
    /// @param[out]     chan_bw_MHz         The channel bandwidth in MHz.
    /// @param[out]     coeffs              The filter coefficients.
    /// @return                             true if the read was successful (all OK or a only warning issued, false
    ///                                     otherwise.
    ///
private:
    bool read_corr_coeffs(std::string filename, 
                          double &chan_bw_MHz, 
                          std::vector<float> &coeffs);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Calculate the number of raw results to be used for averaging based on the current mode and power
    ///                 setting.
    /// @return                             The number of raw correlation results
    ///
private:
    unsigned get_num_raw_results_for_averaging(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Calculate the number of raw results to average.
    /// @details        The default values depend on whether low power consumption or high sensitivity is required, but
    ///                 these are over-ridden if the client-supplied values are greater than zero.
    /// @param[in]      client_search_averages
    ///                                     The client-defined number of raw correlation results to average in search
    ///                                     mode (0 forces the use of the defaults that depend on the low_power
    ///                                     setting).
    /// @param[in]      client_track_averages
    ///                                     The client-defined number of raw correlation results to average in track
    ///                                     mode (0 forces the use of the defaults that depend on the low_power
    ///                                     setting).
    ///
private:
    void calc_averages(unsigned client_search_averages, unsigned client_track_averages);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Calculate the thresholds based on the current mode.
    /// @details        The raw threshold is used by the FPGA to limit the number of correlation peaks that it reports.
    ///                 The averaging threshold is used by the Correlation class after it has performed the averaging
    ///                 and it determines the final false positive rate and sensitivity.
    ///
    ///                 The raw thresholds are based on the results reported in 1QR006, "SRS Performance Comparison" and
    ///                 they should allow the high sensitivity target to be met after the averaging is applied. The
    ///                 averaging thresholds are calculated from the raw thresholds by allowing for the processing gain
    ///                 of the averaging, i.e. sqrt(number of values averaged).
    ///
    ///                 The threshold factor is ony applied to the averaging threshold because applying it to the raw
    ///                 threshold risks too many FPGA peaks being produced so that some may have to be discarded.
    /// @param[in]      search_threshold_factor_dB
    ///                                     The client-defined factor applied to the averaging threshold in search mode.
    /// @param[in]      track_threshold_factor_dB
    ///                                     The client-defined factor applied to the averaging threshold in track mode.
    ///
private:
    void calc_thresholds(double search_threshold_factor_dB, double track_threshold_factor_dB);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Reads the correlator's result FIFO until it is empty.
    /// @details        Retrieve the peaks from the FPGA in a loop and accumulate the results Each iteration should
    ///                 empty the FPGA FIFO and while those peaks are processed the FPGA may generate a few more peaks.
    ///                 Therefore continue to iterate until no more peaks are reported.
    ///
private:
    void read_and_accumulate_fpga_results(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Set the FPGA's test register
    /// @param[in]      val                 The value to write to the register.
    ///
public:
    void set_test_register(uint32_t val);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Set the debug verbosity level.
    /// @param[in]      level               The new verbosity level.
    ///
public:
    void debug_printing(unsigned level);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The functions used to interact with the Fpga class that are replaced by calls to simmulators in sw test mode.
    //

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Fpga class's send_4g_coefs function that writes the coefficients for the channel
    ///                 filters and correlator for the 4G fpga.
    /// @details        If there is a fault with any of the supplied coefficients nothing is progrrammed to the fpga
    /// @param[in]      channel1    Array of int16_t with parameters for channel filter stage 1
    /// @param[in]      channel2    Array of int16_t with parameters for channel filter stage 1
    /// @param[in]      correl_taps Array of 320 correlator taps in IEEE 754 binary32 format
    /// @returns        false if there is a problem with any of the supplied parameters/coefficients
    ///
private:
#ifndef SWTEST_4G_CORR
    bool fpga_send_4g_coefs(const std::vector<int16_t>&channel1,
                            const std::vector<int16_t>&channel2,
                            const std::vector<float>&correl_taps)
    {
        return (fpga.send_4g_coefs(channel1, channel2, correl_taps));
    }
#else
    bool fpga_send_4g_coefs(const std::vector<int16_t>&channel1,
                            const std::vector<int16_t>&channel2,
                            const std::vector<float>&correl_taps);
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Fpga class's get_4g_time function that returns the current FPGA time.
    /// @returns        Current FPGA time in a uint64_t
    ///
private:
#ifndef SWTEST_4G_CORR
    uint64_t fpga_get_4g_time(void)
    {
        return (fpga.get_4g_time());
    }
#else
    uint64_t fpga_get_4g_time_retval;
    uint64_t fpga_get_4g_time(void);
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Fpga class's set_4g_threshold_sqrd that is used to set the threshold.
    /// @param[in]      threshold_sqrd   float squared threshold
    ///
private:
#ifndef SWTEST_4G_CORR
    void fpga_set_4g_threshold_sqrd(float threshold_sqrd)
    {
        fpga.set_4g_threshold_sqrd(threshold_sqrd);
    }
#else
    void fpga_set_4g_threshold_sqrd(float threshold_sqrd);
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Fpga class's get_4g_statistics that is used to control and retrieve results from
    ///                 the statistics block.
    /// @param[in]      restart  Restart engine afterwards?
    /// @param[out]     peak     Statistics engine peak values
    /// @param[out]     sum      Statistics engine sum of values
    /// @returns        Number of values contributing to the results
    ///
private:
#ifndef SWTEST_4G_CORR
    uint32_t fpga_get_4g_statistics(bool restart, float &peak, float &sum)
    {
        return (fpga.get_4g_statistics(restart, peak, sum));
    }
#else
    corr_stat_res_t fpga_get_4g_statistics_results;
    uint32_t fpga_get_4g_statistics(bool restart, float &peak, float &sum);
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Fpga class's setup_4g_correlation that is used to setup a correlation sequence.
    /// @param[in]      cmds  a list of CorrCmd_4g_t type
    /// @returns        unsigned, the number of commands that were written
    ///
private:
#ifndef SWTEST_4G_CORR
    uint32_t fpga_setup_4g_correlation(std::list<Fpga::CorrCmd_4g_t> cmds)
    {
        return (fpga.setup_4g_correlation(cmds));
    }
#else
    uint32_t fpga_setup_4g_correlation(std::list<Fpga::CorrCmd_4g_t> cmds);
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Fpga class's get_4g_peak_data that reads the status of recent colleration
    ///                 commands.
    /// @param[in]      maxNumPeaks the maximum number of peaks to write into the destination array
    /// @param[out]     peaks array of peaks read from the peaks fifo
    /// @param[out]     numPeaks actual number of peaks placed into peaks array
    /// @returns        bool, "false" if the FIFO's overflow flag is set
    ///
private:
#ifndef SWTEST_4G_CORR
    bool fpga_get_4g_peak_data(size_t maxNumPeaks, Fpga::CorrPeak_4g_t peaks[], uint32_t &numPeaks)
    {
        return (fpga.get_4g_peak_data(maxNumPeaks, peaks, numPeaks));
    }
#else
    std::list<Fpga::CorrPeak_4g_t> fpga_get_4g_peak_data_peaks;
    bool fpga_get_4g_peak_data_retval;
    bool fpga_get_4g_peak_data(size_t maxNumPeaks, Fpga::CorrPeak_4g_t peaks[], uint32_t &numPeaks);
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Wrapper for the Fpga class's get_4g_correlation_command_status that teads the status of recent
    ///                 colleration commands
    /// @param[out]     cmd_status a list of CorrCmdStatus_4g_t type showing command ids and associated status
    /// @returns        bool, "false" if the FIFO's overflow flag is set
    ///
private:
#ifndef SWTEST_4G_CORR
    bool fpga_get_4g_correlation_command_status(std::list<Fpga::CorrCmdStatus_4g_t> &cmd_status)
    {
        return (fpga.get_4g_correlation_command_status(cmd_status));
    }
#else
    std::list<Fpga::CorrCmdStatus_4g_t> fpga_get_4g_correlation_command_status_cmd_status;
    bool fpga_get_4g_correlation_command_status_retval;
    bool fpga_get_4g_correlation_command_status(std::list<Fpga::CorrCmdStatus_4g_t> &cmd_status);
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The functions used to test the implementations of the principal functions.
    //
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Print the correlation accumulator results
    /// @details        Only prints the results with non-zero counts.
    ///
private:    
#ifdef SWTEST_4G_CORR
    void test_print_acc_results(void);
#endif    

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Print the correlation statitics results if the count is greater than zero.
    ///
private:    
#ifdef SWTEST_4G_CORR
    void test_print_stat_results(void);
#endif    

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Print the results returned by the read_results function.
    /// @details        Only prints the vector results with non-zero values.
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
#ifdef SWTEST_4G_CORR
    void test_print_read_results_returns(std::vector<unsigned> corr_peak_counts, 
                                         std::vector<float> corr_peak_vals, 
                                         std::vector<float> corr_peak_mean_inputs_adc_lsb_sq, 
                                         float corr_inp_max_mean_adc_lsb_sq,
                                         float corr_inp_mean_adc_lsb_sq);
#endif    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Test the restart_correlation() function
    /// @details        There is no automated checking of the results, which must be manually confirmed.
    ///
private:    
#ifdef SWTEST_4G_CORR
    void test_restart_correlation(void);
#endif    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Test the result_available() function
    /// @details        There is no automated checking of the results, which must be manually confirmed.
    ///
private:    
#ifdef SWTEST_4G_CORR
    void test_result_available(void);
#endif    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Test the read_results() function
    /// @details        There is no automated checking of the results, which must be manually confirmed.
    ///
private:    
#ifdef SWTEST_4G_CORR
    void test_read_results(void);
#endif    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Test the FPGA timing
    /// @details        Tests the FPGA timing in a tight loop that reads the timing value twice and then sleeps for
    ///                 a period of time. The results are stored in arrays and then printed as debug strings after
    ///                 the loop is completed. The analysis of the results is manual.
    ///
    ///                 Caveat the default number of loop iterations and sleep time means that the test takes 34 minutes
    ///                 to complete as this ensures that the upper 32 bits of the FPGA counter is also tested (at least
    ///                 its bit 0).
    ///
private:    
#ifdef SWTEST_4G_FPGA_TIMING
    void test_fpga_timing(void);
#endif    

#ifdef SWTEST_4G_FPGA
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Start correlating continuously
    ///
public:
    void start_cont_correlation(void);
#endif

};

#endif

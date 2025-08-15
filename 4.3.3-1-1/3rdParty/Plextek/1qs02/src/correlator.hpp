/***********************************************************************************************************************
 *  Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 *  --------------------------------------------------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/correlator.hpp $
 * $Revision: 6338 $
 * $Author: pdm $
 * $Date: 2011-07-15 14:57:08 +0100 (Fri, 15 Jul 2011) $
 *******************************************************************************************************************//**
 *  @file
 *  @brief          The declaration of ::Correlator class that controls the correlator in the FPGA.
 *  @details        Wrap up the hardware correlator in a class with any additional processing fluff needed.  Uses the
 *                  low level FPGA interface talk to the hardware.
 **********************************************************************************************************************/
#ifndef __CORRELATOR_HPP__
#define __CORRELATOR_HPP__

#include "correlationsource.hpp"
#include "debug.hpp"
#include "design.hpp"
#include "fpga.hpp"
#include <map>
#include <memory>
#include <list>
#include <vector>

///
/// @brief          Provides an object that takes a source of chips and returns correlation data and peak information.
/// @details        Makes use of the ::FPGA module to talk to the low level hardware.  A source of chips to correlate
///                 against is assigned at construction time.  More than one correlator object shouldn't exist at once
///                 or bad things will happen as they reconfigure and talk to the single correlator we have in the fpga.
///
class Correlator
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The constants.
    //
private:
    //------------------------------------------------------------------------------------------------------------------
    //  Frequency domain coefficient controls.
    //

    ///
    /// @brief          The limits for the components of the frequency domain coefficients.
    /// @details        The limits are symmetric because a minimum of -127 ensures that the result of the complex
    ///                 multiplication has components that always fit in 32-bit words.
    ///
    static const int8_t MAX_ABS_FREQ_DOM_COEFF_VAL = 127;

    //------------------------------------------------------------------------------------------------------------------
    //  Filter sizes.
    //

    ///
    /// @brief          The size of the filter used for calibrating the frequency reference.
    /// @details        The uncertainty in the frequency means that the performance of a longer filter could be
    ///                 seriously affected by the phase rotation across the samples spanned by the filter.
    ///
    ///                 See VAT002, "3G Sensitivity"
    ///
    static const int FREQ_CAL_FILTER_SIZE = 256;

    ///
    /// @brief          The size of the filter used for the search and tracking mode.
    /// @details        The uncertainty in the frequency means that the performance of a longer filter could be
    ///                 seriously affected by the phase rotation across the samples spanned by the filter.
    ///
    ///                 See VAT002, "3G Sensitivity"
    ///
    static const int SEARCH_TRACK_FILTER_SIZE = 2048;

    //------------------------------------------------------------------------------------------------------------------
    //  Averaging.
    //

    ///
    /// @brief          The number of slots per frame to be non-coherently combined in reference calibration mode when
    ///                 low power consumption is the priority.
    ///
    static const unsigned LP_FREQ_CAL_MODE_SLOTS_PER_FRAME_TO_AVG = 1;

    ///
    /// @brief          The frames to be non-coherently combined in reference calibration mode when low power consumption
    ///                 is the priority.
    ///
    static const unsigned LP_FREQ_CAL_MODE_FRAMES_TO_AVG          = 1;

    ///
    /// @brief          The number of slots per frame to be non-coherently combined in search mode when low power
    ///                 consumption is the priority.
    ///
    static const unsigned LP_SEARCH_MODE_SLOTS_PER_FRAME_TO_AVG = 1;

    ///
    /// @brief          The frames to be non-coherently combined in search mode when low power consumption is the
    ///                 priority.
    ///
    static const unsigned LP_SEARCH_MODE_FRAMES_TO_AVG          = 1;

    ///
    /// @brief          The number of slots per frame to be non-coherently combined in track mode when low power
    ///                 consumption is the priority.
    ///
    static const unsigned LP_TRACK_MODE_SLOTS_PER_FRAME_TO_AVG  = 1;

    ///
    /// @brief          The frames to be non-coherently combined in track mode when low power consumption is the
    ///                 priority.
    ///
    static const unsigned LP_TRACK_MODE_FRAMES_TO_AVG           = 1;

    ///
    /// @brief          The number of slots per frame to be non-coherently combined in reference calibration mode when
    ///                 high sensitivity is the priority.
    ///
    static const unsigned HS_FREQ_CAL_MODE_SLOTS_PER_FRAME_TO_AVG = 15;

    ///
    /// @brief          The frames to be non-coherently combined in reference calibration mode when high sensitivity is
    ///                 the priority.
    ///
    static const unsigned HS_FREQ_CAL_MODE_FRAMES_TO_AVG          = 1;

    ///
    /// @brief          The number of slots per frame to be non-coherently combined in search mode when high sensitivity
    ///             is the priority.
    ///
    static const unsigned HS_SEARCH_MODE_SLOTS_PER_FRAME_TO_AVG = 15;

    ///
    /// @brief          The frames to be non-coherently combined in search mode when high sensitivity is the priority.
    ///
    static const unsigned HS_SEARCH_MODE_FRAMES_TO_AVG          = 3;

    ///
    /// @brief          The number of slots per frame to be non-coherently combined in track mode when high sensitivity
    ///                 is the priority.
    ///
    static const unsigned HS_TRACK_MODE_SLOTS_PER_FRAME_TO_AVG  = 15;

    ///
    /// @brief          The frames to be non-coherently combined in track mode when high sensitivity is the priority.
    ///
    static const unsigned HS_TRACK_MODE_FRAMES_TO_AVG           = 2;

    //------------------------------------------------------------------------------------------------------------------
    //  Threshold offsets.
    //

    ///
    /// @brief          The threshold used for calibrating the frequency reference when low power consumption is the
    ///                 priority.
    ///
    static const double LP_FREQ_CAL_MODE_REL_THRESHOLD_DB = -12.1;

    ///
    /// @brief          The threshold used for the search mode when low power consumption is the priority.
    ///
    static const double LP_SEARCH_MODE_REL_THRESHOLD_DB = -21.1;

    ///
    /// @brief          The threshold used for the tracking mode when low power consumption is the priority.
    ///
    static const double LP_TRACK_MODE_REL_THRESHOLD_DB = -21.1;

    ///
    /// @brief          The threshold used for calibrating the frequency reference when high sensitivity is the
    ///                 priority.
    ///
    static const double HS_FREQ_CAL_MODE_REL_THRESHOLD_DB = -19.0;

    ///
    /// @brief          The threshold used for the search mode when high sensitivity is the priority.
    /// @details        The -30.3 dB component is based on theory, which ignores the heuristic filtering.  The -4.0
    ///                 term is based on empirical results at 1.93 GHz.
    ///
    static const double HS_SEARCH_MODE_REL_THRESHOLD_DB = -30.3-4.0;

    ///
    /// @brief          The threshold used for the tracking mode when high sensitivity is the priority.
    /// @details        The -30.3 dB component is based on theory, which ignores the heuristic filtering.  The -2.0
    ///                 term is based on empirical results at 1.93 GHz.
    ///
    static const double HS_TRACK_MODE_REL_THRESHOLD_DB = -30.3-2.0;

    //------------------------------------------------------------------------------------------------------------------
    //  Gain schedules and scaling factors.
    //

    ///
    /// @brief          The gain schedule for the forward FFT used for calibrating the frequency reference.
    /// @details        The scaling is applied after the FFT has been completed and is a right-shift by the sum of the
    ///                 gain schedule taken 2 bits at a time.  This mapping arose because the the forward FFT
    ///                 originally used the same style of scaling as the inverse FFT.
    ///
    ///                 See X1R008, "Gain Control and Threshold Setting"
    ///
    static const uint32_t FREQ_CAL_MF_FWD_FFT_GAINS =   (3 <<  0)
                                                      | (0 <<  2);

    ///
    /// @brief          The gain schedule for the inverse FFT used for calibrating the frequency reference.
    /// @details        The scaling is applied during the inverse FFT.  At the end of every two Radix 2 stages the
    ///                 samples a right-shifted by the amount specified by the corresponding two bits in the gain
    ///                 schedule.
    ///
    ///                 See X1R008, "Gain Control and Threshold Setting"
    ///
    static const uint32_t FREQ_CAL_MF_INV_FFT_GAINS =   (2 <<  0)
                                                      | (1 <<  2)
                                                      | (1 <<  4)
                                                      | (1 <<  6)
                                                      | (1 <<  8)
                                                      | (0 << 10);

    ///
    /// @brief          The combined gain schedule for the forward and inverse FFTs used for calibrating the frequency
    ///                 reference.
    ///
    static const uint32_t FREQ_CAL_MF_FFT_GAINS =   (FREQ_CAL_MF_INV_FFT_GAINS << 16)
                                                  | FREQ_CAL_MF_FWD_FFT_GAINS;

    ///
    /// @brief          The gain schedule for the forward FFT used for the search and tracking mode.
    /// @details        The scaling is applied after the FFT has been completed and is a right-shift by the sum of the
    ///                 gain schedule taken 2 bits at a time.  This mapping arose because the the forward FFT
    ///                 originally used the same style of scaling as the inverse FFT.
    ///
    ///                 See X1R008, "Gain Control and Threshold Setting"
    ///
    static const uint32_t SEARCH_TRACK_MF_FWD_FFT_GAINS =   (3 <<  0)
                                                          | (2 <<  2);

    ///
    /// @brief          The gain schedule for the inverse FFT used for search and tracking mode.
    /// @details        The scaling is applied during the inverse FFT.  At the end of every two Radix 2 stages the
    ///                 samples a right-shifted by the amount specified by the corresponding two bits in the gain
    ///                 schedule.
    ///
    ///                 See X1R008, "Gain Control and Threshold Setting"
    ///
    static const uint32_t SEARCH_TRACK_MF_INV_FFT_GAINS =   (2 <<  0)
                                                          | (1 <<  2)
                                                          | (1 <<  4)
                                                          | (1 <<  6)
                                                          | (1 <<  8)
                                                          | (2 << 10);

    ///
    /// @brief          The combined gain schedule for the forward and inverse FFTs used for search and tracking
    ///                 mode.
    ///
    static const uint32_t SEARCH_TRACK_MF_FFT_GAINS =   (SEARCH_TRACK_MF_INV_FFT_GAINS << 16)
                                                      | SEARCH_TRACK_MF_FWD_FFT_GAINS;

    ///
    /// @brief          The matched-filter power gain factor for calibrating the frequency reference.
    /// @details        The coherent gain is obtained by multplying this factor by the square of the filter length and
    ///                 the non-coherent gain is obtained by multplying this factor by the filter length.
    ///
    ///                 See X1R008, "Gain Control and Threshold Setting"
    ///
    static const double FREQ_CAL_MF_PWR_GAIN_FACTOR = 1.0/(64.0*64.0);

    ///
    /// @brief          The matched-filter power gain factor for the search and tracking mode.
    /// @details        The coherent gain is obtained by multplying this factor by the square of the filter length and
    ///                 the non-coherent gain is obtained by multplying this factor by the filter length.
    ///
    ///                 See X1R008, "Gain Control and Threshold Setting"
    ///
    static const double SEARCH_TRACK_MF_PWR_GAIN_FACTOR = 2.0/(512.0*512.0);

    ///
    /// @brief          The scaling factor to be applied to the peak power results when the matched-filter is in I/Q
    ///                 mode.
    /// @details        The FPGA divides the I^2+Q^2 by 2 to avoid an overflow in the 32-bit result.
    ///
    static const double PEAK_POWER_SCALING_FACTOR_IQ_MODE = 2.0;

    ///
    /// @brief          The scaling factor to be applied to the peak power results when the matched-filter is in power
    ///                 mode.
    /// @details        The FPGA divides the I^2+Q^2 by 512 to allow headroom in the 32-bit samples for the summation of
    ///                 multiple results.
    ///
    static const double PEAK_POWER_SCALING_FACTOR_PWR_MODE = 512.0;

    ///
    /// @brief          The scaling factor to be applied to the power samples when the matched-filter is in I/Q mode.
    /// @details        The software calculates the power samples from the I/Q samples produced by the FPGA.
    ///
    static const double POWER_SAMPLE_SCALING_FACTOR_IQ_MODE = 1.0;

    ///
    /// @brief          The scaling factor to be applied to the power samples when the matched-filter is in power
    ///                 mode.
    /// @details        The FPGA divides the I^2+Q^2 by 512 to allow headroom in the 32-bit samples for the summation of
    ///                 multiple results.
    ///
    static const double POWER_SAMPLE_SCALING_FACTOR_PWR_MODE = 512.0;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The types.
    //

    ///
    /// @brief          The valid correlator mode combinations.
    ///
public:
    typedef enum
    {
        ///
        /// @brief          Reference calibration using antenna 1.
        ///
        CAL_ANT1    = Fpga::OP_MODE_CAL    | Fpga::ANT_MODE_ANT1,

        ///
        /// @brief          Reference calibration using antenna 2.
        ///
        CAL_ANT2    = Fpga::OP_MODE_CAL    | Fpga::ANT_MODE_ANT2,

        ///
        /// @brief          Search using antenna 1.
        ///
        SEARCH_ANT1 = Fpga::OP_MODE_SEARCH | Fpga::ANT_MODE_ANT1,

        ///
        /// @brief          Search using antenna 2.
        ///
        SEARCH_ANT2 = Fpga::OP_MODE_SEARCH | Fpga::ANT_MODE_ANT2,

        ///
        /// @brief          Track using antenna 1.
        ///
        TRACK_ANT1  = Fpga::OP_MODE_TRACK  | Fpga::ANT_MODE_ANT1,

        ///
        /// @brief          Track using antenna 2.
        ///
        TRACK_ANT2  = Fpga::OP_MODE_TRACK  | Fpga::ANT_MODE_ANT2,

        ///
        /// @brief          Track using alternating antennas, starting with antenna 1.
        ///
        TRACK_SWAP  = Fpga::OP_MODE_TRACK  | Fpga::ANT_MODE_SWAP_ANTS_START1
    } mode_t;

    ///
    /// @brief          Information describing an interpolated peak identified by the get_peak() function.
    ///
public:
    typedef struct
    {
        ///
        /// @brief          Frame number of the first frame contributing to the result.
        ///
        int64_t frame;

        ///
        /// @brief          The interpolated offset in samples of the peak in the result.
        ///
        double  offset;

        ///
        /// @brief          The power of the peak normalised to the LSB^2 of the channel filter samples.
        ///
        double  pwr;
    } peak_pos_t;


    ///
    /// @brief          The result returned by the busy() function.
    ///
public:
    typedef enum
    {
        ///
        /// @brief          A result is available.
        ///
        RESULT_AVAILABLE,

        ///
        /// @brief          The FPGA is working to provide a result.
        ///
        RESULT_PENDING,

        ///
        /// @brief          The FPGA has spent too long producing a result - an error must have occurred.
        ///
        RESULT_TIMEOUT
    } busy_result_t;

    ///
    /// @brief          The peak information returned by the get_peak_data() function.
    ///
    typedef struct
    {
        ///
        /// @brief          true if the peak power exceeds the threshold.
        ///
        bool        exceeds_threshold;

        ///
        /// @brief          The offset in samples into the result buffer of the peak.
        ///
        uint32_t    sample_offset;

        ///
        /// @brief          The power of the peak normalised to the LSB^2 of the channel filter samples.
        ///
        double      sample_power;
    } peak_t;

    ///
    /// @brief          Block of chips to correlate against.
    ///
    typedef CorrelationSource::block_t      block_t;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The values.
    //
private:
    ///
    /// @brief          The interface to the FPGA.
    ///
    Fpga &fpga;

    ///
    /// @brief          The source of the reference chip sequence for the correlation.
    ///
    std::auto_ptr<CorrelationSource> cs;

    ///
    /// @brief          Signals that the low-power settings for the averaging should be used.
    ///
    bool low_power;

    ///
    /// @brief          The current mode combination.
    ///
    mode_t current_mode;

    ///
    /// @brief          The size of the reference chip sequence for the correlation.
    ///
    int size;

    ///
    /// @brief          The FFT gain schedule.
    ///
    uint32_t fft_gains;

    ///
    /// @brief          The start of the correlation in a frame or slot.
    ///
    unsigned current_start;

    ///
    /// @brief          The threshold for the peak detector.
    ///
    double current_threshold;

    ///
    /// @brief          The debug stream.
    /// @details        Debug messages are sent to this stream.  If the debugging level has been appropriately
    ///                 configured they appear on the standard output console.  See ::DebugStream.
    ///
    DebugStream debugStr;


    ///
    /// @brief          Controls the writing of working data to debug files.
    ///
    bool dump_files;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The default constructor is declared but not defined in order to prevent a compiler generated
    ///                 version appearing.
    ///
private:
    Correlator(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Constructs an instance of the class.
    /// @details        Read the reference sequence chips from the correlation source, convert them into frequency
    ///                 domain coefficients for the overlap-add implementation iof the matched filter and write the
    ///                 results to the FPGA.  Get the FPGA interface to setup the correlation.
    /// @param[in]      fpga                The FPGA interface.
    /// @param[in]      cs_in               Correlation source object for source of chips.
    /// @param[in]      low_power           True if the low-power settings should be used, false if high-sensitivity is
    ///                                     required.
    /// @param[in]      m                   Mode of correlator.
    /// @param[in]      start               Sample offset in frame to begin correlation.
    /// @param[in]      debug               Controls the debug messages that may be generated.
    /// @param[in]      dump                true = dump fft results to file for debug
    ///
public:
    Correlator(Fpga                             &fpga,
               std::auto_ptr<CorrelationSource> cs_in,
               bool                             low_power,
               mode_t                           m,
               unsigned                         start,
               unsigned                         debug,
               bool                             dump);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The destructor has nothing to do.
    ///
public:
    virtual ~Correlator();

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
    /// @brief          Retrieves the frame number of the first frame that contributed to the current result.
    /// @return                             The frame number.
    ///
public:
    int64_t get_frame(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Retrieves the peak in a search window in the current result.
    /// @details        The function not only retrieves the basic peak data but also performsa quadratic fit to estimate
    ///                 the true position of the peak.
    /// @param[in]      search_start        The first sample in the search window.
    /// @param[in]      search_len          The length of the search window.
    /// @return                             The peak data.
    ///
public:
    peak_pos_t get_peak(uint32_t search_start, uint32_t search_len);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Retrieves the peak in the current result.
    /// @param[out]     raw_peak            The peak data.
    /// @return                             The frame number.
    ///
public:
    int64_t get_peak_data(peak_t &raw_peak);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Retrieves the sample offset of the peak in the current result.
    /// @return                             The sample offset.
    ///
public:
    uint32_t get_peak_sample_offset(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Gets the samples for a window in the current result.
    /// @param[in]      start               The start of the window.
    /// @param[in]      len                 The length of the window.
    /// @param[in]      alt_ant             true if the samples for the alternate antenna should be read, false
    ///                                     otherwise.
    /// @param[out]     data                The destination for the samples.
    /// @param[in]      use_threshold       true if samples below the threshold should be set to 0.0.
    /// @return                             The frame number.
    ///
public:
    int64_t get_raw_corr_data(uint32_t            start,
                              uint32_t            len,
                              bool                alt_ant,
                              std::vector<double> &data,
                              bool                use_threshold=true);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Retrieves the channel RSSI and sets the threshold.
    /// @return                             The RSSI normalised to the LSB^2 of the channel filter samples.
    ///
public:
    double get_channel_power(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Retrieves the ADC RSSI.
    /// @return                             The RSSI normalised to the LSB^2 of the ADC samples.
    ///
public:
    double get_adc_power(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Signals that the software has finished with the current result so that the FPGA can re-use the
    ///                 result memory and registers.
    ///
public:
    void pop_result(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Determines if there is a new result available.
    /// @return                             The result status.
    ///
public:
    busy_result_t busy(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Restart the current correlation with the same settings.
    ///
public:
    void restart_correlation(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Change the correlation mode (and optionally the start offset) and restart it.
    /// @details        Only the antenna mode may be changed.
    /// @todo           Enforce the requirement that only the antenna mode may be changed.
    /// @param[in]      m                   The new mode.
    /// @param[in]      start               Sample offset in frame to begin correlation (in track mode).
    ///
public:
    void change_mode(mode_t m, unsigned start = 0);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Change the correlation start setting and restart it.
    /// @param[in]      start               Sample offset in frame to begin correlation.
    ///
public:
    void change_start(unsigned start);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Gets the coherent power gain.
    /// @details        The coherent gain of the matched filter for such a signal depends on the filter length, as
    ///                 described in X1R008, "Gain Control and Threshold Setting".
    /// @return                             The coherent gain.
    ///
private:
    double get_coherent_pwr_gain(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Gets the non-coherent power gain.
    /// @details        The non-coherent gain of the power-summing process depends on the number of matched-filter
    ///                 results that are summed.
    /// @return                             The non-coherent gain.
    ///
private:
    double get_non_coherent_pwr_gain(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Calculate the threshold based on the mean RRC sample power.
    /// @details        The threshold is defined relative to the peak that would be expected if a perfect signal were
    ///                 being received.  The coherent gain of the matched filter for such a signal depends on the filter
    ///                 length, as described in X1R008, "Gain Control and Threshold Setting".
    /// @param[in]      power               The mean %ADC sample power.
    /// @param[in]      threshold_factor_dB An adjustement to be applied to the built-in threshold.
    /// @return                             The threshold.
    ///
private:
    double calc_threshold(double rrcPower, double threshold_factor_dB);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  @brief          Scale the threshold for the FPGA.
    //  @details        The matched-filter produces either I/Q samples or power samples and the scaling factor for the
    //                  threshold must match the scaling of the peak power result.
    //  @param[in]      threshold           The desired threshold.
    //  @return                             The scaled threshold
    //
private:
    uint32_t scale_threshold_for_fpga(double threshold);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Set the threshold based on the mean RRC sample power.
    /// @param[in]      power               The mean %ADC sample power.
    /// @param[in]      threshold_factor_dB An adjustement to be applied to the built-in threshold.
    ///
public:
    void set_threshold(double rrcPower, double threshold_factor_dB);

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
};

#endif

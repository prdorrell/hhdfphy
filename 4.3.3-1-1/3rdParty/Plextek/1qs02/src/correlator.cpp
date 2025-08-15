/******************************************************************************
 * Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 * All Rights Reserved
 ******************************************************************************
 * THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 * ----------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/correlator.cpp $
 * $Revision: 6338 $
 * $Author: pdm $
 * $Date: 2011-07-15 14:57:08 +0100 (Fri, 15 Jul 2011) $
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file correlator.cpp
 * \brief
 * Wrap up the hardware correlator in a class with any additional
 * processing fluff needed.
 * Uses the low level FPGA interface talk to the hardware.
 *
 *****************************************************************************/
#include "correlator.hpp"
#include "design.hpp"

#include "kissfft.hh"

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class.
//  @details        Read the reference sequence chips from the correlation source, convert them into frequency domain
//                  coefficients for the overlap-add implementation iof the matched filter and write the results to
//                  the FPGA.  Get the FPGA interface to setup the correlation.
//  @param[in]      fpga                The FPGA interface.
//  @param[in]      cs_in               Correlation source object for source of chips.
//  @param[in]      m                   Mode of correlator.
//  @param[in]      low_power           True if the low-power settings should be used, false if high-sensitivity is
//                                      required.
//  @param[in]      start               Sample offset in frame to begin correlation.
//  @param[in]      debug               Controls the debug messages that may be generated.
//  @param[in]      dump                true = dump fft results to file for debug
//
Correlator::Correlator(Fpga                             &fpga,
                       std::auto_ptr<CorrelationSource> cs_in,
                       bool                             low_power,
                       mode_t                           m,
                       unsigned                         start,
                       unsigned                         debug,
                       bool                             dump) :
    fpga(fpga),
    cs(cs_in),
    low_power(low_power),
    current_mode(m),
    size(0),
    fft_gains(0),
    current_start(start),
    current_threshold(0.0),
    debugStr("Correlator.. ", debug),
    dump_files(dump)
{
    unsigned slot;
    double max_coef = 0;
    std::vector< block_t > fft_results( ThreeGPP::SLOTS_PER_FRAME );

    if ((m == CAL_ANT1) | (m == CAL_ANT2))
    {
        size      = FREQ_CAL_FILTER_SIZE;
        fft_gains = FREQ_CAL_MF_FFT_GAINS;
    }
    else
    {
        size      = SEARCH_TRACK_FILTER_SIZE;
        fft_gains = SEARCH_TRACK_MF_FFT_GAINS;
    }

    int fft_size = 2*size;

    // load the begining chunk of each slot into the frame
    // the fpga can then correlate each one with an offset of one slot
    // and average the correlation power non-coherrently
    // the number it actually does is set up later and will effect if realtime operation pos
    for ( slot = 0; slot < ThreeGPP::SLOTS_PER_FRAME; ++slot )
    {
        // get the source to generate the reference chips for this slot
        block_t const & ref_chips = cs->get_block( slot );

        // the matched-filter coefficients are the time-reversed conjugates of the reference chips
        block_t fir_coeffs(ref_chips.size());

        block_t::const_iterator ref_chips_iterator = ref_chips.begin();
        for ( block_t::reverse_iterator fir_coeffs_iterator  = fir_coeffs.rbegin()
            ;                           fir_coeffs_iterator != fir_coeffs.rend()
            ;                         ++fir_coeffs_iterator )
        {
            *fir_coeffs_iterator = conj(*ref_chips_iterator);
            ++ref_chips_iterator;
        }

        // the overlap-add requires the coefficients to be extended to the size of the FFT, twice the FIR size
        fir_coeffs.resize(fft_size, 0);

        if( dump_files )
        {
            std::string filename("dump/ref_chips");
            filename += boost::lexical_cast<std::string>( slot ) + ".txt";
            std::ofstream dumpfile( filename.c_str(), std::ios::out | std::ios::trunc );
            BOOST_FOREACH( block_t::value_type x, ref_chips )
            {
                dumpfile<< x.real() << ' ' << x.imag() <<'\n';
            }
            dumpfile.close();
        }

        if( dump_files )
        {
            std::string filename("dump/corrdump");
            filename += boost::lexical_cast<std::string>( slot ) + ".txt";
            std::ofstream dumpfile( filename.c_str(), std::ios::out | std::ios::trunc );
            BOOST_FOREACH( block_t::value_type x, fir_coeffs )
            {
                dumpfile<< x.real() << ' ' << x.imag() <<'\n';
            }
            dumpfile.close();
        }

        // do fft
        kissfft< double > fft( fir_coeffs.size(), false );
        fft_results[slot].resize( fir_coeffs.size() );
        fft.transform( &fir_coeffs.front(), &fft_results[slot].front() );
        // find max abs re / im
        BOOST_FOREACH( block_t::value_type x, fft_results[slot] )
        {
            if( fabs(x.real()) > max_coef ) max_coef = fabs(x.real());
            if( fabs(x.imag()) > max_coef ) max_coef = fabs(x.imag());
        }
    }

    //
    //  Scale and saturate the components and remove the DC gain.
    //
    double norm_factor = static_cast<double>(MAX_ABS_FREQ_DOM_COEFF_VAL)/(2.0*sqrt(static_cast<double>(fft_size)));

    for ( slot = 0; slot < Design::MAX_CORRELATIONS_PER_FRAME; ++slot )
    {
        std::vector< std::complex< int32_t > > coefs;

        BOOST_FOREACH( block_t::value_type x, fft_results[slot] )
        {
            double real = floor(norm_factor*x.real()+0.5);
            if (real > +MAX_ABS_FREQ_DOM_COEFF_VAL)
            {
                real = +MAX_ABS_FREQ_DOM_COEFF_VAL;
            }
            else
            if (real < -MAX_ABS_FREQ_DOM_COEFF_VAL)
            {
                real = -MAX_ABS_FREQ_DOM_COEFF_VAL;
            }

            double imag = floor(norm_factor*x.imag()+0.5);
            if (imag > +MAX_ABS_FREQ_DOM_COEFF_VAL)
            {
                imag = +MAX_ABS_FREQ_DOM_COEFF_VAL;
            }
            else
            if (imag < -MAX_ABS_FREQ_DOM_COEFF_VAL)
            {
                imag = -MAX_ABS_FREQ_DOM_COEFF_VAL;
            }

            coefs.push_back(
                std::complex<int32_t>( static_cast<int32_t>( real ),
                                       static_cast<int32_t>( imag ) ) );
        }

        //
        //  Set the DC gain to 0 and that of the adjacent bins too.
        //
        coefs[fft_size-1] = 0;
        coefs[0]          = 0;
        coefs[1]          = 0;

        if( dump_files )
        {
            std::string filename("./dump/coefdump");
            filename += boost::lexical_cast<std::string>( slot ) + ".txt";
            std::ofstream dumpfile( filename.c_str(), std::ios::out | std::ios::trunc );
            BOOST_FOREACH( std::complex<int32_t> x, coefs )
            {
                dumpfile<< x.real() << ' ' << x.imag() <<'\n';
            }
            dumpfile.close();
        }

        // load fpga
        fpga.send_coefs( coefs, slot );
    }

    // start the correlation
    restart_correlation();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          The destructor has nothing to do.
//
Correlator::~Correlator()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Enter low power mode.
//
void Correlator::enter_low_power_mode(void)
{
    fpga.enter_low_power_mode();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Exit low power mode.
//
void Correlator::exit_low_power_mode(void)
{
    fpga.exit_low_power_mode();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Retrieves the channel RSSI.
//  @return                             The RSSI normalised to the LSB^2 of the channel filter samples.
//
double Correlator::get_channel_power( void )
{
    return (fpga.get_channel_power());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Retrieves the ADC RSSI.
//  @return                             The RSSI normalised to the LSB^2 of the ADC samples.
//
double Correlator::get_adc_power( void )
{
    return (fpga.get_adc_power());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Signals that the software has finished with the current result so that the FPGA can re-use the
//                  result memory and registers.
//
void Correlator::pop_result( void )
{
    fpga.pop_result();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Restart the current correlation with the same settings.
//
void Correlator::restart_correlation( void )
{
    //
    //  Select the averaging according to the mode.
    //
    unsigned slot_averages;
    unsigned frame_averages;
    if (low_power)
    {
        slot_averages  = LP_FREQ_CAL_MODE_SLOTS_PER_FRAME_TO_AVG;
        frame_averages = LP_FREQ_CAL_MODE_FRAMES_TO_AVG;
        if ((current_mode == SEARCH_ANT1) || (current_mode == SEARCH_ANT2))
        {
            slot_averages  = LP_SEARCH_MODE_SLOTS_PER_FRAME_TO_AVG;
            frame_averages = LP_SEARCH_MODE_FRAMES_TO_AVG;
        }
        else
        if ((current_mode == TRACK_ANT1) || (current_mode == TRACK_ANT2) || (current_mode == TRACK_SWAP))
        {
            slot_averages  = LP_TRACK_MODE_SLOTS_PER_FRAME_TO_AVG;
            frame_averages = LP_TRACK_MODE_FRAMES_TO_AVG;
        }
    }
    else
    {
        slot_averages  = HS_FREQ_CAL_MODE_SLOTS_PER_FRAME_TO_AVG;
        frame_averages = HS_FREQ_CAL_MODE_FRAMES_TO_AVG;
        if ((current_mode == SEARCH_ANT1) || (current_mode == SEARCH_ANT2))
        {
            slot_averages  = HS_SEARCH_MODE_SLOTS_PER_FRAME_TO_AVG;
            frame_averages = HS_SEARCH_MODE_FRAMES_TO_AVG;
        }
        else
        if ((current_mode == TRACK_ANT1) || (current_mode == TRACK_ANT2) || (current_mode == TRACK_SWAP))
        {
            slot_averages  = HS_TRACK_MODE_SLOTS_PER_FRAME_TO_AVG;
            frame_averages = HS_TRACK_MODE_FRAMES_TO_AVG;
        }
    }

    fpga.setup_correlation(current_mode,
                           fft_gains,
                           current_start,
                           slot_averages,
                           frame_averages,
                           scale_threshold_for_fpga(current_threshold));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Change the correlation mode (and optionally the start offset) and restart it.
//  @details        Only the antenna mode may be changed.
//  @todo           Enforce the requirement that only the antenna mode may be changed.
//  @param[in]      m                   The new mode.
//  @param[in]      start               Sample offset in frame to begin correlation (in track mode).
//
void Correlator::change_mode( mode_t m, unsigned start )
{
    current_mode = m;
    current_start = start;
    restart_correlation();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Change the correlation start and length settings and restart it.
//  @param[in]      start               Sample offset in frame to begin correlation.
//
void Correlator::change_start( unsigned start )
{
    current_start = start;
    restart_correlation();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Retrieves the peak in the current result.
//  @param[in]      raw_peak            The peak data.
//  @return                             The frame number.
//
int64_t Correlator::get_peak_data( peak_t &peak )
{
    //
    //  Get the raw peak data.
    //
    Fpga::raw_peak_t raw_peak;

    int64_t frame = fpga.get_peak_data(raw_peak);

    //
    //  Correct the FPGA's power scaling and normalise the power to RRC samples.
    //
#ifdef IQ_RESULTS
    double power_scaling_factor = PEAK_POWER_SCALING_FACTOR_IQ_MODE;
#else
    double power_scaling_factor = PEAK_POWER_SCALING_FACTOR_PWR_MODE;
#endif

    double fpga_power       = raw_peak.sample_power;
    double corrected_power  = power_scaling_factor*fpga_power;
    double normalised_power = corrected_power/get_coherent_pwr_gain()/get_non_coherent_pwr_gain();

    if (raw_peak.exceeds_threshold)
    {
        debugStr(DebugStream::info3) << "Peak data: " << "offset = "           << raw_peak.sample_offset << ", "
                                                      << "FPGA power = "       << fpga_power             << ", "
                                                      << "corrected power = "  << corrected_power        << ", "
                                                      << "normalised power = " << normalised_power       << "\n";
    }

    peak.exceeds_threshold = raw_peak.exceeds_threshold;
    peak.sample_offset     = raw_peak.sample_offset;
    peak.sample_power      = normalised_power;

    return frame;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Retrieves the peak in a search window in the current result.
//  @details        The function not only retrieves the basic peak data but also performsa quadratic fit to estimate the
//                  true position of the peak.
//  @param[in]      search_start        The first sample in the search window.
//  @param[in]      search_len          The length of the search window.
//  @return                             The frame number.
//
Correlator::peak_pos_t Correlator::get_peak(uint32_t search_start, uint32_t search_len)
{
    debugStr(DebugStream::info3) << "get_peak : search_start =" << search_start << ", search_len = " << search_len << "\n";

    peak_pos_t peak =
    {
        0,                  // frame
        0.0,                // offset
        0.0                 // pwr
    };

    unsigned peak_offset = 0;

    std::vector<double> peak_samples(3);    //  3 samples needed for quadratic fit.

    //
    //  Get the result samples.
    //
    std::vector<double> data;
    peak.frame = get_raw_corr_data(search_start, search_len, false, data);

    //
    //  Identify the peak value in the samples.
    //
    double   peak_value  = 0.0;
    for (unsigned sample_num = 0 ; sample_num < search_len ; ++sample_num)
    {
        double sample = data[sample_num];

        if (sample > peak_value)
        {
            peak_offset = sample_num;
            peak_value  = sample;
        }
    }

    //
    //  Get the samples either side of the peak.
    //
    if (peak_offset == 0)
    {
        peak_samples[0] = data[search_len-1];
        peak_samples[1] = data[0];
        peak_samples[2] = data[1];
    }
    else
    if (peak_offset == (search_len-1))
    {
        peak_samples[0] = data[search_len-2];
        peak_samples[1] = data[search_len-1];
        peak_samples[2] = data[0];
    }
    else
    {
        peak_samples[0] = data[peak_offset-1];
        peak_samples[1] = data[peak_offset];
        peak_samples[2] = data[peak_offset+1];
    }

    // do quadratic fit
    // assume points are in form y = c + ax**2 (a is -ve)
    // and they are shifted in x dir so x = (x' - off)
    // y = c + ax'**2 - aoff**2 + 2ax'off
    // cf y = ax**2 + bx + c
    // b = 2aoff, off = -b/(2a)
    //
    // y[-1] = a x[-1]**2 + b x[-1] + c = a - b + c
    // y[0]  = a x[0]**2 + b x[0] + c   = c
    // y[1]  = a x[1]**2 + b x[1] + c   = a + b + c
    // a = (y[-1]+y[1]-2y[0])/2
    // b = (y[1] - y[-1])/2
    // off = -b/(2a)

    peak.offset = - (double(peak_samples[2]) - double(peak_samples[0])) / 2.0 /
                    (double(peak_samples[0]) + double(peak_samples[2]) - 2.0*double(peak_samples[1]) );

    peak.offset += double(peak_offset) + double(search_start);
    if (peak.offset > Design::samples_per_frame)
    {
        peak.offset -= double(Design::samples_per_frame);
    }

    // (small) -ve offset is posible

    //
    //  Normalise the power to RRC samples.
    //
    peak.pwr = peak_samples[1]/get_coherent_pwr_gain()/get_non_coherent_pwr_gain();   // could interpolate this too

    return peak;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Gets the samples for a window in the current result.
//  @param[in]      start               The start of the window.
//  @param[in]      len                 The length of the window.
//  @param[in]      alt_ant             true if the samples for the alternate antenna should be read.
//  @param[out]     data                The destination for the samples.
//  @param[in]      use_threshold       true if samples below the threshold should be set to 0.0.
//  @return                             The frame number.
//
int64_t Correlator::get_raw_corr_data(uint32_t start,
                                      uint32_t len,
                                      bool alt_ant,
                                      std::vector<double> &data,
                                      bool use_threshold)
{
#ifdef IQ_RESULTS
    double power_scaling_factor = POWER_SAMPLE_SCALING_FACTOR_IQ_MODE;
#else
    double power_scaling_factor = POWER_SAMPLE_SCALING_FACTOR_PWR_MODE;
#endif

    //
    //  Get the raw samples.
    //
#ifdef IQ_RESULTS
    std::vector<std::complex<double> > raw_data;
    int64_t frame = fpga.get_raw_iq_mode_corr_data( start, len, alt_ant, raw_data );
#else
    std::vector<double> raw_data;
    int64_t frame = fpga.get_raw_pwr_mode_corr_data( start, len, alt_ant, raw_data );
#endif

    //
    //  Convert the raw samples to powers with the appropriate scaling correction.
    //
    //
    //  Set all powers that are below the threshold to 0 and normalise the power of the remainder to RRC samples.
    //
    double coherent_pwr_gain     = get_coherent_pwr_gain();
    double non_coherent_pwr_gain = get_non_coherent_pwr_gain();

#ifdef IQ_RESULTS
    BOOST_FOREACH( std::complex<double> sample, raw_data )
    {
        double pwr = sample.real()*sample.real()+sample.imag()*sample.imag();
#else
    BOOST_FOREACH( double sample, raw_data )
    {
        double pwr = sample;
#endif
        pwr *= power_scaling_factor;
        if ((pwr < current_threshold) && use_threshold)
        {
            pwr = 0.0;
        }
        else
        {
            pwr *= 1.0/coherent_pwr_gain/non_coherent_pwr_gain;
        }
        data.push_back(pwr);
    }

    return frame;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Retrieves the frame number of the first frame that contributed to the current result.
//  @return                             The frame number.
//
int64_t Correlator::get_frame(void)
{
    return fpga.get_frame();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Retrieves the sample offset of the peak in the current result.
//  @return                             The sample offset.
//
uint32_t Correlator::get_peak_sample_offset(void)
{
    return fpga.get_peak_sample_offset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Determines if there is a new result available.
//  @return                             The result status.
//
Correlator::busy_result_t Correlator::busy(void)
{
    busy_result_t ret_val = RESULT_AVAILABLE;
    Fpga::result_avail_t result_available = fpga.result_available();
    if (result_available == Fpga::RESULT_PENDING)
    {
        ret_val = RESULT_PENDING;
    }
    else
    if (result_available == Fpga::RESULT_TIMEOUT)
    {
        ret_val = RESULT_TIMEOUT;
    }

    return (ret_val);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Gets the coherent power gain.
//  @details        The coherent gain of the matched filter for such a signal depends on the filter length, as described
//                  in X1R008, "Gain Control and Threshold Setting".
//  @return                             The coherent gain.
//
double Correlator::get_coherent_pwr_gain(void)
{
    double coherent_pwr_gain =   SEARCH_TRACK_MF_PWR_GAIN_FACTOR
                               * static_cast<double>(size)
                               * static_cast<double>(size);
    if (size == FREQ_CAL_FILTER_SIZE)
    {
        coherent_pwr_gain =   FREQ_CAL_MF_PWR_GAIN_FACTOR
                            * static_cast<double>(size)
                            * static_cast<double>(size);
    }
    return (coherent_pwr_gain);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Gets the non-coherent power gain.
//  @details        The non-coherent gain of the power-summing process depends on the number of matched-filter results
//                  that are summed.
//  @return                             The non-coherent gain.
//
double Correlator::get_non_coherent_pwr_gain(void)
{
    double non_coherent_pwr_gain;
    if (low_power)
    {
        non_coherent_pwr_gain = LP_FREQ_CAL_MODE_FRAMES_TO_AVG*LP_FREQ_CAL_MODE_SLOTS_PER_FRAME_TO_AVG;
        if ((current_mode == SEARCH_ANT1) || (current_mode == SEARCH_ANT2))
        {
            non_coherent_pwr_gain = LP_SEARCH_MODE_FRAMES_TO_AVG*LP_SEARCH_MODE_SLOTS_PER_FRAME_TO_AVG;
        }
        else
        if ((current_mode == TRACK_ANT1) || (current_mode == TRACK_ANT2) || (current_mode == TRACK_SWAP))
        {
            non_coherent_pwr_gain = LP_TRACK_MODE_FRAMES_TO_AVG*LP_TRACK_MODE_SLOTS_PER_FRAME_TO_AVG;
        }
    }
    else
    {
        non_coherent_pwr_gain = HS_FREQ_CAL_MODE_FRAMES_TO_AVG*HS_FREQ_CAL_MODE_SLOTS_PER_FRAME_TO_AVG;
        if ((current_mode == SEARCH_ANT1) || (current_mode == SEARCH_ANT2))
        {
            non_coherent_pwr_gain = HS_SEARCH_MODE_FRAMES_TO_AVG*HS_SEARCH_MODE_SLOTS_PER_FRAME_TO_AVG;
        }
        else
        if ((current_mode == TRACK_ANT1) || (current_mode == TRACK_ANT2) || (current_mode == TRACK_SWAP))
        {
            non_coherent_pwr_gain = HS_TRACK_MODE_FRAMES_TO_AVG*HS_TRACK_MODE_SLOTS_PER_FRAME_TO_AVG;
        }
    }

    return (non_coherent_pwr_gain);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Calculate the threshold based on the mean RRC sample power.
//  @details        The threshold is defined relative to the peak that would be expected if a perfect signal were being
//                  received.  The coherent gain of the matched filter for such a signal depends on the filter length,
//                  as described in X1R008, "Gain Control and Threshold Setting".
//  @param[in]      power               The mean %ADC sample power.
//  @param[in]      threshold_factor_dB An adjustement to be applied to the built-in threshold.
//  @return                             The threshold.
//
double Correlator::calc_threshold(double rrcPower, double threshold_factor_dB)
{
    //
    //  Calculate the signal gains.
    //
    double matched_filter_pwr_gain = get_coherent_pwr_gain();
    double averaging_pwr_gain      = get_non_coherent_pwr_gain();

    //
    //  Calculate the threshold factor that depends on the current mode.
    //
    double threshold_factor;
    if (low_power)
    {
        threshold_factor = pow(10.0, (LP_FREQ_CAL_MODE_REL_THRESHOLD_DB+threshold_factor_dB)/10.0);
        if ((current_mode == SEARCH_ANT1) || (current_mode == SEARCH_ANT2))
        {
            threshold_factor = pow(10.0, (LP_SEARCH_MODE_REL_THRESHOLD_DB+threshold_factor_dB)/10.0);
        }
        else
        if ((current_mode == TRACK_ANT1) || (current_mode == TRACK_ANT2) || (current_mode == TRACK_SWAP))
        {
            threshold_factor = pow(10.0, (LP_TRACK_MODE_REL_THRESHOLD_DB+threshold_factor_dB)/10.0);
        }
    }
    else
    {
        threshold_factor = pow(10.0, (HS_FREQ_CAL_MODE_REL_THRESHOLD_DB+threshold_factor_dB)/10.0);
        if ((current_mode == SEARCH_ANT1) || (current_mode == SEARCH_ANT2))
        {
            threshold_factor = pow(10.0, (HS_SEARCH_MODE_REL_THRESHOLD_DB+threshold_factor_dB)/10.0);
        }
        else
        if ((current_mode == TRACK_ANT1) || (current_mode == TRACK_ANT2) || (current_mode == TRACK_SWAP))
        {
            threshold_factor = pow(10.0, (HS_TRACK_MODE_REL_THRESHOLD_DB+threshold_factor_dB)/10.0);
        }
    }

    //
    //  Calculate the ideal peak and the threshold, which is specified relative to it.
    //
    double ideal_peak = matched_filter_pwr_gain*averaging_pwr_gain*rrcPower;
    double threshold  = threshold_factor*ideal_peak;

    debugStr(DebugStream::info3) << boost::format("Threshold for RRC power of %.0f and filter length of %4d = %f\n")
                                                  % rrcPower
                                                  % size
                                                  % threshold;

    return (threshold);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Scale the threshold for the FPGA.
//  @details        The matched-filter produces either I/Q samples or power samples and the scaling factor for the
//                  threshold must match the scaling of the peak power result.
//  @param[in]      threshold           The desired threshold.
//  @return                             The scaled threshold
//
uint32_t Correlator::scale_threshold_for_fpga(double threshold)
{
#ifdef IQ_RESULTS
    double fpga_threshold = floor(threshold/PEAK_POWER_SCALING_FACTOR_IQ_MODE +0.5);
#else
    double fpga_threshold = floor(threshold/PEAK_POWER_SCALING_FACTOR_PWR_MODE+0.5);
#endif

    if (fpga_threshold > std::numeric_limits<uint32_t>::max())
    {
        fpga_threshold = std::numeric_limits<uint32_t>::max();
    }

    return (static_cast<uint32_t>(fpga_threshold));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Set the threshold based on the mean RRC sample power.
//  @param[in]      power               The mean %ADC sample power.
//  @param[in]      threshold_factor_dB An adjustement to be applied to the built-in threshold.
//
void Correlator::set_threshold(double rrcPower, double threshold_factor_dB)
{
    current_threshold = calc_threshold(rrcPower, threshold_factor_dB);
    fpga.set_threshold(scale_threshold_for_fpga(current_threshold));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Set the FPGA's test register
//  @param[in]      val                 The value to write to the register.
//
void Correlator::set_test_register(uint32_t val)
{
    fpga.set_test_register(val);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Set the debug verbosity level.
//  @param[in]      level               The new verbosity level.
//
void Correlator::debug_printing(unsigned level)
{
    debugStr.show_level(level);
    cs->debug_printing(level);
}

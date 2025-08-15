/***********************************************************************************************************************
 *
 *
 ***********************************************************************************************************************
 *  Filename:   design.hpp
 *  Author(s):  pdm
 *******************************************************************************************************************//**
 * File Description (User Field)
 * ================
 */
/** \file design.hpp
 * \brief Constants specific to the implementation.
 *
 **********************************************************************************************************************/
#ifndef __DESIGN_HPP__
#define __DESIGN_HPP__

#include "3gpp.h"
#include "4g.hpp"
#include "gsm.hpp"

/// Collection of design constants
namespace Design{
    /// \name self explanatory constants
    //@{
    const unsigned adc_samples_per_chip   = 5;
    const unsigned adc_samples_per_frame  = adc_samples_per_chip * ThreeGPP::CHIPS_PER_FRAME;
    const unsigned adc_samples_per_slot   = adc_samples_per_chip * ThreeGPP::CHIPS_PER_SLOT;
    const unsigned adc_samples_per_second = adc_samples_per_chip * ThreeGPP::CHIPS_PER_SEC;

    const unsigned samples_per_chip   = 2;
    const unsigned samples_per_frame  = samples_per_chip * ThreeGPP::CHIPS_PER_FRAME;
    const unsigned samples_per_slot   = samples_per_chip * ThreeGPP::CHIPS_PER_SLOT;
    const unsigned samples_per_second = samples_per_chip * ThreeGPP::CHIPS_PER_SEC;

    const unsigned ADC_SAMPLES_PER_BIT_GSM   = 48;
    const unsigned ADC_SAMPLES_PER_FRAME_GSM = ADC_SAMPLES_PER_BIT_GSM * Gsm::BITS_PER_FRAME;

    const unsigned SAMPLES_PER_FRAME_GSM     = ADC_SAMPLES_PER_FRAME_GSM/20;

    const unsigned ADC_SAMPLES_PER_SEC_4G      = adc_samples_per_second;
    const unsigned ADC_SAMPLES_PER_SUBFRAME_4G = ADC_SAMPLES_PER_SEC_4G / FourG::SUBFRAMES_PER_SEC;
    
    const unsigned DEC_FACTOR_4G = 4;
    const unsigned SAMPLES_PER_SEC_4G      = ADC_SAMPLES_PER_SEC_4G / DEC_FACTOR_4G;
    const unsigned SAMPLES_PER_SUBFRAME_4G = ADC_SAMPLES_PER_SUBFRAME_4G / DEC_FACTOR_4G;

    ///
    /// @brief          The minimum DAC value.
    ///
    const unsigned MIN_DAC_VAL =    0;

    ///
    /// @brief          The maximum DAC value.
    ///
    const unsigned MAX_DAC_VAL = 4095;

    ///
    /// @brief          The DAC value that corresponds to the minimum of the VCTCXO control range.
    /// @details        The DAC has 12 bits and a 3.2 V supply.  Its output spans the full range from 0 V to 3.2 V.  The
    ///                 VCTCXO has a control input with a valid range of between 0.5 V and 2.5 V, which corresponds to
    ///                 +/-7 ppm.  Therefore the maximum and minimum DAC values are 3200 and 640 respectively and the
    ///                 mid-range value is 1920.
    ///
    const unsigned MIN_VCTCXO_RANGE_DAC_VAL =  640;

    ///
    /// @brief          The DAC value that corresponds to the middle of the VCTCXO control range.
    /// @details        See Design::MIN_VCTCXO_DAC_VAL.
    ///
    const unsigned MID_VCTCXO_RANGE_DAC_VAL = 1920;

    ///
    /// @brief          The DAC value that corresponds to the maximum of the VCTCXO control range.
    /// @details        See Design::MIN_VCTCXO_DAC_VAL.
    ///
    const unsigned MAX_VCTCXO_RANGE_DAC_VAL = 3200;





    const unsigned samples_per_cal_corr_block = samples_per_chip * 256;
    const unsigned samples_per_search_corr_block = samples_per_chip * 2048;

    const unsigned gps_nominal_ticks = 26000000; // measured with 26MHz clock

    const unsigned MAX_CORRELATIONS_PER_FRAME = ThreeGPP::SLOTS_PER_FRAME;

    const int TRACK_MODE_OFFSET_RES  = 64;

    const unsigned NUM_TRACK_MODE_RESULTS = 1024;
    const unsigned TRACK_MODE_SKEW        = NUM_TRACK_MODE_RESULTS/2;

    const int MIN_TRACK_WINDOW_POS_CHANGE = 2;
    const int MAX_TRACK_WINDOW_POS_CHANGE = 5;
    
    ///
    /// @brief          The 4G SRS periodicity in ms (= subframes).
    /// @details        This depends on the SRS Configuration Index (see table 8.2-1 in 36.213) that is always chosen 
    ///                 to produce a periodicity of 2 ms.
    ///
    static const unsigned SRS_4G_PERIODICITY_MS = 2;

    ///
    /// @brief          The number of samples in the 4G SRS periodicity.
    ///
    static const unsigned SAMPLES_PER_4G_SRS_PERIOD = ( SRS_4G_PERIODICITY_MS * SAMPLES_PER_SEC_4G ) / 1000;

    ///
    /// @brief          Maximum number of correlator taps
    ///
    static const unsigned NUM_CORRELATOR_TAPS_4G = 320;

    ///
    /// @brief          The 4G tracking window size.
    /// @details        This width is based on two elements, 125 that is the equivalent in time to the 3G window and
    ///                 allows for any timing drift and twice the length of an SRS burst with extended cyclic prefix.
    ///                 The second term allows for the possibility that detection occurs at the edge of a burst and
    ///                 allows it to centre itself.
    ///
#ifdef WIDE_TRACK_WINDOW_4G
    static const unsigned TRACK_WINDOW_SIZE_4G_SAMPLES =
          125
        + (  static_cast<uint64_t>(4)
           * static_cast<uint64_t>(SAMPLES_PER_SEC_4G)
           * static_cast<uint64_t>(  FourG::BASIC_TIME_UNITS_PER_EXTENDED_CYCLIC_PREFIX
                                   + FourG::BASIC_TIME_UNITS_PER_BASIC_SYMBOL ) )
           / FourG::BASIC_TIME_UNITS_PER_SEC;
#else
    static const unsigned TRACK_WINDOW_SIZE_4G_SAMPLES = 125+NUM_CORRELATOR_TAPS_4G;
#endif
    ///
    /// @brief          The scaling factor to be applied to the 3G power correction values when applying them to 4G
    ///                 results to account for the FPGA filter gain differences.
    /// @details        The correction factors convert the power measurements at the output of the 3G channel filter to
    ///                 power at the antenna input. The signal path for 4G and 3G is the same, but the 3G FPGA filtering
    ///                 has a DC gain of 9.3 (see X1R008). The 4G FPGA has unity gain and so it is necssary to multiply
    ///                 the correction factors by 9.3^2.
    ///                 The factor of 2 is a result of measurements that suggest there is such a factor somewhere, but
    ///                 it is not understood where it comes from.
    ///
    static const double CAL_FACTOR_RATIO_4G_3G_FPGA_FILTERS = 9.3*9.3/2.0;
    
    ///
    /// @brief          The scaling factor to be applied to the power correction values when applying them for a 5 MHz
    ///                 channel bandwidth.
    /// @details        The factor arises because the SRS bandwidth for a 5 MHz channel bandwidth is 4.32 MHz and
    ///                 the bandwidth of the combined filters is about 3.8 MHz and so the SRS signal power should be
    ///                 4.32/3.8 times the measured power.
    ///
    ///                 Note that the 3.8 MHz filter bandwidth is approximate and is slightly different for the 5MHz and
    ///                 10MHz channel bandwidths, but the difference should not be significant.
    ///
    static const double CAL_FACTOR_RATIO_4G_5MHZ_CHAN_BW = CAL_FACTOR_RATIO_4G_3G_FPGA_FILTERS*4.32e6/3.8e6;
    
    ///
    /// @brief          The scaling factor to be applied to the power correction values when applying them for a 10 MHz
    ///                 channel bandwidth.
    /// @details        The factor arises because the SRS bandwidth for a 10 MHz channel bandwidth is 8.64 MHz and
    ///                 the bandwidth of the combined filters is about 3.8 MHz and so the SRS signal power should be
    ///                 8.64/3.8 times the measured power.
    ///
    ///                 Note that the 3.8 MHz filter bandwidth is approximate and is slightly different for the 5MHz and
    ///                 10MHz channel bandwidths, but the difference should not be significant.
    ///
    static const double CAL_FACTOR_RATIO_4G_10MHZ_CHAN_BW = CAL_FACTOR_RATIO_4G_3G_FPGA_FILTERS*8.64e6/3.8e6;
    
    //@}
}

#endif

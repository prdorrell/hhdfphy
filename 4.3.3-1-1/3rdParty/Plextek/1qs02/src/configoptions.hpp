/******************************************************************************
 *
 *
 ******************************************************************************
 *  Filename:   configoptions.hpp
 *  Author(s):  pdm
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file configoptions.hpp
 * \brief collection of command line options
 *
 *****************************************************************************/
#ifndef __CONFIGOPTIONS_HPP__
#define __CONFIGOPTIONS_HPP__

#include "common.hpp"
#include "design.hpp"
#include "ranging.hpp"

#include <string>

/// \brief Collection of command line options.
//
/// Default values defined here too
struct ConfigOptions
{
    double                  ref_cal_threshold_factor_dB;    ///< threshold factor for valid correlation detection
    double                  search_threshold_factor_dB;     ///< threshold factor for valid correlation detection
    double                  track_threshold_factor_dB;      ///< threshold factor for valid correlation detection
    double                  search_threshold_factor_4G_dB;  ///< 4G threshold factor for valid correlation detection
    double                  track_threshold_factor_4G_dB;   ///< 4G threshold factor for valid correlation detection
    unsigned int            search_average_4G;              ///< Number of raw correlation results to average in the 4G search mode (0 => use default)
    unsigned int            track_average_4G;               ///< Number of raw correlation results to average in the 4G track mode (0 => use default)
    std::string             image_name_for_4G;              ///< 4G fpga image file name .bin format
    std::string             image_name_for_3G;              ///< 3G fpga image file name .bin format
    std::string             image_name_for_GSM;             ///< GSM fpga image file name .bin format
    eSystemMode             init_mode;                      ///< initial mode
    bool                    low_power;                      ///< signals that the low-power settings for the averaging should be used
    bool                    load_only;                      ///< load fpga image and exit program
    unsigned                debug_print;                    ///< debug tracing display mask
    Ranging::RangingMode    ranging_mode;                   ///< use auto ranging?
    unsigned int            vctcxo_dac;                     ///< initial dac value to set
    unsigned int            port;                           ///< port to listen for connections on
    bool                    debug_files;                    ///< dump file of some debug info
    unsigned int            searching_pause;                ///< number of idle frames between searching iterations
    unsigned int            tracking_pause;                 ///< number of idle frames between tracking iterations.
    bool                    man_load;                       ///< prompt the user to load the fpga image
    bool                    skip_load;                      ///< don't load the fpga image
    unsigned int            antenna_mode;                   ///< antenna 1 or 2
    std::string             pwr_cal_conf_filename;          ///< filename of the power calibration configuration file

    /// set defaults
    ConfigOptions() :
        ref_cal_threshold_factor_dB    ( 0.0 ),
        search_threshold_factor_dB     ( 0.0 ),
        track_threshold_factor_dB      ( 0.0 ),
        search_threshold_factor_4G_dB  ( 0.0 ),
        track_threshold_factor_4G_dB   ( 0.0 ),
        search_average_4G              ( 0 ),
        track_average_4G               ( 0 ),
        image_name_for_4G              ( "Undefined 4G FPGA image" ),
        image_name_for_3G              ( "Undefined 3G FPGA image" ),
        image_name_for_GSM             ( "Undefined GSM FPGA image" ),
        init_mode                      ( MODE_3G ),
        low_power                      ( false ),
        load_only                      ( false ),
        debug_print                    ( false ),
        ranging_mode                   ( Ranging::AUTO_RANGING ),
        vctcxo_dac                     ( 1913 ),   // Empirically determined to be a good value
        port                           ( 55555 ),
        debug_files                    ( false ),
        searching_pause                ( 0 ),
        tracking_pause                 ( 0 ),
        man_load                       ( false ),
        skip_load                      ( false ),
        antenna_mode                   ( 1 ),
        pwr_cal_conf_filename          ( "hhdfphy_pwr_cal.conf" )
        {}
};

#endif

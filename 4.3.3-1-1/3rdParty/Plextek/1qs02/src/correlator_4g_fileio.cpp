/***********************************************************************************************************************
 *  Copyright (c) 2021 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  Filename:   correlator_4g_fileio.cpp
 *  Author(s):  pdm
 *******************************************************************************************************************//**
 * File Description
 * ================
 *
 *  @file
 *  @brief          The implementation of ::Correlator_4g class functions that handle file IO such as reading the
 *                  coefficient files.
 *  @details        Wrap up the hardware 4G correlator in a class that extends the processing with the averaging of the
 *                  peaks.
 **********************************************************************************************************************/
#include "correlator_4g.hpp"
#include "design.hpp"
#include "radio.hpp"

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include <complex>
#include <memory>
#include <functional>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <cassert>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Read the coefficients for channel filter 1.
//  @details        Read the input file using the format specified in 1QR005, "4G Changes to the HHDFPHY SOftware",
//                  checking the filename format and the content format.
//
//                  Error and warning conditions:
//
//                      - Filename format error,        DebugStream::error,     return false with all-pass filter
//                      - Filter number not 1,          DebugStream::error,     return false with all-pass filter
//                      - File format version wrong,    DebugStream::error,     return false with all-pass filter
//                      - File format error,            DebugStream::error,     return false with all-pass filter
//                      - Scaling factor wrong,         DebugStream::error,     return false with all-pass filter
//                      - Filter size even or greater than the maximum
//                                                      DebugStream::error,     return false with all-pass filter
//                      - Tuning offset not multiple of resolution
//                                                      DebugStream::warn2,     return true with rounded offset
//                      - Coefficient out of range      DebugStream::warn2,     return true with clipped coefficient
//
//  @param[in]      filename            The name of the coefficient file.
//  @param[out]     chan_bw_MHz         The channel bandwidth in MHz.
//  @param[out]     tuning_offset_Hz    The tuning offset in Hz.
//  @param[out]     coeffs              The filter coefficients.
//  @return                             true if the read was successful (all OK or a only warning issued, false
//                                      otherwise.
//
bool Correlator_4g::read_chan_filt_1_coeffs(std::string filename, 
                                            double &chan_bw_MHz,
                                            freq_offset_t &tuning_offset_Hz,
                                            std::vector<int16_t> &coeffs)
{
    bool success = false;
    
    chan_bw_MHz = 0.0;
    tuning_offset_Hz = 0;
    coeffs.assign(1, CHAN_FILT_MAX_ABS_COEFF_VAL);
    
    //
    //  Parse the filename and check those fields that can be checked.
    //
    std::string base_filename = filename.substr(filename.find_last_of("/\\") + 1);
    debugStr(DebugStream::info1) << boost::format(  "Reading channel filter 1 coefficients from %s\n" )
                                                  % filename;
    
    unsigned filter_num;
    unsigned bw_int;
    unsigned bw_dec;
    unsigned file_version;
    int num_args_filled = sscanf(base_filename.c_str(),
                                 "ChanFilt_%u_BW_%u_%u_MHz_v%u.dat",
                                 &filter_num,
                                 &bw_int,
                                 &bw_dec,
                                 &file_version);
    if (4 != num_args_filled)
    {
        debugStr(DebugStream::error) << boost::format(  "Channel filter 1 coefficients filename format error (%s)\n" )
                                                      % filename;
    }
    else
    if (1 != filter_num)
    {
        debugStr(DebugStream::error) << boost::format(  "Channel filter 1 coefficient file filter number (%u) is not 1\n" )
                                                      % filter_num;
    }
    else
    if (COEFF_FILE_FORMAT_VERSION != file_version)
    {
        debugStr(DebugStream::error) << boost::format(  "Channel filter 1 coefficient file version (%u) is unsupported\n" )
                                                      % file_version;
    }
    else
    {
        chan_bw_MHz = static_cast<double>(bw_int)+static_cast<double>(bw_dec)/CHAN_BW_SF;
        
        //
        //  Read the file contents.
        //
        std::ifstream ifs(filename.c_str());
        if (ifs.fail())
        {
            debugStr(DebugStream::error) << boost::format(  "Channel filter 1 file (%s) could not be opened\n" )
                                                          % filename;
        }
        else
        {
            std::string nextLine;
            std::getline(ifs, nextLine);

            num_args_filled = sscanf(nextLine.c_str(), "Tuning offset  = %d Hz", &tuning_offset_Hz);
            if (1 != num_args_filled)
            {
                debugStr(DebugStream::error) << boost::format(  "Channel filter 1 tuning offset format error (%s)\n" )
                                                              % nextLine;
            }
            else
            {
                freq_offset_t rounded_tuning_offset_Hz = 
                    static_cast<freq_offset_t>(  Radio::get_freq_res()
                                               * round(static_cast<double>(tuning_offset_Hz)/Radio::get_freq_res()));
                if (tuning_offset_Hz != rounded_tuning_offset_Hz)
                {
                    debugStr(DebugStream::warn2) << boost::format(  "Channel filter 1 tuning offset (%d) not a multiple of the frequency resolution (%lf)\n" )
                                                                  % tuning_offset_Hz
                                                                  % Radio::get_freq_res();
                    tuning_offset_Hz = rounded_tuning_offset_Hz;
                }

                std::getline(ifs, nextLine);
                unsigned scaling_factor;
                num_args_filled = sscanf(nextLine.c_str(), "Scaling factor = %u", &scaling_factor);
                if (1 != num_args_filled)
                {
                    debugStr(DebugStream::error) << boost::format(  "Channel filter 1 scaling factor format error (%s)\n" )
                                                                  % nextLine;
                }
                else
                if (scaling_factor != CHAN_FILT_COEFF_SCALING_FACTOR)
                {
                    //
                    //  For some reason using a class constant in the Boost formatting below produces a linker error.
                    //
                    unsigned required_sf = CHAN_FILT_COEFF_SCALING_FACTOR;
                    debugStr(DebugStream::error) << boost::format(  "Channel filter 1 scaling factor (%u) is not %u\n" )
                                                                  % scaling_factor
                                                                  % required_sf;
                }
                
                std::getline(ifs, nextLine);
                unsigned filter_length;
                num_args_filled = sscanf(nextLine.c_str(), "Filter length  = %u", &filter_length);
                if (1 != num_args_filled)
                {
                    debugStr(DebugStream::error) << boost::format(  "Channel filter 1 length format error (%s)\n" )
                                                                  % nextLine;
                }
                else
                if ((filter_length > CHAN_FILT_MAX_LENGTH) || ((filter_length % 2) == 0))
                {
                    //
                    //  For some reason using a class constant in the Boost formatting below produces a linker error.
                    //
                    unsigned max_filter_length = CHAN_FILT_MAX_LENGTH;
                    debugStr(DebugStream::error) << boost::format(  "Channel filter 1 length (%u) is either too great (> %u) or even\n" )
                                                                  % filter_length
                                                                  % max_filter_length;
                }
                else
                {
                    success = true;
                    
                    //
                    //  Read the coefficients.
                    //
                    unsigned num_coeffs = (filter_length+1)/2;
                    coeffs.assign(num_coeffs, 0);
                    for (unsigned coeff_num = 0 ; coeff_num < num_coeffs ; ++coeff_num)
                    {
                        std::getline(ifs, nextLine);
                        int coeff;
                        num_args_filled = sscanf(nextLine.c_str(), "%d", &coeff);
                        if (1 != num_args_filled)
                        {
                            coeffs.assign(1, CHAN_FILT_MAX_ABS_COEFF_VAL);
                            debugStr(DebugStream::error) << boost::format(  "Channel filter 1 coefficient format error (%s)\n" )
                                                                          % nextLine;
                            success = false;
                            break;
                        }
                        else
                        if (CHAN_FILT_MAX_ABS_COEFF_VAL < static_cast<unsigned>(abs(coeff)))
                        {
                            //
                            //  For some reason using a class constant in the Boost formatting below produces a linker
                            //  error.
                            //
                            unsigned max_abs_coeff_val = CHAN_FILT_MAX_ABS_COEFF_VAL;
                            debugStr(DebugStream::warn2) << boost::format(  "Channel filter 1 coefficient too large or small (abs(%d) > %u)\n" )
                                                                          % coeff
                                                                          % max_abs_coeff_val;
                            if (coeff > 0)
                            {
                                coeff = CHAN_FILT_MAX_ABS_COEFF_VAL;
                            }
                            else
                            {
                                coeff = -CHAN_FILT_MAX_ABS_COEFF_VAL;
                            }
                        }
                        coeffs[coeff_num] = coeff;
                    }
                }
            }
        }
    }
    
    return success;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Read the coefficients for channel filter 2.
//  @details        Read the input file using the format specified in 1QR005, "4G Changes to the HHDFPHY SOftware",
//                  checking the filename format and the content format.
//
//                  Error and warning conditions:
//
//                      - Filename format error,        DebugStream::error,     return false with all-pass filter
//                      - Filter number not 1,          DebugStream::error,     return false with all-pass filter
//                      - File format version wrong,    DebugStream::error,     return false with all-pass filter
//                      - File format error,            DebugStream::error,     return false with all-pass filter
//                      - Scaling factor wrong,         DebugStream::error,     return false with all-pass filter
//                      - Filter size even or greater than the maximum
//                                                      DebugStream::error,     return false with all-pass filter
//                      - Coefficient out of range      DebugStream::warn2,     return true with clipped coefficient
//
//  @param[in]      filename            The name of the coefficient file.
//  @param[out]     chan_bw_MHz         The channel bandwidth in MHz.
//  @param[out]     coeffs              The filter coefficients.
//  @return                             true if the read was successful (all OK or a only warning issued, false
//                                      otherwise.
//
bool Correlator_4g::read_chan_filt_2_coeffs(std::string filename, 
                                            double &chan_bw_MHz,
                                            std::vector<int16_t> &coeffs)
{
    bool success = false;
    
    chan_bw_MHz = 0.0;
    coeffs.assign(1, CHAN_FILT_MAX_ABS_COEFF_VAL);
    
    //
    //  Parse the filename and check those fields that can be checked.
    //
    std::string base_filename = filename.substr(filename.find_last_of("/\\") + 1);
    debugStr(DebugStream::info1) << boost::format(  "Reading channel filter 2 coefficients from %s\n" )
                                                  % filename;
    
    unsigned filter_num;
    unsigned bw_int;
    unsigned bw_dec;
    unsigned file_version;
    int num_args_filled = sscanf(base_filename.c_str(),
                                 "ChanFilt_%u_BW_%u_%u_MHz_v%u.dat",
                                 &filter_num,
                                 &bw_int,
                                 &bw_dec,
                                 &file_version);
    if (4 != num_args_filled)
    {
        debugStr(DebugStream::error) << boost::format(  "Channel filter 2 coefficients filename format error (%s)\n" )
                                                      % filename;
    }
    else
    if (2 != filter_num)
    {
        debugStr(DebugStream::error) << boost::format(  "Channel filter 2 coefficient file filter number (%u) is not 2\n" )
                                                      % filter_num;
    }
    else
    if (COEFF_FILE_FORMAT_VERSION != file_version)
    {
        debugStr(DebugStream::error) << boost::format(  "Channel filter 2 coefficient file version (%u) is unsupported\n" )
                                                      % file_version;
    }
    else
    {
        chan_bw_MHz = static_cast<double>(bw_int)+static_cast<double>(bw_dec)/CHAN_BW_SF;
        
        //
        //  Read the file contents.
        //
        std::ifstream ifs(filename.c_str());
        if (ifs.fail())
        {
            debugStr(DebugStream::error) << boost::format(  "Channel filter 2 file (%s) could not be opened\n" )
                                                          % filename;
        }
        else
        {
            std::string nextLine;
            std::getline(ifs, nextLine);
            unsigned scaling_factor;
            num_args_filled = sscanf(nextLine.c_str(), "Scaling factor = %u", &scaling_factor);
            if (1 != num_args_filled)
            {
                debugStr(DebugStream::error) << boost::format(  "Channel filter 2 scaling factor format error (%s)\n" )
                                                              % nextLine;
            }
            else
            if (scaling_factor != CHAN_FILT_COEFF_SCALING_FACTOR)
            {
                //
                //  For some reason using a class constant in the Boost formatting below produces a linker error.
                //
                unsigned required_sf = CHAN_FILT_COEFF_SCALING_FACTOR;
                debugStr(DebugStream::error) << boost::format(  "Channel filter 2 scaling factor (%u) is not %u\n" )
                                                              % scaling_factor
                                                              % required_sf;
            }
            
            std::getline(ifs, nextLine);
            unsigned filter_length;
            num_args_filled = sscanf(nextLine.c_str(), "Filter length  = %u", &filter_length);
            if (1 != num_args_filled)
            {
                debugStr(DebugStream::error) << boost::format(  "Channel filter 2 length format error (%s)\n" )
                                                              % nextLine;
            }
            else
            if ((filter_length > CHAN_FILT_MAX_LENGTH) || ((filter_length % 2) == 0))
            {
                //
                //  For some reason using a class constant in the Boost formatting below produces a linker error.
                //
                unsigned max_filter_length = CHAN_FILT_MAX_LENGTH;
                debugStr(DebugStream::error) << boost::format(  "Channel filter 2 length (%u) is either too great (> %u) or even\n" )
                                                              % filter_length
                                                              % max_filter_length;
            }
            else
            {
                success = true;
                
                //
                //  Read the coefficients.
                //
                unsigned num_coeffs = (filter_length+1)/2;
                coeffs.assign(num_coeffs, 0);
                for (unsigned coeff_num = 0 ; coeff_num < num_coeffs ; ++coeff_num)
                {
                    std::getline(ifs, nextLine);
                    int coeff;
                    num_args_filled = sscanf(nextLine.c_str(), "%d", &coeff);
                    if (1 != num_args_filled)
                    {
                        coeffs.assign(1, CHAN_FILT_MAX_ABS_COEFF_VAL);
                        debugStr(DebugStream::error) << boost::format(  "Channel filter 2 coefficient format error (%s)\n" )
                                                                      % nextLine;
                        success = false;
                        break;
                    }
                    else
                    if (CHAN_FILT_MAX_ABS_COEFF_VAL < static_cast<unsigned>(abs(coeff)))
                    {
                        //
                        //  For some reason using a class constant in the Boost formatting below produces a linker
                        //  error.
                        //
                        unsigned max_abs_coeff_val = CHAN_FILT_MAX_ABS_COEFF_VAL;
                        debugStr(DebugStream::warn2) << boost::format(  "Channel filter 2 coefficient too large or small (abs(%d) > %u)\n" )
                                                                      % coeff
                                                                      % max_abs_coeff_val;
                        if (coeff > 0)
                        {
                            coeff = CHAN_FILT_MAX_ABS_COEFF_VAL;
                        }
                        else
                        {
                            coeff = -CHAN_FILT_MAX_ABS_COEFF_VAL;
                        }
                    }
                    coeffs[coeff_num] = coeff;
                }
            }
        }
    }
    
    return success;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Read the coefficients for the correlator.
//  @details        Read the input file using the format specified in 1QR005, "4G Changes to the HHDFPHY SOftware",
//                  checking the filename format and the content format.
//
//                  Error and warning conditions:
//
//                      - Filename format error,        DebugStream::error,     return false with all-zero coefficients
//                      - File format version wrong,    DebugStream::error,     return false with all-zero coefficients
//                      - File format error,            DebugStream::error,     return false with all-zero coefficients
//                      - Wrong number of coefficients  DebugStream::error,     return false with all-zero coefficients
//                      - Physical Channel ID out of range
//                                                      DebugStream::warn2,     return true
//                      - Coefficient mean not 0.0 or variance not 1.0 (error > 1e-3)
//                                                      DebugStream::warn2,     return true
//                      - Coefficient mean not 0.0 or variance not 1.0 (error > float epsilon)
//                                                      DebugStream::infor3,    return true
//
//  @param[in]      filename            The name of the coefficient file.
//  @param[out]     chan_bw_MHz         The channel bandwidth in MHz.
//  @param[out]     coeffs              The correlator coefficients.
//  @return                             true if the read was successful (all OK or a only warning issued, false
//                                      otherwise.
//
bool Correlator_4g::read_corr_coeffs(std::string filename, 
                                     double &chan_bw_MHz, 
                                     std::vector<float> &coeffs)
{
    bool success = false;
    
    chan_bw_MHz = 0.0;
    coeffs.assign(NUM_CORR_COEFFS, 0.0);
    
    //
    //  Parse the filename and check those fields that can be checked.
    //
    std::string base_filename = filename.substr(filename.find_last_of("/\\") + 1);
    debugStr(DebugStream::info1) << boost::format(  "Reading correlator coefficients from %s\n" )
                                                  % filename;
    
    unsigned bw_int;
    unsigned bw_dec;
    unsigned id;
    unsigned cp;        //  Will read and ignore - is only informational.
    unsigned file_version;
    int num_args_filled = sscanf(base_filename.c_str(),
                                 "Correlator_BW_%u_%u_MHz_ID_%u_CP_%u_v%u.dat",
                                 &bw_int,
                                 &bw_dec,
                                 &id,
                                 &cp,
                                 &file_version);
    if (5 != num_args_filled)
    {
        debugStr(DebugStream::error) << boost::format(  "Correlator coefficients filename format error (%s)\n" )
                                                      % filename;
    }
    else
    if (COEFF_FILE_FORMAT_VERSION != file_version)
    {
        debugStr(DebugStream::error) << boost::format(  "Correlator coefficient file version (%u) is unsupported\n" )
                                                      % file_version;
    }
    else
    {
        chan_bw_MHz = static_cast<double>(bw_int)+static_cast<double>(bw_dec)/CHAN_BW_SF;
        
        if (MAX_PHY_CHAN_ID < id)
        {
            //
            //  For some reason using a class constant in the Boost formatting below produces a linker error.
            //
            unsigned max_phy_chan_id = MAX_PHY_CHAN_ID;
            debugStr(DebugStream::warn2) << boost::format(  "Correlator coefficient ID too large (%u > %u)\n" )
                                                          % id
                                                          % max_phy_chan_id;
        }
        
        //
        //  Read the file contents.
        //
        std::ifstream ifs(filename.c_str());
        if (ifs.fail())
        {
            debugStr(DebugStream::error) << boost::format(  "Coefficient file (%s) could not be opened\n" )
                                                          % filename;
        }
        else
        {
            std::string nextLine;
            std::getline(ifs, nextLine);
            unsigned corr_len;
            num_args_filled = sscanf(nextLine.c_str(), "Correlator length = %u", &corr_len);
            if (1 != num_args_filled)
            {
                debugStr(DebugStream::error) << boost::format(  "Correlator coefficient format error (%s)\n" )
                                                              % nextLine;
            }
            else
            if (corr_len != NUM_CORR_COEFFS)
            {
                //
                //  For some reason using a class constant in the Boost formatting below produces a linker error.
                //
                unsigned num_corr_coeffs = NUM_CORR_COEFFS;
                debugStr(DebugStream::error) << boost::format(  "Number of correlation coefficients (%u) is not %u\n" )
                                                              % corr_len
                                                              % num_corr_coeffs;
            }
            else
            {
                success = true;
                
                //
                //  Read the coefficients and calculate the mean and variance and check that they are 0 and 1 respectively.
                //
                double coeffs_sum = 0.0;
                double coeffs_sq_sum = 0.0;
                for (unsigned coeff_num = 0 ; coeff_num < NUM_CORR_COEFFS ; ++coeff_num)
                {
                    std::string nextLine;
                    std::getline(ifs, nextLine);
                    float coeff;
                    num_args_filled = sscanf(nextLine.c_str(), "%f", &coeff);
                    if (1 != num_args_filled)
                    {
                        coeffs.assign(NUM_CORR_COEFFS, 0.0);
                        debugStr(DebugStream::error) << boost::format(  "Correlator coefficient format error (%s)\n" )
                                                                      % nextLine;
                        success = false;
                        break;
                    }
                    coeffs[coeff_num] = coeff;
                    
                    coeffs_sum += static_cast<double>(coeff);
                    coeffs_sq_sum += static_cast<double>(coeff)*static_cast<double>(coeff);
                }

                if (success)
                {
                    //
                    //  Check the mean and variance.
                    //
                    double mean = coeffs_sum/static_cast<double>(NUM_CORR_COEFFS);
                    double var  = coeffs_sq_sum/static_cast<double>(NUM_CORR_COEFFS)-mean*mean;
                    
                    double mean_err = fabs(mean);
                    double var_err  = fabs(var-1.0);
                    
                    double warn2_limit = 1e-3;
                    double info3_limit = static_cast<double>(std::numeric_limits<float>::epsilon());
                    if ((mean_err > warn2_limit) || (var_err > warn2_limit))
                    {
                        debugStr(DebugStream::warn2) << boost::format(  "Correlator coefficient mean (%e) not 0.0 or variance (%e) not 1.0\n" )
                                                                      % mean
                                                                      % var;
                    }
                    else
                    if ((mean_err > info3_limit) || (var_err > info3_limit))
                    {
                        debugStr(DebugStream::info3) << boost::format(  "Correlator coefficient mean (%e) not 0.0 or variance (%e) not 1.0\n" )
                                                                      % mean
                                                                      % var;
                    }
                }
            }
        }
    }
    
    return success;
}

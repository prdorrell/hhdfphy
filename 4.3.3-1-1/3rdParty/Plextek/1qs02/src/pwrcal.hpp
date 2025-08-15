/***********************************************************************************************************************
 *  Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 *  --------------------------------------------------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/pwrcal.hpp $
 * $Revision: 5327 $
 * $Author: pdm $
 * $Date: 2011-04-12 11:35:01 +0100 (Tue, 12 Apr 2011) $
 *******************************************************************************************************************//**
 *  @file
 *  @brief          The declaration of the ::PwrCal class that provides power calibration data.
 **********************************************************************************************************************/
#ifndef __PWRCAL_HPP__
#define __PWRCAL_HPP__

#include "radio.hpp"

#include <boost/utility.hpp>
#include <map>

///
/// @brief          Provides power calibration data based on the contents of the configuration file.
/// @details        The power calibration data is stored in a configuration file that is accessed through an instance of
///                 the ::ConfigFile class.  The fields in the configuration file are:
///
///                     -   "FILE_FORMAT_VERSION", a single digit that should increment if the file format changes.
///                     -   "CALIBRATION_DATE", a date string or "None" if no calibration has been performed.
///                     -   "CAL_POINT_nnn", the calibration points (nnn is a 3-digit integer with leading zeros).  The
///                         value is a comma-separated list with the following format
///                         "<band>, <frequency>, <gain>, <value>".
///
///                 There may be up to 1000 calibration points.  Linear interpolation of the calibration values (in dB)
///                 in the same band is performed to calculate the values for arbitrary frequencies and gains.
///
///                 When implementing the linear interpolation, the algorithm first interpolates between calibration
///                 points on the same frequency and then interpolates between frequencies.  If the gain or frequency
///                 setting does not lie between two calibration points then the nearest is used and no interpolation
///                 is performed.
///
class PwrCal : private boost::noncopyable
{
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // The constants.
        //
    private:
        ///
        /// @brief          The default calibration value that is used when no other is known.
        ///
        static const double DEFAULT_CAL_VAL_DB;

        ///
        /// @brief          The key string for the file format version number.
        ///
        static const char *FILE_FORMAT_VERSION_KEY;

        ///
        /// @brief          The key string for the calibration date.
        ///
        static const char *CALIBRATION_DATE_KEY;

        ///
        /// @brief          The key string for the calibration points.
        ///
        static const char *CALIBRATION_POINT_KEY;

        ///
        /// @brief          The key string format for the calibration points.
        ///
        static const char *CALIBRATION_POINT_KEY_FORMAT;

        ///
        /// @brief          The maximum number of calibration points.
        ///
        static const unsigned MAX_NUM_CAL_POINTS;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // The types.
        //
    private:
        ///
        /// @brief          Used in the lookup table for interpreting the configuration file band strings.
        ///
        typedef std::map<std::string, Radio::band_t>    StrToBandTab;

        ///
        /// @brief          The calibration points for a frequency, indexed by the gain.
        ///
        typedef std::map<uint16_t, double>              GainCalTab;

        ///
        /// @brief          The calibration points for a band, indexed by the frequency.
        ///
        typedef std::map<uint32_t, GainCalTab>          FreqCalTab;

        ///
        /// @brief          The calibration points, indexed by the band.
        ///
        typedef std::map<Radio::band_t, FreqCalTab>     CalTab;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // The values.
        //
    private:
        ///
        /// @brief          The current band.
        ///
        Radio::band_t   currBand;

        ///
        /// @brief          The current frequency.
        ///
        uint32_t        currFreq_Hz;

        ///
        /// @brief          The calibration table.
        ///
        CalTab          calTab;

        ///
        /// @brief          The output stream for debug messages.
        ///
        DebugStream     debugStr;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          Constructs an instance of the class from the contents of the specified file.
        /// @details        The file is parsed and the calibration data is cached.  If an error occurs default
        ///                 calibration values will be returned.
        /// @param[in]      filename            The name of the configuration file.
        /// @param[in]      debugMask           The mask controlling the active debug levels.
        ///
    public:
        PwrCal(std::string filename, unsigned debugMask);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          The destructor has nothing to do.
        ///
    public:
        ~PwrCal(void);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          Sets the band for which subsequent requests for calibration data apply.
        /// @param[in]      newBand             The band.
        ///
    public:
        void setBand(Radio::band_t newBand);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          Sets the frequency for which subsequent requests for calibration data apply.
        /// @param[in]      newFreq_Hz          The frequency in Hz.
        ///
    public:
        void setFreq(uint32_t newFreq_Hz);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          Retrieves the linear power calibration factor.
        /// @details        The factor should be applied to linear power samples normalised to the channel filter
        ///                 output.  It includes a component that corresponds to the frontend gain.
        /// @param[in]      gain_dB             The frontend gain in dB.
        /// @return                             The calibration factor.
        ///
    public:
        double getFactor(uint16_t gain_dB) const;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// @brief          Retrieves the linear power calibration factor for a particular gain.
        /// @details        Identifies the gains for which calibration factors are known and calculates the calibration
        ///                 factor for the current gain.
        ///
        ///                 If there are no entries in the lookup table then the default calibration factor is returned.
        ///
        ///                 If there is just one entry then it is used for all gains.
        ///
        ///                 If there is more than one entry linear interpolation (of the dB values) is performed unless
        ///                 the current gain is larger than the largest gain in the table or smaller than the smallest.
        ///                 In this case the calibration factor for the largest or smallest gains is used without
        ///                 interpolation.
        /// @param[in]      gainCalTab          The lookup table of calibration factors.
        /// @param[in]      gain_dB             The frontend gain in dB.
        /// @return                             The calibration factor for the given frontend gain.
        ///
    private:
        double getFactorByGain(const GainCalTab &gainCalTab, uint16_t gain_dB) const;
};

#endif

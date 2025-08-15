/***********************************************************************************************************************
 *  Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 *  --------------------------------------------------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/pwrcal.cpp $
 * $Revision: 6285 $
 * $Author: pdm $
 * $Date: 2011-07-12 17:45:49 +0100 (Tue, 12 Jul 2011) $
 *******************************************************************************************************************//**
 *  @file
 *  @brief          The implementation of the ::PwrCal class that provides power calibration data.
 **********************************************************************************************************************/
#include "configfile.hpp"
#include "debug.hpp"
#include "pwrcal.hpp"

#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <cmath>

//
//  @brief          The default calibration value that is used when no other is known.
//
const double PwrCal::DEFAULT_CAL_VAL_DB          = -68.0;

//
//  @brief          The key string for the file format version number.
//
const char *PwrCal::FILE_FORMAT_VERSION_KEY      = "FILE_FORMAT_VERSION";

//
//  @brief          The key string for the calibration date.
//
const char *PwrCal::CALIBRATION_DATE_KEY         = "CALIBRATION_DATE";

//
//  @brief          The key string for the calibration points.
//
const char *PwrCal::CALIBRATION_POINT_KEY        = "CAL_POINT_000";

//
//  @brief          The key string format for the calibration points.
//
const char *PwrCal::CALIBRATION_POINT_KEY_FORMAT = "CAL_POINT_%03u";

//
//  @brief          The maximum number of calibration points.
//
const unsigned PwrCal::MAX_NUM_CAL_POINTS = 1000;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class from the contents of the specified file.
//  @details        The file is parsed and the keys and values are cached in the PwrCal::lookup_table.  If an error
//                  occurs the cache will be empty and requests for the values will yield an empty string.
//  @param[in]      filename            The name of the configuration file.
//  @param[in]      debugMask           The mask controlling the active debug levels.
//
PwrCal::PwrCal(std::string filename, unsigned debugMask) :
    currBand(Radio::INVALID_BAND),
    currFreq_Hz(0),
    calTab(),
    debugStr("PwrCal...... ", debugMask)
{
    //
    //  Populate the calibration table from the configuration file.
    //
    ConfigFile pwr_cal_config(filename);

    std::string fileFormatVersion = pwr_cal_config.getValue(FILE_FORMAT_VERSION_KEY);
    std::string calDate           = pwr_cal_config.getValue(CALIBRATION_DATE_KEY);

    if (fileFormatVersion != "1")
    {
        debugStr(DebugStream::error) << "Unsupported file format - using default calibration factors.\n";

#ifdef EXIT_ON_CAL_FILE_ERROR
        exit(1);
#endif
    }
    else
    {
        if (calDate == "None")
        {
            debugStr(DebugStream::warn1) << "No calibration data in file  - using default calibration factors.\n";
        }
        else
        {
            debugStr(DebugStream::info1) << "Power calibration data:\n"
                                         << "\n"
                                         << "    FILE_FORMAT_VERSION = " << fileFormatVersion << "\n"
                                         << "    CALIBRATION_DATE    = " << calDate           << "\n"
                                         << "\n";

            for (unsigned nextCalPointNum = 0 ; nextCalPointNum < MAX_NUM_CAL_POINTS ; ++nextCalPointNum)
            {
                char calPointKey[strlen(CALIBRATION_POINT_KEY)+1];
                sprintf(calPointKey, CALIBRATION_POINT_KEY_FORMAT, nextCalPointNum);

                std::string calPointVal = pwr_cal_config.getValue(std::string(calPointKey));
                if (calPointVal.empty())
                {
                    break;
                }
                else
                {
                    //
                    //  Extract the fields, stripping the commas and spaces, and convert the strings into their internal
                    //  representations.
                    //
                    boost::char_separator<char> sep(", ");

                    boost::tokenizer<boost::char_separator<char> > tokens(calPointVal, sep);

                    boost::tokenizer<boost::char_separator<char> >::iterator tok_iter = tokens.begin();

                    std::string bandStr = *tok_iter++;
                    std::string freqStr = *tok_iter++;
                    std::string gainStr = *tok_iter++;
                    std::string calStr  = *tok_iter++;

                    StrToBandTab strToBandTab;
                    strToBandTab["UL800" ] = Radio::UL800;
                    strToBandTab["DL800" ] = Radio::DL800;
                    strToBandTab["UL850" ] = Radio::UL850;
                    strToBandTab["DL850" ] = Radio::DL850;
                    strToBandTab["UL900" ] = Radio::UL900;
                    strToBandTab["DL900" ] = Radio::DL900;
                    strToBandTab["UL1700"] = Radio::UL1700;
                    strToBandTab["DL1700"] = Radio::DL1700;
                    strToBandTab["UL1800"] = Radio::UL1800;
                    strToBandTab["DL1800"] = Radio::DL1800;
                    strToBandTab["UL1900"] = Radio::UL1900;
                    strToBandTab["DL1900"] = Radio::DL1900;
                    strToBandTab["UL2100"] = Radio::UL2100;
                    strToBandTab["DL2100"] = Radio::DL2100;

                    StrToBandTab::iterator bandEntry = strToBandTab.find(bandStr);
                    if (bandEntry == strToBandTab.end())
                    {
                        debugStr(DebugStream::error) << "Corrupt file format - using default calibration factors.\n";

#ifdef EXIT_ON_CAL_FILE_ERROR
        exit(1);
#endif
                        break;
                    }
                    Radio::band_t band = bandEntry->second;

                    uint32_t freq_Hz;
                    uint16_t gain_dB;
                    float    cal_dB;
                    try
                    {
                        freq_Hz = boost::lexical_cast<uint32_t>(freqStr);
                        gain_dB = boost::lexical_cast<uint16_t>(gainStr);
                        cal_dB  = boost::lexical_cast<float>(calStr);
                    }
                    catch(boost::bad_lexical_cast &)
                    {
                        debugStr(DebugStream::error) << "Corrupt file format - using default calibration factors.\n";

#ifdef EXIT_ON_CAL_FILE_ERROR
        exit(1);
#endif
                        break;
                    }

                    //
                    //  Find the entry in the full calibration table for this band.
                    //
                    CalTab::iterator calEntry = calTab.find(band);
                    if (calEntry == calTab.end())
                    {
                        //
                        //  Create a new entry.
                        //
                        GainCalTab freqCalTabEntry;
                        freqCalTabEntry[gain_dB] = cal_dB;

                        FreqCalTab bandCalTabEntry;
                        bandCalTabEntry[freq_Hz] = freqCalTabEntry;

                        calTab[band] = bandCalTabEntry;
                    }
                    else
                    {
                        //
                        //  Find the entry in the band calibration table for this frequency.
                        //
                        FreqCalTab::iterator freqEntry = calEntry->second.find(freq_Hz);
                        if (freqEntry == calEntry->second.end())
                        {
                            //
                            //  Create a new entry.
                            //
                            GainCalTab freqCalTabEntry;
                            freqCalTabEntry[gain_dB] = cal_dB;

                            calEntry->second[freq_Hz] = freqCalTabEntry;
                        }
                        else
                        {
                            //
                            //  Find the entry in the frequency calibration table for this gain.
                            //
                            GainCalTab::iterator gainEntry = freqEntry->second.find(gain_dB);
                            if (gainEntry == freqEntry->second.end())
                            {
                                //
                                //  Create a new entry.
                                //
                                freqEntry->second[gain_dB] = cal_dB;
                            }
                            else
                            {
                                //
                                //  An entry already exists - ignore the new one.
                                //
                                debugStr(DebugStream::error) << "Duplicate calibration point - ignoring new one.\n";

#ifdef EXIT_ON_CAL_FILE_ERROR
        exit(1);
#endif
                            }
                        }
                    }
                }
            }

            //
            //  Report the table for debugging.
            //
            for ( CalTab::iterator bandTabIter = calTab.begin()
                ; bandTabIter != calTab.end()
                ; ++bandTabIter)
            {
                debugStr(DebugStream::info1) << bandTabIter->first << "\n";

                for ( FreqCalTab::iterator freqTabIter = bandTabIter->second.begin()
                    ; freqTabIter != bandTabIter->second.end()
                    ; ++freqTabIter)
                {
                    debugStr(DebugStream::info1) << "    " << freqTabIter->first << "\n";

                    for ( GainCalTab::iterator gainTabIter = freqTabIter->second.begin()
                        ; gainTabIter != freqTabIter->second.end()
                        ; ++gainTabIter)
                    {
                        debugStr(DebugStream::info1) << "        " << gainTabIter->first
                                                          << " : " << gainTabIter->second << "\n";
                    }
                }
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          The destructor has nothing to do.
//
PwrCal::~PwrCal(void)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Sets the band for which subsequent requests for calibration data apply.
//  @param[in]      newBand             The band.
//
void PwrCal::setBand(Radio::band_t newBand)
{
    currBand = newBand;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Sets the frequency for which subsequent requests for calibration data apply.
//  @param[in]      freq_Hz             The frequency in Hz.
//
void PwrCal::setFreq(uint32_t newFreq_Hz)
{
    currFreq_Hz = newFreq_Hz;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Retrieves the linear power calibration factor.
//  @details        The factor should be applied to linear power samples normalised to the channel filter
//                  output.  It includes a component that corresponds to the frontend gain.
//  @param[in]      gain_dB             The frontend gain in dB.
//  @return                             The calibration factor.
//
double PwrCal::getFactor(uint16_t gain_dB) const
{
    double calVal_dB = DEFAULT_CAL_VAL_DB;

    //
    //  Find the calibration table entry for the current band.  If there isn't one then use the default factor.
    //
    CalTab::const_iterator calEntry = calTab.find(currBand);
    if (calEntry != calTab.end())
    {
        if (calEntry->second.size() >= 2)
        {
            //
            //  Find the entries either side of the current frequency.
            //
            //  Note about lower_bound:
            //
            //      "Returns an iterator pointing to the first element in the container whose key does not compare less
            //       than x (using the container's comparison object), i.e. it is either equal or greater."
            //
            //  So first identify the entry for a calibration frequency that is greater than or equal to the current
            //  frequency.
            //
            FreqCalTab::const_iterator freqTabAbove = calEntry->second.lower_bound(currFreq_Hz);

            if (freqTabAbove == calEntry->second.end())
            {
                //
                //  If there is no such entry use the last entry.
                //
                --freqTabAbove;
                calVal_dB = getFactorByGain(freqTabAbove->second, gain_dB);
            }
            else
            if (freqTabAbove == calEntry->second.begin())
            {
                //
                //  If the entry is the first in the table use it.
                //
                calVal_dB = getFactorByGain(freqTabAbove->second, gain_dB);
            }
            else
            if (freqTabAbove->first == currFreq_Hz)
            {
                //
                //  If the entry is an exact match use it without interpolation.
                //
                calVal_dB = getFactorByGain(freqTabAbove->second, gain_dB);
            }
            else
            {
                //
                //  Otherwise perform the interpolation.
                //
                FreqCalTab::const_iterator freqTabBelow = freqTabAbove;
                --freqTabBelow;

                uint32_t    freqAbove_Hz   = freqTabAbove->first;
                double      calValAbove_dB = getFactorByGain(freqTabAbove->second, gain_dB);

                uint32_t    freqBelow_Hz   = freqTabBelow->first;
                double      calValBelow_dB = getFactorByGain(freqTabBelow->second, gain_dB);

                calVal_dB =   (calValAbove_dB-calValBelow_dB)
                            * (static_cast<double>(currFreq_Hz )-static_cast<double>(freqBelow_Hz))
                            / (static_cast<double>(freqAbove_Hz)-static_cast<double>(freqBelow_Hz))
                            + calValBelow_dB;
            }
        }
        else
        if (calEntry->second.size() == 1)
        {
            //
            //  Use the only entry for all values.
            //
            FreqCalTab::const_iterator freqTab = calEntry->second.begin();
            calVal_dB = getFactorByGain(freqTab->second, gain_dB);
        }
    }

    return (pow(10.0, (+calVal_dB-static_cast<double>(gain_dB))/10.0));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Retrieves the linear power calibration factor for a particular gain.
//  @details        Identifies the gains for which calibration factors are known and calculates the calibration factor
//                  for the current gain.
//
//                  If there are no entries in the lookup table then the default calibration factor is returned.
//
//                  If there is just one entry then it is used for all gains.
//
//                  If there is more than one entry linear interpolation (of the dB values) is performed unless the
//                  current gain is larger than the largest gain in the table or smaller than the smallest.  In this
//                  case the calibration factor for the largest or smallest gains is used without interpolation.
//
//  @param[in]      gainCalTab          The lookup table of calibration factors.
//  @param[in]      gain_dB             The frontend gain in dB.
//  @return                             The calibration factor for the given frontend gain.
//
double PwrCal::getFactorByGain(const GainCalTab &gainCalTab, uint16_t gain_dB) const
{
    double calVal_dB = DEFAULT_CAL_VAL_DB;

    if (gainCalTab.size() >= 2)
    {
        //
        //  Find the entries either side of the current gain.
        //
        //  Note about lower_bound:
        //
        //      "Returns an iterator pointing to the first element in the container whose key does not compare
        //       less than x (using the container's comparison object), i.e. it is either equal or greater."
        //
        //  So first identify the entry for a calibration gain that is greater than or equal to the current gain.
        //
        GainCalTab::const_iterator entryAbove = gainCalTab.lower_bound(gain_dB);

        if (entryAbove == gainCalTab.end())
        {
            //
            //  If there is no such entry use the last entry.
            //
            --entryAbove;
            calVal_dB  = entryAbove->second;
        }
        else
        if (entryAbove == gainCalTab.begin())
        {
            //
            //  If the entry is the first in the table use it.
            //
            calVal_dB  = entryAbove->second;
        }
        else
        if (entryAbove->first == gain_dB)
        {
            //
            //  If the entry is an exact match use it without interpolation.
            //
            calVal_dB  = entryAbove->second;
        }
        else
        {
            //
            //  Otherwise perform the interpolation.
            //
            GainCalTab::const_iterator entryBelow = entryAbove;
            --entryBelow;

            calVal_dB =   (entryAbove->second-entryBelow->second)
                        * (static_cast<double>(gain_dB          )-static_cast<double>(entryBelow->first))
                        / (static_cast<double>(entryAbove->first)-static_cast<double>(entryBelow->first))
                        + entryBelow->second;
        }
    }
    else
    if (gainCalTab.size() == 1)
    {
        //
        //  Use the only entry for all values.
        //
        calVal_dB = gainCalTab.begin()->second;
    }

    return (calVal_dB);
}

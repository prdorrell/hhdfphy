/***********************************************************************************************************************
 *  Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 *  --------------------------------------------------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/configfile.hpp $
 * $Revision: 5452 $
 * $Author: pdm $
 * $Date: 2011-04-21 17:27:26 +0100 (Thu, 21 Apr 2011) $
 *******************************************************************************************************************//**
 *  @file
 *  @brief          The declaration of the ::ConfigFile class that provides an interface to configuration files.
 **********************************************************************************************************************/
#ifndef __CONFIGFILE_HPP__
#define __CONFIGFILE_HPP__

#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>

///
/// @brief          Provides access to configuration files.
/// @details        This class provides access configuration files.
///
///                 Configuration files have a simple line-based structure:
///
///                     -   Leading whitespaces are ignored.
///                     -   Lines that begin with '#' are comments and are ignored.
///                     -   Lines with "=" define a key on the lefthand side of the "=" and a value on the righthand
///                         side.  Leading whitespaces are stripped from the value, and all whitespaces are stripped
///                         from the key.
///
///                 Both key and value are strings.  Any structure within the value is imposed by the class's client,
///                 for example the line:
///
///                     "CALIBRATION_DATE = 25/03/2011"
///
///                 Is parsed to produce a key string "CALIBRATION_DATE" and a value string "25/03/2011".
///
class ConfigFile : private boost::noncopyable
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The constants.
    //
private:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The types.
    //
public:
    ///
    /// @brief          The map used to store the cached keys and values.
    ///
    typedef std::map<std::string, std::string> LookupTableT;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The values.
    //
private:
    ///
    /// @brief          The cached keys and values.
    ///
    LookupTableT    lookup_table;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Constructs an instance of the class from the contents of the specified file.
    /// @details        The file is parsed and the keys and values are cached in the ConfigFile::lookup_table.  If an
    ///                 error occurs the cache will be empty and calls to ConfigFile::getValue() will yield an empty
    ///                 string.
    /// @param[in]      filename            The name of the configuration file.
    ///
public:
    ConfigFile(std::string filename);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The destructor has nothing to do.
    ///
public:
    ~ConfigFile(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Removes spaces from a key string.
    /// @param[in]      key                 The key string.
    /// @return                             The key string with all whitespaces removed.
    ///
private:
    std::string pack_key(const char *key);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Retrieves the value string corresponding to a key string.
    /// @param[in]      key                 The key string.
    /// @return                             The value string - empty if the key is not found.
    ///
public:
    std::string getValue(const std::string &key);
};

#endif

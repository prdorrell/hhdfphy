/***********************************************************************************************************************
 *  Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 *  --------------------------------------------------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/configfile.cpp $
 * $Revision: 5327 $
 * $Author: pdm $
 * $Date: 2011-04-12 11:35:01 +0100 (Tue, 12 Apr 2011) $
 *******************************************************************************************************************//**
 *  @file
 *  @brief          The implementation of the ::ConfigFile class that provides an interface to configuration files.
 **********************************************************************************************************************/
#include "configfile.hpp"

#include <cstdio>
#include <iostream>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class from the contents of the specified file.
//  @details        The file is parsed and the keys and values are cached in the ConfigFile::lookup_table.  If an error
//                  occurs the cache will be empty and requests for the values will yield an empty string.
//  @param[in]      filename            The name of the configuration file.
//
ConfigFile::ConfigFile(std::string filename) :
    lookup_table()
{
    FILE *infile = fopen(filename.c_str(), "r");

    if (infile)
    {
        const int LINEBUFFSIZE=1024;
        char linebuff[LINEBUFFSIZE];

        while (!feof(infile))
        {
            if (!fgets(linebuff, LINEBUFFSIZE, infile))
            {
                break;
            }

            linebuff[LINEBUFFSIZE-1] = 0;
            char *key = linebuff;;
            char *value;

            // Strip leading whitespaces from the line
            while (*key && isspace(*key ))
            {
                key++;
            }

            // If first non-whitespace character in line is a '#' then ignore the line (ie its a comment line)
            if (*key == '#')
            {
                continue;
            }

            // Get rid of line endings
            value = strchr(key, '\n');
            if (value != 0)
            {
                *value = 0;
            }
            value = strchr(key, '\r');
            if (value != 0)
            {
                *value = 0;
            }

            // Ignore anything that does not have an equals sign in it. The equals is used to delimit
            // the key and value fields
            //
            value = strchr(key, '=');
            if (value != 0)
            {
                // Seperate the key from the value.
                // value will now point to the start of the value field - remove any leading whitespace and
                // it is ready to go
                //
                *(value++) = 0;

                while (isspace(*value))
                {
                    value++;
                }

                lookup_table.insert(std::make_pair(pack_key(key), std::string(value)));
            }
        }

        fclose(infile);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          The destructor has nothing to do.
//
ConfigFile::~ConfigFile(void)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Removes spaces from a key string.
//  @param[in]      key                 The key string.
//  @return                             The key string with all whitespaces removed.
//
std::string ConfigFile::pack_key(const char *key)
{
    std::string key_str;

    for (; key && *key; key++)
    {
        if (!isspace(*key))
        {
            key_str += *key;
        }
    }

    return key_str;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Retrieves the value string corresponding to a key string.
//  @param[in]      key                 The key string.
//  @return                             The value string - empty if the key is not found.
//
std::string ConfigFile::getValue(const std::string &key)
{
    std::string value;

    LookupTableT::iterator pos = lookup_table.find(pack_key(key.c_str()));

    if (pos != lookup_table.end())
    {
        value = pos->second;
    }

    return (value);
}

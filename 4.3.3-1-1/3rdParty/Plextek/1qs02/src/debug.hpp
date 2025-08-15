/******************************************************************************
 * Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 * All Rights Reserved
 ******************************************************************************
 * THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 * ----------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/debug.hpp $
 * $Revision: 4801 $
 * $Author: pdm $
 * $Date: 2011-02-22 16:25:52 +0000 (Tue, 22 Feb 2011) $
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file debug.hpp
 * \brief A stream that can be set for different levels.
 * Preppends prefix to lines (set when object instantiated).
 * Passes written output on to cout when \\n received.
 * Bit mask to control whether level is displayed or not.
 * Text is also coloured depending on level.
 *
 *****************************************************************************/

#ifndef __DEBUG_HPP__
#define __DEBUG_HPP__

#include <iostream>
#include <streambuf>
#include <string>

/// Special buffer of characters based on std::streambuf
class DebugBuffer: public std::streambuf{
    std::string buffer;     ///< line based buffer
    std::string prefix;     ///< added to begining of line
    std::string code;       ///< colour escape code
    unsigned show_level;    ///< Level (bit mask) at which message should be shown.
    unsigned level;         ///< Current level of message in buffer.

    /// write out the line
    void flush()
    {
        std::cout << prefix << code << buffer << "\e[0m";
        buffer.clear();
    }

    /// gets called as charactors added
    int_type overflow( int_type i )
    {
        if( (show_level&level) && i != traits_type::eof() ){
            char_type c = traits_type::to_char_type(i);
            buffer.push_back( c );
            if( c == '\n' ){
                flush();
            }
        }
        return traits_type::not_eof(i);
    }

public:
    /// ctor
    /// \param [in] p prefix to use
    /// \param [in] s initial mask controlling if lines get written out
    DebugBuffer( std::string p, unsigned s )
    {
        prefix = p;
        show_level = s;
        level = 0xff;
    }

    /// Make sure it has been written
    ~DebugBuffer()
    {
        if( buffer != "" ) flush();
    }

    /// Set the mask of shown levels.
    void set_show_level( unsigned l )
    {
        if( buffer != "" ) flush();
        buffer.clear();
        show_level = l;
    }

    /// Get the mask which levels are shown.
    unsigned get_state( void )
    {
        return show_level;
    }

    /// Set level of stream.
    void set_state( unsigned l )
    {
        level = l;
        if     ( level & 0xc0 ) code = "\e[36m"; // info1,2 cyan
        else if( level & 0x20 ) code = "\e[37m"; // info3 white
        else if( level & 0x10 ) code = "\e[32m"; // info4 green
        else if( level & 0x0c ) code = "\e[33m"; // warning yellow
        else if( level & 0x03 ) code = "\e[31m"; // error red
    }
};

/// \brief Stream class that passes on charactors to our buffer class.
//
/// Adds functionality to set level on top of standard stream << operator
// technically this should problaby be a templated thing and use ostream_basic as base
// but I just don't care about wchars etc for now
class DebugStream: public std::ostream{
    DebugBuffer buffer;     ///< line based buffer
public:
    /// \name Verbosity Levels
    /// \brief Named levels the stream can be set to.
    //@{
    static unsigned const off   = 0;
    static unsigned const fatal = 1;
    static unsigned const error = 2;
    static unsigned const warn2 = 4;    ///< something bit wrong but not error
    static unsigned const warn1 = 8;    ///< unimplemented things
    static unsigned const info4 = 16;   ///< important info
    static unsigned const info3 = 32;   ///< verbose other info
    static unsigned const info2 = 64;   ///< verbose port writes etc
    static unsigned const info1 = 128;  ///< verbose controller detail
    static unsigned const all   = 0xff;
    //@}

    /// ctor
    DebugStream( std::string const & prefix, unsigned show_level = off )
        :
        std::ostream( &buffer ),
        buffer( prefix, show_level )
    {}

    /// Set the mask of shown levels.
    void show_level( unsigned l )
    {
        buffer.set_show_level( l );
    }

    /// Get the mask which levels are shown.
    int  get_state( void )
    {
        return buffer.get_state();
    }

    /// Set level of stream. Use one of the verbosity level constants
    /// defined in this class.
    DebugStream& operator()( unsigned l )
    {
        buffer.set_state( l );
        return *this;
    }
};

#endif

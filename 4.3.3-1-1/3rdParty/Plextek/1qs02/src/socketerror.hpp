/******************************************************************************
 * Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 * All Rights Reserved
 ******************************************************************************
 * THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 * ----------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/socketerror.hpp $
 * $Revision: 4104 $
 * $Author: dfc $
 * $Date: 2010-12-07 17:23:23 +0000 (Tue, 07 Dec 2010) $
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file socketerror.hpp
 * \brief definition of SocketError and derived.
 * Exception that can be thrown when theres a tcp/ip socket error.
 *
 *****************************************************************************/
#ifndef __SOCKETERROR_HPP__
#define __SOCKETERROR_HPP__

#include <stdexcept>

/// \brief Base - this is used for serious miscilanious errors
class SocketError : public std::runtime_error
{
    public:
        /// just add text and pass on
        explicit SocketError( std::string const & s ) : runtime_error("SocketError: " + s) {}
};

/// \brief Operation was interupted by signal, could continue
class SocketErrorSignal : public SocketError
{
    public:
        /// just add text and pass on
        explicit SocketErrorSignal( std::string const & s ) : SocketError("Signal: " + s) {}
};

/// \brief Disconnected socket - can try to reconnect
class SocketErrorDisconnect : public SocketError
{
    public:
        /// just add text and pass on
        explicit SocketErrorDisconnect( std::string const & s ) : SocketError("Disconnect: " + s) {}
};


#endif

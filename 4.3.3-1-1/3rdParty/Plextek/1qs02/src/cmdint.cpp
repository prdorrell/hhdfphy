/******************************************************************************
 *
 *
 ******************************************************************************
 *  Filename:   cmdint.cpp
 *  Author(s):  pdm
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file cmdint.cpp
 * \brief cmdInt_t class listens to TCP/IP socket for client to connect.
 * It then accepts \\n terminated commands defined in interface.def and
 * passes them to a parser function that converts the text into internal
 * representation.
 *
 * It is called from phy.cpp where the parsed command is processed.
 *
 *****************************************************************************/

#include "cmdint.hpp"
#include "socketerror.hpp"
#include "design.hpp"

#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <iostream>
#include <stdexcept>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;

/// The class is only designed to support one client at a time.
const int       MAX_NUM_CONTROLLERS = 1;

// /////////////////////////////////////////////////////////////////////////////
/// Create socket files and initialise handler dispatch tables
//
/// The constructor establishes TCP/IP socket endpoint on which the instance can
/// later listen for incoming connection requests by a TCP/IP client.  If this
/// fails then the instance will be left in the disabled state from which the
/// only recovery is to destroy the instance and create a new one.  However, as
/// failure indicates a fundamental problem with the TCP/IP sockets (most likely
/// is that the port is already in use), it is unlikely that recovery will
/// possible.
///
/// Establishing the endpoint consists of two stages.  First a socket
/// endpoint must be defined.  This provides a file-descriptor that is used for
/// listening for service requests on the socket.  Secondly the endpoint must be
/// bound to a TCP/IP port.
///
/// Both stages should always succeed immediately and if either fails the
/// instance will be left in the disabled state.
///
/// The function also initialises the maps used to convert acknowledgement codes
/// into the strings and the dictionary of parser functions.
///
/// \param [in] p       port number to listen on
/// \param [in] debug   verbosity level
/// \throws SocketError on failure to create/bind socket
//
cmdInt_t::cmdInt_t( unsigned int p, unsigned int debug ) :
    listen_fd(-1),
    io_fd(-1),
    inpLine(),
    port(p),
    debugStr("cmdInt_t.... ", debug )
{
    //
    // Setup the listening socket.
    //
    struct sockaddr_in servaddr;

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family      = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port        = htons( port );

    listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0)
    {
        //
        // Some errors indicate conditions that should never arise and there is
        // something wrong with the code, and others indicate resource
        // shortages.  In either case there is nothing to be done about it.
        //
        throw SocketError( std::string("cmdInt_t : socket error : ") + strerror(errno) );
    }
    else
    {
        int optval = 1;
        setsockopt (listen_fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
        if (bind(listen_fd, (sockaddr *)&servaddr, sizeof(servaddr)) < 0)
        {
            //
            // Some errors indicate conditions that should never arise and there is
            // something wrong with the code, and others indicate resource
            // shortages.  In either case there is nothing to be done about it.
            //
            throw SocketError( std::string("cmdInt_t : socket bind error : ") + strerror(errno) );
        }
    }

    /// \cond
    // setup map of command strings to parser functions as defined in interface.def
    #define COMMAND( id, text, fn ) cmd_map.insert( cmd_map_t::value_type( string(text), &cmdInt_t::parse_##fn ) );
    // setup map of command aliases to parser functions as defined in interface.def
    #define ALIAS( id, text, fn )   cmd_map.insert( cmd_map_t::value_type( string(text), &cmdInt_t::parse_##fn ) );
    #include "interface.def"

    // Setup the acknowledgement code table as defined in interface.def
    ack_table.resize( ACK_TABLE_SIZE );
    #define ACK( id, text ) ack_table[id] = text;
    #include "interface.def"
    /// \endcond
}

// /////////////////////////////////////////////////////////////////////////////
/// The destructor closes the socket.
//
cmdInt_t::~cmdInt_t(void)
{
    if (listen_fd >= 0)
    {
        close(listen_fd);
    }
    if (io_fd >= 0)
    {
        close(io_fd);
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Listen for socket connection.
//
/// Establishes a link to the Client Application by waiting for its service
/// request.  This function blocks until a controller has requested access or an
/// error or signal occurs.
///
/// There are two stages to this operation.  In the first the function listens
/// for service requests using the file descriptor set up when the instance was
/// constructed.  When a request is received a second file descriptor is created
/// by accepting the service request and it is this file descriptor that will be
/// used for future communications with the new client.
///
/// \throws SocketError and derived on failure to connect
//
void cmdInt_t::connToController( void )
{
#ifdef IQ_RESULTS
    debugStr(DebugStream::warn1) << "Using IQ result format\n";
#endif

    if( !isEnabled() )
    {
        throw SocketError( "cmdInt_t : Attempt to connect to controller through a disabled interface" );
    }
    if( isConnected() )
    {
        throw SocketError( "cmdInt_t : Attempt to connect to a 2nd controller" );
    }

    debugStr(DebugStream::info4) << "waiting for connection...\n";
    if( listen(listen_fd, MAX_NUM_CONTROLLERS) < 0 )
    {
        // All errors indicate conditions that should never arise so
        // there is something wrong with the code.
        disable();
        throw SocketError( std::string("connToController : ") + strerror(errno) );
    }
    else
    {
        struct sockaddr_in cliaddr;
        socklen_t          clilen = sizeof(cliaddr);

        io_fd = accept(listen_fd,
                       (struct sockaddr *)&cliaddr,
                       &clilen);
        if (io_fd < 0)
        {
            switch (errno)
            {
                //
                // EINTR
                //      Let the invoking function decide whether or not
                //      to re-attempt the link-up.
                //
                case EINTR:
                    throw SocketErrorSignal( "connToController" );

                //
                // ECONNABORTED
                //      Let the invoking function decide whether or not
                //      to re-attempt the link-up.
                //
                case ECONNABORTED:
                    throw SocketErrorDisconnect( "connToController" );

                //
                // EAGAIN, EWOULDBLOCK, EINVAL
                //      Give the "listen" above this should never
                //      occur.
                //
                // EBADF, ENOTSOCK, EOPNOTSUPP, EFAULT, EPROTO
                //      These should never arise so there is something
                //      wrong with the code.
                //
                // EMFILE, ENFILE, ENOBUFS, ENOMEM
                //      Its unlikely that anything can be done about
                //      this.
                //
                // EPERM
                //      Some sort of system setup problem.
                //
                default :
                    disable();
                    throw SocketError( std::string("connToController: ") + strerror(errno) );
            }
        }
        else
        {
            debugStr(DebugStream::info4) << "cmdInt_t : Controller connected" << endl;
        }
    }
}


// /////////////////////////////////////////////////////////////////////////////
/// Gets a command from the controller.
//
/// If there is no pending command the
/// function returns immediately.
/// Otherwise it passes controll to the appropriate function to parse the
/// command, checking that the correct number and types of parameters are present.
/// If a formatting error is detected then these functions send an acknowledgement
/// indicating an internal error and this function will signal that no command has been
/// received.  If the format of the parameters is correct the function fills in
/// the command code and the parameters.
///
/// Not all socket errors are fatal (for example the Client Application may
/// have closed the connection).  This is communicated via the types
/// derived from socketError.
///
/// \param [out] cmd is the received command structure to be filled in
/// \throws SocketError propogated from getCommand( string ).
//
void cmdInt_t::getCommand( cmd_t &cmd )
{
    cmd.code = CMD_NONE;
    string cmdLine;
    getCommand( cmdLine );  // returns empty if no complete line
    if (!cmdLine.empty())
    {
        // There is an incoming command so check the command code and if it
        // is recognised call the appropriate function to parse the
        // parameters.  If a formatting error is detected then the parser
        // function sends an acknowledgement indicating the error.
        //
        // Commands are trimmed of leading and trailing whiespace
        // the first charactor is checked in the dictionary for old
        // short codes if not it is split on ' ' or ','.
        // The first element is the command and the remaining the parameters.
        vector< string > split_line;
        cmd_map_t::const_iterator it;

        boost::trim( cmdLine );
        if( cmdLine.empty() ) return;

        string c( 1, cmdLine[0] );
        it = cmd_map.find( c );  // check the first letter for short commands
        if( it != cmd_map.end() )
        {
            // found short command
            string params = cmdLine.substr(1);
            boost::trim_if( params, boost::is_any_of(" ,\t") );
            boost::split( split_line, params, boost::is_any_of(" ,\t"), boost::token_compress_on );
            split_line.insert( split_line.begin(), c );
            (this->*(it->second))( cmd, split_line );       // call command param parser
        }
        else
        {
            // look for long command
            boost::split( split_line, cmdLine, boost::is_any_of(" ,\t"), boost::token_compress_on );
            it = cmd_map.find( split_line[0] );
            if( it != cmd_map.end() )
            {
                // found long command
                (this->*(it->second))( cmd, split_line );   // call command param parser
            }
            else
            {
                // command not recognised
                sendMessage("?");
            }
        }
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Low level string fetch part
//
/// Gets a command from the controller.  If no command has been sent, or it is
/// incomplete, the function returns an empty command line.
///
/// The function reads the socket one character at a time because this
/// simplifies the code structure.  By checking each character in turn the end
/// of a command can be spotted so that if there is another one pending it can
/// be left in the socket while the first is processed.  The processing cost of
/// doing it this way is negligible given the rate at which commands are
/// expected.
///
/// All characters except for \\r, which is ignored, are accepted and written
/// into an internal buffer.  Thus the command may be built up over multiple
/// invocations until a \\n signals the end of a command.  The contents of the
/// internal buffer are then moved into the cmdLine string and the function
/// returns.  While the characters are being read there are a number of error
/// conditions that might arise:
///
///  - The controller has shutdown the connection : the instance is marked as
///    disconnected.
///  - A signal is received : the function raises an exception but leaves
///    the instance enabled and connected.
///  - All other errors : the instance is disabled.
///
/// \param [out]    cmdLine is the string that receives the command line.
///                 It will be empty if there is no pending command.
/// \throws         SocketError and derived if a socket error occurs
//
void cmdInt_t::getCommand( std::string &cmdLine )
{
    cmdLine.clear();

    if( !isEnabled() )
    {
        throw SocketError( "getCommand - attempt to get from disabled interface" );
    }

    if( !isConnected() )
    {
        throw SocketErrorDisconnect( "getCommand - attempt to get from unconnected controller" );
    }

    //
    // Read any incoming command, character by character, until a new-
    // line character is received (carriage-return characters are
    // discarded), no more characters are available or an error occurs.
    //
    while (true) // blocks until complete string or no read or crisis
    {
        char c;
        int  n = recv(io_fd, &c, 1, MSG_DONTWAIT);
        if (n < 0)
        {
            switch (errno)
            {
                //
                // EINTR
                //      Let the invoking function decide what to do.
                //      Any incomplete command will be completed during
                //      a later invocation.
                //
                case EINTR:
                {
                    throw SocketErrorSignal( "getCommand" );
                    break;
                }

                //
                // EAGAIN
                //      There is no pending character so do nothing.
                //      Any incomplete command will be completed during
                //      a later invocation.
                //
                case EAGAIN:
                {
                    break;
                }

                //
                // ECONNRESET
                //      Connection reset by peer
                //
                case ECONNRESET:
                {
                    disconnect();
                    throw SocketErrorDisconnect( "getCommand - connection reset by peer" );
                    break;
                }

                //
                // EBADF, ECONNREFUSED, ENOTCONN, ENOTSOCK, EFAULT, EINVAL
                //      These should never arise so there is something
                //      wrong with the code.
                //
                // ENOMEM
                //      Its unlikely that anything can be done about
                //      this.
                //
                default :
                {
                    string str("getCommand - error ");
                    str += boost::lexical_cast<string>(errno);
                    str += strerror(errno);
                    disable();
                    throw SocketError( str );
                    break;
                }
            }
            break; // break out of while(true)
        }
        else
        if (n == 0)
        {
            //
            // The controller has shut down its end of the connection.
            //
            disconnect();
            throw SocketErrorDisconnect( "getCommand - controller shut down connection" );
            break; // break out of while(true)
        }
        else
        if (c == '\n')
        {
            cmdLine = inpLine;
            if( inpLine.length() > 80 )
            {
                debugStr(DebugStream::info3) << "Command received (len="
                    << inpLine.length() << ") prefix: \""
                    << inpLine.substr(0,25) << "...\"\n";
            }
            else
            {
                debugStr(DebugStream::info3) << "Command received (len="
                    << inpLine.length() << ") \"" << inpLine << "\"\n";
            }
            inpLine.clear();
            break; // break out of while(true)
        }
        else
        if (c != '\r')
        {
            inpLine += c;
        }
    }// while(true)
}

// /////////////////////////////////////////////////////////////////////////////
/// Sends a message to the controller.
//
/// The function blocks until the message has been sent or an error
/// or a signal occurs.
//
/// Note that the send operation may not actually send the whole message if the
/// socket buffers are full.  As this implies that the controller is not reading
/// the messages (the socket buffers on a typical system are much longer than
/// any single message) the condition is treated as an error.  The instance is
/// left enabled and connected.
//
/// The function first appends a \\n character to the message and then attempts
/// to send it.  There are a number of error conditions that might arise:
//
///  - The controller has shutdown the connection : the instance is marked as
///    disconnected.
///  - A signal is received : the function raises an exception but leaves the instance
///    enabled and connected so that if the invoking function ignores the signal
///    a subsequent invocation of the function will allow an incoming command to
///    be completed.
///  - All other errors : the instance is disabled.
//
/// \param [in] msg is the string to be sent.
/// \throws SocketError and derived if there is a comms error
//
void cmdInt_t::sendMessage( const std::string &msg )
{
    if( !isEnabled() )
    {
        throw SocketError( "attempt to send message to disabled interface");
    }
    if( !isConnected() )
    {
        throw SocketErrorDisconnect( "attempt to send message to unconnected controller");
    }


    string outLine = msg + "\n"; //"\x0D\x0A";
    size_t n = send(io_fd, outLine.c_str(), outLine.length(), 0);
    if (n < 0)
    {
        switch (errno)
        {
            //
            // EINTR
            //      signal -let the invoking function decide what to do.
            //
            case EINTR:
            {
                throw SocketErrorSignal( "sendMessage" );
                break;
            }

            //
            // ECONNRESET
            //      The controller has shut down its end of the
            //      connection.
            //
            case ECONNRESET:
            {
                disconnect();
                throw SocketErrorDisconnect( "sendMessage controller shut down connection" );
                break;
            }

            //
            // EAGAIN, EWOULDBLOCK, EINVAL
            //      Given the parameters this should never occur.
            //
            // EBADF, EDESTADDRREQ, EFAULT, EISCONN, EMSGSIZE,
            // ENOTCONN, ENOTSOCK, EOPNOTSUPP, EPIPE
            //      These should never arise so there is something
            //      wrong with the code.
            //
            // ENOBUFS, ENOMEM
            //      Its unlikely that anything can be done about
            //      this.
            //
            default :
            {
                string str( "error " );
                str += boost::lexical_cast<string>(errno);
                str += strerror(errno);

                cerr << "cmdInt_t : \"send\" "<< str <<'\n';
                disable();
                throw SocketError( str );
                break;
            }
        }
    }
    else if (n < msg.length())
    {
        // hmm how bad is this, perhaps we could just continue
        throw SocketError( "incomplete message sent "+msg );
    }
    else
    {
        if( msg.length() > 80 )
        {
            debugStr(DebugStream::info3) << "Message sent (len="
                << msg.length() <<") prefix: \"" << msg.substr(0,15) << "\"\n";
        }
        else
        {
            debugStr(DebugStream::info3) << "Message sent (len="
                << msg.length() <<") \"" << msg << "\"\n";
        }
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Posts ack code as text up the comms socket. Overloaded version with
/// additional data to send
//
/// \param [in] ackCode the code
/// \param [in] param additional text to send after the code
/// \throws     SocketError
//
void cmdInt_t::send_ack( ackCode_t ackCode, std::string const & param )
{
    sendMessage( std::string(ack_table[ackCode]) + ',' + param );
}

// /////////////////////////////////////////////////////////////////////////////
/// Posts ack code as text up the comms socket.
//
/// \param [in] ackCode the code
/// \throws     SocketError
//
void cmdInt_t::send_ack( ackCode_t ackCode )
{
    sendMessage( std::string(ack_table[ackCode]) );
}



// /////////////////////////////////////////////////////////////////////////////
//
// command parsers
//
// all have the same signature
//
// parameters:  cmd = cmd structure to fill in
//              params = split string of text that followed command id
//
// /////////////////////////////////////////////////////////////////////////////

// /////////////////////////////////////////////////////////////////////////////
/// Parse quit command
//
/// \param [out] cmd    parsed data placed here
/// \param [in] params  split string containing text command, parameters ignored
/// \memberof cmdInt_t
void cmdInt_t::parse_quit( cmd_t& cmd, std::vector<std::string>const& params )
{
    cmd.code = CMD_QUIT;
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse abort command
//
/// \param [out] cmd    parsed data placed here
/// \param [in] params  split string containing text command, parameters ignored
/// \memberof cmdInt_t
void cmdInt_t::parse_abort( cmd_t& cmd, std::vector<std::string>const& params )
{
    cmd.code = CMD_ABORT;
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse set band command
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - band names UL850, DL850, etc as defined in radio.hpp
/// \memberof cmdInt_t
void cmdInt_t::parse_set_band( cmd_t& cmd, std::vector<std::string> const & params )
{
    try
    {
        Radio::band_t band = Radio::string_to_band( params.at(1) );
        if( band == Radio::INVALID_BAND ) throw invalid_argument( "'" + params.at(1) + "' not a recognised band" );
        cmd.set_band.band = band;
        cmd.code = CMD_SET_BAND;
    }
    catch( exception & e )
    {
        debugStr(DebugStream::warn1) << "parse_set_band error ( " << e.what() << " ) \n";
        send_ack( ACK_SET_BAND_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse_cal_ref_clock
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - Type of cal = GPS / CPICH / PSCH
/// - Frequency in Hz needed for CPICH and PSCH cal
/// - Optional code number needed for CPICH cal
/// - Optional diversity antenna number used fo CPICH cal (defaults to 1)
/// \memberof cmdInt_t
void cmdInt_t::parse_cal_ref_clock( cmd_t &cmd, std::vector<std::string> const & params )
{
    using boost::lexical_cast;
    using boost::to_upper;

    try
    {
        // type, freq, [code], [antenna=1]
        // type
        string str = params.at(1);
        to_upper( str );
        if      ( str == "GPS"   ) cmd.cal_ref.cal_type = paramCalRef_t::GPS_CAL;
        else if ( str == "CPICH" ) cmd.cal_ref.cal_type = paramCalRef_t::CPICH_CAL;
        else if ( str == "PSCH"  ) cmd.cal_ref.cal_type = paramCalRef_t::PSCH_CAL;
        else throw invalid_argument("cal type '" +str+"' should be GPS/CPICH/PSCH");
        // [frequency]
        if( cmd.cal_ref.cal_type != paramCalRef_t::GPS_CAL )
        {
            cmd.cal_ref.freq = lexical_cast<freq_t>(params.at(2));
        }
        // [code]
        if( cmd.cal_ref.cal_type == paramCalRef_t::CPICH_CAL )
        {
            cmd.cal_ref.code = lexical_cast< uint32_t >( params.at(3) );

            //
            //  Check that the code number is for a valid primary code.
            //
            unsigned pri_code = cmd.cal_ref.code/ThreeGPP::PRIMARY_SCRAMBLING_CODE_SEP;
            if(   (pri_code >= ThreeGPP::NUM_PRIMARY_SCRAMBLING_CODES)
               || ((pri_code*ThreeGPP::PRIMARY_SCRAMBLING_CODE_SEP) != cmd.cal_ref.code) )
            {
                throw invalid_argument("invalid cpich code");
            }
        }
        // [antenna = 1]
        cmd.cal_ref.antenna = 1;
        if( params.size() > 4 ){
            cmd.cal_ref.antenna = lexical_cast< uint32_t >( params.at(4) );
        }

        cmd.code = CMD_CAL_REF_CLOCK;
    }
    catch( exception & e )
    {
        debugStr(DebugStream::warn1) << "cal_ref_clock parameter error ( "<<e.what() << " ) \n";
        send_ack( ACK_CAL_REF_CLOCK_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////
/// Parse uplink search and track command
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - code number
/// - Frequency in Hz
/// - number of pilot bits
///
/// or "restart"
/// \memberof cmdInt_t
void cmdInt_t::parse_ul_search( cmd_t &cmd, std::vector<std::string> const & params  )
{
    try
    {
        string s = params.at(1);
        boost::trim( s );
        boost::to_lower( s );
        if( s == "restart" )
        {
            cmd.start_ul_search.restart = true;
        }
        else
        {
            cmd.start_ul_search.restart = false;
            // code, frequency, pilots
            cmd.start_ul_search.code   = boost::lexical_cast<uint32_t>( params.at(1) );
            cmd.start_ul_search.freq   = boost::lexical_cast<freq_t>(   params.at(2) );
            cmd.start_ul_search.pilots = boost::lexical_cast<uint32_t>( params.at(3) );
            if( cmd.start_ul_search.pilots < 3 || cmd.start_ul_search.pilots > 8 )
            {
                throw invalid_argument("number of pilots out of range");
            }
        }

        cmd.code = CMD_UPLINK_ST;
    }
    catch( exception & e )
    {
        debugStr(DebugStream::warn1) << "parse_ul_search parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_UPLINK_ST_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////
/// Parse downlink search and track command
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - code number
/// - Frequency in Hz
/// - number of bits
///
/// or "restart"
/// \memberof cmdInt_t
void cmdInt_t::parse_dl_search( cmd_t &cmd, std::vector<std::string> const & params  )
{
    try
    {
        string s = params.at(1);
        boost::trim( s );
        boost::to_lower( s );
        if( s == "restart" )
        {
            cmd.start_dl_search.restart = true;
        }
        else
        {
            cmd.start_dl_search.restart = false;
            // code, frequency, pilots
            cmd.start_dl_search.code = boost::lexical_cast<uint32_t>( params.at(1) );
            cmd.start_dl_search.freq = boost::lexical_cast<freq_t>(   params.at(2) );
            cmd.start_dl_search.syms = boost::lexical_cast<uint32_t>( params.at(3) );

            //
            //  Check that the code number is for a valid primary code.
            //
            unsigned pri_code = cmd.start_dl_search.code/ThreeGPP::PRIMARY_SCRAMBLING_CODE_SEP;
            if(   (pri_code >= ThreeGPP::NUM_PRIMARY_SCRAMBLING_CODES)
               || ((pri_code*ThreeGPP::PRIMARY_SCRAMBLING_CODE_SEP) != cmd.start_dl_search.code) )
            {
                throw invalid_argument("invalid cpich code");
            }

            //
            //  Check that the number o symbols is valid.
            //
            if( cmd.start_dl_search.syms < 1 || cmd.start_dl_search.syms > 8 )
            {
                throw invalid_argument("number of bits out of range");
            }
        }

        cmd.code = CMD_DOWNLINK_ST;
    }
    catch( exception & e )
    {
        debugStr(DebugStream::warn1) << "parse_dl_search parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_DOWNLINK_ST_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////
/// Parse lte search and track command
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - frequency in Hz
/// - file containing the coefficients for channel filter 1
/// - file containing the coefficients for channel filter 2
/// - file containing the correlator coefficients
///
/// or "restart"
/// \memberof cmdInt_t
void cmdInt_t::parse_lte_search( cmd_t &cmd, std::vector<std::string> const & params  )
{
    try
    {
        string s = params.at(1);
        boost::trim( s );
        boost::to_lower( s );
        if( s == "restart" )
        {
            cmd.start_lte_search.restart = true;
        }
        else
        {
            cmd.start_lte_search.restart = false;
            // code, channel_filter1, channel_filter2, correlator_coefficients
            cmd.start_lte_search.freq = boost::lexical_cast<freq_t>(   params.at(1) );
            strncpy(cmd.start_lte_search.ch_file_1_coeff, 
                    params.at(2).c_str(),
                    MAX_FILENAME_LENGTH - 1);
            cmd.start_lte_search.ch_file_1_coeff[MAX_FILENAME_LENGTH - 1] = '\0'; // allow for over-length string
            strncpy(cmd.start_lte_search.ch_file_2_coeff, 
                    params.at(3).c_str(),
                    MAX_FILENAME_LENGTH - 1);
            cmd.start_lte_search.ch_file_2_coeff[MAX_FILENAME_LENGTH - 1] = '\0'; // allow for over-length string
            strncpy(cmd.start_lte_search.corr_coeff,
                    params.at(4).c_str(),
                    MAX_FILENAME_LENGTH - 1);
            cmd.start_lte_search.corr_coeff[MAX_FILENAME_LENGTH - 1] = '\0'; // allow for over-length string
        }

        cmd.code = CMD_4G_UPLINK_ST;
    }
    catch( exception & e )
    {
        debugStr(DebugStream::warn1) << "parse_dl_search parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_4G_UPLINK_ST_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////
/// Parse lte search and track command
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - file containing the coefficients for channel filter 1
/// - file containing the coefficients for channel filter 2
/// - file containing the correlator coefficients
/// - file containing the test vector values
///
/// \memberof cmdInt_t
void cmdInt_t::parse_regression_4g( cmd_t &cmd, std::vector<std::string> const & params  )
{
    try
    {
        // channel_filter1, channel_filter2, correlator_coefficients, test_vec
        strncpy(cmd.regression_4g.ch_file_1_coeff, 
                params.at(1).c_str(),
                MAX_FILENAME_LENGTH - 1);
        cmd.regression_4g.ch_file_1_coeff[MAX_FILENAME_LENGTH - 1] = '\0'; // allow for over-length string
        strncpy(cmd.regression_4g.ch_file_2_coeff, 
                params.at(2).c_str(),
                MAX_FILENAME_LENGTH - 1);
        cmd.regression_4g.ch_file_2_coeff[MAX_FILENAME_LENGTH - 1] = '\0'; // allow for over-length string
        strncpy(cmd.regression_4g.corr_coeff,
                params.at(3).c_str(),
                MAX_FILENAME_LENGTH - 1);
        cmd.regression_4g.corr_coeff[MAX_FILENAME_LENGTH - 1] = '\0'; // allow for over-length string
        strncpy(cmd.regression_4g.test_vec,
                params.at(4).c_str(),
                MAX_FILENAME_LENGTH - 1);
        cmd.regression_4g.test_vec[MAX_FILENAME_LENGTH - 1] = '\0'; // allow for over-length string

        cmd.code = CMD_4G_REGR4G_ST;
    }
    catch( exception & e )
    {
        debugStr(DebugStream::warn1) << "parse_regression_4g parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_4G_REGR4G_ST_PARAM );
    }
}


// /////////////////////////////////////////////////////////////////////////////
/// Parse get version command
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - none in this case
/// \memberof cmdInt_t
void cmdInt_t::parse_get_version( cmd_t& cmd, std::vector<std::string> const & params )
{
    cmd.code = CMD_GET_VERSION;
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse set mode command
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - none in this case
/// \memberof cmdInt_t
void cmdInt_t::parse_set_mode( cmd_t& cmd, std::vector<std::string> const & params )
{
    try
    {
        string s = params.at(1);
        boost::trim( s );
        boost::to_upper( s );
        if( s == "4G" )
        {
            cmd.set_mode.mode = MODE_4G;
        }
        else if( s == "3G" )
        {
            cmd.set_mode.mode = MODE_3G;
        }
        else if( s == "GSM" )
        {
            cmd.set_mode.mode = MODE_GSM;
        }
        else
        {
            throw invalid_argument("unknown mode");
        }

        cmd.code = CMD_SET_MODE;
    }
    catch( exception & e )
    {
        debugStr(DebugStream::warn1) << "parse_set_mode parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_SET_MODE_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse set standby command
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - none in this case
/// \memberof cmdInt_t
void cmdInt_t::parse_set_standby( cmd_t& cmd, std::vector<std::string> const & params )
{
    try
    {
        string s = params.at(1);
        boost::trim( s );
        boost::to_upper( s );
        if( s == "ACTIVE" )
        {
            cmd.set_standby = ACTIVE;
        }
        else
        if( s == "FPGA_IDLE" )
        {
            cmd.set_standby = FPGA_IDLE;
        }
        else
        if( s == "ADC_IDLE" )
        {
            cmd.set_standby = ADC_IDLE;
        }
        else
        if( s == "FPGA_ADC_IDLE" )
        {
            cmd.set_standby = FPGA_ADC_IDLE;
        }
        else
        if( s == "RADIO_FPGA_ADC_IDLE" )
        {
            cmd.set_standby = RADIO_FPGA_ADC_IDLE;
        }
        else
        {
            throw invalid_argument("unknown standby mode");
        }

        cmd.code = CMD_SET_STANDBY;
    }
    catch( exception & e )
    {
        debugStr(DebugStream::warn1) << "parse_set_standby parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_SET_STANDBY_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse set_pauses
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - minimum report rate in frames
/// \memberof cmdInt_t
void cmdInt_t::parse_set_pauses( cmd_t& cmd, std::vector<std::string>const& params )
{
    using boost::lexical_cast;

    try
    {
        cmd.set_pauses.searching_pause = lexical_cast< uint32_t >( params.at(1) );
        cmd.set_pauses.tracking_pause  = lexical_cast< uint32_t >( params.at(2) );
        cmd.code = CMD_SET_PAUSES;
    }
    catch( exception& e )
    {
        debugStr(DebugStream::warn1) << "parse_set_pauses parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_SET_PAUSES_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse set_low_power
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - minimum report rate in frames
/// \memberof cmdInt_t
void cmdInt_t::parse_set_low_power( cmd_t& cmd, std::vector<std::string>const& params )
{
    using boost::lexical_cast;

    try
    {
        string s = params.at(1);
        boost::trim( s );
        boost::to_upper( s );
        if( s == "LOW_POWER" )
        {
            cmd.set_low_power.low_power = true;
        }
        else
        if( s == "HIGH_SENS" )
        {
            cmd.set_low_power.low_power = false;
        }
        else
        {
            throw invalid_argument("unknown low power settings");
        }

        cmd.code = CMD_SET_LOW_POWER;
    }
    catch( exception& e )
    {
        debugStr(DebugStream::warn1) << "parse_set_low_power parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_SET_LOW_POWER_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////
/// Parse channel power command
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - Frequency in Hz
/// \memberof cmdInt_t
void cmdInt_t::parse_chan_pwr( cmd_t &cmd, std::vector<std::string> const & params  )
{
    try
    {
        cmd.chan_pwr.freq   = boost::lexical_cast<freq_t>(params.at(1));
        cmd.code = CMD_CHAN_PWR;
    }
    catch( exception & e )
    {
        debugStr(DebugStream::warn1) << "parse_chan_pwr parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_CHAN_PWR_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse set_antenna
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - minimum report rate in frames
/// \memberof cmdInt_t
void cmdInt_t::parse_set_antenna( cmd_t& cmd, std::vector<std::string>const& params )
{
    using boost::lexical_cast;

    try
    {
        unsigned antenna = boost::lexical_cast<unsigned>(params.at(1));;
        if (antenna == 1)
        {
            cmd.set_antenna.antenna = paramSetAntenna_t::SET_ANTENNA_1;
        }
        else
        if (antenna == 2)
        {
            cmd.set_antenna.antenna = paramSetAntenna_t::SET_ANTENNA_2;
        }
        else
        {
            throw invalid_argument("unknown antenna setting");
        }

        cmd.code = CMD_SET_ANTENNA;
    }
    catch( exception& e )
    {
        debugStr(DebugStream::warn1) << "parse_set_antenna parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_SET_ANTENNA_PARAM );
    }
}

///////////////////////////////////////////////////////////////////////////////
//
// debug command parsers
//
///////////////////////////////////////////////////////////////////////////////


// /////////////////////////////////////////////////////////////////////////////
/// Parse debug_print
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - verbosity level
/// \memberof cmdInt_t
void cmdInt_t::parse_debug_print( cmd_t& cmd, std::vector<std::string>const& params )
{
    try
    {
        cmd.debug.uinta = boost::lexical_cast<unsigned int>( params.at(1) );
        cmd.code = CMD_DEBUG_PRINT;
    }
    catch( exception & e )
    {
        debugStr(DebugStream::warn1) << "parse_debug_print parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_DEBUG_PRINT_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse debug_vctcxo
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - 12bit unsigned int to send to dac
/// \memberof cmdInt_t
void cmdInt_t::parse_debug_vctcxo( cmd_t& cmd, std::vector<std::string>const& params )
{
    try
    {
        cmd.debug.uinta = boost::lexical_cast<unsigned int>( params.at(1) );
        if( cmd.debug.uinta > Design::MAX_DAC_VAL )
        {
            throw std::range_error("dac val out of range");
        }
        cmd.code = CMD_DEBUG_VCTCXO;
    }
    catch( exception& e )
    {
        debugStr(DebugStream::warn1) << "parse_debug_vctcxo parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_DEBUG_VCTCXO_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse debug_ref_cal_threshold
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - threshold factor (double)
/// \memberof cmdInt_t
void cmdInt_t::parse_debug_ref_cal_threshold( cmd_t& cmd, std::vector<std::string>const& params )
{
    try
    {
        cmd.debug.frac = boost::lexical_cast<double>( params.at(1) );
        cmd.code = CMD_DEBUG_REF_CAL_THRESHOLD;
    }
    catch( exception& e )
    {
        debugStr(DebugStream::warn1) << "parse_debug_ref_cal_threshold parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_DEBUG_REF_CAL_THRESHOLD_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse debug_search_threshold
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - threshold factor (double)
/// \memberof cmdInt_t
void cmdInt_t::parse_debug_search_threshold( cmd_t& cmd, std::vector<std::string>const& params )
{
    try
    {
        cmd.debug.frac = boost::lexical_cast<double>( params.at(1) );
        cmd.code = CMD_DEBUG_SEARCH_THRESHOLD;
    }
    catch( exception& e )
    {
        debugStr(DebugStream::warn1) << "parse_debug_search_threshold parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_DEBUG_SEARCH_THRESHOLD_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse debug_track_threshold
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - threshold factor (double)
/// \memberof cmdInt_t
void cmdInt_t::parse_debug_track_threshold( cmd_t& cmd, std::vector<std::string>const& params )
{
    try
    {
        cmd.debug.frac = boost::lexical_cast<double>( params.at(1) );
        cmd.code = CMD_DEBUG_TRACK_THRESHOLD;
    }
    catch( exception& e )
    {
        debugStr(DebugStream::warn1) << "parse_debug_track_threshold parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_DEBUG_TRACK_THRESHOLD_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse debug_search_threshold_4G
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - threshold factor (double)
/// \memberof cmdInt_t
void cmdInt_t::parse_debug_search_threshold_4G( cmd_t& cmd, std::vector<std::string>const& params )
{
    try
    {
        cmd.debug.frac = boost::lexical_cast<double>( params.at(1) );
        cmd.code = CMD_DEBUG_SEARCH_THRESHOLD_4G;
    }
    catch( exception& e )
    {
        debugStr(DebugStream::warn1) << "parse_debug_search_threshold_4G parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_DEBUG_SEARCH_THRESHOLD_4G_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse debug_track_threshold_4G
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - threshold factor (double)
/// \memberof cmdInt_t
void cmdInt_t::parse_debug_track_threshold_4G( cmd_t& cmd, std::vector<std::string>const& params )
{
    try
    {
        cmd.debug.frac = boost::lexical_cast<double>( params.at(1) );
        cmd.code = CMD_DEBUG_TRACK_THRESHOLD_4G;
    }
    catch( exception& e )
    {
        debugStr(DebugStream::warn1) << "parse_debug_track_threshold_4G parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_DEBUG_TRACK_THRESHOLD_4G_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse debug_switches
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - antenna switch  = 1/2
/// - input switch    = text name
/// - adf input       = text name
/// - adf gpio        = 0-15
/// \memberof cmdInt_t
void cmdInt_t::parse_debug_switches( cmd_t& cmd, std::vector<std::string>const& params )
{
    using boost::trim;
    using boost::to_upper;
    using boost::lexical_cast;

    // <antenna switch> <input switch> <adf input> <adf gpio>
    try
    {
        // <input switch>
        string name = params.at(1);
        trim( name );
        to_upper( name );
        // hardly seems worth adding these few to a map,
        // shame there isn't a way to create compile time const dictionarys
        if      ( name == "GSM_RX_1"  ) cmd.debug.band = Radio::GSM_RX_1;
        else if ( name == "GSM_RX_2"  ) cmd.debug.band = Radio::GSM_RX_2;
        else if ( name == "GSM_RX_3"  ) cmd.debug.band = Radio::GSM_RX_3;
        else if ( name == "GSM_LB_TX" ) cmd.debug.band = Radio::GSM_LB_TX;
        else if ( name == "GSM_HB_TX" ) cmd.debug.band = Radio::GSM_HB_TX;
        else if ( name == "WCDMA_1"   ) cmd.debug.band = Radio::WCDMA_1;
        else if ( name == "WCDMA_2"   ) cmd.debug.band = Radio::WCDMA_2;
        else if ( name == "WCDMA_3"   ) cmd.debug.band = Radio::WCDMA_3;
        else if ( name == "LOAD50"    ) cmd.debug.band = Radio::LOAD50;
        else throw invalid_argument("unrecognised band parameter");

        // <adf input>
        name = params.at(2);
        trim( name );
        to_upper( name );
        if      ( name == "RXLBRF"  ) cmd.debug.ADF_input = Radio::RXLBRF;
        else if ( name == "RXHB1RF" ) cmd.debug.ADF_input = Radio::RXHB1RF;
        else if ( name == "RXHB2RF" ) cmd.debug.ADF_input = Radio::RXHB2RF;
        else throw invalid_argument("unrecongnised ADF4602 input parameter");

        // <adf gpio>
        cmd.debug.uinta = lexical_cast<unsigned int>( params.at(3) );
        if( cmd.debug.uinta > 15 ) throw range_error( "invalid gpio parameter" );

        cmd.code = CMD_DEBUG_SWITCHES;
    }
    catch( exception& e )
    {
        debugStr(DebugStream::warn1) << "parse_debug_switches error ( "<<e.what() << " )\n";
        send_ack( ACK_DEBUG_SWITCHES_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse debug_ADF4602
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - read/write
/// - address
/// - data
/// \bug read not implemented at lower level
/// \memberof cmdInt_t
void cmdInt_t::parse_debug_ADF4602( cmd_t &cmd, std::vector<std::string> const & params )
{
    using boost::lexical_cast;
    using boost::to_upper;

    try{
        string rw = params.at(1);
        to_upper( rw );
        if( rw == "READ" ){
            cmd.debug.boolean = false;
        }
        else if( rw == "WRITE" ){
            cmd.debug.boolean = true;
        }
        else throw invalid_argument("first arg should be read/write not " + params.at(1) );

        // address format is xx or 0.xxx for sub addresses
        double faddr = lexical_cast< double >( params.at(2) );
        if( faddr < 1.0 )
        {
            // sub address
            cmd.debug.uinta = 0;
            cmd.debug.uintb = static_cast<uint32_t>(faddr*1000);
            if( cmd.debug.uintb > 0xff )
            {
                throw range_error("sub address out of range");
            }
        }
        else
        {
            // normal address
            cmd.debug.uinta = static_cast<uint32_t>(faddr);
            if( cmd.debug.uinta > 0x1f )
            {
                throw range_error("address out of range");
            }
        }

        // data
        cmd.debug.uintc = lexical_cast< uint32_t >( params.at(3) );
        if( cmd.debug.uintc > 0xffff )
        {
            throw range_error("data out of range");
        }
        cmd.code = CMD_DEBUG_ADF4602;
    }
    catch( exception& e )
    {
        debugStr(DebugStream::warn1) << "parse_debug_adf4602 parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_DEBUG_ADF4602_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse debug_adc
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - read/write
/// - address
/// - data
/// \bug read not implemented at lower level
/// \memberof cmdInt_t
void cmdInt_t::parse_debug_adc( cmd_t &cmd, std::vector<std::string> const & params )
{
    using boost::lexical_cast;
    using boost::to_upper;

    try{
        string rw = params.at(1);
        to_upper( rw );
        if      ( rw == "READ" ){
            cmd.debug.boolean = false;
        }
        else if ( rw == "WRITE" ){
            cmd.debug.boolean = true;
        }
        else throw invalid_argument("first arg should be read/write not " + params.at(1) );

        // address
        cmd.debug.uinta = lexical_cast< uint32_t >( params.at(2) );
        if( cmd.debug.uinta > 0x3f )
        {
            throw range_error("address out of range");
        }

        // data
        cmd.debug.uintc = lexical_cast< uint32_t >( params.at(3) );
        if( cmd.debug.uintc > 0xff )
        {
            throw range_error("data out of range");
        }
        cmd.code = CMD_DEBUG_ADC;
    }
    catch( exception& e )
    {
        debugStr(DebugStream::warn1) << "parse_debug_adc parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_DEBUG_ADC_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse debug_ranging
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - on/off
/// - optional level 0-127 (dB) to set
/// \memberof cmdInt_t
void cmdInt_t::parse_debug_ranging( cmd_t &cmd, std::vector<std::string> const & params )
{
    using boost::lexical_cast;
    using boost::to_upper;

    try{
        string s = params.at(1);
        to_upper( s );
        if      ( s == "ON" ){
            cmd.debug.uintb = 1;
        }
        else if ( s == "OFF" ){
            cmd.debug.uintb = 0;
        }
        else throw invalid_argument("first arg should be on/off/now not " + params.at(1) );

        // optional value
        if( params.size() == 3 )
        {
            cmd.debug.uinta = lexical_cast< uint32_t >( params.at(2) );
            if( cmd.debug.uinta > 0xff )
            {
                throw range_error("level out of range");
            }
        }
        else
        {
            cmd.debug.uinta = 0x100;
        }
        cmd.code = CMD_DEBUG_RANGING;
    }
    catch( exception& e )
    {
        debugStr(DebugStream::warn1) << "parse_debug_range parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_DEBUG_RANGING_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse debug_peek
//
/// Read fpga's memory.
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - word address start
/// - number of words
/// \memberof cmdInt_t
void cmdInt_t::parse_debug_peek( cmd_t& cmd, std::vector<std::string>const& params )
{
    using boost::lexical_cast;

    try
    {
        // address
        cmd.debug.uinta = lexical_cast<uint32_t>( params.at(1) );
        // number of words
        cmd.debug.uintb = lexical_cast<uint32_t>( params.at(2) );
        cmd.code = CMD_DEBUG_PEEK;
    }
    catch( exception& e )
    {
        debugStr(DebugStream::warn1) << "parse_debug_peek parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_DEBUG_PEEK_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse debug_poke
//
/// Write words to fpga's memory.
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - start address
/// - N words of data to write
/// \memberof cmdInt_t
void cmdInt_t::parse_debug_poke( cmd_t& cmd, std::vector<std::string>const& params )
{
    using boost::lexical_cast;

    try
    {
        // start address
        cmd.debug.uinta = lexical_cast<uint32_t>( params.at(1) );
        // get all the words
        cmd.uint_vector.clear();
        for( size_t i = 2; i < params.size(); i += 1 )
        {
            cmd.uint_vector.push_back( lexical_cast<uint32_t>(params.at(i)) );
        }
        cmd.code = CMD_DEBUG_POKE;
    }
    catch( exception& e )
    {
        debugStr(DebugStream::warn1) << "parse_debug_poke parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_DEBUG_POKE_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse debug_reg
//
/// Read or write words to fpga's registers
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - W/R
/// - register word address
/// - [data word to write]
/// \memberof cmdInt_t
void cmdInt_t::parse_debug_reg( cmd_t& cmd, std::vector<std::string>const& params )
{
    using boost::lexical_cast;
    using boost::trim;
    using boost::to_upper;

    try
    {
        // r/w
        string str = params.at( 1 );
        trim( str );
        to_upper( str );

        if     ( str == "W" ) cmd.debug.boolean = true;
        else if( str == "R" ) cmd.debug.boolean = false;
        else throw invalid_argument( "param 1 should be W/R" );

        // start address
        cmd.debug.uinta = lexical_cast<uint32_t>( params.at(2) );

        // data
        if( cmd.debug.boolean )
        {
            cmd.debug.uintb = lexical_cast<uint32_t>( params.at(3) );
        }

        cmd.code = CMD_DEBUG_REG;
    }
    catch( exception& e )
    {
        debugStr(DebugStream::warn1) << "parse_debug_reg parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_DEBUG_REG_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// parse_debug_gpio
//
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - high/low/get
/// - gpio file number
/// \memberof cmdInt_t
void cmdInt_t::parse_debug_gpio( cmd_t& cmd, std::vector<std::string>const& params )
{
    using boost::lexical_cast;
    using boost::trim;
    using boost::to_upper;
    using namespace std;

    try
    {
        string str = params.at( 1 );
        trim( str );
        to_upper( str );
        if      ( str == "HIGH" ) cmd.debug.uinta = 1;
        else if ( str == "LOW"  ) cmd.debug.uinta = 0;
        else if ( str == "GET"  ) cmd.debug.uinta = 2;
        else throw invalid_argument("should be high/low/get not:'"+str+"'" );

        cmd.debug.uintb = lexical_cast< uint32_t >( params.at(2) );

        cmd.code = CMD_DEBUG_GPIO;
    }
    catch( exception& e )
    {
        debugStr(DebugStream::warn1) << "parse_debug_gpio parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_DEBUG_GPIO_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse debug_gsm
//
/// Read fpga's memory.
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - number of values
/// \memberof cmdInt_t
void cmdInt_t::parse_debug_gsm( cmd_t& cmd, std::vector<std::string>const& params )
{
    using boost::lexical_cast;

    try
    {
        // number of values
        cmd.debug.uinta = lexical_cast<uint32_t>( params.at(1) );
        cmd.code = CMD_DEBUG_GSM;
    }
    catch( exception& e )
    {
        debugStr(DebugStream::warn1) << "parse_debug_gsm parameter error ( "<<e.what() << " )\n";
        send_ack( ACK_DEBUG_GSM_PARAM );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse debug_float
//
/// Confirm compiler float representation matches that of the FPGA
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - number of values
/// \memberof cmdInt_t
void cmdInt_t::parse_debug_float( cmd_t& cmd, std::vector<std::string>const& params )
{
    using boost::lexical_cast;

    cmd.code = CMD_DEBUG_FLOAT;
}

// /////////////////////////////////////////////////////////////////////////////
/// Parse debug_help
//
/// Post back the list of known commands from interface.def and get usage
/// info for a command.
/// Processing done here, handler in phy.cpp never gets called.
/// \param [out] cmd    Parsed data placed here
/// \param [in] params  Split string containing text command and parameters:
/// - optional command to look up usage help for
/// \memberof cmdInt_t
/// \cond
void cmdInt_t::parse_debug_help( cmd_t &cmd, std::vector<std::string> const & params )
{
    using namespace std;
    string str;
    if( params.size() == 1 )
    {
        str = "known commands:\n"
        #define COMMAND( id, text, fn ) text "\n"
        #define ALIAS( id, text, fn ) text "\n"
        #include "interface.def"
        ;
    }
    else
    {
        map<string,string> lookup;
        #define COMMAND( id, text, fn ) ; lookup[text] =
        #define ALIAS( id, text, fn ) lookup[text] =
        #define USAGE( text ) text "\n"
        #include "interface.def"
        ;

        map<string,string>::iterator it = lookup.find( params[1] );
        if( it != lookup.end() )
        {
            str = it->first + "\n";
            str += it->second;
        }
        else
        {
            str = "unknown command (use long name when requesting help)";
        }
    }

    send_ack( ACK_DEBUG_HELP_OK, str );
    cmd.code = CMD_NONE;
}
/// \endcond


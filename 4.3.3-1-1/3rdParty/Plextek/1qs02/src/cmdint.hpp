/******************************************************************************
 *
 *
 ******************************************************************************
 *  Filename:   cmdint.hpp
 *  Author(s):  pdm
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file cmdint.hpp
 * \brief class to process commands coming over the tcp/ip socket
 *
 *****************************************************************************/

#ifndef __CMDINT_H__
#define __CMDINT_H__

#include "3gpp.h"
#include "common.hpp"
#include "debug.hpp"
#include "radio.hpp"
#include "types.hpp"

#include <list>
#include <map>
#include <string>
#include <vector>
#include <complex>

// /////////////////////////////////////////////////////////////////////////////
/// Command interface class.
//
/// This class provides an interface to the Client Application and is the
/// counterpart to the phyInt_t class.
///
/// The physical interface is based on TCP/IP sockets.  These provide a byte-
/// stream interface between applications that is based on the TCP/IP protocol.
/// The sockets require a server/client relationship between the applications.
/// The Physical Layer Program is the server and the Client Application is
/// the client.  A single bi-directional socket is used with new-line terminated
/// strings forming the basis for all communications.
///
/// There are a number of states in which an instance may find itself, depending
/// on the condition of the interface.  The instance is enabled if the interface
/// is usable (tested with the isEnabled function).  Generally the interface
/// becomes unusable if some error occurs while accessing the socket.  If the
/// instance is enabled it may be connected (tested with the isConnected
/// function), which means that messages may be exchanged with the Client
/// Application.  The Client Application may break the connection but as long as
/// the instance remains enabled the connection may be re-established (the
/// connToController function gives the Client Application the opportunity to
/// establish the connection).
///
/// The class is designed to support a system design based around a polling
/// loop.  The getCommand function should be invoked at regular intervals and
/// each time it will check the socket for incoming commands, returning
/// immediately if there is no command i.e. it is non-blocking function.  The
/// sendMessage command on the other hand always blocks until the message has
/// been successfully written to the socket.
///
/// The interface protocol is based on the principle
/// that commands issued by the Client Application are always acknowledged and
/// that these acknowledgements only signal acceptance or rejection of the
/// command.  The response to a command is always sent as an independent
/// indication.  Thus the response to an incoming command requires the following
/// sequence:
///
/// - The getCommand function must be invoked to check if any incoming
///   command is pending.  This is a polling function that may return without
///   any command.  If there is a command then the function parses the
///   parameters and if there is a formatting error it issues a negative
///   acknowledgement indicating an internal error and returns signalling
///   that no command was received.  If there is no error it returns the
///   command and the parameters.
///
/// - The calling function/task/process/class is responsible for checking
///   that the command is acceptable while the system is in its current state
///   and that the parameter values are valid.  It must issue an
///   acknowledgement through the cmdInt_t instance that indicates whether or
///   not the command has been accepted.
///
/// - The calling function/task/process/class must next carry out the
///   operations required by the command.
//
class cmdInt_t
{
    public:
        /// The command codes returned by the getCommand function.
        enum cmdCode_t
        {
            /// \cond
            #define COMMAND( id, text, fn ) id,
            #include "interface.def"
            /// \endcond
            CMD_NONE
        };

        /// The acknowledgement codes.
        enum ackCode_t
        {
            /// \cond
            #define ACK( id, text ) id,
            #include "interface.def"
            /// \endcond
            ACK_TABLE_SIZE
        };

    public:
        /// \name The command parameter structures.
        //@{

        /// For set band
        struct paramSetBand_t
        {
            Radio::band_t band; ///< ul900 etc
        };

        /// Reference calibration
        struct paramCalRef_t
        {
            enum{ GPS_CAL, CPICH_CAL, PSCH_CAL } cal_type;
            ///< 3 types of cal currently possible
            freq_t freq;        ///< for cpich and psch
            uint32_t code;      ///< for cpich cal
            uint32_t antenna;   ///< for cpich cal
        };

        /// Uplink search and track
        struct paramStartUlSearch_t
        {
            uint32_t    code;   ///< ul scrambling code
            freq_t      freq;   ///< frequency to work at
            uint32_t    pilots; ///< number of pilot bits
            bool        restart;///< restart an in progress search and track
        };

        /// Downlink search and track
        struct paramStartDlSearch_t
        {
            uint32_t    code;   ///< ul scrambling code
            freq_t      freq;   ///< frequency to work at
            uint32_t    syms;   ///< number of symbols
            bool        restart;///< restart an in progress search and track
        };

        static const uint8_t MAX_FILENAME_LENGTH = 255u;

        /// lte search and track
        struct paramStartLteSearch_t
        {
            freq_t      freq;           ///< frequency to work at
            char        ch_file_1_coeff[MAX_FILENAME_LENGTH];///< file containing the coefficients for channel filter 1
            char        ch_file_2_coeff[MAX_FILENAME_LENGTH];///< file containing the coefficients for channel filter 2
            char        corr_coeff[MAX_FILENAME_LENGTH];     ///< file containing the correlator coefficients
            bool        restart;        ///< restart an in progress search and track
        };

        /// regression 4g test
        struct paramRegression4g_t
        {
            char        ch_file_1_coeff[MAX_FILENAME_LENGTH];///< file containing the coefficients for channel filter 1
            char        ch_file_2_coeff[MAX_FILENAME_LENGTH];///< file containing the coefficients for channel filter 2
            char        corr_coeff[MAX_FILENAME_LENGTH];     ///< file containing the correlator coefficients
            char        test_vec[MAX_FILENAME_LENGTH];       ///< file containing the test vector
        };

        /// For set mode
        struct paramSetMode_t
        {
            eSystemMode    mode;
        };

        /// For set standby
        enum paramSetStandby_t
        {
            ACTIVE,
            ADC_IDLE,
            FPGA_IDLE,
            FPGA_ADC_IDLE,
            RADIO_FPGA_ADC_IDLE     //  Radio supplies clocks so all must be idle
        };

        /// For set mode
        struct paramSetPauses_t
        {
            unsigned searching_pause;
            unsigned tracking_pause;
        };

        /// For set low power
        struct paramSetLowPower_t
        {
            bool low_power;
        };

        /// Channel power
        struct paramChanPwr_t
        {
            freq_t      freq;   ///< frequency to work at
        };

        /// Set Antenna
        struct paramSetAntenna_t
        {
            enum
            {
                SET_ANTENNA_1,
                SET_ANTENNA_2
            } antenna;
        };

        /// All debug commands use this
        struct paramDebug_t
        {
            /// \name general purpose values
            //@{
            double                  frac;
            bool                    boolean;
            uint32_t                uinta;
            uint32_t                uintb;
            uint32_t                uintc;
            //@}
            Radio::RF1480_band_t    band;       ///< multi way switch setting
            Radio::ADF4602_input_t  ADF_input;  ///< which rf input of chip
        };
        //@}

        /// All possible command parameters grouped
        struct cmd_t
        {
            /// id of command
            cmdCode_t   code;
            /// pod types can be unioned
            union
            {
                paramDebug_t            debug;
                paramSetBand_t          set_band;
                paramCalRef_t           cal_ref;
                paramStartUlSearch_t    start_ul_search;
                paramStartDlSearch_t    start_dl_search;
                paramStartLteSearch_t   start_lte_search;
                paramRegression4g_t     regression_4g;
                paramSetMode_t          set_mode;
                paramSetStandby_t       set_standby;
                paramSetPauses_t        set_pauses;
                paramSetLowPower_t      set_low_power;
                paramChanPwr_t          chan_pwr;
                paramSetAntenna_t       set_antenna;
            };
            /// for debug_poke, can't be in union
            std::vector<uint32_t> uint_vector;
        };

        /// The type for mapping acknowledgement codes onto message strings.
        typedef std::vector< char const * > ack_table_t;

        /// Type of parser function
        typedef void ( cmdInt_t::*parser_fn_t)( cmd_t &, std::vector<std::string> const & );
        /// Type for mapping command codes to parser functions
        typedef std::map< std::string, parser_fn_t > cmd_map_t;

    private:
        int             listen_fd;  ///< listening socket descriptor
        int             io_fd;      ///< connected socket descriptor
        std::string     inpLine;    ///< incomming command string
        cmd_map_t       cmd_map;    ///< lookup command string to parser function
        ack_table_t     ack_table;  ///< lookup ack code to string
        unsigned int    port;       ///< listening port number
        DebugStream     debugStr;   ///< debug message stream

    public:
        cmdInt_t( unsigned int port, unsigned int debug = DebugStream::off );

        // /////////////////////////////////////////////////////////////////////
        /// prevent complier gernerated copy constructor
    private:
        cmdInt_t( const cmdInt_t &src );

    public:
        virtual ~cmdInt_t( void );

        // /////////////////////////////////////////////////////////////////////
        /// prevent complier gernerate operator=
    private:
        cmdInt_t &operator=(const cmdInt_t &rhs);

        // /////////////////////////////////////////////////////////////////////
        /// Indicates if the initialisation was successful and no subsequent
        /// errors have been serious enough to invalidate it.
        //
        /// \return true = the interface can be used.
        //
    public:
        bool isEnabled(void) const
        {
            return (listen_fd >= 0);
        }

        // /////////////////////////////////////////////////////////////////////
        /// Indicates if the command interface is connected to a controller.
        //
        /// \return true = the instance is connected to a controller.
    public:
        bool isConnected(void) const
        {
            return (io_fd >= 0);
        }

        // /////////////////////////////////////////////////////////////////////
        /// Permanently disables the interface.
    private:
        void disable(void)
        {
            disconnect();
            if (isEnabled())
            {
                close(listen_fd);
                listen_fd = -1;
            }
        }

        // /////////////////////////////////////////////////////////////////////
        /// Disconnects the interface - a future re-connection may be possible.
    private:
        void disconnect(void)
        {
            if (isConnected())
            {
                close(io_fd);
                io_fd = -1;
            }
        }

    public:
        void connToController(void);
        void getCommand( cmd_t &cmd );

    private:
        void getCommand(std::string &cmdLine);

        // /////////////////////////////////////////////////////////////////////
        // parsing functions
        //
        // all have signature void fn( cmd_t&, std::vector<std::string> const& )
        //
        // param  cmd_t&          - to be filled in with parsed params
        //        vector<string>  - list of parameters split on ',' or ' '
        //                          element 0 is command itself
    private:
        /// \cond
        #define COMMAND( id, text, fn ) void parse_##fn( cmd_t&, std::vector<std::string> const & );
        #include "interface.def"
        /// \endcond

    public:
        void send_ack( ackCode_t );
        void send_ack( ackCode_t, std::string const & );
        void sendMessage(const std::string &msg);

        // /////////////////////////////////////////////////////////////////////
        /// Set debug printing verbosity
        //
        /// \param [in] l level
    public:
        void debug_printing( unsigned l )
        {
            debugStr.show_level( l );
        }
};

#endif // __CMDINT_H__

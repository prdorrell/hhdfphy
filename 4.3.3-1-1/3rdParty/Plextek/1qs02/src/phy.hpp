/***********************************************************************************************************************
 *
 *
 ***********************************************************************************************************************
 *  Filename:   phy.hpp
 *  Author(s):  pdm, pmd
 *******************************************************************************************************************//**
 * File Description (User Field)
 * ================
 *
 *  @file
 *  @brief          Main class that checks for incoming commands and implements handlers
 *
 **********************************************************************************************************************/

#ifndef __PHY_H__
#define __PHY_H__

#include "cmdint.hpp"
#include "common.hpp"
#include "debug.hpp"
#include "configoptions.hpp"
#include "pwrcal.hpp"
#include "searchandtrack.hpp"
#include "searchandtrack_4g.hpp"
#include "timeconttrack.hpp"
#include "ranging.hpp"
#include "refcal.hpp"

#include <memory>


// /////////////////////////////////////////////////////////////////////////////
/// The Physical Layer class.
//
/// This class implements the main functionality of the physical layer software.
/// Its activate function contains the processing loop that checks for incoming
/// commands from a client and controls the requested operations.  It only
/// returns if the client issues a terminate command or if a error is detected
/// on the socket.
//
class phy_t
{
    private:
        /// The phy processing states.
#ifdef SWTEST_4G_FPGA        
        enum phyState_t { STANDBY, IDLE, UL_SEARCHANDTRACK, DL_SEARCHANDTRACK,
            LTE_SEARCHANDTRACK, CAL_REF_CLOCK, CHAN_PWR, REGRESSION_4G };
#else
        enum phyState_t { STANDBY, IDLE, UL_SEARCHANDTRACK, DL_SEARCHANDTRACK,
            LTE_SEARCHANDTRACK, CAL_REF_CLOCK, CHAN_PWR };
#endif    

        /// The generic command handler function type.
        typedef void (phy_t::*cmdHandlerFunc)( cmdInt_t::cmd_t const & cmd );

        /// The type of the dictionary associating cmd codes with function
        /// handler procedures
        typedef std::map<cmdInt_t::cmdCode_t, cmdHandlerFunc> cmdHandlerTab_t;

    private:
        Fpga &fpga;

        /// A specific type of ref cal object will be build and stored here
        /// to handle reference calibration
        std::auto_ptr<RefCal>           ref_cal;
        /// This is filled with specific object to handle different
        /// 3G cal/track functionality
        std::auto_ptr<SearchAndTrack>       search_and_track;
        /// This is filled with specific object to handle 4G cal/track functionality
        std::auto_ptr<SearchAndTrack_4g>    search_and_track_4g;
        /// This is filled with the object to handle the GSM mode time-continuous tracking
        std::auto_ptr<TimeContTrack>        time_cont_track;

        cmdInt_t            cmdInt;                 ///< The command interface receives commands from the client application.
        cmdHandlerTab_t     cmdHandlerTab;          ///< The table of functions for handling commands.
        Ranging             rangeCtrl;              ///< The ranging controller.
        PwrCal              pwrCal;                 ///< The power calibration controller.


        /// \name current settings
        //@{
        cmdInt_t::paramSetStandby_t
                            cur_standby_state;              ///< current standby state
        std::string         image_name_for_4G;              ///< 4G fpga image file name .bin format
        std::string         image_name_for_3G;              ///< 3G fpga image file name .bin format
        std::string         image_name_for_GSM;             ///< GSM fpga image file name .bin format
        eSystemMode         cur_mode;                       ///< mode selected
        bool                low_power;                      ///< signals that the low-power settings for the averaging should be used.
        Radio::band_t       cur_band;                       ///< band selected
        freq_t              cur_freq;                       ///< frequency tuned to
        phyState_t          state;                          ///< tracking / idle etc
        double              ref_cal_threshold_factor_dB;    ///< added to the built-in threshold for the reference calibration
        double              search_threshold_factor_dB;     ///< added to the built-in threshold for the search mode
        double              track_threshold_factor_dB;      ///< added to the built-in threshold for the track mode
        double              search_threshold_factor_4G_dB;  ///< added to the built-in threshold for the 4G search mode
        double              track_threshold_factor_4G_dB;   ///< added to the built-in threshold for the 4G track mode
        unsigned int        search_average_4G;              ///< Number of raw correlation results to average in the 4G search mode (0 => use default)
        unsigned int        track_average_4G;               ///< Number of raw correlation results to average in the 4G track mode (0 => use default)
        unsigned            antenna_mode;                   ///< 0 = norm, 1 = force 1 always, 2 = force 2 always
        unsigned int        searching_pause;                ///< number of idle frames between searching iterations
        unsigned int        tracking_pause;                 ///< number of idle frames between tracking iterations
        bool                active;                         ///< true = phy loop running, false = not yet running or due to quit
        //@}

        bool                debug_files;            ///< dump coefs etc to files
        DebugStream         debugStr;               ///< debug tracing output

    public:
        phy_t( ConfigOptions const &, Fpga &fpga );
        virtual ~phy_t(void);
        void activate( bool const & quit_flag );

        // /////////////////////////////////////////////////////////////////////
        /// Generate all the comand handler prototypes
        //
        /// They all have signature void fn( cmdInt_t::cmd_t& ).
        /// They take the command info as a param and do the deed.
        //
        #define COMMAND( id, text, fn ) void handle_##fn( cmdInt_t::cmd_t const & );
        #include "interface.def"

    private:
        void force_full_standby_state( void );
        void restore_standby_state( void );
        void abort_processing( void );
        void proc_search_and_track( void );
        void proc_search_and_track_4g( void );
        void proc_cal_ref( void );
        void proc_chan_pwr( void );
        void proc_regression_4g( void );

        // /////////////////////////////////////////////////////////////////////
        /// debug_printing
        //
        /// \param [in] l show level
        //
    public:
        void debug_printing( unsigned l )
        {
            debugStr.show_level( l );
        }

    private:
        phy_t &operator=(const phy_t &rhs); ///< unused, supress compiler generated
        phy_t(const phy_t &src);            ///< unused, supress compiler generated
        phy_t( void );                      ///< unused, supress compiler generated

};

#endif // __PHY_H__

/******************************************************************************
 *
 *
 ******************************************************************************
 *  Filename:   phy.cpp
 *  Author(s):  pdm, pmd
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file phy.cpp
 * \brief main class that checks for incoming commands and implements handlers
 *
 *****************************************************************************/

#include "phy.hpp"
#include "radio.hpp"
#include "adc.hpp"
#include "dac.hpp"
#include "fpga.hpp"
#include "socketerror.hpp"
#include "design.hpp"
#include "gpio.hpp"
#include "pcb.hpp"
#include "version.hpp"

#include <cassert>

#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>

using namespace std;

// /////////////////////////////////////////////////////////////////////////////
/// Build command handler table from x-macro definitions
//
/// Various command line options copied to change settings
phy_t::phy_t( ConfigOptions const & co, Fpga &fpga )
    :
    fpga(fpga),
    cmdInt( co.port, co.debug_print ),
    cmdHandlerTab(),
    rangeCtrl(co.ranging_mode, co.debug_print),
    pwrCal(co.pwr_cal_conf_filename, co.debug_print),
    cur_standby_state(cmdInt_t::ACTIVE),
    image_name_for_4G(co.image_name_for_4G),
    image_name_for_3G(co.image_name_for_3G),
    image_name_for_GSM(co.image_name_for_GSM),
    cur_mode(co.init_mode),
    low_power(co.low_power),
    cur_band( Radio::LOAD_BAND ),
    state( IDLE ),
    ref_cal_threshold_factor_dB( co.ref_cal_threshold_factor_dB ),
    search_threshold_factor_dB( co.search_threshold_factor_dB ),
    track_threshold_factor_dB( co.track_threshold_factor_dB ),
    search_threshold_factor_4G_dB( co.search_threshold_factor_4G_dB ),
    track_threshold_factor_4G_dB( co.track_threshold_factor_4G_dB ),
    search_average_4G( co.search_average_4G ),
    track_average_4G( co.track_average_4G ),
    antenna_mode( co.antenna_mode ),
    searching_pause( co.searching_pause ),
    tracking_pause( co.tracking_pause ),
    debug_files( co.debug_files ),
    debugStr("phy_t....... ", co.debug_print)
{
    //
    // Set up command handler table
    /// \cond
    #define COMMAND( id, text, fn ) cmdHandlerTab[cmdInt_t::id] = &phy_t::handle_##fn;
    #include "interface.def"
    /// \endcond
}

// /////////////////////////////////////////////////////////////////////////////
/// nothing is explicitly destroyed
phy_t::~phy_t(void)
{
}

// /////////////////////////////////////////////////////////////////////////////
/// Activates the instance.
//
/// This function implements a loop that only
/// terminates if an error condition occurs or a termination command is
/// received.  While it runs the loop accepts commands from the command
/// interface and acts on them.
///
/// The loop polls the command-interface to see
/// if there is any pending command.  If there is a command
/// pending then the cmdHandlerTab is used to identify the appropriate
/// function to handle the command.
/// State dependent handlers are also called as appropriate each time
/// around the loop.
//
/// \exception SocketError exceptions coming from the tcp/ip
/// interface may thrown if they can not be usefuly handled
void phy_t::activate( bool const& quit_flag )
{
    //
    //  Nothing to do, so put hardware elements into low power mode.
    //
    force_full_standby_state();

    // Find a controller.
    // blocking call till connect
    // any exceptions are passed up
    cmdInt.connToController();

    //
    //  Connected so restore the client-defined low power mode of the hardware elements.
    //
    restore_standby_state();

    // main loop that
    // * polls command interface
    // * calls (state dependent) data processes (cooperative MT)
    // * tries to wait for reconnection if connection is dropped
    // * quit on quit command or unhandled error
    active = true;
    do
    {
        try
        {
            // Check for any commands.
            cmdInt_t::cmd_t cmd;
            cmdInt.getCommand( cmd );

            // check quit flag (ctrl-c handler will set this)
            if( quit_flag )
            {
                abort_processing();
                active = false;
                continue;
            }

            // Dispatch cmds to appropriate handler
            cmdHandlerTab_t::iterator cmdHandler = cmdHandlerTab.find(cmd.code);

            if( cmdHandler != cmdHandlerTab.end() )
            {
                (this->*(cmdHandler->second))( cmd );
            }
            // Ignore unknown commands.

            // Now perform any required processing.
            // (Poll data processes, co-operative multitasking)
            //debugStr(DebugStream::info1) << boost::format("phy_t::activate(%u)\n") % state;
            switch( state )
            {
                case UL_SEARCHANDTRACK:
                case DL_SEARCHANDTRACK:
                    proc_search_and_track();
                    break;
                case LTE_SEARCHANDTRACK:
                    proc_search_and_track_4g();
                    break;
                case CAL_REF_CLOCK:
                    proc_cal_ref();
                    break;
                case CHAN_PWR:
                    proc_chan_pwr();
                    break;
#ifdef SWTEST_4G_FPGA        
                case REGRESSION_4G:
                    proc_regression_4g();
                    break;
#endif
                case IDLE:
                    break;
                case STANDBY:
                    break;
            }
            // Allow other applications/processes a chance to run. The LTE correlation needs to keep up with the FPGA
            // that produces a large number of raw results. 
            //debugStr(DebugStream::warn2) << "Main Sleeping (10 ms)\n";
            if (LTE_SEARCHANDTRACK == state)
            {
                usleep(4000);
            }
            else
            {
                usleep(10000);
            }
        }
        catch( SocketErrorSignal& e )
        {
            // signal aborted socket operation, just continue and hope all will be well
            debugStr(DebugStream::warn2) << e.what() << '\n';
        }
        catch( SocketErrorDisconnect& e )
        {
            // we've been disconnected
            // stop processing
            // try to connect again
            debugStr(DebugStream::warn2) << e.what() << '\n';
            abort_processing();

            //
            //  Nothing to do, so put hardware elements into low power mode.
            //
            force_full_standby_state();

            cmdInt.connToController(); // give up if we get an exception from this as well

            //
            //  Connected so restore the client-defined low power mode of the hardware elements.
            //
            restore_standby_state();
        }// other errors more serious, will get passed higher up
    }while( active );
}

// /////////////////////////////////////////////////////////////////////////////
/// Set all the the hardware elements into their standby state.
//
/// \memberof phy_t
//
void phy_t::force_full_standby_state( void )
{
    //
    //  Put the elements into standby, Radio last because it provides the clocks.
    //
    fpga.enter_low_power_mode();
    ADC::enter_low_power_mode();
    Radio::enter_low_power_mode();
}

// /////////////////////////////////////////////////////////////////////////////
/// Restore the standby state of the hardware elements to the client-defined
/// setting.
//
/// \memberof phy_t
//
void phy_t::restore_standby_state( void )
{
    //
    // Bring the appropriate elements out of standby, Radio first because it provides the clocks.
    //
    if (cur_standby_state == cmdInt_t::ACTIVE)
    {
        Radio::exit_low_power_mode((cur_mode == MODE_3G) || (cur_mode == MODE_4G));   // 4G uses the 3G configuration.
        ADC::exit_low_power_mode((cur_mode == MODE_3G) || (cur_mode == MODE_4G));     // 4G uses the 3G sample rate

        //
        //  Allow the clocks to stabilise.
        //
        usleep(10000);
        fpga.exit_low_power_mode();
    }
    else
    if (cur_standby_state == cmdInt_t::ADC_IDLE)
    {
        Radio::exit_low_power_mode((cur_mode == MODE_3G) || (cur_mode == MODE_4G));   // 4G uses the 3G configuration.

        //
        //  Allow the clocks to stabilise.
        //
        usleep(10000);
        fpga.exit_low_power_mode();
    }
    else
    if (cur_standby_state == cmdInt_t::FPGA_IDLE)
    {
        Radio::exit_low_power_mode((cur_mode == MODE_3G) || (cur_mode == MODE_4G));   // 4G uses the 3G configuration.
        ADC::exit_low_power_mode((cur_mode == MODE_3G) || (cur_mode == MODE_4G));     // 4G uses the 3G sample rate
    }
    else
    if (cur_standby_state == cmdInt_t::FPGA_ADC_IDLE)
    {
        Radio::exit_low_power_mode((cur_mode == MODE_3G) || (cur_mode == MODE_4G));   // 4G uses the 3G configuration.
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Handler: Perform clean exit of program
//
/// \memberof phy_t
//
void phy_t::handle_quit( cmdInt_t::cmd_t const & cmd )
{
    active = false;
}

// /////////////////////////////////////////////////////////////////////////////
/// Handler: Abort current processing task
//
/// \memberof phy_t
//
void phy_t::handle_abort( cmdInt_t::cmd_t const & cmd )
{
    abort_processing();
    cmdInt.send_ack( cmdInt_t::ACK_ABORT_OK );
}

// /////////////////////////////////////////////////////////////////////////////
/// Abort current processing task
//
/// May have originated internally or via handle_abort command
//
void phy_t::abort_processing( void )
{
    debugStr(DebugStream::info4) << "aborting...\n";
    switch( state )
    {
        case STANDBY:
            break;
        case IDLE:
            break;
        case UL_SEARCHANDTRACK:
        case DL_SEARCHANDTRACK:
            search_and_track.reset(); // destroy auto_ptr contained object
            state = IDLE;
            break;
        case LTE_SEARCHANDTRACK:
            search_and_track_4g.reset(); // destroy auto_ptr contained object
            state = IDLE;
            break;
        case CAL_REF_CLOCK:
            ref_cal.reset();
            state = IDLE;
            break;
        case CHAN_PWR:
            time_cont_track.reset();
            state = IDLE;
            break;
#ifdef SWTEST_4G_FPGA        
        case REGRESSION_4G:
            fpga.stop_correlation();
            fpga.regression_4g_block_enable(false);
            state = IDLE;
            break;
#endif            
    }

    rangeCtrl.reset_ranging();
    Radio::set_RF1480(Radio::LOAD50);
}

// /////////////////////////////////////////////////////////////////////////////
/// Handler: Start a ref clock cal
//
/// \memberof phy_t
//
void phy_t::handle_cal_ref_clock( cmdInt_t::cmd_t const & cmd )
{
    bool mode_type_comb_ok =    (MODE_3G == cur_mode)       //  3G mode is always OK.
                             || (   (MODE_4G == cur_mode)   //  4G mode is only OK for GPS type.
                                 && (   cmdInt_t::paramCalRef_t::GPS_CAL
                                     == cmd.cal_ref.cal_type));
    
    if( (state != IDLE) || !mode_type_comb_ok )
    {
        debugStr(DebugStream::warn2) << "need to be in idle state to start a cal,\n"
                                     << "in 3G mode any type may be started, but in 4G mode only GPS can be started\n";
        cmdInt.send_ack( cmdInt_t::ACK_CAL_REF_CLOCK_STATE );
        return;
    }

    if( cmd.cal_ref.cal_type != cmdInt_t::paramCalRef_t::GPS_CAL )
    {
        if( !Radio::set_frequency( cur_band, cmd.cal_ref.freq ) )
        {
            debugStr(DebugStream::warn2) << "freq not in band\n";
            cmdInt.send_ack( cmdInt_t::ACK_CAL_REF_CLOCK_FREQ );
            return;
        }
    }

    unsigned debug = debugStr.get_state();
    switch( cmd.cal_ref.cal_type )
    {
        case cmdInt_t::paramCalRef_t::GPS_CAL:
            ref_cal = auto_ptr<RefCal>( new RefCalGPS( fpga, debug ) );
            break;

        case cmdInt_t::paramCalRef_t::CPICH_CAL:
            rangeCtrl.reset_ranging();
            ref_cal = auto_ptr<RefCal>( new RefCalCPICH( fpga,
                                                         rangeCtrl,
                                                         low_power,
                                                         cmd.cal_ref.code,
                                                         cmd.cal_ref.antenna,
                                                         ref_cal_threshold_factor_dB,
                                                         antenna_mode,
                                                         debug,
                                                         debug_files
                                                       ) );
            break;

        case cmdInt_t::paramCalRef_t::PSCH_CAL:
            rangeCtrl.reset_ranging();
            ref_cal = auto_ptr<RefCal>( new RefCalPSCH( fpga,
                                                        rangeCtrl,
                                                        low_power,
                                                        ref_cal_threshold_factor_dB,
                                                        antenna_mode,
                                                        debug,
                                                        debug_files
                                                      ) );
            break;
    }

    state = CAL_REF_CLOCK;

    cmdInt.send_ack( cmdInt_t::ACK_CAL_REF_CLOCK_OK );
}

// /////////////////////////////////////////////////////////////////////////////
/// Handler: Set band
//
/// \memberof phy_t
//
void phy_t::handle_set_band( cmdInt_t::cmd_t const & cmd )
{
    if( state != IDLE )
    {
        // has to be idle
        cmdInt.send_ack( cmdInt_t::ACK_SET_BAND_STATE );
        return;
    }

    cur_band = cmd.set_band.band;

    cmdInt.send_ack( cmdInt_t::ACK_SET_BAND_OK );
}

// /////////////////////////////////////////////////////////////////////////////
/// Handler: Start an uplink search and track operation
//
/// Build a search and track object from the appropriate correlator
/// (currently only do search/track on uplinks)
/// \memberof phy_t
//
void phy_t::handle_ul_search( cmdInt_t::cmd_t const & cmd )
{
    using namespace std;

    if( cmd.start_ul_search.restart )
    {
        if( state != UL_SEARCHANDTRACK )
        {
            debugStr(DebugStream::warn2) << "must already be in search and track mode to restart\n";
            cmdInt.send_ack( cmdInt_t::ACK_UPLINK_ST_STATE );
            return;
        }
        search_and_track->restart();
        cmdInt.send_ack( cmdInt_t::ACK_UPLINK_ST_OK );
        return;
    }

    if( (state != IDLE) || (cur_mode != MODE_3G) )
    {
        debugStr(DebugStream::warn2) << "must be idle and in 3G mode to start ul search\n";
        cmdInt.send_ack( cmdInt_t::ACK_UPLINK_ST_STATE );
        return;
    }

    if( !Radio::set_frequency( cur_band, cmd.start_ul_search.freq ) )
    {
        debugStr(DebugStream::warn2) << cmd.start_ul_search.freq << " not in band " << Radio::band_to_string(cur_band) <<'\n';
        cmdInt.send_ack( cmdInt_t::ACK_UPLINK_ST_FREQ );
        return;
    }

    pwrCal.setBand( cur_band );
    pwrCal.setFreq( cmd.start_ul_search.freq );

    rangeCtrl.reset_ranging();

    uint32_t pilots = cmd.start_ul_search.pilots;

    unsigned debug = debugStr.get_state();

    auto_ptr<CorrelationSource> cs( new UplinkDPCCHSource( cmd.start_ul_search.code, pilots, debug ) );

    auto_ptr<Correlator> c
    (
        new Correlator( fpga,
                        cs,
                        low_power,
                        Correlator::SEARCH_ANT1,  // antenna num will get updated by seach_and_track later
                        0,
                        debug,
                        debug_files
                      )
    );

    assert( search_and_track.get() == 0 );  // there shouldn't be a live tracker at this point
    search_and_track = auto_ptr<SearchAndTrack>
    (
        new SearchAndTrack( rangeCtrl,
                            pwrCal,
                            c,
                            antenna_mode,
                            search_threshold_factor_dB,
                            track_threshold_factor_dB,
                            searching_pause,
                            tracking_pause,
                            debug
                          )
    );

    state = UL_SEARCHANDTRACK;

    cmdInt.send_ack( cmdInt_t::ACK_UPLINK_ST_OK );
}

// /////////////////////////////////////////////////////////////////////////////
/// Handler: Start an downlink search and track operation
//
/// Build a search and track object from the appropriate correlator
/// (currently only do search/track on downlinks)
/// \memberof phy_t
//
void phy_t::handle_dl_search( cmdInt_t::cmd_t const & cmd )
{
    using namespace std;

    if( cmd.start_dl_search.restart )
    {
        if( state != DL_SEARCHANDTRACK )
        {
            debugStr(DebugStream::warn2) << "must already be in search and track mode to restart\n";
            cmdInt.send_ack( cmdInt_t::ACK_DOWNLINK_ST_STATE );
            return;
        }
        search_and_track->restart();
        cmdInt.send_ack( cmdInt_t::ACK_DOWNLINK_ST_OK );
        return;
    }

    if( (state != IDLE) || (cur_mode != MODE_3G) )
    {
        debugStr(DebugStream::warn2) << "must be idle and in 3G mode to start dl search\n";
        cmdInt.send_ack( cmdInt_t::ACK_DOWNLINK_ST_STATE );
        return;
    }

    if( !Radio::set_frequency( cur_band, cmd.start_dl_search.freq ) )
    {
        debugStr(DebugStream::warn2) << cmd.start_dl_search.freq << " not in band " << Radio::band_to_string(cur_band) <<'\n';
        cmdInt.send_ack( cmdInt_t::ACK_DOWNLINK_ST_FREQ );
        return;
    }

    pwrCal.setBand( cur_band );
    pwrCal.setFreq( cmd.start_dl_search.freq );

    rangeCtrl.reset_ranging();

    uint32_t antenna = 1;
    auto_ptr<CorrelationSource> cs(new DownlinkPCPICHSource(cmd.start_dl_search.code,
                                                            antenna,
                                                            cmd.start_dl_search.syms,
                                                            debugStr.get_state(),
                                                            debug_files));

    auto_ptr<Correlator> corr(new Correlator(fpga,
                                              cs,
                                              low_power,
                                              Correlator::SEARCH_ANT1,  // antenna num will get updated by seach_and_track later
                                              0,
                                              debugStr.get_state(),
                                              debug_files));

    assert( search_and_track.get() == 0 );  // there shouldn't be a live tracker at this point
    search_and_track = auto_ptr<SearchAndTrack>(new SearchAndTrack(rangeCtrl,
                                                                   pwrCal,
                                                                   corr,
                                                                   antenna_mode,
                                                                   search_threshold_factor_dB,
                                                                   track_threshold_factor_dB,
                                                                   searching_pause,
                                                                   tracking_pause,
                                                                   debugStr.get_state()));

    state = DL_SEARCHANDTRACK;

    cmdInt.send_ack( cmdInt_t::ACK_DOWNLINK_ST_OK );
}

// /////////////////////////////////////////////////////////////////////////////
/// Handler: Start a lte search and track operation
//
/// Build a search and track object from the appropriate correlator files
/// (currently only do search/track on uplinks)
/// \memberof phy_t
//
void phy_t::handle_lte_search( cmdInt_t::cmd_t const & cmd )
{
    using namespace std;

    if( cmd.start_lte_search.restart )
    {
        if( state != LTE_SEARCHANDTRACK )
        {
            debugStr(DebugStream::warn2) << "must already be in search and track mode to restart\n";
            cmdInt.send_ack( cmdInt_t::ACK_4G_UPLINK_ST_STATE );
            return;
        }
        search_and_track_4g->restart();
        cmdInt.send_ack( cmdInt_t::ACK_4G_UPLINK_ST_OK );
        return;
    }

    if( (state != IDLE) || (cur_mode != MODE_4G) )
    {
        debugStr(DebugStream::warn2) << "must be idle and in 4G mode to start lte search\n";
        cmdInt.send_ack( cmdInt_t::ACK_4G_UPLINK_ST_STATE );
        return;
    }

    try
    {
        if (1 == antenna_mode)
        {
            fpga.set_4g_antenna(Fpga::ANT_MODE_ANT1);
        }
        else
        {
            fpga.set_4g_antenna(Fpga::ANT_MODE_ANT2);
        }
        
        unsigned debug = debugStr.get_state();
        
        auto_ptr<Correlator_4g> c(new Correlator_4g( fpga,
                                                     std::string(cmd.start_lte_search.ch_file_1_coeff),
                                                     std::string(cmd.start_lte_search.ch_file_2_coeff),
                                                     std::string(cmd.start_lte_search.corr_coeff),
                                                     search_average_4G,
                                                     track_average_4G,
                                                     search_threshold_factor_4G_dB,
                                                     track_threshold_factor_4G_dB,
                                                     low_power,
                                                     debug ));

        freq_t eff_freq = static_cast<freq_t>(cmd.start_lte_search.freq + c->get_tuning_offset());
        if( !Radio::set_frequency( cur_band, eff_freq ) )
        {
            debugStr(DebugStream::warn2) << eff_freq << " not in band " << Radio::band_to_string(cur_band) <<'\n';
            cmdInt.send_ack( cmdInt_t::ACK_4G_UPLINK_ST_FREQ );
            return;
        }

        pwrCal.setBand( cur_band );
        pwrCal.setFreq( cmd.start_lte_search.freq );

        rangeCtrl.reset_ranging();
        
        double factor1 = 10*log10(pwrCal.getFactor(0));
        double factor2 = 10*log10(pwrCal.getFactor(100));
        debugStr(DebugStream::warn2) <<   boost::format("f = %.0f Hz + %.0f Hz (band %s), c = %.1f dB,  %.1f dB\n") 
                                        % cmd.start_lte_search.freq
                                        % c->get_tuning_offset()
                                        % Radio::band_to_string(cur_band)
                                        % factor1
                                        % factor2;

        assert( search_and_track_4g.get() == 0 );  // there shouldn't be a live tracker at this point
        search_and_track_4g = auto_ptr<SearchAndTrack_4g>
        (
            new SearchAndTrack_4g( rangeCtrl,
                                   pwrCal,
                                   c,
                                   searching_pause,
                                   tracking_pause,
                                   debug
                                 )
        );

        state = LTE_SEARCHANDTRACK;

        cmdInt.send_ack( cmdInt_t::ACK_4G_UPLINK_ST_OK );
    }
    catch( exception & e )
    {
        cmdInt.send_ack( cmdInt_t::ACK_4G_UPLINK_ST_PARAM );
        debugStr(DebugStream::warn1) << "Failed to start the LTE search ( " << e.what() << " ) \n";
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Handler: Start a 4G regression test
//
/// Build a search and track object from the appropriate correlator files (which loads the
/// correlator files).  Load the test vectors to the regression FIFO, trigger it and wait
/// until it is empty.
/// \memberof phy_t
//
void phy_t::handle_regression_4g( cmdInt_t::cmd_t const & cmd )
{
    using namespace std;
#ifdef SWTEST_4G_FPGA
    std::list<Fpga::CorrCmdStatus_4g_t> cmd_status_list;

    if( (state != IDLE) || (cur_mode != MODE_4G) )
    {
        debugStr(DebugStream::warn2) << "must be idle and in 4G mode to start regression_4g test\n";
        cmdInt.send_ack( cmdInt_t::ACK_4G_REGR4G_ST_STATE );
        return;
    }

    try
    {
        if (!fpga.regression_4g_block_enable(true)) // enable regression block
        {
            throw std::runtime_error("Could not enable 4G regression block");
        }
        debugStr(DebugStream::info1) << "Regression block enabled\n";

        fpga.get_4g_correlation_command_status(cmd_status_list);  // Monitor status of the correlator command queue
        debugStr(DebugStream::info1) << "Correlator queue checked\n";

        unsigned debug = debugStr.get_state();
        
        auto_ptr<Correlator_4g> c(new Correlator_4g( fpga,
                                                     std::string(cmd.regression_4g.ch_file_1_coeff),
                                                     std::string(cmd.regression_4g.ch_file_2_coeff),
                                                     std::string(cmd.regression_4g.corr_coeff),
                                                     search_average_4G,
                                                     track_average_4G,
                                                     search_threshold_factor_4G_dB,
                                                     track_threshold_factor_4G_dB,
                                                     low_power,
                                                     debug,
                                                     false ));  // do not restart correlator

        rangeCtrl.reset_ranging();
        // Ensure result FIFO is empty - empty it if it is not!
        size_t maxNumPeaks = 1024;
        Fpga::CorrPeak_4g_t peaks[maxNumPeaks];
        uint32_t numPeaks;
        do {
            fpga.get_4g_peak_data(maxNumPeaks, peaks, numPeaks);
            debugStr(DebugStream::info1) << boost::format("handle_regression_4g: Found %d peaks in correlator at start\n")
                                                        % numPeaks;
        } while (numPeaks != 0);
        
        c->start_cont_correlation(); // by this point regression interface is enabled so all ADC samples are zeroes
        debugStr(DebugStream::info1) << "Started continuous correlation\n";

        do {
            if (!fpga.get_4g_peak_data(maxNumPeaks, peaks, numPeaks))
            {
                debugStr(DebugStream::info1) << boost::format("handle_regression_4g: correlator overflow\n");
            }
            debugStr(DebugStream::info1) << boost::format("handle_regression_4g: Found %d peaks in correlator, no samples being played\n")
                                                        % numPeaks;
        } while (numPeaks != 0);
        debugStr(DebugStream::info1) << "Reading and clearing statistics AFTER starting correlation\n";
        float peak, sum;
        fpga.get_4g_statistics(true, peak, sum); // debug

        std::list <uint32_t> adc_samples;
        if (EXIT_FAILURE == fpga.read_regression_4g_samples(std::string(cmd.regression_4g.test_vec), adc_samples))
        {
            debugStr(DebugStream::error) << boost::format("read_regression_4g_samples: Error reading test_vec file %s\n")
                                                           % cmd.regression_4g.test_vec;
            throw runtime_error("Error reading test_vec file");
        }

        if (EXIT_FAILURE == fpga.regression_4g_program_fifo(adc_samples))
        {
            debugStr(DebugStream::error) << "regression_4g_program_fifo: Not EMPTY at start or FULL before programming complete\n";
            throw runtime_error("Error programming regression_4g FIFO");
        }

        fpga.get_4g_correlation_command_status(cmd_status_list);  // Monitor status of the correlator command queue

        if (EXIT_FAILURE == fpga.regression_4g_trigger(false))  // and let it go
        {
            debugStr(DebugStream::error) << "regression_4g_trigger: EMPTY at start or interface not enabled\n";
            throw runtime_error("Error programming regression_4g FIFO");
        }

        state = REGRESSION_4G;

        cmdInt.send_ack( cmdInt_t::ACK_4G_REGR4G_ST_ACCEPTED );
    }
    catch( exception & e )
    {
        cmdInt.send_ack( cmdInt_t::ACK_4G_REGR4G_ST_PARAM );
        debugStr(DebugStream::warn1) << "Failed to start the 4g regression test ( " << e.what() << " ) \n";
    }
#else
        std::string ans = "Not enabled.  SWTEST_4G_FPGA not defined in build";
        //
        //  Send the reply.
        //
        cmdInt.send_ack( cmdInt_t::ACK_4G_REGR4G_ST_OK, ans );
#endif
}

// /////////////////////////////////////////////////////////////////////////////
/// Handler: Get version
//
/// \memberof phy_t
//
void phy_t::handle_get_version( cmdInt_t::cmd_t const & cmd )
{
    if( state == STANDBY )
    {
        cmdInt.send_ack( cmdInt_t::ACK_GET_VERSION_STATE );
    }
    else
    {
        stringstream versions;

        versions << Pcb::getPcbRev()                           << ","
                 << static_cast<unsigned>(fpga.get_version()) << ","
                 << SW_VERSION_SHORT;

        cmdInt.send_ack( cmdInt_t::ACK_GET_VERSION_OK, versions.str() );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Handler: Set mode
//
/// \memberof phy_t
//
void phy_t::handle_set_mode( cmdInt_t::cmd_t const & cmd )
{
    if( state != IDLE )
    {
        //
        //  Has to be idle
        //
        cmdInt.send_ack( cmdInt_t::ACK_SET_MODE_STATE );
    }
    else
    {
        //
        //  Send the command acknowledgement first
        //
        cmdInt.send_ack( cmdInt_t::ACK_SET_MODE_OK );

        //
        //  If the mode has changed then load the appropriate image and change the radio and ADC configurations.
        //
        if (cur_mode != cmd.set_mode.mode)
        {
            cur_mode = cmd.set_mode.mode;

            // 4G uses the 3G Radio and ADC settings.
            if ((cur_mode == MODE_3G) || (cur_mode == MODE_4G))
            {
                Radio::set_3G_mode();
                ADC::set_3G_mode();
            }
            else
            {
                Radio::set_GSM_mode();
                ADC::set_GSM_mode();
            }

            string image_name;
            switch (cur_mode)
            {
            case MODE_GSM:
                image_name = image_name_for_GSM;
                break;
            case MODE_3G:
                image_name = image_name_for_3G;
                break;
            case MODE_4G:
                image_name = image_name_for_4G;
                break;
            default:
                assert(false);
                break;
            }

            fpga.load_image(image_name, cur_mode);
        }

        //
        //  Construct a versions message.
        //
        stringstream versions;

        versions << Pcb::getPcbRev()                          << ","
                 << static_cast<unsigned>(fpga.get_version()) << ","
                 << SW_VERSION_SHORT;

        cmdInt.send_ack(cmdInt_t::ACK_SET_MODE_DONE, versions.str());
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Handler: Set standby
//
/// \memberof phy_t
//
void phy_t::handle_set_standby( cmdInt_t::cmd_t const & cmd )
{
    if( (state != IDLE) && (state != STANDBY) )
    {
        //
        //  Has to be idle
        //
        cmdInt.send_ack( cmdInt_t::ACK_SET_STANDBY_STATE );
    }
    else
    {
        //
        //  First bring all the the elements out of standby, Radio first because it provides the clocks.
        //
        if (cur_standby_state == cmdInt_t::ADC_IDLE)
        {
            ADC::exit_low_power_mode((cur_mode == MODE_3G) || (cur_mode == MODE_4G));     // 4G uses the 3G sample rate
        }
        else
        if (cur_standby_state == cmdInt_t::FPGA_IDLE)
        {
            fpga.exit_low_power_mode();
        }
        if (cur_standby_state == cmdInt_t::FPGA_ADC_IDLE)
        {
            ADC::exit_low_power_mode((cur_mode == MODE_3G) || (cur_mode == MODE_4G));     // 4G uses the 3G sample rate

            //
            //  Allow the clocks to stabilise.
            //
            usleep(10000);
            fpga.exit_low_power_mode();
        }
        if (cur_standby_state == cmdInt_t::RADIO_FPGA_ADC_IDLE)
        {
            Radio::exit_low_power_mode((cur_mode == MODE_3G) || (cur_mode == MODE_4G));   // 4G uses the 3G configuration.
            ADC::exit_low_power_mode((cur_mode == MODE_3G) || (cur_mode == MODE_4G));     // 4G uses the 3G sample rate

            //
            //  Allow the clocks to stabilise.
            //
            usleep(10000);
            fpga.exit_low_power_mode();
        }

        //
        //  Put the requested elements into standby, Radio last because it provides the clocks.
        //
        if (cmd.set_standby == cmdInt_t::ADC_IDLE)
        {
            ADC::enter_low_power_mode();
        }
        else
        if (cmd.set_standby == cmdInt_t::FPGA_IDLE)
        {
            fpga.enter_low_power_mode();
        }
        if (cmd.set_standby == cmdInt_t::FPGA_ADC_IDLE)
        {
            fpga.enter_low_power_mode();
            ADC::enter_low_power_mode();
        }
        if (cmd.set_standby == cmdInt_t::RADIO_FPGA_ADC_IDLE)
        {
            fpga.enter_low_power_mode();
            ADC::enter_low_power_mode();
            Radio::enter_low_power_mode();
        }

        //
        //  Update the local standby flags and the state.
        //
        cur_standby_state = cmd.set_standby;

        if (cur_standby_state == cmdInt_t::ACTIVE)
        {
            state = IDLE;
        }
        else
        {
            state = STANDBY;
        }

        //
        //  Send the command acknowledgement
        //
        cmdInt.send_ack( cmdInt_t::ACK_SET_STANDBY_OK );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// change pause frames between searching and tracking iterations
//
/// \memberof phy_t
//
void phy_t::handle_set_pauses( cmdInt_t::cmd_t const & cmd )
{
    if( (state != IDLE) && (state != STANDBY) )
    {
        //
        //  Has to be idle
        //
        cmdInt.send_ack( cmdInt_t::ACK_SET_STANDBY_STATE );
    }
    else
    {
        searching_pause = cmd.set_pauses.searching_pause;
        tracking_pause  = cmd.set_pauses.tracking_pause;
        cmdInt.send_ack( cmdInt_t::ACK_SET_PAUSES_OK );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Handler: Set low power
//
/// \memberof phy_t
//
void phy_t::handle_set_low_power( cmdInt_t::cmd_t const & cmd )
{
    if( (state != IDLE) && (state != STANDBY) )
    {
        //
        //  Has to be idle
        //
        cmdInt.send_ack( cmdInt_t::ACK_SET_LOW_POWER_STATE );
    }
    else
    {
        low_power = cmd.set_low_power.low_power;

        cmdInt.send_ack( cmdInt_t::ACK_SET_LOW_POWER_OK );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Handler: Start a channel power operation
//
/// Build a search and track object from the appropriate correlator
/// (currently only do search/track on uplinks)
/// \memberof phy_t
//
void phy_t::handle_chan_pwr( cmdInt_t::cmd_t const & cmd )
{
    using namespace std;

    if( (state != IDLE) || (cur_mode != MODE_GSM) )
    {
        debugStr(DebugStream::warn2) << "must be idle and in GSM mode to start channel power\n";
        cmdInt.send_ack( cmdInt_t::ACK_CHAN_PWR_STATE );
        return;
    }

    if( !Radio::set_frequency( cur_band, cmd.chan_pwr.freq ) )
    {
        debugStr(DebugStream::warn2) << cmd.chan_pwr.freq << " not in band " << Radio::band_to_string(cur_band) <<'\n';
        cmdInt.send_ack( cmdInt_t::ACK_CHAN_PWR_FREQ );
        return;
    }

    pwrCal.setBand( cur_band );
    pwrCal.setFreq( cmd.chan_pwr.freq );

    rangeCtrl.reset_ranging();

    unsigned debug = debugStr.get_state();

    assert( time_cont_track.get() == 0 );  // there shouldn't be a live tracker at this point
    time_cont_track = auto_ptr<TimeContTrack>(new TimeContTrack(fpga, rangeCtrl, pwrCal, antenna_mode, debug));

    state = CHAN_PWR;
    cmdInt.send_ack( cmdInt_t::ACK_CHAN_PWR_OK );
}

// /////////////////////////////////////////////////////////////////////////////
/// Handler: Set antenna
//
/// \memberof phy_t
//
void phy_t::handle_set_antenna( cmdInt_t::cmd_t const & cmd )
{
    if( (state != IDLE) && (state != STANDBY) )
    {
        //
        //  Has to be idle
        //
        cmdInt.send_ack( cmdInt_t::ACK_SET_ANTENNA_STATE );
    }
    else
    {
        antenna_mode = 1;
        if (cmd.set_antenna.antenna == cmdInt_t::paramSetAntenna_t::SET_ANTENNA_2)
        {
            antenna_mode = 2;
        }

        cmdInt.send_ack( cmdInt_t::ACK_SET_ANTENNA_OK );
    }
}

///////////////////////////////////////////////////////////////////////////////
//
// handle debug cmds
//
///////////////////////////////////////////////////////////////////////////////

// /////////////////////////////////////////////////////////////////////////////
/// Debug Handler: Change debug output verbosity
//
/// \memberof phy_t
//
void phy_t::handle_debug_print( cmdInt_t::cmd_t const & cmd )
{
    debug_printing( cmd.debug.uinta );
    cmdInt.debug_printing( cmd.debug.uinta );
    fpga.debug_printing( cmd.debug.uinta );
    Radio::debug_printing( cmd.debug.uinta );
    ADC::debug_printing( cmd.debug.uinta );
    DAC::debug_printing( cmd.debug.uinta );
    if( search_and_track.get() ) search_and_track->debug_printing( cmd.debug.uinta );
    if( search_and_track_4g.get() ) search_and_track_4g->debug_printing( cmd.debug.uinta );

    cmdInt.send_ack( cmdInt_t::ACK_DEBUG_PRINT_OK );
}

// /////////////////////////////////////////////////////////////////////////////
/// Debug Handler: set the vctcxo dac
//
/// \memberof phy_t
//
void phy_t::handle_debug_vctcxo( cmdInt_t::cmd_t const & cmd )
{
    if( state == STANDBY )
    {
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_VCTCXO_STATE );
    }
    else
    {
        DAC::set( cmd.debug.uinta );
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_VCTCXO_OK );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Debug Handler: Set reference calibration threshold adjustment factor
//
/// Set the threshold factor that is added to the built-in threshold to adjust
/// it
/// \memberof phy_t
//
void phy_t::handle_debug_ref_cal_threshold( cmdInt_t::cmd_t const & cmd )
{
    ref_cal_threshold_factor_dB = cmd.debug.frac;
    cmdInt.send_ack( cmdInt_t::ACK_DEBUG_REF_CAL_THRESHOLD_OK );
}

// /////////////////////////////////////////////////////////////////////////////
/// Debug Handler: Set search mode threshold adjustment factor
//
/// Set the threshold factor that is added to the built-in threshold to adjust
/// it
/// \memberof phy_t
//
void phy_t::handle_debug_search_threshold( cmdInt_t::cmd_t const & cmd )
{
    search_threshold_factor_dB = cmd.debug.frac;
    cmdInt.send_ack( cmdInt_t::ACK_DEBUG_SEARCH_THRESHOLD_OK );
}

// /////////////////////////////////////////////////////////////////////////////
/// Debug Handler: Set track mode threshold adjustment factor
//
/// Set the threshold factor that is added to the built-in threshold to adjust
/// it
/// \memberof phy_t
//
void phy_t::handle_debug_track_threshold( cmdInt_t::cmd_t const & cmd )
{
    track_threshold_factor_dB = cmd.debug.frac;
    cmdInt.send_ack( cmdInt_t::ACK_DEBUG_TRACK_THRESHOLD_OK );
}

// /////////////////////////////////////////////////////////////////////////////
/// Debug Handler: Set 4G search mode threshold adjustment factor
//
/// Set the 4G threshold factor that is added to the built-in threshold to
/// adjust it
/// \memberof phy_t
//
void phy_t::handle_debug_search_threshold_4G( cmdInt_t::cmd_t const & cmd )
{
    search_threshold_factor_4G_dB = cmd.debug.frac;
    cmdInt.send_ack( cmdInt_t::ACK_DEBUG_SEARCH_THRESHOLD_4G_OK );
}

// /////////////////////////////////////////////////////////////////////////////
/// Debug Handler: Set 4G track mode threshold adjustment factor
//
/// Set the 4G threshold factor that is added to the built-in threshold to
/// adjust it
/// \memberof phy_t
//
void phy_t::handle_debug_track_threshold_4G( cmdInt_t::cmd_t const & cmd )
{
    track_threshold_factor_4G_dB = cmd.debug.frac;
    cmdInt.send_ack( cmdInt_t::ACK_DEBUG_TRACK_THRESHOLD_4G_OK );
}

// /////////////////////////////////////////////////////////////////////////////
/// Debug Handler: Set the front end switches
//
/// \memberof phy_t
//
void phy_t::handle_debug_switches( cmdInt_t::cmd_t const & cmd )
{
    if( state == STANDBY )
    {
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_SWITCHES_STATE );
    }
    else
    {
        Radio::set_RF1480( cmd.debug.band );
        Radio::set_ADF4602_switches( cmd.debug.ADF_input, cmd.debug.uinta );

        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_SWITCHES_OK );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Debug Handler: read / write ADF4602 registers
//
/// \memberof phy_t
//
void phy_t::handle_debug_ADF4602( cmdInt_t::cmd_t const & cmd )
{
    if( state != IDLE )
    {
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_ADF4602_STATE );
        return;
    }

    if( cmd.debug.boolean )
    {
        // write
        if( cmd.debug.uinta == 0 ){
            // sub addr
            Radio::ADF4602_SPI_write( cmd.debug.uinta, cmd.debug.uintb, cmd.debug.uintc );
        }else{
            // norm addr
            Radio::ADF4602_SPI_write( cmd.debug.uinta, cmd.debug.uintc );
        }
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_ADF4602_OK );
    }
    else
    {
        // read
        std::string str = "to do - read ADF4602 spi";
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_ADF4602_OK, str );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Debug Handler: read / write the adc registers
//
/// \memberof phy_t
//
void phy_t::handle_debug_adc( cmdInt_t::cmd_t const & cmd )
{
    if( state != IDLE )
    {
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_ADC_STATE );
        return;
    }

    //cmd.debug.uinta = address
    //cmd.debug.uintc = data
    //cmd.debug.boolean : true = write, false = read

    if( cmd.debug.boolean )
    {
        // write
        ADC::SPI_write( cmd.debug.uinta, cmd.debug.uintc );
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_ADC_OK );
    }
    else
    {
        // read
        std::string str = "to do - read ADC spi";
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_ADC_OK, str );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Debug Handler: Set ranging on/off, optionally force a level
//
/// \memberof phy_t
//
void phy_t::handle_debug_ranging( cmdInt_t::cmd_t const & cmd )
{
    //cmd.debug.uinta = value (128 = none given)
    //cmd.debug.uintb : 1 = on, 0 = off

    if ( cmd.debug.uintb == 0 )
    {
        rangeCtrl.disable_auto_ranging();
    }
    else if ( cmd.debug.uintb == 1 )
    {
        rangeCtrl.enable_auto_ranging();
    }

    if( cmd.debug.uinta != 0x100 )
    {
        rangeCtrl.set_gain_dB(cmd.debug.uinta );
    }

    cmdInt.send_ack( cmdInt_t::ACK_DEBUG_RANGING_OK );
}

// /////////////////////////////////////////////////////////////////////////////
/// \brief Debug Handler: write to fpga ddr
/// \memberof phy_t
//
void phy_t::handle_debug_poke( cmdInt_t::cmd_t const & cmd )
{
    if( state == STANDBY )
    {
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_POKE_STATE );
    }
    else
    {
        fpga.mem_raw_write( cmd.debug.uinta, cmd.uint_vector );
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_POKE_OK );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// \brief Debug Handler: read from fpga ddr
/// \memberof phy_t
//
void phy_t::handle_debug_peek( cmdInt_t::cmd_t const & cmd )
{
    if( state == STANDBY )
    {
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_PEEK_STATE );
    }
    else
    {
        using namespace std;
        using boost::lexical_cast;

        vector<uint32_t> data;
        fpga.mem_raw_read( cmd.debug.uinta, cmd.debug.uintb, data );
        // we'll do this binary so can get whole frame at bit better speed
        string ans( (char*)&data[0], data.size()*4 );
        // old text version
        //~ string ans;
        //~ BOOST_FOREACH( uint32_t d, data )
        //~ {
            //~ ans += " ";
            //~ ans += lexical_cast<string>( d );
        //~ }
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_PEEK_OK, ans );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// \brief Debug Handler: read/write fpga register
/// \memberof phy_t
//
void phy_t::handle_debug_reg( cmdInt_t::cmd_t const & cmd )
{
    if( state == STANDBY )
    {
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_REG_STATE );
    }
    else
    {
        if( cmd.debug.boolean )
        {
            fpga.debug_reg_write( cmd.debug.uinta, cmd.debug.uintb );
            cmdInt.send_ack( cmdInt_t::ACK_DEBUG_REG_OK );
        }
        else
        {
            uint32_t v = fpga.debug_reg_read( cmd.debug.uinta );
            cmdInt.send_ack( cmdInt_t::ACK_DEBUG_REG_OK, boost::lexical_cast<std::string>(v) );
        }
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// read or write a gpio line
//
/// \memberof phy_t
//
void phy_t::handle_debug_gpio( cmdInt_t::cmd_t const & cmd )
{
    if( state == STANDBY )
    {
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_GPIO_STATE );
    }
    else
    {
        using boost::lexical_cast;
        // uinta 0 = set low, 1 = set high, 2 = read
        // uintb gpio number

        std::string ans;

        if( cmd.debug.uinta == 2 )
        {
            gpio temp( cmd.debug.uintb, gpio::in, debugStr.get_state() );
            ans = lexical_cast<string>( temp.get() );
        }
        else if( cmd.debug.uinta == 0 )
        {
            gpio temp( cmd.debug.uintb, gpio::out_low, debugStr.get_state() );
        }
        else if( cmd.debug.uinta == 1 )
        {
            gpio temp( cmd.debug.uintb, gpio::out_high, debugStr.get_state() );
        }

        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_GPIO_OK, ans );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// \brief Debug Handler: read from fpga ddr
/// \memberof phy_t
//
void phy_t::handle_debug_gsm( cmdInt_t::cmd_t const & cmd )
{
    if( state == STANDBY )
    {
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_GSM_STATE );
    }
    else
    {
        using namespace std;
        using boost::lexical_cast;

        vector<uint32_t> data;

        //
        //  Empty the FIFO.
        //
        fpga.empty_gsm_fifo();

        //
        //  Discard the out-of-date samples.
        //
        data.clear();

        //
        //  Get the new samples.
        //
        unsigned samples_to_read = cmd.debug.uinta;
        while (data.size() < samples_to_read)
        {
            fpga.read_gsm_fifo(samples_to_read-data.size(), data);
        }

        //
        //  Convert the samples to ASCII and send the reply.
        //
        string ans;
        BOOST_FOREACH( uint32_t d, data )
        {
            ans += " ";
            ans += lexical_cast<string>( d );
        }
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_GSM_OK, ans );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// \brief Debug Handler: Confirm 4G FPGA internal float matches compiler representation
/// \memberof phy_t
//
void phy_t::handle_debug_float( cmdInt_t::cmd_t const & cmd )
{
    if( state == STANDBY )
    {
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_FLOAT_STATE );
    }
    else
    {
        using namespace std;
        using boost::lexical_cast;

        string ans = "";
        #ifdef SWTEST_4G_FPGA
            int result = fpga.checkFpgaFloatRepresentation();
            if (EXIT_SUCCESS == result)
            {
                ans = "Passed";
            }
            else
            {
                ans = "Failed";
            }
        #else
            ans = "Not enabled.  SWTEST_4G_FPGA not defined in build";
        #endif

        //
        //  Send the reply.
        //
        cmdInt.send_ack( cmdInt_t::ACK_DEBUG_FLOAT_OK, ans );
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// dummy - never called
//
/// \memberof phy_t
//
void phy_t::handle_debug_help( cmdInt_t::cmd_t const & cmd )
{
}


///////////////////////////////////////////////////////////////////////////////
//
// processing functions
//
///////////////////////////////////////////////////////////////////////////////

// /////////////////////////////////////////////////////////////////////////////
/// Perform reference calibration iteration
//
/// Gets called repeatedly while in calibrate reference state.
/// The type of ref_cal object built will effect type of calibration performed
//
void phy_t::proc_cal_ref( void )
{
    assert( ref_cal.get() );

    switch( ref_cal->iterate() )
    {
        case RefCal::REF_CAL_ACTIVE:
        {
            break;
        }
        case RefCal::REF_CAL_DONE:
        {
            stringstream dac_val;

            dac_val << DAC::get();

            cmdInt.send_ack( cmdInt_t::ACK_CAL_REF_CLOCK_DONE, dac_val.str() );
            ref_cal.reset();
            state = IDLE;
            break;
        }
        case RefCal::REF_CAL_ABORT_TIMEOUT:
        {
            cmdInt.send_ack( cmdInt_t::ACK_CAL_REF_CLOCK_ABORT_TIMEOUT );
            ref_cal.reset();
            state = IDLE;
            break;
        }
        default :
        {
            break;
        }
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Perform and search and track operation
//
/// Gets called repeatedly while in search and track state.
//
void phy_t::proc_search_and_track( void )
{
    using namespace std;
    using boost::format;

    assert( search_and_track.get() );

    cmdInt_t::ackCode_t ack_searching    = cmdInt_t::ACK_UPLINK_ST_SEARCHING;
    cmdInt_t::ackCode_t ack_tracking     = cmdInt_t::ACK_UPLINK_ST_TRACKING;
    cmdInt_t::ackCode_t ack_track_result = cmdInt_t::ACK_UPLINK_ST_RESULT;
    cmdInt_t::ackCode_t ack_timeout      = cmdInt_t::ACK_UPLINK_ST_ABORT_TIMEOUT;
    if (state == DL_SEARCHANDTRACK)
    {
        ack_searching    = cmdInt_t::ACK_DOWNLINK_ST_SEARCHING;
        ack_tracking     = cmdInt_t::ACK_DOWNLINK_ST_TRACKING;
        ack_track_result = cmdInt_t::ACK_DOWNLINK_ST_RESULT;
        ack_timeout      = cmdInt_t::ACK_DOWNLINK_ST_ABORT_TIMEOUT;
    }

    switch( search_and_track->iterate() )
    {
        case SearchAndTrack::WAITING:
            break;
        case SearchAndTrack::START_SEARCH:
            cmdInt.send_ack( ack_searching );
            break;
        case SearchAndTrack::START_TRACK:
            cmdInt.send_ack( ack_tracking );
            break;
        case SearchAndTrack::NEW_RESULT:
        {
            SearchAndTrack::PathList_t const & paths = search_and_track->get_paths();

            //
            //  Add all valid paths to the result string.
            //
            // <chan pwr dB>,<numPaths>,{<power ant1 dB>,<power ant2 dB>,<sample offset>}
            std::string str;

            int c = 0;
            BOOST_FOREACH( SearchAndTrack::Path_t const & p, paths )
            {
                if( p.valid_count >= SearchAndTrack::VALID_COUNT_CONFIRMED )
                {
                    str +=  (format(", %6.2f, %6.2f, %5d") %
                            (10.0 * log10( p.power ) ) %            // pwr ant1
                            (10.0 * log10( p.power_alt ) ) %        // pwr ant2
                            (p.pos % Design::samples_per_frame )    // offset
                            ).str();
                    ++c;
                }
            }

            //
            //  Insert the RSSI and path count at the start of the result string.
            //
            str =   (format("%6.2f, %2d") %
                    (10.0 * log10(search_and_track->get_power())) % // ch pwr
                    c                                               // path count
                    ).str() + str;

            cmdInt.send_ack( ack_track_result, str );

            //
            //  Help debugging by explaining why the path count is zero.
            //
            if( c == 0 )
            {
                if( paths.size() == 0 )
                {
                    debugStr(DebugStream::info1) << "path list empty!\n";
                }
                else
                {
                    debugStr(DebugStream::info1) << paths.size() << " invalid paths\n";
                }
            }
            break;
        }
        case SearchAndTrack::ABORT_TIMEOUT:
        {
            cmdInt.send_ack(ack_timeout);
            search_and_track.reset();
            state = IDLE;
            break;
        }
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Perform 4G search and track operation
//
/// Gets called repeatedly while in search and track state.
//
void phy_t::proc_search_and_track_4g( void )
{
    using namespace std;
    using boost::format;

    assert( search_and_track_4g.get() );

    cmdInt_t::ackCode_t ack_searching            = cmdInt_t::ACK_4G_UPLINK_ST_SEARCHING;
    cmdInt_t::ackCode_t ack_tracking             = cmdInt_t::ACK_4G_UPLINK_ST_TRACKING;
    cmdInt_t::ackCode_t ack_track_result         = cmdInt_t::ACK_4G_UPLINK_ST_RESULT;
#ifdef ALWAYS_USE_LL_RESULT_CODE    
    cmdInt_t::ackCode_t ack_track_result_ranging = cmdInt_t::ACK_4G_UPLINK_ST_RESULT;
#else
    cmdInt_t::ackCode_t ack_track_result_ranging = cmdInt_t::ACK_4G_UPLINK_ST_RESULT_RANGING;
#endif

    SearchAndTrack_4g::iter_res_t iterate_result = search_and_track_4g->iterate();
    switch( iterate_result )
    {
        case SearchAndTrack_4g::WAITING:
            break;
        case SearchAndTrack_4g::START_SEARCH:
            cmdInt.send_ack( ack_searching );
            break;
        case SearchAndTrack_4g::START_TRACK:
            cmdInt.send_ack( ack_tracking );
            break;
        case SearchAndTrack_4g::NEW_RESULT:
        case SearchAndTrack_4g::NEW_RESULT_RANGING:
        {
            SearchAndTrack_4g::PathList_t const & paths = search_and_track_4g->get_paths();

            //
            //  Add the first few valid paths to the result string.
            //
            // <ch pwr dB>,<num paths>,{<det pwr dB>,<sample offset>}
            std::string str;

            int c = 0;
            int c_lim = 3;
            BOOST_FOREACH( SearchAndTrack_4g::Path_t const & p, paths )
            {
                if( p.valid_count >= SearchAndTrack_4g::VALID_COUNT_CONFIRMED )
                {
                    str +=  (format(", %6.2f, %4u") %
                            (10.0 * log10( p.power ) ) %
                            (p.offset)
                            ).str();
                    ++c;
                    if (c > c_lim)
                    {
                        break;
                    }
                }
            }

            //
            //  Insert the RSSI and path count at the start of the result string.
            //
            str =   (format("%6.2f, %2d") %
                    (10.0 * log10(search_and_track_4g->get_power())) % // ch pwr
                    c                                                  // path count
                    ).str() + str;

            if (SearchAndTrack_4g::NEW_RESULT == iterate_result)
            {
                cmdInt.send_ack( ack_track_result, str );
            }
            else
            {
                cmdInt.send_ack( ack_track_result_ranging, str );
            }

            //
            //  Help debugging by explaining why the path count is zero.
            //
            if( c == 0 )
            {
                if( paths.size() == 0 )
                {
                    debugStr(DebugStream::info1) << "path list empty!\n";
                }
                else
                {
                    debugStr(DebugStream::info1) << paths.size() << " invalid paths\n";
                }
            }
            break;
        }
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Perform and GSM time-continuous track operation
//
/// Gets called repeatedly while in channel power state.
//
void phy_t::proc_chan_pwr( void )
{
    using namespace std;
    using boost::format;

    assert( time_cont_track.get() );

    switch( time_cont_track->iterate() )
    {
        case TimeContTrack::TC_WAITING:
        {
            break;
        }
        case TimeContTrack::TC_NEW_RESULT:
        {
            vector<double> pwr_samples;
            double chan_pwr = time_cont_track->get_results(pwr_samples);

            //
            //  Add all valid paths to the result string.
            //
            // <avg ch pwr dB>, {<sub-slot ch pwr dB>}*8/24
            std::string str;

            str += (format("%7.2f, ") % (10.0*log10(chan_pwr))).str();

            unsigned n;
            for (n = 0 ; n < (pwr_samples.size()-1) ; ++n)
            {
                str += (format("%7.2f, ") % (10.0*log10(pwr_samples[n]))).str();
            }
            str += (format("%7.2f") % (10.0*log10(pwr_samples[n]))).str();

            cmdInt.send_ack( cmdInt_t::ACK_CHAN_PWR_RESULT, str );

            //
            //  In GSM tracking the ranging may be upset by RACH bursts, or other high-power bursts.  If so the internal
            //  mode will have been switched so that the gain is only ever reduced.  However such effects tend to be
            //  transient and so it is worth restoring the auto-ranging mode once a successful measurement has been
            //  completed.  If the conditions persist then measurement rates will tend to be low.
            //
            if (rangeCtrl.decrease_gain_forced())
            {
                rangeCtrl.enable_auto_ranging();
            }
            break;
        }
    }
}

// /////////////////////////////////////////////////////////////////////////////
/// Perform 4G regression test
//
/// Gets called repeatedly while in regression 4g state
//
void phy_t::proc_regression_4g( void )
{
#ifdef SWTEST_4G_FPGA
    using namespace std;
    using boost::format;

    static bool seenPeaks = false;

    cmdInt_t::ackCode_t ack_running      = cmdInt_t::ACK_4G_REGR4G_ST_RUNNING;
    cmdInt_t::ackCode_t ack_empty        = cmdInt_t::ACK_4G_REGR4G_ST_EMPTY;
    cmdInt_t::ackCode_t ack_stats        = cmdInt_t::ACK_4G_REGR4G_ST_STATS;
    cmdInt_t::ackCode_t ack_regr4g_result = cmdInt_t::ACK_4G_REGR4G_ST_RESULT;
    cmdInt_t::ackCode_t ack_overflow = cmdInt_t::ACK_4G_REGR4G_ST_OVERFLOW;
    cmdInt_t::ackCode_t ack_abort_timeout = cmdInt_t::ACK_4G_REGR4R_ST_TIMEOUT;
    cmdInt_t::ackCode_t ack_ok = cmdInt_t::ACK_4G_REGR4G_ST_OK; // we have finished with command

    switch( fpga.regression_4g_iterate())
    {
        case Fpga::REGRESSION_4G_ST_NOENABLED:
        {
            cmdInt.send_ack( ack_ok );
            state = IDLE;
            break;
        }
        case Fpga::REGRESSION_4G_ST_RUNNING:
            cmdInt.send_ack( ack_running );
            seenPeaks = false;
            break;
        case Fpga::REGRESSION_4G_ST_EMPTY:
            cmdInt.send_ack( ack_empty );
            break;
        case Fpga::REGRESSION_4G_ST_RESULT:
        {
            std::string str;
            size_t maxNumPeaks = 1; // can increase size whilst the command interface still copes
            Fpga::CorrPeak_4g_t peaks[maxNumPeaks];
            uint32_t numPeaks;
            if (!fpga.get_4g_peak_data(maxNumPeaks, peaks, numPeaks))
            {
                cmdInt.send_ack( ack_overflow ); // indicate correlation FIFO has overflowed
            }
            if (numPeaks != 0)
            {
                for (unsigned index = 0; index < numPeaks; index++)
                {
                    str += (format("%llu, %6.2f, %6.2f, %6.2f\n")
                            % peaks[index].time
                            % peaks[index].CorrPeakNum
                            % peaks[index].CorrPeakDenSqrd
                            % peaks[index].SumCorrInpAtPeak).str();
                }
                cmdInt.send_ack( ack_regr4g_result, str );
                seenPeaks = true;
            }
            break;
        }
        case Fpga::REGRESSION_4G_ST_NORESULT:
        {
            // Correlator still running, regression block still enabled
            std::string str;
            float peak, sum;
            uint32_t numvals = fpga.get_4g_statistics(true, peak, sum); // SS says read statistics *before* stopping correlator
            str = (boost::format("%d, %f, %f\n")
                  % numvals
                  % peak
                  % sum).str();
            cmdInt.send_ack( ack_stats, str );
            break;
        }
        case Fpga::REGRESSION_4G_ST_WAITING:
            // do nothing - we are waiting
            break;
        case Fpga::REGRESSION_4G_ST_FINISHED:
            // do nothing - we are waiting for REGRESSION_4G_ST_NOENABLED
            if (!seenPeaks)
            {
                cmdInt.send_ack( ack_abort_timeout ); // no peaks observed.
            }
            seenPeaks = false;
            break;
    }
#else
    state = IDLE;   // should not get here but just in case
#endif // ifdef SWTEST_4G_FPGA
}

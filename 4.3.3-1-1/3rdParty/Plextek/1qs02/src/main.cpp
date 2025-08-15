/***********************************************************************************************************************
 *
 *
 ***********************************************************************************************************************
 *  Filename:   main.cpp
 *  Author(s):  pdm, pmd
 *******************************************************************************************************************//**
 * File Description
 * ================
 *
 *  @file
 *  @brief          Entry point
 *  @details         - Command line option processing
 *                   - Call hardware initialistion
 *                   - Instatiation of phy object
 *
 **********************************************************************************************************************/

#include <errno.h>
#include <getopt.h>

#include <iostream>
#include <stdexcept>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include "phy.hpp"
#include "configoptions.hpp"
#include "fpga.hpp"
#include "radio.hpp"
#include "design.hpp"
#include "adc.hpp"
#include "tables.hpp"
#include "dac.hpp"
#include "debug.hpp"
#include "pcb.hpp"
#include "version.hpp"

#include <stdlib.h>
#include <signal.h>

/// \name Compiler defines these to show some versioning info when program run
//@{
///
#ifndef COMPILE_INFO
/// compiler version number
#define COMPILE_INFO "unknown"
#endif
#ifndef HOST_MACHINE
/// build machine name
#define HOST_MACHINE "unknown"
#endif
#ifndef GIT_HASH
/// Git hash
#define GIT_HASH "unknown"
#endif
//@}


// /////////////////////////////////////////////////////////////////////////////
/// Displays a helpful usage message to assist command line operation
//
void usage
(
    const std::string& progname ///< basename of program obtained from argc
)
{
    using namespace std;

    cerr << '\n';
    cerr << "Usage: " << progname << " [OPTIONS]\n";
    cerr << "Options:\n\n";
    cerr << "help                               - this message\n";
    cerr << "ref-cal-thresh-factor=<double>     - threshold adjustment factor in dB for the reference calibration\n";
    cerr << "search-thresh-factor=<double>      - threshold adjustment factor in dB for the search mode\n";
    cerr << "track-thresh-factor=<double>       - threshold adjustment factor in dB for the track mode\n";
    cerr << "4G-search-thresh-factor=<double>   - threshold adjustment factor in dB for the 4G search mode\n";
    cerr << "4G-track-thresh-factor=<double>    - threshold adjustment factor in dB for the 4G track mode\n";
    cerr << "4G-search-average=<double>         - number of raw correlation results to average in the 4G search mode (0 => use default)\n";
    cerr << "4G-track-average=<double>          - number of raw correlation results to average in the 4G track mode (0 => use default)\n";
    cerr << "fpga-4G-image=<filename>           - define 4G fpga image filename\n";
    cerr << "fpga-3G-image=<filename>           - define 3G fpga image filename\n";
    cerr << "fpga-GSM-image=<filename>          - define GSM fpga image filename\n";
    cerr << "init-mode=[4G|3G|GSM]              - set the initial mode to 4G, 3G or GSM\n";
    cerr << "low-power=[LOW_POWER|HIGH_SENS]    - select low-power or high sensitivity (default) settings for the averaging\n";
    cerr << "fpga-man-load                      - prompt the user to load the FPGA manually\n";
    cerr << "fpga-load-only                     - load the fpga image and quit\n";
    cerr << "fpga-skip-load                     - run as usual by skip the image load\n";
    cerr << "debug=[level]                      - debug printing, option verbosity mask\n";
    cerr << "debug-files                        - generate data files during correlator ctor\n";
    cerr << "no-auto-range                      - turn off power auto ranging\n";
    cerr << "vctcxo-dac=<int>                   - set initial vctcxo dac value\n";
    cerr << "port=<int>                         - port to listen to commands on\n";
    cerr << "searching-pause=<int>              - number of idle frames between searching iterations (interval = pause + 8 or 9 frames)\n";
    cerr << "tracking-pause=<int>               - number of idle frames between tracking iterations (interval = pause + 12 or 13 frames)\n";
    cerr << "antenna-mode=[int]                 - 0=normal(default), 1=ant1 aways, 2=ant2 always\n";
    cerr << "pwr_cal_conf_filename=<filename>   - overide default name for the power calibration configuration file\n";
    /// Here is the extract of the option descriptions:
    /// \dontinclude main.cpp
    /// \skip cerr << "help
    /// \until cerr << "antenna-mode
}


namespace handler
{
    bool quit_flag = 0; ///< set by ctrl-c signal handler

    /// Catches ctrl-c and sets quit_flag to allow program to exit gracefully
    void sigint( int sig )
    {
        std::cerr << "sig-handler. received ctrl-c\n";
        quit_flag = true;
    }
}

///////////////////////////////////////////////////////////////////////////////
/// The main program entry-point
//
int main( int argc, char *argv[] )
{
    using boost::lexical_cast;
    using boost::bad_lexical_cast;

    // set up ctrl-c handler
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sigemptyset( &sa.sa_mask );
    sa.sa_handler = handler::sigint;
    sigaction( SIGINT, &sa, NULL );

    int result = EXIT_SUCCESS;

    std::string prog_name( argv[0] );
    size_t n = prog_name.rfind('/');
    prog_name = prog_name.substr( n+1 );

    std::cout << prog_name << '\n';
    std::cout << "      built: " << __DATE__ << ", " __TIME__ << '\n';
    std::cout << "   compiler: " << COMPILE_INFO     << '\n';
    std::cout << "         on: " << HOST_MACHINE     << '\n';
    std::cout << "sw  version: " << SW_VERSION_LONG  << "\n";
    std::cout << "   git hash: " << GIT_HASH << "\n";
    std::cout << "pcb version: " << Pcb::getPcbRev() << "\n\n";

    ConfigOptions co;

    // boost::program_options is probably cleaner but requires compiled boost libs
    // how easy will it be to cross compile that?
    static struct option options[] ={
        {"help",                    no_argument,        NULL, 'h'},
        {"ref-cal-thresh-factor",   required_argument,  NULL, 1000},
        {"search-thresh-factor",    required_argument,  NULL, 1001},
        {"track-thresh-factor",     required_argument,  NULL, 1002},

        {"fpga-load-only",          no_argument,        NULL, 1004},
        {"debug",                   optional_argument,  NULL, 1005},
        {"no-auto-range",           no_argument,        NULL, 1006},
        {"vctcxo-dac",              required_argument,  NULL, 1007},
        {"port",                    required_argument,  NULL, 1008},
        {"debug-files",             no_argument,        NULL, 1010},

        {"fpga-man-load",           no_argument,        NULL, 1013},
        {"fpga-skip-load",          no_argument,        NULL, 1014},

        {"antenna-mode",            optional_argument,  NULL, 1015},

        {"pwr_cal_conf_filename",   required_argument,  NULL, 1018},

        {"fpga-4G-image",           required_argument,  NULL, 1019},
        {"fpga-3G-image",           required_argument,  NULL, 1020},
        {"fpga-GSM-image",          required_argument,  NULL, 1021},
        {"init-mode",               required_argument,  NULL, 1022},

        {"low-power",               required_argument,  NULL, 1025},

        {"searching-pause",         required_argument,  NULL, 1030},
        {"tracking-pause",          required_argument,  NULL, 1031},

        {"4G-search-thresh-factor", required_argument,  NULL, 1041},
        {"4G-track-thresh-factor",  required_argument,  NULL, 1042},

        {"4G-search-average",       required_argument,  NULL, 1043},
        {"4G-track-average",        required_argument,  NULL, 1044},

        //to do - loadsaoptions here
        {0, 0, 0, 0 }
    };

    int opt=0,index=0;
    try{
        while ((opt = getopt_long( argc, argv, "h:", options, &index )) >= 0 && result == EXIT_SUCCESS)
        {
            switch (opt)
            {

                case 1000:
                    std::cout << "ref-cal-thresh-factor = " << optarg << '\n';
                    co.ref_cal_threshold_factor_dB = lexical_cast<double>( optarg );
                    break;

                case 1001:
                    std::cout << "search-thresh-factor = " << optarg << '\n';
                    co.search_threshold_factor_dB = lexical_cast<double>( optarg );
                    break;

                case 1002:
                    std::cout << "track-thresh-factor = " << optarg << '\n';
                    co.track_threshold_factor_dB = lexical_cast<double>( optarg );
                    break;

                case 1004:
                    std::cout << "fpga-load-only\n";
                    co.load_only = true;
                    break;

                case 1005:
                    if( optarg )
                    {
                        if ((optarg[0] == '0') && ((optarg[1] == 'x') || (optarg[1] == 'X')))
                        {
                            if (sscanf(&optarg[2], "%x", &(co.debug_print)) != 1)
                            {
                                throw bad_lexical_cast();
                            }
                        }
                        else
                        {
                            co.debug_print = lexical_cast<unsigned>( optarg );
                        }
                    }
                    else
                    {
                        co.debug_print = DebugStream::all;
                    }
                    std::cout << "debug = " << co.debug_print << "\n";
                    break;

                case 1006:
                    std::cout << "no-auto-ranging\n";
                    co.ranging_mode = Ranging::FIXED_RANGE;
                    break;

                case 1007:
                    std::cout << "vctcxo-dac = ";
                    co.vctcxo_dac = lexical_cast<unsigned int>( optarg );
                    std::cout << co.vctcxo_dac <<'\n';
                    if( co.vctcxo_dac > Design::MAX_DAC_VAL )
                    {
                        std::cout << "invalid dac 0 - "<<Design::MAX_DAC_VAL<<'\n';
                        result = EXIT_FAILURE;
                    }
                    break;

                case 1008:
                    std::cout << "port = ";
                    co.port = lexical_cast<unsigned int>( optarg );
                    std::cout << co.port <<'\n';
                    break;

                case 1010:
                    std::cout << "debug_files\n";
                    co.debug_files = true;
                    break;

                case 1013:
                    std::cout << "fpga-man-load\n";
                    co.man_load = true;
                    break;

                case 1014:
                    std::cout << "fpga-skip-load\n";
                    co.skip_load = true;
                    break;

                case 1015:
                    co.antenna_mode = lexical_cast<unsigned int>( optarg );
                    if( (co.antenna_mode < 1) || (co.antenna_mode > 2) )
                    {
                        throw std::invalid_argument("antenna mode not 1 or 2");
                    }
                    std::cout << "antenna-mode = " << co.antenna_mode << "\n";
                    break;

                case 1018:
                    std::cout << "pwr_cal_conf_filename = "<<optarg<<'\n';
                    co.pwr_cal_conf_filename = std::string( optarg );
                    break;

                case 1019:
                    std::cout << "fpga-4G-image = "<<optarg<<'\n';
                    co.image_name_for_4G = std::string( optarg );
                    break;

                case 1020:
                    std::cout << "fpga-3G-image = "<<optarg<<'\n';
                    co.image_name_for_3G = std::string( optarg );
                    break;

                case 1021:
                    std::cout << "fpga-GSM-image = "<<optarg<<'\n';
                    co.image_name_for_GSM = std::string( optarg );
                    break;

                case 1022:
                    if (std::string(optarg) == "4G")
                    {
                        std::cout << "init-mode = "<<optarg<<'\n';
                        co.init_mode = MODE_4G;
                    }
                    else if (std::string(optarg) == "3G")
                    {
                        std::cout << "init-mode = "<<optarg<<'\n';
                        co.init_mode = MODE_3G;
                    }
                    else
                    if (std::string(optarg) == "GSM")
                    {
                        std::cout << "init-mode = "<<optarg<<'\n';
                        co.init_mode = MODE_GSM;
                    }
                    else
                    {
                        std::cout << "invalid initial mode: \""<<optarg<<"\"\n";
                        result = EXIT_FAILURE;
                    }
                    break;

                case 1025:
                    if (std::string(optarg) == "LOW_POWER")
                    {
                        std::cout << "low-power = "<<optarg<<'\n';
                        co.low_power = true;
                    }
                    else
                    if (std::string(optarg) == "HIGH_SENS")
                    {
                        std::cout << "low-power = "<<optarg<<'\n';
                        co.low_power = false;
                    }
                    else
                    {
                        std::cout << "invalid low-power setting: \""<<optarg<<"\"\n";
                        result = EXIT_FAILURE;
                    }
                    break;

                case 1030:
                    co.searching_pause = lexical_cast<unsigned int>( optarg );
                    std::cout << "searching-pause = " << co.searching_pause << "\n";
                    break;

                case 1031:
                    co.tracking_pause = lexical_cast<unsigned int>( optarg );
                    std::cout << "tracking-pause = " << co.tracking_pause << "\n";
                    break;

                case 1041:
                    co.search_threshold_factor_4G_dB = lexical_cast<double>( optarg );
                    std::cout << "4G-search-thresh-factor = " << co.search_threshold_factor_4G_dB << '\n';
                    break;

                case 1042:
                    co.track_threshold_factor_4G_dB = lexical_cast<double>( optarg );
                    std::cout << "4G-track-thresh-factor = " << co.track_threshold_factor_4G_dB << '\n';
                    break;

                case 1043:
                    co.search_average_4G = lexical_cast<unsigned int>( optarg );
                    //
                    //  This limit is arbitrary and could be raised, but is equivalent to 0.5 s.
                    //
                    if( 240 < co.search_average_4G )
                    {
                        throw std::invalid_argument("4G search averaging limited to 240 or less.");
                    }
                    std::cout << "4G-search-average = " << co.search_average_4G << '\n';
                    break;

                case 1044:
                    co.track_average_4G = lexical_cast<unsigned int>( optarg );
                    //
                    //  This limit is imposed by timing and is equivalent to 0.25 s.
                    //  It could be rised if Correlator_4g::CORR_START_TIME_OFFSET_RESULT_BUFFERS were raised, but abort
                    //  higher limit is the depth of the FPGA's correlation command FIFO (Fpga::MAX_NUM_CORR_CMDS_4G)
                    //  that requires an FPGA change to increase.
                    //
                    if( 120 < co.track_average_4G )
                    {
                        throw std::invalid_argument("4G search averaging limited to 120 or less.");
                    }
                    std::cout << "4G-track-average = " << co.track_average_4G << '\n';
                    break;

                case 'h':
                default:
                {
                    usage( prog_name );
                    result = EXIT_FAILURE;
                    break;
                }
            }
        }
    }catch( std::exception & e ){
        std::cerr<< "invalid parameter " << optarg << " for ";
        if (opt < 1000) std::cerr<< static_cast<char>(opt) << '\n';
        else std::cerr<< options[index].name << '\n';
        std::cerr<< e.what() <<'\n';
        result = EXIT_FAILURE;
    }

    if( result == EXIT_FAILURE ) return result;

    //
    // Now to do the work.
    //
    try
    {
        Radio::debug_printing( co.debug_print );
        DAC::debug_printing( co.debug_print );
        ADC::debug_printing( co.debug_print );

        Radio::initialise((co.init_mode == MODE_3G) || (co.init_mode == MODE_4G));  // needed to set up clocks
        DAC::set( co.vctcxo_dac );
        ADC::initialise((co.init_mode == MODE_3G) || (co.init_mode == MODE_4G));
        Fpga fpga(co.debug_print);

        if( co.man_load )
        {
            std::cout << "Load the FPGA" << std::endl;
            int a;
            std::cin >> a;
            fpga.reset_image();
        }
        else
        if( co.skip_load )
        {
            fpga.reset_image();
        }
        else
        switch (co.init_mode)
        {
            case MODE_GSM:
                fpga.load_image(co.image_name_for_GSM, co.init_mode);
                break;
            case MODE_3G:
                fpga.load_image(co.image_name_for_3G, co.init_mode);
                break;
            case MODE_4G:
                fpga.load_image(co.image_name_for_4G, co.init_mode);
                break;
            default:
                throw std::invalid_argument("co.init_mode has invalid value");
                break;
        }

        if( co.load_only )
        {
            return EXIT_SUCCESS;
        }

        //
        //  Instantiate the Physical Layer and activate it.
        //
        phy_t phy( co, fpga );
        phy.activate( handler::quit_flag );
    }
    catch ( std::exception& e )
    {
        std::cerr << "\e[31mexception... " << e.what() << '\n';
        result = EXIT_FAILURE;
        // could automatically try again setting up a new phy etc
        // if that was useful behaviour
    }
    catch ( ... )
    {
        std::cerr << "\e[31mexception... unhandled\n";
        result = EXIT_FAILURE;
    }

    std::cout << "\e[37m" << std::flush;    // make sure we go back to white text

    return (result);
}

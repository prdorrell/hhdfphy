/***********************************************************************************************************************
 *
 *
 ***********************************************************************************************************************
 *  Filename:   fpga.cpp
 *  Author(s):  pdm
 *******************************************************************************************************************//**
 *  @file
 *  @brief          The implementation of ::Fpga class that controls the FPGA.
 *  @details        The fpga uses two buffers 0/1 (even/odd) for many results.  This allows the FPGA to write
 *                  correlation results, frame counters etc to one block of memory/registers while the software reads
 *                  another.  If the software is too slow the fpga can just keep overwriting the same bank.  When the
 *                  software is ready it will request the fpga use the other buffer.  After the fpga has finished
 *                  writing the current set of results it will start using the requested buffer and the software will be
 *                  able to read a completed set of results from the other buffer.  This stops partial results and
 *                  results from different frames getting confused.
 *
 **********************************************************************************************************************/
#include "3gpp.h"
#include "fpga.hpp"
#include "debug.hpp"
#include "gpio.hpp"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include <stdexcept>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

#include <sys/mman.h>
#include <sys/time.h>
#include <fcntl.h>
#include <time.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class.
//  @details        Initialises that pointers that are used to access the memory mapped registers.  The hardware
//                  addresses are mapped by the mmap function into software addresses that are stored in the pointers.
//                  Two groups of hardware addresses are used:
//                  -   The general control registers are mapped into the region enabled by chip select 0.
//                  -   The DDR memory interface registers are mapped into the region enabled by chip select 1.
//  @param[in]      debug               Controls the debug messages that may be generated.
//
Fpga::Fpga( unsigned debug ) :
    reg_base(                 NULL),
    reg_firmware_rev_ptr(     NULL),
    reg_test_register_ptr(    NULL),
    reg_mode_ptr(             NULL),
    reg_average_ctrl_ptr(     NULL),
    reg_start_ptr(            NULL),
    reg_length_ptr(           NULL),
    reg_threshold_ptr(        NULL),
    reg_swing_buffer_ctrl_ptr(NULL),
    reg_ant_sw_ctrl_ptr(      NULL),
    reg_even_frame_count_ptr( NULL),
    reg_even_peak_address_ptr(NULL),
    reg_even_peak_data_ptr(   NULL),
    reg_even_num_peaks_ptr(   NULL),
    reg_chan_rssi_ptr(        NULL),
    reg_fsm_debug1_ptr(       NULL),
    reg_1pps_ptr(             NULL),
    reg_fft_gain_control_ptr( NULL),
    reg_fsm_debug2_ptr(       NULL),
    reg_odd_frame_count_ptr(  NULL),
    reg_odd_peak_address_ptr( NULL),
    reg_odd_peak_data_ptr(    NULL),
    reg_odd_num_peaks_ptr(    NULL),
    reg_adc_rssi_ptr(         NULL),
    mcb_read_address_ptr(     NULL),
    mcb_read_fifo_ptr(        NULL),
    mcb_read_length_ptr(      NULL),
    mcb_write_address_ptr(    NULL),
    mcb_write_fifo_ptr(       NULL),
    mcb_write_length_ptr(     NULL),
    mcb_control_ptr(          NULL),
    mcb_fifo_status_ptr(      NULL),
    coeff_base(),
    cur_mode(MODE_3G),
    frame_data_valid(false),
    num_samples_over_threshold(0),
    peak_sample_offset(        0),
    peak_pwr(                  0),
    big_frame_count_started(false),
    big_frame_count(            0),
    last_fpga_frame(            0),
    time_of_last_frame_update(   ),
    proc_waiting_for_even_buffer(true),
    start_time(),
    last_pop_time(),
    debugStr("Fpga........ ", debug)
{
    //
    //  Open the memory device so that the mapping between the register's hardware addresses and the software pointers
    //  can be configured.
    //
    int fd;
    if( (fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1 )
    {
        throw std::runtime_error("can't open /dev/mem");
    }

    //
    //  cs1 is used for ddr interface, this is accessed through imx physical memory at 0xA8000000
    //
    uint32_t* ptr = (uint32_t*) mmap( 0, 0x4000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0xA8000000 );
    if( ptr == MAP_FAILED )
    {
        close( fd );
        throw std::runtime_error( "can't mmap 0xA800 0000" );
    }

    mcb_read_address_ptr    = ptr++;
    mcb_read_fifo_ptr       = ptr++;    // A800 0004
    mcb_read_length_ptr     = ptr++;    // A800 0008
    mcb_write_address_ptr   = ptr++;    // A800 000c
    mcb_write_fifo_ptr      = ptr++;    // A800 0010
    mcb_write_length_ptr    = ptr++;    // A800 0014
    mcb_control_ptr         = ptr++;    // A800 0018
    mcb_fifo_status_ptr     = ptr++;    // A800 001c

    //
    //  cs0 is used for the general registers, which are is accessed through imx physical memory at 0xA0000000
    //
    ptr = (uint32_t*) mmap( 0, 0x80000*4, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0xA0000000 );
    if( ptr == MAP_FAILED )
    {
        close( fd );
        throw std::runtime_error( "can't mmap 0xA000 0000" );
    }

    reg_base                  = ptr;
    reg_firmware_rev_ptr      = reg_base+REG_FIRMWARE_REV_OFFSET;
    reg_test_register_ptr     = reg_base+REG_TEST_REGISTER_OFFSET;
    reg_mode_ptr              = reg_base+REG_MODE_OFFSET;
    reg_average_ctrl_ptr      = reg_base+REG_AVERAGE_CTRL_OFFSET;
    reg_start_ptr             = reg_base+REG_CORRELATION_START_OFFSET;
    reg_length_ptr            = reg_base+REG_CORRELATION_LENGTH_OFFSET;
    reg_threshold_ptr         = reg_base+REG_THRESHOLD_OFFSET;
    reg_swing_buffer_ctrl_ptr = reg_base+REG_SWING_BUFFER_CONTROL_OFFSET;
    reg_ant_sw_ctrl_ptr       = reg_base+REG_ANT_SW_CTRL_OFFSET;
    reg_even_frame_count_ptr  = reg_base+REG_EVEN_FRAME_COUNT_OFFSET;
    reg_even_peak_address_ptr = reg_base+REG_EVEN_PEAK_ADDRESS_OFFSET;
    reg_even_peak_data_ptr    = reg_base+REG_EVEN_PEAK_DATA_OFFSET;
    reg_even_num_peaks_ptr    = reg_base+REG_EVEN_NUM_PEAKS_OFFSET;
    reg_chan_rssi_ptr         = reg_base+REG_CHAN_RSSI_OFFSET;
    reg_fsm_debug1_ptr        = reg_base+REG_FSM_DEBUG1_OFFSET;
    reg_1pps_ptr              = reg_base+REG_PPS_COUNT_OFFSET;
    reg_fft_gain_control_ptr  = reg_base+REG_FFT_GAIN_CTRL_OFFSET;
    reg_fsm_debug2_ptr        = reg_base+REG_FSM_DEBUG2_OFFSET;
    reg_odd_frame_count_ptr   = reg_base+REG_ODD_FRAME_COUNT_OFFSET;
    reg_odd_peak_address_ptr  = reg_base+REG_ODD_PEAK_ADDRESS_OFFSET;
    reg_odd_peak_data_ptr     = reg_base+REG_ODD_PEAK_DATA_OFFSET;
    reg_odd_num_peaks_ptr     = reg_base+REG_ODD_NUM_PEAKS_OFFSET;
    reg_adc_rssi_ptr          = reg_base+REG_ADC_RSSI_OFFSET;
    reg_gsm_result_fifo_ptr   = reg_base+REG_GSM_RESULT_FIFO_OFFSET;
    reg_gsm_result_status_ptr = reg_base+REG_GSM_RESULT_STATUS_OFFSET;
    reg_gsm_dc_offset_ptr     = reg_base+REG_GSM_DC_OFFSET;

    for (unsigned slotNum = 0 ; slotNum < Design::MAX_CORRELATIONS_PER_FRAME ; ++slotNum)
    {
        coeff_base[slotNum] =   reg_base
                              + CORR_BASE_COEFF_OFFSET
                              + slotNum*CORR_MAX_NUM_COEFFS_PER_SLOT/CORR_NUM_COEFFS_PER_WORD32;
    }

    // 4G specific configuration
    reg_test_register_4g_ptr = reg_base + REG_TEST_REGISTER_4G_OFFSET;
    reg_antenna_select_4g_ptr = reg_base + REG_ANTENNA_SELECT_4G_OFFSET;
    reg_sample_time_ctrl_4g_ptr = reg_base + REG_SAMPLE_TIME_4G_OFFSET;
    reg_sample_time_lsw_4g_ptr =  reg_base + REG_SAMPLE_TIME_4G_OFFSET + 1;
    reg_sample_time_msw_4g_ptr =  reg_base + REG_SAMPLE_TIME_4G_OFFSET + 2;


    channel_filter_1_4g_ptr = reg_base+CHANNEL_FILTER_1_4G_OFFSET;
    channel_filter_2_4g_ptr = reg_base+CHANNEL_FILTER_2_4G_OFFSET;

    reg_float_format_test_4g_ptr = reg_base+REG_FLOAT_FORMAT_TEST_4G_OFFSET;
    reg_correlator_threshold_sqrd_4g_ptr = reg_base+CORRELATOR_PWR_MULTIPLICATION_4G_OFFSET;
    reg_correlator_cmd_queue_4g_ptr = reg_base+CORRELATOR_CMD_QUEUE_4G_OFFSET;
    reg_correlator_cmd_queue_start_4g_ptr = (uint64_t *)(reg_base + CORRELATOR_CMD_START_TIME_4G_OFFSET);
    reg_correlator_cmd_queue_stop_4g_ptr = (uint64_t *)(reg_base + CORRELATOR_CMD_STOP_TIME_4G_OFFSET);
    // Information about Correlator command queue, information on command status FIFO
    // and the status FIFO itself
    reg_correlator_cmd_queue_status_ptr = reg_base + CORRELATOR_CMD_QUEUE_STATUS_4G_OFFSET;
    reg_correlator_cmd_status_info_ptr = reg_base + CORRELATOR_CMD_STATUS_INFO_4G_OFFSET;
    reg_correlator_cmd_status_fifo_ptr = reg_base + CORRELATOR_CMD_STATUS_FIFO_4G_OFFSET;

    reg_correlation_result_info_ptr = reg_base + CORRELATION_RESULT_INFO_4G_OFFSET;
    reg_correlation_result_fifo_ls_time_ptr = reg_base + CORRELATION_RESULT_FIFO_BASE_4G_OFFSET;
    reg_correlation_result_fifo_ms_time_ptr = reg_base + CORRELATION_RESULT_FIFO_BASE_4G_OFFSET + 1;
    reg_correlation_result_fifo_correl_num_ptr = reg_base + CORRELATION_RESULT_FIFO_BASE_4G_OFFSET + 2;
    reg_correlation_result_fifo_correl_den_ptr = reg_base + CORRELATION_RESULT_FIFO_BASE_4G_OFFSET + 3;
    reg_correlation_result_fifo_sum_pwr_ptr = reg_base + CORRELATION_RESULT_FIFO_BASE_4G_OFFSET + 4;

    //Mean statistics block contains 4 separate values offset from a starting base address
    reg_correlation_statistics_control_ptr = reg_base + CORRELATION_MEAN_STATISTICS_BASE_4G_OFFSET;
    reg_correlation_statistics_num_means_ptr = reg_base + CORRELATION_MEAN_STATISTICS_BASE_4G_OFFSET + 1;
    reg_correlation_statistics_peak_power_ptr = reg_base + CORRELATION_MEAN_STATISTICS_BASE_4G_OFFSET + 2;
    reg_correlation_statistics_sum_power_ptr = reg_base + CORRELATION_MEAN_STATISTICS_BASE_4G_OFFSET + 3;

    for (uint32_t cortaps = 0; cortaps < NUM_CORRELATOR_TAPS_4G; ++cortaps)
    {
        correlator_taps_4G[cortaps] = (volatile uint32_t *)(reg_base
                                    + CORRELATOR_BASE_4G_OFFSET + cortaps);
    }

    reg_regression_test_ctrl_4g_ptr = reg_base + REG_REGRESSION_TEST_CONTROL_4G_OFFSET;
    reg_regression_test_value_4g_ptr = reg_base + REG_REGRESSION_TEST_VALUE_4G_OFFSET;

    //
    //  Close the memory device.  The address mappings remain until the process is closed.
    //
    close( fd );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          The destructor has nothing to do.
//
Fpga::~Fpga()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Reverse the order of the bits in a byte.
//  @param[in,out]  b               The byte to be modified.
//
void Fpga::swap_bits( uint8_t& b )
{
    b = ( ( b & 0xf0 ) >> 4 ) | ( ( b & 0x0f ) << 4 );
    b = ( ( b & 0xcc ) >> 2 ) | ( ( b & 0x33 ) << 2 );
    b = ( ( b & 0xaa ) >> 1 ) | ( ( b & 0x55 ) << 1 );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Load the FPGA image using the WEIM bus.
//  @details        The WEIM bus is mapped into the chip-select 3 address space so the function first maps this onto
//                  a range of corresponding software addresses.  It then sets the FPGA image reset line so that the
//                  non-essential essential elements in the image are held in reset even after it has been loaded.
//
//                  The programming of the image is started when the function clears the PROGRAM_B GPIO pin, which
//                  resets the FPGA device.  It then forces the configuration lines to be inputs by clearing the RDWR_B
//                  GPIO pin, before bringing the FPGA device out of reset by setting the PROGRAM_B GPIO pin.
//
//                  The function waits for the INIT_B GPIO pin to be driven high, a signal the the FPGA is ready to be
//                  programmed.  It then writes the image, adjusting the bit and byte ordering as necessary and when it
//                  is finished it clears FPGA image reset line.
//
//                  Finally the fuunction waits for the DONE GPIO pin to be driven high, a signal that the FPGA has been
//                  configured.  It then reads the status of the CRC and reports its value (the software used to abort
//                  if the CRC failed, but experience has shown that the signal is unreliable) and unmaps the WEIM bus.
//
//  @param[in]      fin                 The file stream to get data from.
//  @param[in]      swap                true if the bit-ordering in the bytes needs to be reversed, false otherwise.
//  @param[in]      bigend              true if the bytes in half words are in big-endian order, false otherwise.
//  @param[in]      mode                GSM, 3G or 4G?
//
void Fpga::send_config_weim(std::ifstream& fin, bool swap, bool bigend, eSystemMode mode)
{
    using namespace std;
    int i = 0;

    // cs3 connected to csi_b
    // writes to physical 0xB200 0000 - 0xB3ff ffff are on cs3
    // we map one word of this physical address range into user logical memory space
    // writes to this will then trigger cs3, which will sequencially load config words
    int fd;
    if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
    {
        throw runtime_error("can't open /dev/mem");
    }

    // can we map just the halfword we need or does it have to be a whole page?
    // I think offset needs to be page aligned
    // void * mmap (void *address, size_t length,int protect, int flags, int filedes, off_t offset)
    uint16_t* ptr = (uint16_t*) mmap( 0, 2, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0xB2000000 );
    if( ptr == MAP_FAILED )
    {
        throw runtime_error( "can't mmap 0xB200 0000" );
    }

    //
    //
    //
    gpio image_reset( gpio::FPGA_IMG_RST,   gpio::out_high,   debugStr.get_state() ); // hold high = image reset
    gpio suspend    ( gpio::FPGA_SUSPEND,   gpio::out_low,    debugStr.get_state() ); // hold low = not in suspend state
    gpio programb   ( gpio::FPGA_PROGRAM_B, gpio::out_low,    debugStr.get_state() ); // hold low = device reset
    gpio initb      ( gpio::FPGA_INIT_B,    gpio::in,         debugStr.get_state() ); // read 1 = crc ok
    gpio done       ( gpio::FPGA_DONE,      gpio::in,         debugStr.get_state() ); // read 1 = config ok
    gpio wdrtb      ( gpio::FPGA_RDWR_B,    gpio::out_low,    debugStr.get_state() ); // make fpga config lines inputs

#ifdef WAIT_FOR_MCB_AND_CLOCKS
    gpio mcb_cal_done( gpio::FPGA_MCB_CAL_DONE,    gpio::in,         debugStr.get_state() ); // read 1 = MCB calibration is complete
    gpio clk_lock_192( gpio::FPGA_192_CLK_LOCK,    gpio::in,         debugStr.get_state() ); // read 1 = 192 MHz clock is locked
    gpio clk_lock_195( gpio::FPGA_195_CLK_LOCK,    gpio::in,         debugStr.get_state() ); // read 1 = 195 MHz clock is locked
#endif

    programb.set( 1 );     // pull device out of reset

    // wait for initb to come high
    i = 0;
    while( !initb.get() )
    {
        if( i > 1000 ) throw runtime_error( "time out waiting for initb to go high");
        ++i;
        timespec t;
        t.tv_sec = 0;
        t.tv_nsec = 1000000; // 1ms
        nanosleep( &t, &t );
    }

    // get file length
    int len = fin.seekg( 0, ios_base::end ).tellg();
    fin.seekg( 0, ios_base::beg );

    i = 0;
    int p = -1;
    debugStr(DebugStream::info4) << "loading... \n";
    cout << "\e[32m\e[1F\e[24G\e[s" << flush;

    while( fin.good() )
    {
        if( debugStr.get_state() && p != 100*i/len )
        {
            // % counter
            p = 100*i/len;
            cout << "\e[u\e[s" << p << '%' << flush;
        }
        uint8_t bytes[2];
        fin.read( reinterpret_cast<char*>(bytes), 2 );
        if( fin.gcount() == 0 ) break;
        if( fin.gcount() == 1 ) bytes[1] = 0;

        if( swap )
        {
            swap_bits( bytes[0] );
            swap_bits( bytes[1] );
        }

        uint16_t hword;
        if( bigend )
        {
            hword = bytes[0];
            hword = (hword << 8) | bytes[1];
        }
        else
        {
            hword = bytes[1];
            hword = (hword << 8) | bytes[0];
        }

        *ptr = hword;
        i += 2;
    }

    cout << "\e[39m\n";

    //
    //  Allow the image to run
    //
    image_reset.set( 0 );

    //
    // Wait until all initialisation is complete
    //
    for( int j=0; j<10; ++j )
    {
        usleep(10000);
        if(   done.get()
#ifdef WAIT_FOR_MCB_AND_CLOCKS
           && mcb_cal_done.get() && (!image_3G || clk_lock_192.get()) && clk_lock_195.get()
#endif
           )
        {
            break;
        }

        // just in case it wants some more junk at end to finish it off
        for( i=0; i<32; ++i )
        {
            *ptr = 0xffff;
        }

        debugStr(DebugStream::warn1) << "done still low " << j << "\n";
    }

    string msg="";
    // check config done flag
    if( !done.get() )
    {
        msg += "done line low; ";
    }
#ifdef WAIT_FOR_MCB_AND_CLOCKS
    // check MCB calibration flag
    if( !mcb_cal_done.get() )
    {
        msg += "MCB calibration line low; ";
    }
    // check 192 MHz clock lock flag
    if( mode != MODE_GSM && !clk_lock_192.get() )
    {
        msg += "192 MHz clock lock line low; ";
    }
    // check 195 MHz clock lock flag
    if( !clk_lock_195.get() )
    {
        msg += "195 MHz clock lock line low; ";
    }
#endif
    if( msg != "" )
    {
        throw runtime_error( "error configuring fpga: " + msg );
    }

    if( munmap( ptr, 2 ) == -1 )
    {
        throw runtime_error( "failed to unmap cs3 address space");
    }
    close(fd);

    // report the crc error flag status
    if( !initb.get() )
    {
        debugStr(DebugStream::warn1) << "FPGA CRC failure\n";
    }

    // success
    cur_mode = mode;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Load the FPGA image.
//  @details        Automatically detects whether the image uses big or little endian bytes in halfwords and bit-
//                  reversal in each byte based on the sync words that should be at the begining of the file.
//  @param[in]      filename            The name of the ".bin" format image file.
//  @param[in]      mode                GSM, 3G or 4G?
//
void Fpga::load_image(std::string filename, eSystemMode mode)
{
    using namespace std;
    debugStr(DebugStream::info4) << "loading image " << filename << '\n';
    ifstream f;
    f.open( filename.c_str(), ios::binary );

    if( !f.good() )
    {
        throw runtime_error("error loading " + filename );
    }

    bool bigend;
    bool swap;
    // have a go at detecting the header
    // and determining endianess and bit swap based on sync pattern
    const int header_size = 100;    // padding appears to be 16 bytes and
                                    // sync is 4 so this is should be plenty
    uint8_t bytes[header_size];
    f.read( reinterpret_cast<char*>(bytes), header_size );
    int i = 0;
    while( i < header_size && bytes[i] == 0xff ) ++i;   // skip padding
    if( i <= (header_size-4) && f.good() )
    {
        if( (bytes[i]   == 0x99) && (bytes[i+1] == 0xaa) &&
            (bytes[i+2] == 0x66) && (bytes[i+3] == 0x55) )
        {
            debugStr(DebugStream::info1) << "little endian, needs bit swap\n";
            bigend  = false;
            swap    = true;
        }
        else if( (bytes[i]   == 0x99) && (bytes[i+1] == 0x55) &&
                 (bytes[i+2] == 0x66) && (bytes[i+3] == 0xaa) )
        {
            debugStr(DebugStream::info1) << "little endian, already bit swapped\n";
            bigend  = false;
            swap    = false;
        }
        else if( (bytes[i]   == 0xaa) && (bytes[i+1] == 0x99) &&
                 (bytes[i+2] == 0x55) && (bytes[i+3] == 0x66) )
        {
            debugStr(DebugStream::info1) << "big endian, needs bit swap\n";
            bigend  = true;
            swap    = true;
        }
        else if( (bytes[i]   == 0x55) && (bytes[i+1] == 0x99) &&
                 (bytes[i+2] == 0xaa) && (bytes[i+3] == 0x66) )
        {
            debugStr(DebugStream::info1) << "big endian, already bit swapped\n";
            bigend  = true;
            swap    = false;
        }
        else
        {
            throw runtime_error("unrecognised image format");
        }
    }
    else
    {
        throw runtime_error("unrecognised image format");
    }

    // do the load
    send_config_weim( f, swap, bigend, mode );

    f.close();

    debugStr(DebugStream::info4) << "done\n";
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Reset the FPGA image.
//
void Fpga::reset_image(void)
{
    gpio image_reset( gpio::FPGA_IMG_RST,  gpio::out_high,   debugStr.get_state() );  // high = image reset

    gpio mcb_cal_done( gpio::FPGA_MCB_CAL_DONE,    gpio::in,         debugStr.get_state() ); // read 1 = MCB calibration is complete
    gpio clk_lock_192( gpio::FPGA_192_CLK_LOCK,    gpio::in,         debugStr.get_state() ); // read 1 = 192 MHz clock is locked
    gpio clk_lock_195( gpio::FPGA_195_CLK_LOCK,    gpio::in,         debugStr.get_state() ); // read 1 = 195 MHz clock is locked

    image_reset.set( 0 );                                                             // allow the image to run

    //
    // Wait until all reset is complete
    //
    for( int j=0; j<10; ++j )
    {
        usleep(10000);
        if( mcb_cal_done.get() && (cur_mode == MODE_GSM || clk_lock_192.get()) && clk_lock_195.get() )
        {
            break;
        }

        debugStr(DebugStream::warn1) << "done still low " << j << "\n";
    }

    std::string msg="";
    // check MCB calibration flag
    if( !mcb_cal_done.get() )
    {
        msg += "MCB calibration line low; ";
    }
    // check 192 MHz clock lock flag
    if( cur_mode != MODE_GSM && !clk_lock_192.get() )
    {
        msg += "192 MHz clock lock line low; ";
    }
    // check 195 MHz clock lock flag
    if( !clk_lock_195.get() )
    {
        msg += "195 MHz clock lock line low; ";
    }

    if( msg != "" )
    {
        throw std::runtime_error( "error reseting fpga: " + msg );
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Read from the DDR memory.
//  @details        The memory is read by requesting the data from the FPGA's Memory Controller Block (MCB) in batches
//                  of up to 64 words.  The MCB writes the words into a FIFO from where the software may read it.
//
//                  The function first checks that the MCB is ready to process a new read command and that no errors
//                  have occurred.  If there is a problem it asserts an error, otherwise it begins the read process.
//
//                  While more the 64 words are required the read loop requests 64 words and waits for the FIFO to be
//                  full before reading the new words.  If on the final iteration fewer than 64 words are needed then
//                  FIFO is read while it is not empty.
//
//  @param[in]      address             The start address.
//  @param[in]      length              The number of 32-bit words to read.
//  @param[out]     data                The destination for the data.
//
void Fpga::mem_raw_read( uint32_t address, uint32_t length, std::vector<uint32_t>& data )
{
    using namespace std;

    //
    // check for fifo errors
    //
    if( *mcb_fifo_status_ptr & (MCB_WR_ERROR|MCB_RD_ERROR|MCB_WR_UNDERRUN|MCB_RD_OVERFLOW) )
    {
        throw runtime_error( "mcb fifo w/r error" );
    }

    //
    // check last read was sent (indicates error if not)
    //
    if( *mcb_control_ptr & MCB_READ_CMD )
    {
        throw runtime_error( "previous command not accepted" );
    }

    //
    // check fifo empty (indicates an error in sw logic if not)
    //
    if( !(*mcb_fifo_status_ptr & MCB_RD_EMPTY) )
    {
        throw runtime_error( "read fifo not empty" );
    }

    //
    // read the memory in blocks of up to 64 words
    //
    size_t const retry = 100000;
    size_t i;

    while( length > 0 )
    {
        *mcb_read_address_ptr = address;            // address to request data from

        unsigned words_to_get = length;
        if( words_to_get > 64 )
        {
            words_to_get = 64;
        }

        length -= words_to_get;
        address += 64;                              // address measured in 32 bit words
        *mcb_read_length_ptr = words_to_get-1;      // number of words to request (max 64)

        *mcb_control_ptr = MCB_READ_CMD;

        //
        // wait for command to be accepted
        //
        for( i = 0; i < retry; ++i )
        {
            if( !(*mcb_control_ptr & MCB_READ_CMD)  )
            {
                break;
            }
        }

        if( i == retry )
        {
            throw runtime_error( "gave up waiting for ddr read cmd to be accepted");
        }

        if ( words_to_get == 64 )
        {
            //
            // wait until the FIFO is full and then empty it
            //
            uint32_t status;
            for( i = 0; i < retry; ++i )
            {
                status = *mcb_fifo_status_ptr;
                if( status & (MCB_WR_ERROR | MCB_RD_ERROR | MCB_WR_UNDERRUN | MCB_RD_OVERFLOW) )
                {
                    throw runtime_error( "mcb fifo error while waiting to read data" );
                }
                if( status & MCB_RD_FULL )
                {
                    break;
                }
            }
            if( i == retry )
            {
                throw runtime_error( "gave up wainting for ddr read fifo to fill");
            }

            while( words_to_get > 0 )
            {
                data.push_back( (uint32_t)*mcb_read_fifo_ptr );
                --words_to_get;
            }
        }
        else
        {
            //
            // read individual words from the FIFO
            //
            while( words_to_get > 0 )
            {
                uint32_t status;
                for( i = 0; i < retry; ++i )
                {
                    status = *mcb_fifo_status_ptr;
                    if( status & (MCB_WR_ERROR | MCB_RD_ERROR | MCB_WR_UNDERRUN | MCB_RD_OVERFLOW) )
                    {
                        throw runtime_error( "mcb fifo error while waiting to read data" );
                    }
                    if( !(status & MCB_RD_EMPTY) )
                    {
                        break;
                    }
                }
                if( i == retry )
                {
                    throw runtime_error( "gave up wainting for ddr read fifo to fill");
                }

                data.push_back( (uint32_t)*mcb_read_fifo_ptr );
                --words_to_get;
            }
        }
        if( !(*mcb_fifo_status_ptr & MCB_RD_EMPTY) )
        {
            throw runtime_error("fifo not empty after read");
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Write to the DDR memory.
//  @details        The memory is written to by requesting FPGA's Memory Controller Block (MCB) to write it in batches
//                  of up to 64 words.  The software writes the words into a FIFO from where the MCB reads it.
//
//                  The function first checks that the MCB is ready to process a new write command and that no errors
//                  have occurred.  If there is a problem it asserts an error, otherwise it begins the write process.
//
//                  The function writes the data into the FIFO until it is full orthere are no more words to write.  It
//                  then requests MCB to carry out the write operation and waits for the FIFO to be emptied.
//
//  @param[in]      address             The start address.
//  @param[in]      data                The data to be written.
//
void Fpga::mem_raw_write( uint32_t address, std::vector<uint32_t> const & data )
{
    using namespace std;

    // check for fifo errors
    if( *mcb_fifo_status_ptr & (MCB_WR_ERROR|MCB_RD_ERROR|MCB_WR_UNDERRUN|MCB_RD_OVERFLOW) )
        throw runtime_error( "mcb fifo w/r error" );
    // check last write was sent (indicates error if not)
    if( *mcb_control_ptr & MCB_WRITE_CMD ) throw runtime_error( "previous command not accepted" );
    // check fifo empty (indicates an error in sw logic if not)
    if( !(*mcb_fifo_status_ptr & MCB_WR_EMPTY) ) throw runtime_error( "write fifo not empty" );

    size_t length = data.size();
    vector<uint32_t>::const_iterator it = data.begin();
    size_t const retry = 100000;
    size_t i;
    while( length > 0 )
    {
        *mcb_write_address_ptr = address;           // address to write to

        unsigned words_to_send = length;
        if( words_to_send > 64 ) words_to_send = 64;
        length -= words_to_send;
        address += 64;                              // address measured 32 bit words
        *mcb_write_length_ptr = words_to_send-1;    // number of words to request (max 64)

        // copy the words
        while( words_to_send > 0 )
        {
            *mcb_write_fifo_ptr = *it;
            --words_to_send;
            ++it;
        }

        *mcb_control_ptr = MCB_WRITE_CMD;

        // wait for command to be accepted
        for( i = 0; i < retry; ++i )
        {
            if( !(*mcb_control_ptr & MCB_WRITE_CMD) ) break;
        }
        if( i == retry ) throw runtime_error( "ddr write cmd not accepted" );

        // wait for fifo to empty
        for( i = 0; i < retry; ++i )
        {
            if( *mcb_fifo_status_ptr & MCB_WR_EMPTY ) break;
        }
        if( i == retry ) throw runtime_error( "ddr write fifo didn't empty" );
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Write to an FPGA register in chip select 0 address space.
//  @param[in]      offset              The offset of the register from the base address.
//  @param[in]      data                The destination for the data.
//
void Fpga::debug_reg_write(uint32_t offset, uint32_t data)
{
    *(reg_base+offset) = data;
    debugStr(DebugStream::info2) << "write " << ((void *)(reg_base+offset)) << " = " << data << "\n";
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Read from an FPGA register in chip select 0 address space.
//  @param[in]      offset              The offset of the register from the base address.
//  @return                             The register value.
//
uint32_t Fpga::debug_reg_read(uint32_t offset)
{
    return *(reg_base+offset);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Report whether or not the FPGA has finished using the even buffer.
//  @return         true if the FPGA has finished using the even buffer.
//
bool Fpga::fpga_finished_with_even_buffer(void)
{
    uint32_t swing_buffer_ctrl_reg = *reg_swing_buffer_ctrl_ptr;

    debugStr(DebugStream::info1) << "swing buffer status = " << swing_buffer_ctrl_reg
                                 << ", fsm1 = 0x" << std::hex << *reg_fsm_debug1_ptr << std::dec
                                 << ", fsm2 = 0x" << std::hex << *reg_fsm_debug2_ptr << std::dec
                                 << "\n";

    if ((swing_buffer_ctrl_reg & FFT_OVERFLOW) != 0)
    {
        debugStr(DebugStream::error) << boost::format(  "FFT overflow error : swing buffer status = 0x%04x, , fsm1 = 0x%08x, fsm2 = 0x%08x\n")
                                                      % swing_buffer_ctrl_reg
                                                      % *reg_fsm_debug1_ptr
                                                      % *reg_fsm_debug2_ptr;
    }

    return ((swing_buffer_ctrl_reg & EVEN_BUFFER_VALID) != 0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Report whether or not the FPGA has finished using the odd buffer.
//  @return         true if the FPGA has finished using the odd buffer.
//
bool Fpga::fpga_finished_with_odd_buffer(void)
{
    uint32_t swing_buffer_ctrl_reg = *reg_swing_buffer_ctrl_ptr;

    debugStr(DebugStream::info1) << "swing buffer status = " << swing_buffer_ctrl_reg
                                 << ", fsm1 = 0x" << std::hex << *reg_fsm_debug1_ptr << std::dec
                                 << ", fsm2 = 0x" << std::hex << *reg_fsm_debug2_ptr << std::dec
                                 << "\n";

    if ((swing_buffer_ctrl_reg & FFT_OVERFLOW) != 0)
    {
        debugStr(DebugStream::error) << boost::format(  "FFT overflow error : swing buffer status = 0x%04x, , fsm1 = 0x%08x, fsm2 = 0x%08x\n")
                                                      % swing_buffer_ctrl_reg
                                                      % *reg_fsm_debug1_ptr
                                                      % *reg_fsm_debug2_ptr;
    }

    return ((swing_buffer_ctrl_reg & ODD_BUFFER_VALID) != 0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Set the controls that indicate that the FPGA may use the even buffer.
//
void Fpga::fpga_may_use_even_buffer(void)
{
    uint32_t swing_buffer_ctrl_reg = PROC_FINISHED_EVEN_BUFFER;

    debugStr(DebugStream::info1) << "swing buffer control = " << swing_buffer_ctrl_reg << " (FPGA gets even buffer)\n";

    *reg_swing_buffer_ctrl_ptr = swing_buffer_ctrl_reg;
    debugStr(DebugStream::info2) << "write " << ((void *)(reg_swing_buffer_ctrl_ptr)) << " = " << swing_buffer_ctrl_reg << "\n";
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Set the controls that indicate that the FPGA may use the odd buffer.
//
void Fpga::fpga_may_use_odd_buffer(void)
{
    uint32_t swing_buffer_ctrl_reg = PROC_FINISHED_ODD_BUFFER;

    debugStr(DebugStream::info1) << "swing buffer control = " << swing_buffer_ctrl_reg << " (FPGA gets odd buffer)\n";

    *reg_swing_buffer_ctrl_ptr = swing_buffer_ctrl_reg;
    debugStr(DebugStream::info2) << "write " << ((void *)(reg_swing_buffer_ctrl_ptr)) << " = " << swing_buffer_ctrl_reg << "\n";
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Reads the current channel power result.
//  @details        The channel power is measured at the output of the channl filter and is in units of channel filter
//                  LSB^2.
//  @return                             The channel power.
//
double Fpga::get_channel_power( void )
{
    uint32_t p = *reg_chan_rssi_ptr;

    //
    //  Apply the correction to the power measurement.
    //
    double pwr = CHAN_RSSI_CORRECTION_3G*static_cast<double>(p);

    debugStr(DebugStream::info3) << "Channel RSSI power = " << pwr  << " (" << p << ")\n";

    return (pwr);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Reads the current ADC power result.
//  @details        The ADC power is measured at the output of the ADC and is in units of ADC LSB^2.
//  @return                             The ADC power.
//
double Fpga::get_adc_power( void )
{
    uint32_t p = *reg_adc_rssi_ptr;

    //
    //  Apply the correction to the power measurement.
    //
    double pwr = static_cast<double>(p);
    switch (cur_mode)
    {
    case MODE_GSM:
        pwr *= ADC_RSSI_CORRECTION_GSM;
        break;
    case MODE_3G:
        pwr *= ADC_RSSI_CORRECTION_3G;
        break;
    case MODE_4G:   //  Not supported by the 4G FPGA.
	default:
		assert(false);
		break;
    }

    debugStr(DebugStream::info3) << "ADC RSSI power = " << pwr << " (" << p << ")\n";

    return (pwr);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Writes the correlator coefficients to the FPGA.
//  @details        Converts the 32-bit coefficient components into 8-bit values, packs them into a 32-bit words and
//                  writes them to the FPGA coefficient memory.
//  @param[in]      data                Vector of complex integer values to load
//  @param[in]      slot                Slot number to send coeffients to
//
void Fpga::send_coefs(const std::vector<std::complex<int32_t> > &data, int slot)
{
    typedef std::vector<std::complex<int32_t> > v_t;

    int32_t minre=0, minim=0, maxre=0, maxim=0;
    volatile uint32_t * coef_ptr = coeff_base[slot];
    for( v_t::const_iterator it = data.begin(); it != data.end(); /* increments in the loop */ )
    {
        int8_t re0 = it->real();
        int8_t im0 = it->imag();
        it++;

        int8_t re1 = it->real();
        int8_t im1 = it->imag();
        it++;

        uint32_t word =   ( static_cast<uint8_t>( im1 ) << 24 )
                        | ( static_cast<uint8_t>( re1 ) << 16 )
                        | ( static_cast<uint8_t>( im0 ) <<  8 )
                        | ( static_cast<uint8_t>( re0 ) <<  0 );

        *coef_ptr = word;
        ++coef_ptr;

        if( re0 < minre ) minre = re0;
        if( im0 < minim ) minim = im0;
        if( re0 > maxre ) maxre = re0;
        if( im0 > maxim ) maxim = im0;

        if( re1 < minre ) minre = re1;
        if( im1 < minim ) minim = im1;
        if( re1 > maxre ) maxre = re1;
        if( im1 > maxim ) maxim = im1;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Configure the correlator engine.
//  @details        Reset the engine and then leave it idle while writing the setup to the control registers.  Reset the
//                  pending buffer flag, start the engine, record the time and mark the frame data as being invalid.
//  @param[in]      mode                Mode bits controlling size/type of correlation.
//  @param[in]      fft_gains           The FFT gain schedule.
//  @param[in]      start               Offset in samples from fpga clock frame/sample counter.
//  @param[in]      slots               Number of slots per frame to correlate and add non-coherently.
//  @param[in]      frames              Number of frames to correlate and add non-coherently.
//  @param[in]      threshold           For correlation peak detector.
//
void Fpga::setup_correlation(uint32_t mode,
                             uint32_t fft_gains,
                             uint32_t start,
                             uint32_t slots,
                             uint32_t frames,
                             uint32_t threshold)
{
    using namespace std;

    //
    //  Reset the correlator and then leave it idle while the controls are setup.
    //
    *reg_mode_ptr = GEN_MODE_CORR_RESET;
    debugStr(DebugStream::info2) << "write " << ((void *)(reg_mode_ptr)) << " = " << GEN_MODE_CORR_RESET << "\n";

    *reg_mode_ptr = OP_MODE_CORR_IDLE;
    debugStr(DebugStream::info2) << "write " << ((void *)(reg_mode_ptr)) << " = " << OP_MODE_CORR_IDLE << "\n";

    //
    //  Define the start and length of the correlation in a frame or slot.
    //
    *reg_start_ptr  = start;
    debugStr(DebugStream::info2) << "write " << ((void *)(reg_start_ptr)) << " = " << start << "\n";

    //
    //  Define the averaging.
    //
    uint32_t average_ctrl = ((slots-1) << 8) | (frames-1);

    *reg_average_ctrl_ptr = average_ctrl;
    debugStr(DebugStream::info2) << "write " << ((void *)(reg_average_ctrl_ptr)) << " = " << average_ctrl << "\n";

    //
    //  Define the peak detection threshold.
    //
    set_threshold(threshold);

    //
    //  Setup the FFT/IFFT gains.
    //
    *reg_fft_gain_control_ptr = fft_gains;
    debugStr(DebugStream::info2) << "write " << ((void *)(reg_fft_gain_control_ptr)) << " = " << fft_gains << "\n";

    //
    //  Reset the test register
    *reg_test_register_ptr = 0;
    debugStr(DebugStream::info2) << "write " << ((void *)(reg_test_register_ptr)) << " = " << 0 << "\n";

    //
    //  Read and display the state machine status
    debugStr(DebugStream::info1) <<   "fsm1 = 0x" << std::hex << *reg_fsm_debug1_ptr << std::dec
                                 << ", fsm2 = 0x" << std::hex << *reg_fsm_debug2_ptr << std::dec
                                 << "\n";

    //
    //  The even buffer is the first one to be filled by the FPGA.
    //
    proc_waiting_for_even_buffer = true;

    //
    //  Start the correlator.
    //
    uint32_t mode_reg = GEN_MODE_CHAN_FILT_ENABLE | GEN_MODE_CORR_START;

#ifndef IQ_RESULTS
    mode_reg = mode_reg | GEN_MODE_CORR_PWR_RESULT;
#endif

    mode_reg = mode_reg | mode;

    *reg_mode_ptr = mode_reg;
    debugStr(DebugStream::info2) << "write " << ((void *)(reg_mode_ptr)) << " = " << mode_reg << "\n";

    //
    //  Set the start time.
    //
    gettimeofday(&start_time, NULL);
    last_pop_time = start_time;

    //
    //  Mark the frame data as invalid.
    //
    frame_data_valid = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Determines if a new correlator is available.
//  @details        Calculate the time for which the FPGA has been working on this results so that a timeout can be
//                  declared if necessary.  Check the engine's result status to determine if the expected result is
//                  available and if not check if a timeout should be asserted.
//  @return                             The status of the new result, available, pending or timeout.
//
Fpga::result_avail_t Fpga::result_available(void)
{
    result_avail_t ret_val = RESULT_PENDING;

    //
    //  Determine how long the FPGA has been busy so that a timeout can be implemented.
    //
    struct timeval curr_time;
    gettimeofday(&curr_time, NULL);

    double milli_secs_since_start =   double(curr_time.tv_sec -start_time.tv_sec )*1e3
                                    + double(curr_time.tv_usec-start_time.tv_usec)/1e3;

    double milli_secs_since_last_pop  =   double(curr_time.tv_sec -last_pop_time.tv_sec )*1e3
                                        + double(curr_time.tv_usec-last_pop_time.tv_usec)/1e3;

    debugStr(DebugStream::info1) << boost::format(  "Time since test started = %10.3f, since buffer last released = %10.3f\n")
                                                  % milli_secs_since_start
                                                  % milli_secs_since_last_pop;

    //
    //  Check if the appropriate buffer has been filled with results and read the associated data.
    //
#ifdef ONLY_USE_EVEN_BUFFERS
    if (fpga_finished_with_even_buffer() && fpga_finished_with_odd_buffer())
    {
        read_even_frame_data();
        ret_val = RESULT_AVAILABLE;
    }
#else
    if (proc_waiting_for_even_buffer)
    {
        if (fpga_finished_with_even_buffer())
        {
            read_even_frame_data();
            ret_val = RESULT_AVAILABLE;
        }
    }
    else
    {
        if (fpga_finished_with_odd_buffer())
        {
            read_odd_frame_data();
            ret_val = RESULT_AVAILABLE;
        }
    }
#endif

    //
    //  Check if a timeout should be declared.
    //
    if ((ret_val != RESULT_AVAILABLE) && (milli_secs_since_last_pop > RESULT_TIMEOUT_VAL_MS))
    {
        ret_val = RESULT_TIMEOUT;
        debugStr(DebugStream::error) << boost::format(  "Aborted: time since test started = %10.3f, since buffer last released = %10.3f\n")
                                                      % milli_secs_since_start
                                                      % milli_secs_since_last_pop;

#ifdef EXIT_ON_LOCKUP
        exit(1);
#endif
    }

    return ret_val;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Releases the current result buffer back to the FPGA.
//  @details        Determine which result buffer is currently being used by the software, release it back to the FPGA
//                  and record the fact that the software must now wait for it to be refilled before using it again.
//                  Also store the current time so that the time taken to produce the next result can be checked and
//                  mark the frame data as being invalid.
//
void Fpga::pop_result( void )
{
#ifdef ONLY_USE_EVEN_BUFFERS
    fpga_may_use_even_buffer();
    fpga_may_use_odd_buffer();
#else
    if (proc_waiting_for_even_buffer)
    {
        fpga_may_use_even_buffer();
    }
    else
    {
        fpga_may_use_odd_buffer();
    }

    proc_waiting_for_even_buffer = !proc_waiting_for_even_buffer;
#endif

    //
    //  Store the time when the last buffer was released to the FPGA.
    //
    gettimeofday(&last_pop_time, NULL);

    //
    //  Mark the frame data as invalid.
    //
    frame_data_valid = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Update the 64-bit frame counter.
//  @details        The FPGA's frame counter is a 16-bit value that wraps every 11 minutes or so.  This function
//                  maintains the software' 64-bit frame counter.
//
//                  It first checks the overrun and underrun bits that are mapped into the 32-bit frame word.  If either
//                  of these is set it indicates an FPGA problem and the software issues an error message and clears
//                  them both.
//
//                  If the 64-bit counter is being reset the function simply sets 64-bit counter to the FPGA value,
//                  stores the FPGA value for use the next time that the function is called, records the time and clears
//                  the reset flag.
//
//                  Otherwise it attempts to determine the amount by which the 64-bit counter should be incremented.  It
//                  uses the difference between the current and last FPGA frame count and the time difference between
//                  the two function calls.  The time difference is used to estimate the number of counter wraps that
//                  might have occurred and this is added to the difference in the FPGA values to ensure a positive
//                  increment.
//
//                  (A bug in the FPGA used to result in the frame counter decrementing and this is detected as a
//                  negative increment after the elapsed time has been taken into account - the problem was fixed in
//                  build 23 of the FPGA image but the detection is left the code - it issues an error message and
//                  attempts to recover.)
//
//  @param[in]      frame               The 16-bit FPGA frame count.
//
void Fpga::update_big_frame_count( uint32_t frame )
{
    //
    //  Check the over/underrun bit of the frame number.
    //
    if (   ((frame & FPGA_FRAME_FIFO_EMPTY_MASK) == FPGA_FRAME_FIFO_EMPTY_MASK)
        || ((frame & FPGA_FRAME_FIFO_FULL_MASK ) == FPGA_FRAME_FIFO_FULL_MASK ))
    {
        debugStr(DebugStream::error) << boost::format(  "FPGA frame number FIFO full/empty : original = %6u, number = %5u\n")
                                                      % frame
                                                      % (frame & ~(FPGA_FRAME_FIFO_FULL_MASK | FPGA_FRAME_FIFO_EMPTY_MASK));
        frame = frame & ~(FPGA_FRAME_FIFO_FULL_MASK | FPGA_FRAME_FIFO_EMPTY_MASK);
#ifdef EXIT_ON_FRAME_ERROR
        exit(1);
#endif
    }

    if (!big_frame_count_started)
    {
        big_frame_count = frame;
        gettimeofday(&time_of_last_frame_update, NULL);
        last_fpga_frame = frame;
        big_frame_count_started = true;
    }
    else
    {
        //
        //  Calculate the time elapsed since the last update.
        //
        struct timeval curr_time;
        gettimeofday(&curr_time, NULL);

        double milli_secs_since_last_update =   double(curr_time.tv_sec -time_of_last_frame_update.tv_sec )*1e3
                                              + double(curr_time.tv_usec-time_of_last_frame_update.tv_usec)/1e3;

        //
        //  Estimate the number of counter wraps that should have occurred and calculate an unwrapped current frame
        //  number.
        //
        const double MILLI_SECS_IN_FRAME               = 1e3 / static_cast<double>(ThreeGPP::FRAMES_PER_SEC);
        const double MILLI_SECS_IN_FRAME_WRAP_INTERVAL =   static_cast<double>(MAX_FPGA_FRAME+1)
                                                         * MILLI_SECS_IN_FRAME;

        int64_t num_wraps = static_cast<int64_t>(floor(  (  milli_secs_since_last_update
                                                          + static_cast<double>(last_fpga_frame)*MILLI_SECS_IN_FRAME
                                                          - static_cast<double>(frame)          *MILLI_SECS_IN_FRAME)
                                                       / MILLI_SECS_IN_FRAME_WRAP_INTERVAL
                                                       + 0.5));
        int64_t unwrapped_frame = static_cast<int64_t>(frame)+num_wraps*(MAX_FPGA_FRAME+1);

        //
        //  Calculate the most likely increment for the frame number.
        //
        int64_t big_frame_count_inc = unwrapped_frame-static_cast<int64_t>(last_fpga_frame);

        //
        //  The unwrapped frame number should be greater than the last frame number.  However a bug in the FPGA results
        //  in an old frame number occasionally being presented as the new one.
        //
        if (big_frame_count_inc <= 0)
        {
            //
            //  The FPGA frame counter bug has struck again.  The elapsed time is probably not accurate enough to
            //  correct the frame number, so use an increment of 1.  Since the normal minimum increment is 2 this
            //  should allow the system to recover.
            //
            debugStr(DebugStream::error) << boost::format(  "FPGA frame number error : current = %5u, last = %5u, elapsed time = %f ms, num_wraps = %1u, unwrapped_frame = %6u, big_frame_count_inc = %6d (%f, %f)\n")
                                                          % frame
                                                          % last_fpga_frame
                                                          % milli_secs_since_last_update
                                                          % num_wraps
                                                          % unwrapped_frame
                                                          % big_frame_count_inc
                                                          % MILLI_SECS_IN_FRAME
                                                          % MILLI_SECS_IN_FRAME_WRAP_INTERVAL;

#ifdef EXIT_ON_FRAME_ERROR
            exit(1);
#endif

            big_frame_count_inc = 1;
            frame = last_fpga_frame + 1;
            if (frame > MAX_FPGA_FRAME)
            {
                frame = frame-MAX_FPGA_FRAME;
            }
        }

        big_frame_count = big_frame_count + big_frame_count_inc;
        time_of_last_frame_update = curr_time;
        last_fpga_frame = frame;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Read the data associated with the even-numbered result.
//  @details        Read the FPGA's frame counter and peak detector results for the even buffer and store them in the
//                  software cache.  Use the frame number to update the internal 64.bit counter.  Mark the frame data as
//                  being valid.
//
void Fpga::read_even_frame_data(void)
{
    uint32_t frame             = *reg_even_frame_count_ptr;
    num_samples_over_threshold = *reg_even_num_peaks_ptr;
    peak_sample_offset         = *reg_even_peak_address_ptr-CORR_EVEN_RESULT_BASE_ADDRESS;
    peak_pwr                   = *reg_even_peak_data_ptr;

    debugStr(DebugStream::info3) << "even frame: FN = "  << frame                                         << ", "
                                 <<          "N > T = "  << num_samples_over_threshold                    << ", "
                                 <<    "Peak Offset = "  << peak_sample_offset                            << ", "
                                 <<     "Peak Value = "  << peak_pwr                                      << "\n";

    update_big_frame_count( frame );

    debugStr(DebugStream::info3) << "even frame: unwrapped FN = "  << big_frame_count << "\n";

    //
    //  Mark the frame data as valid.
    //
    frame_data_valid = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Read the data associated with the odd-numbered result.
//  @details        Read the FPGA's frame counter and peak detector results for the odd buffer and store them in the
//                  software cache.  Use the frame number to update the internal 64.bit counter.  Mark the frame data as
//                  being valid.
//
void Fpga::read_odd_frame_data(void)
{
    uint32_t frame             = *reg_odd_frame_count_ptr;
    num_samples_over_threshold = *reg_odd_num_peaks_ptr;
    peak_sample_offset         = *reg_odd_peak_address_ptr-CORR_ODD_RESULT_BASE_ADDRESS;
    peak_pwr                   = *reg_odd_peak_data_ptr;

    debugStr(DebugStream::info3) << "odd frame:  FN = "  << frame                                         << ", "
                                 <<          "N > T = "  << num_samples_over_threshold                    << ", "
                                 <<    "Peak Offset = "  << peak_sample_offset                            << ", "
                                 <<     "Peak Value = "  << peak_pwr                                      << "\n";

    update_big_frame_count( frame );

    debugStr(DebugStream::info3) << "odd frame:  unwrapped FN = "  << big_frame_count << "\n";

    //
    //  Mark the frame data as valid.
    //
    frame_data_valid = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Get the frame number for the current result.
//  @details        If the cached value is valid it is returned, otherwise an exception is thrown.
//  @return                             The frame number of the first frame that contributed to the current
//                                      result.
//
int64_t Fpga::get_frame( void )
{
    if (!frame_data_valid)
    {
        throw std::runtime_error("attempt to read invalid frame number");
    }

    return big_frame_count;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Get the number of samples in the current result that exceeded the threshold.
//  @details        If the cached value is valid it is returned, otherwise an exception is thrown.
//  @return                             The number of samples over the threshold.
//
uint32_t Fpga::get_num_samples_over_threshold(void)
{
    if (!frame_data_valid)
    {
        throw std::runtime_error("attempt to read invalid number of samples over the threshold");
    }

    return num_samples_over_threshold;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Get the sample of the peak in the current result.
//  @details        If the cached value is valid it is returned, otherwise an exception is thrown.
//  @return                             The sample offset of the peak.
//
uint32_t Fpga::get_peak_sample_offset( void )
{
    if (!frame_data_valid)
    {
        throw std::runtime_error("attempt to read invalid peak offset");
    }

    return peak_sample_offset;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Get the power of the peak in the current result.
//  @details        If the cached value is valid it is returned, otherwise an exception is thrown.
//  @return                             The power of the peak.
//
uint32_t Fpga::get_peak_pwr(void)
{
    if (!frame_data_valid)
    {
        throw std::runtime_error("attempt to read invalid peak data");
    }

    return peak_pwr;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Gets the details of the peak in the current correlator result.
//  @details        Reads the peak data for the current result from the software cache.  If the number of samples over
//                  the threshold is greater than zero the peak is valid and its details are added to the result.
//  @param[out]     raw_peak            The raw peak data.
//  @return                             The frame number of the first frame that contributed to the result.
//
int64_t Fpga::get_peak_data(raw_peak_t &raw_peak)
{
    uint32_t num_samples_over_threshold = get_num_samples_over_threshold();
    uint32_t peak_sample_offset         = get_peak_sample_offset();
    uint32_t peak_data                  = get_peak_pwr();
    int64_t  frame_number               = get_frame();

    //
    //  Correct the factor of two introduced by the FPGA to the peak power result when the matched filter produces IQ
    //  results.
    //
    debugStr(DebugStream::info3) << "Peak data: " << "frame number = "   << frame_number               << ", "
                                                  << "number samples = " << num_samples_over_threshold << ", "
                                                  << "peak offset = "    << peak_sample_offset         << ", "
                                                  << "peak data = "      << peak_data                  << "\n";

#ifdef USE_SW_PEAK_DETECTION
    /*
     * Use software peak detection until hardware works.
     */
    uint32_t start = 0;
    uint32_t len = 76800;
    bool alt_ant = false;
    std::vector<double> pwrs;

#ifdef IQ_RESULTS
    get_raw_iq_mode_corr_data( start, len, alt_ant, pwrs );
#else
    get_raw_pwr_mode_corr_data( start, len, alt_ant, pwrs );
#endif

    double   raw_peak_pwr       = 0.0;
    unsigned raw_peak_pwr_index = 0;
    for (unsigned n = 0 ; n < len ; ++n)
    {
        if (pwrs[n] > raw_peak_pwr)
        {
            raw_peak_pwr = pwrs[n];
            raw_peak_pwr_index = n;
        }
    }

    debugStr(DebugStream::info3) << "Raw Peak data: " << raw_peak_pwr_index << " "
                                                      << raw_peak_pwr       << "\n";
    num_samples_over_threshold = 1;
    peak_sample_offset         = raw_peak_pwr_index;
    peak_data                  = uint32_t(raw_peak_pwr);
#endif

    raw_peak.exceeds_threshold = (num_samples_over_threshold > 0);
    raw_peak.sample_offset     = peak_sample_offset;
    raw_peak.sample_power      = static_cast<double>(peak_data);

    return frame_number;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Gets the peak sample in the current correlator result and the two samples either side of it.
//  @details        Gets the position of the peak from the software cache, then reads the three samples that span the
//                  true peak.  Fill out the result with the frame number of the first frame that contributed to the
//                  result.
//  @param[out]     peak_array          The destination for the result.
//
void Fpga::get_peak_array(peak_array_t &peak_array)
{
    peak_array.sample_offset = get_peak_sample_offset();

    //
    //  Get the peak and the samples either side of it.
    //
    const unsigned NUM_SAMPLES_BEFORE_PEAK = (SAMPLES_IN_PEAK_ARRAY-1)/2;

    uint32_t firstSample = peak_array.sample_offset-NUM_SAMPLES_BEFORE_PEAK;
    if (peak_array.sample_offset < NUM_SAMPLES_BEFORE_PEAK)
    {
        firstSample = peak_array.sample_offset+Design::samples_per_frame-NUM_SAMPLES_BEFORE_PEAK;
    }

#ifdef IQ_RESULTS
    std::vector<std::complex<double> > data;
    int64_t frame = get_raw_iq_mode_corr_data( firstSample, SAMPLES_IN_PEAK_ARRAY, false, data );

    for (unsigned sample_num = 0 ; sample_num < SAMPLES_IN_PEAK_ARRAY ; ++sample_num)
    {
        peak_array.peak[sample_num] =   data[sample_num].real()*data[sample_num].real()
                                      + data[sample_num].imag()*data[sample_num].imag();
    }
#else
    std::vector<double> data;
    int64_t frame = get_raw_pwr_mode_corr_data( firstSample, SAMPLES_IN_PEAK_ARRAY, false, data );

    for (unsigned sample_num = 0 ; sample_num < SAMPLES_IN_PEAK_ARRAY ; ++sample_num)
    {
        peak_array.peak[sample_num] = data[sample_num];
    }
#endif

    peak_array.frame = frame;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Reads a block of power result samples.
//  @details        Reads the samples from the result buffer as 32-bit words and then converts them to doubles.
//  @param[in]      start               Offset in samples from the start of the result buffer.
//  @param[in]      len                 The number of samples to read.
//  @param[in]      alt_ant             true if the samples for the alternate antenna should be read, false otherwise.
//  @param[out]     data                The destination for the samples.
//  @return                             The frame number of the first frame that contributed to the result.
//
int64_t Fpga::get_raw_pwr_mode_corr_data(uint32_t             start,
                                         uint32_t             len,
                                         bool                 alt_ant,
                                         std::vector<double>& data)
{
    debugStr(DebugStream::info1) << "get_raw_pwr_mode_corr_data : start =" << start << ", len = " << len << "\n";

    uint32_t address = CORR_EVEN_RESULT_BASE_ADDRESS;
    if (!proc_waiting_for_even_buffer)
    {
        address = CORR_ODD_RESULT_BASE_ADDRESS;
    }

    std::vector<uint32_t> raw_data;
    if (start+len <= Design::samples_per_frame)
    {
        mem_raw_read( address+start, len, raw_data );
    }
    else
    {
        uint32_t start1 = start;
        uint32_t len1   = Design::samples_per_frame-start;
        mem_raw_read( address+start1, len1, raw_data );

        uint32_t start2 = 0;
        uint32_t len2   = len-len1;
        mem_raw_read( address+start2, len2, raw_data );
    }

    // convert to double power value for internal use
    BOOST_FOREACH( uint32_t d, raw_data )
    {
        double sample = static_cast<double>(d);
        data.push_back( sample );
    }

    return get_frame();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Reads a block of IQ result samples.
//  @details        Reads the samples from the result buffer as 32-bit words then separates the 16-bit components and
//                  constructs the result samples as complex doubles.
//  @param[in]      start               Offset in samples from the start of the result buffer.
//  @param[in]      len                 The number of samples to read.
//  @param[in]      alt_ant             true if the samples for the alternate antenna should be read, false otherwise.
//  @param[out]     data                The destination for the samples.
//  @return                             The frame number of the first frame that contributed to the result.
//
int64_t Fpga::get_raw_iq_mode_corr_data(uint32_t                            start,
                                        uint32_t                            len,
                                        bool                                alt_ant,
                                        std::vector<std::complex<double> >& data)
{
    debugStr(DebugStream::info1) << "get_raw_iq_mode_corr_data : start =" << start << ", len = " << len << "\n";

    uint32_t address = CORR_EVEN_RESULT_BASE_ADDRESS;
    if (!proc_waiting_for_even_buffer)
    {
        address = CORR_ODD_RESULT_BASE_ADDRESS;
    }

    std::vector<uint32_t> raw_data;
    if (start+len <= Design::samples_per_frame)
    {
        mem_raw_read( address+start, len, raw_data );
    }
    else
    {
        uint32_t start1 = start;
        uint32_t len1   = Design::samples_per_frame-start;
        mem_raw_read( address+start1, len1, raw_data );

        uint32_t start2 = 0;
        uint32_t len2   = len-len1;
        mem_raw_read( address+start2, len2, raw_data );
    }

    // convert to double component values for internal use
    BOOST_FOREACH( uint32_t d, raw_data )
    {
        int16_t re = (d >>  0) & 0xFFFF;
        int16_t im = (d >> 16) & 0xFFFF;

        std::complex<double> sample(static_cast<double>(re), static_cast<double>(im));
        data.push_back( sample );
    }

    return get_frame();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Get sample counter value of the last 1pps event.
//  @details        The time of the 1 pps event is latched by the hardware.  It is measured by a 25-bit counter clocked
//                  by the 26 MHz reference clock.  Thus the value wraps every 1.27 s.  As long as this function is
//                  called at least every 100 ms the interval between events can be unambiguously calculated.
//
//                  The FPGA interface reads the latch register as separate high and low half words, and so if an event
//                  occurs during the read a false value may be read.  However an a change in the latched value always
//                  indicates an event and a second read will produce the correct counter value.
//  @param[out]     count               The number of 26 MHz clock cycles bteween the last two 1pps events.
//  @return                             true if an event has occurred since the last time that this function was called.
//
bool Fpga::get_1pps_count(uint32_t &count)
{
    // count number of 26MHz clocks, modulo 25bit counter
    static uint32_t last=0;
    uint32_t current = *reg_1pps_ptr;

    if( current == last )
    {
        return false;
    }
    // else new counter value

    current = *reg_1pps_ptr;    // read again as may be corrupt if changing
    if( current > last )
    {
        count = current - last;
    }
    else
    {
        count = current + (1<<25) - last;
    }
    last = current;

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Set a new threshold for the peak detection.
//  @param[in]      threshold           The new threshold value.
//
void Fpga::set_threshold(uint32_t threshold)
{
    *reg_threshold_ptr = threshold;
    debugStr(DebugStream::info2) << "write " << ((void *)(reg_threshold_ptr)) << " = " << threshold << "\n";
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Get the FPGA image version number.
//  @return                             The version number.
//
uint8_t Fpga::get_version(void)
{
    return (static_cast<uint8_t>(*reg_firmware_rev_ptr & 0xFF));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Set the test register word.
//  @param[in]      val                 The new value.
//
void Fpga::set_test_register(uint32_t val)
{
    *reg_test_register_ptr = val;
    debugStr(DebugStream::info2) << "write " << ((void *)(reg_test_register_ptr)) << " = " << val << "\n";
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Enter low power mode.
//
void Fpga::enter_low_power_mode(void)
{
#ifdef USE_FPGA_LOW_POWER_MODE
    gpio power_mode_ctrl(gpio::FPGA_LOW_PWR_MODE, gpio::out_high, debugStr.get_state());    // high = low power mode
    debugStr(DebugStream::info1) << "Entering low power mode\n";
#endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Exit low power mode.
//
void Fpga::exit_low_power_mode(void)
{
#ifdef USE_FPGA_LOW_POWER_MODE
    debugStr(DebugStream::info1) << "Exiting low power mode\n";
    gpio power_mode_ctrl(gpio::FPGA_LOW_PWR_MODE, gpio::out_low, debugStr.get_state());     // low  = normal power mode

    gpio mcb_cal_done( gpio::FPGA_MCB_CAL_DONE,    gpio::in,         debugStr.get_state() ); // read 1 = MCB calibration is complete
    gpio clk_lock_192( gpio::FPGA_192_CLK_LOCK,    gpio::in,         debugStr.get_state() ); // read 1 = 192 MHz clock is locked
    gpio clk_lock_195( gpio::FPGA_195_CLK_LOCK,    gpio::in,         debugStr.get_state() ); // read 1 = 195 MHz clock is locked

    for(int j = 0 ; j < 10 ; ++j)
    {
        usleep(10000);
        if(mcb_cal_done.get() && (cur_mode == MODE_GSM || clk_lock_192.get()) && clk_lock_195.get())
        {
            break;
        }
    }

    std::string msg="";
    // check MCB calibration flag
    if( !mcb_cal_done.get() )
    {
        msg += "MCB calibration line low; ";
    }
    // check 192 MHz clock lock flag
    if( cur_mode != MODE_GSM && !clk_lock_192.get() )
    {
        msg += "192 MHz clock lock line low; ";
    }
    // check 195 MHz clock lock flag
    bool clk_195_is_locked = clk_lock_195.get();
    debugStr(DebugStream::info1) << "195 MHz clock status : " << clk_195_is_locked << "\n";
    if( !clk_195_is_locked )
    {
        msg += "195 MHz clock lock line low; ";
    }
    if( msg != "" )
    {
        throw std::runtime_error( "Error exiting fpga low power mode: " + msg );
    }

    big_frame_count_started = false;
#endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Configure the GSM power meter engine.
//  @details        Reset the engine and then start it.
//  @param[in]      one_result_per_slot True if one power results per slot is required, false if three are needed.
//  @param[in]      antenna_mode        The antenna mode to be used.
//
void Fpga::setup_gsm_pwr_meter(bool one_result_per_slot, unsigned antenna_mode)
{
    using namespace std;

    //
    //  Disable the channel filter.
    //
    *reg_mode_ptr = 0;
    debugStr(DebugStream::info2) << "write " << ((void *)(reg_mode_ptr)) << " = " << 0 << "\n";

    //
    //  Setup the DC offset.
    //
    int16_t inph_dc = 0;
    int16_t quad_dc = 0;
    set_dc_offset(inph_dc, quad_dc);

    //
    //  Get rid of pending data.
    //
    (void)empty_gsm_fifo();

    //
    //  Report the FIFO status.
    //
    uint32_t gsm_result_status = *reg_gsm_result_status_ptr;
    debugStr(DebugStream::info3) << boost::format("FIFO status = 0x%08x\n") % gsm_result_status;

    //
    //  Start the power meter, selecting the required result rate and antenna.
    //
    uint32_t mode_reg = GEN_MODE_CHAN_FILT_ENABLE | GSM_MODE_ENABLE_DC_OFFSET;

    if (one_result_per_slot)
    {
        mode_reg = mode_reg | GSM_MODE_ONE_RESULT_PER_SLOT;
    }

    if (antenna_mode == 2)
    {
        mode_reg = mode_reg | ANT_MODE_ANT2;
    }
    else
    {
        mode_reg = mode_reg | ANT_MODE_ANT1;
    }

    *reg_mode_ptr = mode_reg;
    debugStr(DebugStream::info2) << "write " << ((void *)(reg_mode_ptr)) << " = " << mode_reg << "\n";
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Empty the GSM result FIFO.
//  @details        The FIFO is read until it is empty and no errors are issued if an overflow or underflow is detected.
// @return                             The number of discarded samples.
//
unsigned Fpga::empty_gsm_fifo(void)
{
    using namespace std;

    uint32_t gsm_result_status = *reg_gsm_result_status_ptr;

    //
    //  Determine the number of results to read.
    //
    unsigned samples_to_read = gsm_result_status & GSM_FIFO_COUNT;

    //
    //  Read the samples.
    //
    for (unsigned n = 0 ; n < samples_to_read ; ++n)
    {
        *reg_gsm_result_fifo_ptr;
    }

    return (samples_to_read);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Read from the GSM result FIFO.
//  @details        The FIFO is read until it is empty or the number of required samples have been read.
//
//  @param[in]      max_num_samples     The maximum number of 32-bit words to read.
//  @param[out]     data                The destination for the data.
//
void Fpga::read_gsm_fifo(uint32_t max_num_samples, std::vector<uint32_t>& data)
{
    using namespace std;

    uint32_t gsm_result_status = *reg_gsm_result_status_ptr;

    //
    // report any fifo errors
    //
    if ((gsm_result_status & GSM_FIFO_OVERFLOW ) == GSM_FIFO_OVERFLOW )
    {
        debugStr(DebugStream::warn2) << "GSM FIFO overflow\n";
    }

    if ((gsm_result_status & GSM_FIFO_UNDERFLOW) == GSM_FIFO_UNDERFLOW)
    {
        debugStr(DebugStream::warn2) << "GSM FIFO underflow\n";
    }

    //
    //  Determine the number of results to read.
    //
    unsigned samples_to_read = gsm_result_status & GSM_FIFO_COUNT;
    if (samples_to_read > max_num_samples)
    {
        samples_to_read = max_num_samples;
    }

    //
    //  Read the samples.
    //
    for (unsigned n = 0 ; n < samples_to_read ; ++n)
    {
        uint32_t sample = *reg_gsm_result_fifo_ptr;
        data.push_back(sample);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Gets the DC offset estimates.
//  @param[out]     inph_dc             The DC offset of the in-phase component.
//  @param[out]     quad_dc             The DC offset of the quadrature component.
//
void Fpga::get_dc_offset(int16_t &inph_dc, int16_t &quad_dc)
{
    uint32_t reg = *reg_gsm_dc_offset_ptr;
    inph_dc = static_cast<int16_t>((reg >>  0) & 0xFFFF);
    quad_dc = static_cast<int16_t>((reg >> 16) & 0xFFFF);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Sets the DC offset estimates.
//  @param[in]      inph_dc             The DC offset of the in-phase component.
//  @param[in]      quad_dc             The DC offset of the quadrature component.
//
void Fpga::set_dc_offset(int16_t inph_dc, int16_t quad_dc)
{
    uint32_t reg =   ((static_cast<uint32_t>(inph_dc) & 0xFFF) <<  0)
                   | ((static_cast<uint32_t>(quad_dc) & 0xFFF) << 16);

    debugStr(DebugStream::info2) << "write " << ((void *)(reg_gsm_dc_offset_ptr)) << " = " << reg << "\n";
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Set the debug verbosity level.
//  @param[in]      level               The new verbosity level.
//
void Fpga::debug_printing(unsigned level)
{
    debugStr.show_level(level);
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief          Writes the coefficients for the channel filters and correlator for the 4G fpga
/// @details        If there is a fault with any of the supplied coefficients nothing is progrrammed to the fpga
/// @param[in]      channel1    Array of int16_t with parameters for channel filter stage 1
/// @param[in]      channel2    Array of int16_t with parameters for channel filter stage 1
/// @param[in]      correl_taps Array of 320 correlator taps in IEEE 754 binary32 format
/// @returns        false if there is a problem with any of the supplied parameters/coefficients
///
///
/// Note on channel filter coefficients:
/// The filter coefficients are symetrical around a central coefficient.
/// In the presented vectors the first coefficient is the central one with the remaining items being one side of
/// the complete filter.
/// The FPGA expects all the coefficients to be presented.
bool Fpga::send_4g_coefs(const std::vector<int16_t>&channel1,
                   const std::vector<int16_t>&channel2,
                   const std::vector<float>&correl_taps)
{
    bool result = true;
    uint32_t channel1_elements = channel1.size();
    uint32_t channel2_elements = channel2.size();
    uint32_t correltaps_elements = correl_taps.size();
    // union prevents negative signed values having '1's placed in the MS-16 bits of the channel_filter registers
    union {
        int16_t signed_val;
        uint16_t unsigned_val;
    } channel_filter_vals;
    IEEE754Hex correl_tap;

    if (NUM_CHANNEL_FILTER_TAP_VALS_4G != channel1_elements)
    {
        debugStr(DebugStream::error) << "Channel Filter 1 given incorrect number of elements: " << channel1_elements << "\n";
        result = false;
    }

    if (NUM_CHANNEL_FILTER_TAP_VALS_4G != channel2_elements)
    {
        debugStr(DebugStream::error) << "Channel Filter 2 given incorrect number of elements: " << channel2_elements << "\n";
        result = false;
    }

    if (NUM_CORRELATOR_TAPS_4G != correltaps_elements)
    {
        debugStr(DebugStream::error) << "Correlator given incorrect number of elements: " << correltaps_elements << "\n";
        result = false;
    }

    if (result)
    {
        for (uint32_t elem = 0; elem < channel1_elements; elem++)
        {
            channel_filter_vals.signed_val = channel1[elem];
            *channel_filter_1_4g_ptr = (uint32_t)(channel_filter_vals.unsigned_val);
        }

        for (uint32_t elem = 0; elem < channel2_elements; elem++)
        {
            channel_filter_vals.signed_val = channel2[elem];
            *channel_filter_2_4g_ptr = (uint32_t)(channel_filter_vals.unsigned_val);
        }

        for (uint32_t elem = 0; elem < correltaps_elements; elem++)
        {
            correl_tap.ieee754 = correl_taps[elem];
            *correlator_taps_4G[elem] = correl_tap.hex & BINARY_16_8_SET_ZEROES_MASK;
        }
    }
    return result;
}

/// @brief          Writes the squared threshold to be used in the correlator peak detection
/// @param[in]      threshold_sqrd   float squared threshold
///
/// Note on formatting.  The FPGA expects binary32 IEEE-754 internally interpretted as float(16,8)

void Fpga::set_4g_threshold_sqrd(float threshold_sqrd)
{
    IEEE754Hex threshold;
    threshold.ieee754 = threshold_sqrd;
    *reg_correlator_threshold_sqrd_4g_ptr = threshold.hex & BINARY_16_8_SET_ZEROES_MASK;
}

/// @brief          Disable statistics engine.  Read results.  Reset statistics engine if requested
/// @param[in]      restart  Restart engine afterwards?
/// @param[out]     peak     Statistics engine peak values
/// @param[out]     sum      Statistics engine sum of values
/// @returns        Number of values contributing to the results
///
/// According to 1QR00301 the power measurements are IEEE-754 format with 8-bit exponent (24-bits total)
/// These are returned as uint32_t to calling function for them to interpret as they wish

uint32_t Fpga::get_4g_statistics(bool restart, float &peak, float &sum)
{
    uint32_t num_values = 0;
    IEEE754Hex peak_pwr;
    IEEE754Hex sum_pwr;

    // disable statistics engine but do not clear it
    *reg_correlation_statistics_control_ptr = (~CORR_STATISTICS_CTRL_EN & ~CORR_STATISTICS_CTRL_CLR);

    num_values = *reg_correlation_statistics_num_means_ptr;
    peak_pwr.hex = *reg_correlation_statistics_peak_power_ptr;
    peak = peak_pwr.ieee754;
    sum_pwr.hex = *reg_correlation_statistics_sum_power_ptr;
    sum = sum_pwr.ieee754;
    
    // Reset (clear) the statistics engine but do not enable it
    *reg_correlation_statistics_control_ptr = CORR_STATISTICS_CTRL_CLR;

    // restart statistics engine (if requested)
    if (restart)
    {
        *reg_correlation_statistics_control_ptr = CORR_STATISTICS_CTRL_EN;
    }

#ifdef SWTEST_4G_FPGA        
    debugStr(DebugStream::info1) << boost::format("get_4g_statistics: NumValues = %d\nPeak_Pwr = %f\nSum = %f\n")
                                                  % num_values
                                                  % peak
                                                  % sum;
#endif
    return num_values;
}

///         1)  More than MAX_NUM_CORR_CMDS_4G commands: DebugStream::fatal and return without writing anything
///             (it is TBC in 1QS03, but the correlator class is currently implemented on the assumption that it can
///             fit all commands for a correlation into a call).
///         2)  The FPGA's command queue is not empty: DebugStream::fatal and return without writing anything
///             (the current correlator class implementation starts a correlation and should not start another until
///             it is complete).
///         3)  The FPGA's command queue status indicates an overflow: DebugStream::error but write list (with the
///             two tests above it should be impossible unless MAX_NUM_CORR_CMDS_4G does not match the FPGA
///             implementation).
///         4)  Any start or stop time less than the current FPGA time: DebugStream::error and stop writing list
///             (if it is triggered it will probably be on the first element so nothing will be written. It would be
///             the result of either a counter wrap or the correlator class not allowing a large enough margin).
///         4)  Consider detecting odd sequences within the current list (e.g. CORRELATE_CONTINUOUSLY followed by
///             anything else, STOP_IMMEDIATELY as anything other than the first element).
/// @brief          Starts a sequence of correlation commands
/// @param[in]      cmds  a list of CorrCmd_4g_t type
/// @returns        unsigned, the number of commands that were written
///
uint32_t Fpga::setup_4g_correlation(std::list<CorrCmd_4g_t> cmds)
{
    uint32_t cmds_written = 0;
    bool overflow = false;

    BOOST_FOREACH(CorrCmd_4g_t the_cmd, cmds)
    {
        // Command specific programming of start/stop times
        switch (the_cmd.cmd)
        {
        case CORRELATE_CONTINUOUSLY:
            *reg_correlator_cmd_queue_start_4g_ptr = the_cmd.start;
            break;
        case STOP_IMMEDIATELY:
            break;
        case CORRELATE_FROM_TO:
            *reg_correlator_cmd_queue_start_4g_ptr = the_cmd.start;
            *reg_correlator_cmd_queue_stop_4g_ptr = the_cmd.stop;
            break;
        default:
            assert(false);
            break;
        }
        uint32_t my_cmd = (uint8_t)the_cmd.cmd + (the_cmd.id << 8);
        *reg_correlator_cmd_queue_4g_ptr = my_cmd;

        overflow = ((*reg_correlator_cmd_status_info_ptr & CORR_INFO_4G_OVF) == CORR_INFO_4G_OVF);
        if (overflow)
        {
            *reg_correlator_cmd_status_info_ptr = CORR_INFO_4G_OVF; // clear overflow
            break;  // we have over-flowed the command queue
        }
        else
        {
            cmds_written++;
        }
    }
    return cmds_written;
}

/// @brief          Reads the status of recent correlation commands
/// @param[out]     cmd_status a list of CorrCmdStatus_4g_t type showing command ids and associated status
/// @returns        bool, "false" if the FIFO's overflow flag is set
///
bool Fpga::get_4g_correlation_command_status(std::list<CorrCmdStatus_4g_t> &cmd_status)
{
    uint32_t latest_status_id;
    CorrCmdStatus_4g_t cmd_status_elem;

    bool overflow = ((*reg_correlator_cmd_status_info_ptr & CORR_CMD_STATUS_4G_OVF) == CORR_CMD_STATUS_4G_OVF);
    if (overflow)
    {
        *reg_correlator_cmd_status_info_ptr = CORR_CMD_STATUS_4G_OVF; // clear overflow
#ifdef SWTEST_4G_FPGA        
        debugStr(DebugStream::error) << "get_4g_correlation_command_status::OVERFLOW\n";
#endif
    }

    // RDY flag indicates there is still data to read from the cmd fifo
    while ((*reg_correlator_cmd_status_info_ptr & CORR_CMD_STATUS_4G_RDY) != 0)
    {
        latest_status_id = *reg_correlator_cmd_status_fifo_ptr;
        cmd_status_elem.status = (eCorrCmdStatus_4g)(latest_status_id & CORR_CMD_STATUS_FIFO_STATUS_MASK_4G);
        cmd_status_elem.id = (latest_status_id >> CORR_CMD_STATUS_FIFO_STATUS_BITS_4G) & 0xFF; // cmd_id is the next 8-bits
        cmd_status.push_back(cmd_status_elem);

#ifdef SWTEST_4G_FPGA        
        std::string str;
        switch (cmd_status_elem.status)
        {
            case UNUSED0:
                str = "UNUSED0";
                break;
            case REJECTED:
                str = "REJECTED";
                break;
            case FETCHED:
                str = "FETCHED";
                break;
            case UNUSED3:
                str = "UNUSED3";
                break;
        }
        int id = cmd_status_elem.id;
        debugStr(DebugStream::info1) << boost::format("get_4g_correlation_command_status::FPGA cmd ID = %d, status = %s\n")
                                                        % id
                                                        % str;
#endif                                                        
    }

    return !overflow;
}

/// @brief          Reads the status of recent colleration commands
/// @param[in]      maxNumPeaks the maximum number of peaks to write into the destination array
/// @param[out]     peaks array of peaks read from the peaks fifo
/// @param[out]     numPeaks actual number of peaks placed into peaks array
/// @returns        bool, "false" if the FIFO's overflow flag is set
///
bool Fpga::get_4g_peak_data(size_t maxNumPeaks, CorrPeak_4g_t peaks[], uint32_t &numPeaks)
{
    bool overflow;
    numPeaks = 0;
    uint32_t peak_time_ls;
    volatile uint32_t peak_time_ms;  // will be ignored - but must ensure FIFO register is actually read
    IEEE754Hex correl_num;
    IEEE754Hex correl_den;
    IEEE754Hex sum_pwr;

    // RDY indicataes there is still data to read from the results fifo
    while (((*reg_correlation_result_info_ptr & CORR_RESULT_4G_RDY) != 0)
        && numPeaks < maxNumPeaks)
    {
        peak_time_ls = *reg_correlation_result_fifo_ls_time_ptr;    // contains LS 32-bits of 40-bit time
        peak_time_ms = *reg_correlation_result_fifo_ms_time_ptr;    // contains MS 8-bits of 40-bit time
        correl_num.hex = (*reg_correlation_result_fifo_correl_num_ptr);
        correl_den.hex = *reg_correlation_result_fifo_correl_den_ptr;
        sum_pwr.hex = *reg_correlation_result_fifo_sum_pwr_ptr;

        peaks[numPeaks].time = peak_time_ls + ((uint64_t)(peak_time_ms) << 32U);
        peaks[numPeaks].CorrPeakNum = correl_num.ieee754;
        peaks[numPeaks].CorrPeakDenSqrd = correl_den.ieee754;
        peaks[numPeaks].SumCorrInpAtPeak = sum_pwr.ieee754;

        numPeaks++;
    }

    // Read at end in case FIFO overflowed whilst we were processing
    uint32_t reg_value = *reg_correlation_result_info_ptr;
    overflow = ((reg_value & CORR_RESULT_4G_OVF) == CORR_RESULT_4G_OVF);
    if (overflow)
    {
        *reg_correlation_result_info_ptr = reg_value; // clear overflow by writing back with overflow flag set
#ifdef SWTEST_4G_FPGA        
        debugStr(DebugStream::info1) << boost::format("get_4g_peak_data:: OVERFLOW detected\n");
#endif
    }

#ifdef SWTEST_4G_FPGA        
    debugStr(DebugStream::info1) << boost::format("get_4g_peak_data:: returning %u peaks\n")
                                                  % numPeaks;
#endif
    return !overflow;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief          Reads the current FPGA time
/// @returns        Current FPGA time in a uint64_t
///
uint64_t Fpga::get_4g_time(void)
{
    uint64_t the_time = 1;

    *reg_sample_time_ctrl_4g_ptr = SAMPLE_TIME_CTRL_4G_CAPTURE; // capture sample time

    the_time = *reg_sample_time_lsw_4g_ptr;
    the_time += (uint64_t)(*reg_sample_time_msw_4g_ptr) << 32U;
    // Release sample time control
    *reg_sample_time_ctrl_4g_ptr = (~SAMPLE_TIME_CTRL_4G_CAPTURE) & SAMPLE_TIME_CTRL_4G_CAPTURE;

    return the_time;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief          Set the antenna input of the 4G FPGA
/// @details        Uses the previously defined values of ANT_MODE_ANT1 and ANT_MODE_ANT2 from the gsm
///                 antenna mode selection.
///                 All other values other than ANT_MODE_ANT1 and ANT_MODE_ANT2 are invalid and will
///                 result in the system displaying an error message.
/// @param[in]     antenna_mode    ANT_MODE_ANT1 or ANT_MODE_ANT2 only
///
void Fpga::set_4g_antenna(uint32_t antenna_mode)
{
    switch (antenna_mode)
    {
    case ANT_MODE_ANT1:
        *reg_antenna_select_4g_ptr = 0;
        break;
    case ANT_MODE_ANT2:
        *reg_antenna_select_4g_ptr = 1;
        break;
    default:
        debugStr(DebugStream::error) << boost::format("Invalid antenna_mode for set_4g_antenna: 0x%x\n")
                                                      % antenna_mode;
        break;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief         Stop any ongoing correlation
///
void Fpga::stop_correlation(void)
{
    debugStr(DebugStream::info1) << "Stopping correlation\n";

    std::list<Fpga::CorrCmd_4g_t> cmds;

    CorrCmd_4g_t cmd =
    {
        Fpga::STOP_IMMEDIATELY,
        255,
        0,
        0
    };
    cmds.push_back(cmd);
        
    uint64_t current_time_samples = get_4g_time();
    debugStr(DebugStream::info1) << "STOP_IMMEDIATELY: "
                                << "current time = "    << current_time_samples << ", "
                                << "low power mode = N/A, "
                                << "raw threshold = N/A, "
                                << "FPGA cmd ID = "     << static_cast<unsigned>(cmd.id) << ", "
                                << "FPGA start time = " << cmd.start << ", "
                                << "FPGA stop time = "  << cmd.stop << "\n";
        
    debugStr(DebugStream::error) << boost::format(  "Setting up a correlation sequence of length %u\n" )
                                                    % cmds.size();
        
    setup_4g_correlation(cmds);
}


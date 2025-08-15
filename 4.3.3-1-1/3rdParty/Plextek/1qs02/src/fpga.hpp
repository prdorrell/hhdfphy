/***********************************************************************************************************************
 *
 *
 ***********************************************************************************************************************
 *  Filename:   fpga.hpp
 *  Author(s):  pdm
 *******************************************************************************************************************//**
 *  @file
 *  @brief          The declaration of ::Fpga class that controls the FPGA.
 *  @details        Access to the FPGA uses memory-mapped registers.
 **********************************************************************************************************************/
#ifndef __FPGA_HPP__
#define __FPGA_HPP__

#include "common.hpp"
#include "debug.hpp"
#include "design.hpp"

#include <string>
#include <complex>
#include <list>
#include <vector>
#include <map>
#include <inttypes.h>

#include <boost/utility.hpp>

///
/// @brief          Controls access to the FPGA functionality.
///
class Fpga : private boost::noncopyable
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The constants.
    //
private:
    //------------------------------------------------------------------------------------------------------------------
    //  Offsets of the general control and status registers - Common for all FPGAs
    //

    ///
    /// @brief          The offset of the firmware revision register.
    /// @details        Structure:
    ///                 -   Bits  0 to  7 : the revision number
    ///                 -   Bits  8 to 15 : fixed pattern = 0xA3
    ///
    static const unsigned REG_FIRMWARE_REV_OFFSET         = 0;

    ///
    /// @brief          The offset of the test control register.
    /// @details        Structure:
    ///                 -   Bit   0       : debug LED (low = on)
    ///
    static const unsigned REG_TEST_REGISTER_OFFSET        = 1;

    //------------------------------------------------------------------------------------------------------------------
    //  Offsets of the general control and status registers - GSM and 3G.
    //

    ///
    /// @brief          The offset of the mode control register.
    /// @details        See Fpga::mode_t
    ///
    static const unsigned REG_MODE_OFFSET                 = 2;

    ///
    /// @brief          The offset of the averaging control register.
    /// @details        Structure:
    ///                 -   Bits  0 to  7 : frames to average
    ///                 -   Bits  8 to 15 : slots to average
    ///
    static const unsigned REG_AVERAGE_CTRL_OFFSET         = 3;

    ///
    /// @brief          The offset of the correlation start-offset register.
    ///
    static const unsigned REG_CORRELATION_START_OFFSET    = 4;

    ///
    /// @brief          The offset of the correlation length register.
    ///
    static const unsigned REG_CORRELATION_LENGTH_OFFSET   = 5;

    ///
    /// @brief          The offset of the peak detector threshold register.
    ///
    static const unsigned REG_THRESHOLD_OFFSET            = 6;

    ///
    /// @brief          The offset of the buffer control and status register.
    /// @details        Structure:
    ///                 -   Bit   0       : status  : the even result is ready for the software
    ///                 -   Bit   1       : status  : the odd result is ready for the software
    ///                 -   Bit   2       : control : the software has finished with the even result
    ///                 -   Bit   3       : control : the software has finished with the odd result
    ///                 -   Bit   4       : status  : an FFT overflow occurred
    ///
    static const unsigned REG_SWING_BUFFER_CONTROL_OFFSET = 7;

    ///
    /// @brief          The offset of the antenna swap control register.
    ///
    static const unsigned REG_ANT_SW_CTRL_OFFSET          = 8;

    ///
    /// @brief          The offset of the even-result frame number register.
    /// @details        Structure:
    ///                 -   Bits  0 to 15 : the frame number
    ///                 -   Bit  16       : the frame number FIFO empty flag
    ///                 -   Bit  17       : the frame number FIFO full flag
    ///
    static const unsigned REG_EVEN_FRAME_COUNT_OFFSET     = 9;

    ///
    /// @brief          The offset of the even-result peak address register.
    ///
    static const unsigned REG_EVEN_PEAK_ADDRESS_OFFSET    = 10;

    ///
    /// @brief          The offset of the even-result peak power register.
    ///
    static const unsigned REG_EVEN_PEAK_DATA_OFFSET       = 11;

    ///
    /// @brief          The offset of the even-result "number of samples over the threshold" register.
    ///
    static const unsigned REG_EVEN_NUM_PEAKS_OFFSET       = 12;

    ///
    /// @brief          The offset of the channel filter RSSI register.
    ///
    static const unsigned REG_CHAN_RSSI_OFFSET            = 13;

    ///
    /// @brief          The offset of the state machine debug register 1.
    ///
    static const unsigned REG_FSM_DEBUG1_OFFSET           = 14;

    ///
    /// @brief          The offset of the 1 PPS event-time register.
    ///
    static const unsigned REG_PPS_COUNT_OFFSET            = 15;

    ///
    /// @brief          The offset of the FFT gain control register.
    /// @details        Structure:
    ///                 -   Bits  0 to 15 : control the forward FFT gain schedule
    ///                 -   Bits 16 to 31 : control the inverse FFT gain schedule
    ///
    static const unsigned REG_FFT_GAIN_CTRL_OFFSET        = 16;

    ///
    /// @brief          The offset of the state machine debug register 2.
    ///
    static const unsigned REG_FSM_DEBUG2_OFFSET           = 17;

    ///
    /// @brief          The offset of the odd-result frame number register.
    /// @details        Structure:
    ///                 -   Bits  0 to 15 : the frame number
    ///                 -   Bit  16       : the frame number FIFO empty flag
    ///                 -   Bit  17       : the frame number FIFO full flag
    ///
    static const unsigned REG_ODD_FRAME_COUNT_OFFSET      = 18;

    ///
    /// @brief          The offset of the odd-result peak address register.
    ///
    static const unsigned REG_ODD_PEAK_ADDRESS_OFFSET     = 19;

    ///
    /// @brief          The offset of the odd-result peak power register.
    ///
    static const unsigned REG_ODD_PEAK_DATA_OFFSET        = 20;

    ///
    /// @brief          The offset of the odd-result "number of samples over the threshold" register.
    ///
    static const unsigned REG_ODD_NUM_PEAKS_OFFSET        = 21;

    ///
    /// @brief          The offset of the ADC RSSI register.
    ///
    static const unsigned REG_ADC_RSSI_OFFSET             = 22;

    ///
    /// @brief          The offset of the GSM result FIFO register.
    ///
    static const unsigned REG_GSM_RESULT_FIFO_OFFSET      = 23;

    ///
    /// @brief          The offset of the GSM result status register.
    ///
    static const unsigned REG_GSM_RESULT_STATUS_OFFSET    = 24;

    ///
    /// @brief          The offset of the GSM DC offset register.
    ///
    static const unsigned REG_GSM_DC_OFFSET               = 25;

    //  Offsets of the general control and status registers - 4G
    //

    /// @brief          Offset of the Test Register: 0xA000 0004
    /// @details        Structure:
    ///                 -   Bit   0       : debug LED (low = on)
    ///
    static const unsigned REG_TEST_REGISTER_4G_OFFSET   = 1;

    /// @brief          Offset of the Antenna Selection Register: 0xA000 0008
    /// @details        Bit 0 used only
    ///
    static const unsigned REG_ANTENNA_SELECT_4G_OFFSET   = 2;

    /// @brief          Offset of the Time capture control register for the 40-bit Sample Time Register: 0xA000 0010
    /// @details        40-bit time value via 2 32-bit words in neighbouring locations
    ///
    static const unsigned REG_SAMPLE_TIME_4G_OFFSET = 4;

    /// @brief          Channel Filter Stage 1 coefficient programming location
    /// @details        0xA000 1000 as 32-bit access
    ///
    static const unsigned CHANNEL_FILTER_1_4G_OFFSET = 0x0400;

    /// @brief          Channel Filter Stage 2 coefficient programming location
    /// @details        0xA000 1004 as 32-bit access
    ///
    static const unsigned CHANNEL_FILTER_2_4G_OFFSET = 0x0401;

    /// @brief          Floating point format test register - currently same for both input and output
    /// @details        0xA0002000 as 32-bit access
    ///
    static const unsigned REG_FLOAT_FORMAT_TEST_4G_OFFSET = 0x800;

    /// @brief          Correlator power multiplier
    /// @details        0xA0002004 as 32-bit access (float 16-8)
    ///
    static const unsigned CORRELATOR_PWR_MULTIPLICATION_4G_OFFSET = 0x801;

    /// @brief          Base of correlator command queue
    /// @details        0xA000 2008 as 32-bit access
    ///                 Structure 
    ///                 - 0xA0002008 8-bits command
    ///                 - 0xA0002009 8-bits command ID
    ///                 (write 32-bits with commnand in lsb and command id in next lowest byte)
    ///                 - 0xA000200C 64-bits (lower 40 valid) Start sample time
    ///                 - 0xA0002014 64-bits (lower 40 valid) Stop sample time
    ///
    static const unsigned CORRELATOR_CMD_QUEUE_4G_OFFSET = 0x802;
    static const unsigned CORRELATOR_CMD_START_TIME_4G_OFFSET = 0x803; // as 64-bit
    static const unsigned CORRELATOR_CMD_STOP_TIME_4G_OFFSET = 0x805; // as 64-bit

    /// @brief          Correlation Command Status Register offset
    /// @details        0xA000 201C as 32-bit access
    ///                 - bits 7:0 Command queue count (7=MSB)
    ///                 - bit 8 RDY
    ///                 - bit 9 FULL
    ///                 - bit 10 OVF
    static const unsigned CORRELATOR_CMD_QUEUE_STATUS_4G_OFFSET = 0x807;

    /// @brief          Correlation Command queue information about the status FIFO
    /// @details        0xA000 2020 as 32-bit access
    static const unsigned CORRELATOR_CMD_STATUS_INFO_4G_OFFSET = 0x808;

    /// @brief          Correlation Command queue status FIFO (as opposed to information about the FIFO)
    /// @details        0xA000 2024 as 32-bit access
    static const unsigned CORRELATOR_CMD_STATUS_FIFO_4G_OFFSET = 0x809;

    ///
    /// @brief          Address of the correlation result fifo status/info register
    /// @details        0xA000 2028 as 32-bit access
    ///
    static const unsigned CORRELATION_RESULT_INFO_4G_OFFSET = 0x80A;

    ///
    /// @brief          Address of the base of the Correlation FIFO
    /// @details        0xA000 202C as 32-bit access
    ///
    static const unsigned CORRELATION_RESULT_FIFO_BASE_4G_OFFSET = 0X80B;

    ///
    /// @brief          Address of the base of the mean statistics block
    /// @details        0xA000 2040 as 32-bit access
    ///
    static const unsigned CORRELATION_MEAN_STATISTICS_BASE_4G_OFFSET = 0x810;

    /// @brief          Base of correlator taps
    /// @details        0xA0004000 as 32-bit access (tap 0, tap 319 is at 0xA00044FC)
    ///
    static const unsigned CORRELATOR_BASE_4G_OFFSET = 0x1000;

    /// @brief          Offset of the Regression Test Interface Control: 0xA000 8000
    /// @details        Structure
    ///
    static const unsigned REG_REGRESSION_TEST_CONTROL_4G_OFFSET   = 0x2000;

    /// @brief          In-phase and Quadrature phase component of test impulse
    /// @details        Structure: Write all 32-bits simultaneously
    ///                 -   Bits 0:15 In-phase (16-bit int @ 0xA000 8004)
    ///                 -   Bits 16:31 Quadrature (16-bit int @ 0xA000 8006)
    ///
    static const unsigned REG_REGRESSION_TEST_VALUE_4G_OFFSET   = 0x2001;

    //------------------------------------------------------------------------------------------------------------------
    // Details of the 4G correlation Command Fifo
    //
    ///
    /// @brief          The maximum number of commands that the FIFO can hold (see
    ///                 Fpga::CORRELATOR_CMD_QUEUE_4G_OFFSET).
    ///
    static const unsigned MAX_NUM_CORR_CMDS_4G = 256U;

    //------------------------------------------------------------------------------------------------------------------
    // Bit fields for the 4G correlation Command queue register
    //
    ///
    /// @brief          Set when the correlator command queue is empty  (see
    ///                 Fpga::CORRELATOR_CMD_QUEUE_STATUS_4G_OFFSET).
    ///
    static const uint32_t CORR_INFO_4G_EMP = 1U << 8;

    ///
    /// @brief          Set when the correlator command queue is empty  (see
    ///                 Fpga::CORRELATOR_CMD_QUEUE_STATUS_4G_OFFSET).
    ///
    static const uint32_t CORR_INFO_4G_FULL = 1U << 9;

    ///
    /// @brief          Set if there is an attempt to write to the command queue when it was full  (see
    ///                 Cleared by writing to the OVF bit location with all other bits set to logic zero (see
    ///                 Fpga::CORRELATOR_CMD_QUEUE_STATUS_4G_OFFSET).
    ///
    static const uint32_t CORR_INFO_4G_OVF = 1U << 10;

    //------------------------------------------------------------------------------------------------------------------
    // Bit/Enums fields for the 4G correlation Command Status Fifo
    //
    ///
    /// @brief          Set when there is an unread, message waiting in the status FIFO (see
    ///                 Fpga::CORRELATOR_CMD_STATUS_INFO_4G_OFFSET).
    ///
    static const uint32_t CORR_CMD_STATUS_4G_RDY = 1U << 9;

    ///
    /// @brief          Set when command FIFO is full (see
    ///                 Fpga::CORRELATOR_CMD_STATUS_INFO_4G_OFFSET).
    ///
    static const uint32_t CORR_CMD_STATUS_4G_FULL = 1U << 10;

    ///
    /// @brief          Set if the status FIFO was full when the FPGA attempted to write another status message
    ///                 Cleared by writing to the OVF bit location with all other bits set to logic zero (see
    ///                 Fpga::CORRELATOR_CMD_STATUS_INFO_4G_OFFSET).
    ///
    static const uint32_t CORR_CMD_STATUS_4G_OVF = 1U << 11;

    //------------------------------------------------------------------------------------------------------------------
    // Bit fields for the 4G correlation results Fifo 
    //
    ///
    /// @brief          Value in bits 0:1 when COMMAND_REJECTED (see
    ///                 Fpga::CORRELATOR_CMD_STATUS_FIFO_4G_OFFSET).
    ///
    static const uint32_t CORR_FIFO_4G_REJECTED = 1U;
    ///
    /// @brief          Value in bits 0:1 when COMMAND_FETECHED_FOR_EXECUTION (see
    ///                 Fpga::CORRELATOR_CMD_STATUS_FIFO_4G_OFFSET).
    ///
    static const uint32_t CORR_FIFO_4G_FETCHED = 2U;
    ///
    /// @brief          Set when there is an unread detection waiting in the correlation results FIFO (see
    ///                 Fpga::CORRELATION_RESULT_INFO_4G_OFFSET).
    ///
    static const uint32_t CORR_RESULT_4G_RDY = 1U <<10;

    ///
    /// @brief          Set when correleation results FIFO has overflowed (see
    ///                 Fpga::CORRELATION_RESULT_INFO_4G_OFFSET).
    ///
    static const uint32_t CORR_RESULT_4G_OVF = 1U << 11;

    //------------------------------------------------------------------------------------------------------------------
    // Bit fields for the 4G sample time control register
    //
    ///
    /// @brief          Set to capture the sample time at that instant. (see
    ///                 Fpga::CORRELATION_MEAN_STATISTICS_BASE_4G_OFFSET).
    ///
    static const uint32_t SAMPLE_TIME_CTRL_4G_CAPTURE = 1U;

    //------------------------------------------------------------------------------------------------------------------
    // Bit fields for the 4G statistics control register
    //
    ///
    /// @brief          Set to clear the mean statistics block (see
    ///                 Fpga::CORRELATION_MEAN_STATISTICS_BASE_4G_OFFSET).
    ///
    static const uint32_t CORR_STATISTICS_CTRL_CLR = 1U;

    ///
    /// @brief          Set to enable statistics collection (see
    ///                 Fpga::CORRELATION_MEAN_STATISTICS_BASE_4G_OFFSET).
    ///
    static const uint32_t CORR_STATISTICS_CTRL_EN = 1U << 1;

    //------------------------------------------------------------------------------------------------------------------
    //  Bit fields for the buffer control and status register
    //

    ///
    /// @brief          Set by the FPGA to signal that an FFT overflow occurred (see
    ///                 Fpga::REG_SWING_BUFFER_CONTROL_OFFSET).
    ///
    static const unsigned FFT_OVERFLOW              = 1 << 4;

    ///
    /// @brief          Set by the software to signal that it has finished with the odd result (see
    ///                 Fpga::REG_SWING_BUFFER_CONTROL_OFFSET).
    ///
    static const unsigned PROC_FINISHED_ODD_BUFFER  = 1 << 3;

    ///
    /// @brief          Set by the software to signal that it has finished with the even result (see
    ///                 Fpga::REG_SWING_BUFFER_CONTROL_OFFSET).
    ///
    static const unsigned PROC_FINISHED_EVEN_BUFFER = 1 << 2;

    ///
    /// @brief          Set by the FPGA to signal that a new odd result is ready (see
    ///                 Fpga::REG_SWING_BUFFER_CONTROL_OFFSET).
    ///
    static const unsigned ODD_BUFFER_VALID          = 1 << 1;

    ///
    /// @brief          Set by the FPGA to signal that a new even result is ready (see
    ///                 Fpga::REG_SWING_BUFFER_CONTROL_OFFSET).
    ///
    static const unsigned EVEN_BUFFER_VALID         = 1 << 0;

    //------------------------------------------------------------------------------------------------------------------
    //  Details of the frame counter.
    //

    ///
    /// @brief          The maximum value of the FPGA's 16-bit frame counter (see Fpga::REG_EVEN_FRAME_COUNT_OFFSET
    ///                 and Fpga::REG_ODD_FRAME_COUNT_OFFSET).
    ///
    static const uint32_t MAX_FPGA_FRAME             = 0x0FFFF;

    ///
    /// @brief          The mask for accessing the frame number FIFO empty flag (see
    ///                 Fpga::REG_EVEN_FRAME_COUNT_OFFSET and Fpga::REG_ODD_FRAME_COUNT_OFFSET).
    ///
    static const uint32_t FPGA_FRAME_FIFO_EMPTY_MASK = 0x10000;

    ///
    /// @brief          The mask for accessing the frame number FIFO full flag (see
    ///                 Fpga::REG_EVEN_FRAME_COUNT_OFFSET and Fpga::REG_ODD_FRAME_COUNT_OFFSET).
    ///
    static const uint32_t FPGA_FRAME_FIFO_FULL_MASK  = 0x20000;

    //------------------------------------------------------------------------------------------------------------------
    //  Offsets of the MCB control and status registers.
    //

    ///
    /// @brief          The offset of the read address register.
    /// @details        Defines the memory address from which the MCB should read data on behalf of the software.
    ///
    static const unsigned MCB_READ_ADDRESS_OFFSET  = 0;

    ///
    /// @brief          The offset of the read FIFO register.
    /// @details        Data read from the memory by the MCB may be retrieved from the FIFO.
    ///
    static const unsigned MCB_READ_FIFO_OFFSET     = MCB_READ_ADDRESS_OFFSET +1;

    ///
    /// @brief          The offset of the read length register.
    /// @details        The value written to this register is one less than the number of 32-bit words to be read.
    ///                 Thus the maximum value of 63 will cause 64 words to be read.
    ///
    static const unsigned MCB_READ_LENGTH_OFFSET   = MCB_READ_FIFO_OFFSET    +1;

    ///
    /// @brief          The offset of the write address register.
    /// @details        Defines the memory address to which the MCB should write data on behalf of the software.
    ///
    static const unsigned MCB_WRITE_ADDRESS_OFFSET = MCB_READ_LENGTH_OFFSET  +1;

    ///
    /// @brief          The offset of the write FIFO register.
    /// @details        Data to be written to the memory by the MCB should be sent to the FIFO.
    ///
    static const unsigned MCB_WRITE_FIFO_OFFSET    = MCB_WRITE_ADDRESS_OFFSET+1;

    ///
    /// @brief          The offset of the write length register.
    /// @details        The value written to this register is one less than the number of 32-bit words to be
    ///                 written.  Thus the maximum value of 63 will cause 64 words to be written.
    ///
    static const unsigned MCB_WRITE_LENGTH_OFFSET  = MCB_WRITE_FIFO_OFFSET   +1;

    ///
    /// @brief          The offset of the MCB control register.
    /// @details        Structure:
    ///                 -   Bit   0       : set to instruct the MCB to perform a read operation.
    ///                 -   Bit   1       : set to instruct the MCB to perform a write operation.
    ///
    static const unsigned MCB_CONTROL_OFFSET       = MCB_WRITE_LENGTH_OFFSET +1;

    ///
    /// @brief          The offset of the MCB status register.
    /// @details        Structure:
    ///                 -   Bit   0       : command FIFO is full - no more commands will be accepted.
    ///                 -   Bit   1       : command FIFO is empty - more commands can be given.
    ///                 -   Bits  2 to  4 : reserved.
    ///                 -   Bit   5       : read FIFO overflow (cleared when status register is read).
    ///                 -   Bit   6       : read error (cleared when status register is read, MCB reset required).
    ///                 -   Bit   7       : read FIFO full.
    ///                 -   Bit   8       : read FIFO empty.
    ///                 -   Bits  9 to 15 : number of words in the read FIFO.
    ///                 -   Bits 16 to 20 : reserved.
    ///                 -   Bit  21       : write FIFO underrun (cleared when status register is read).
    ///                 -   Bit  22       : write error (cleared when status register is read, MCB reset required).
    ///                 -   Bit  23       : write FIFO full.
    ///                 -   Bit  24       : write FIFO empty.
    ///                 -   Bits 25 to 31 : number of words in the write FIFO.
    ///
    static const unsigned MCB_FIFO_STATUS_OFFSET   = MCB_CONTROL_OFFSET      +1;

    //------------------------------------------------------------------------------------------------------------------
    //  Bit fields for the MCB control and status registers
    //

    ///
    /// @brief          Set by the software to signal that a read operation should be carried out (see
    ///                 Fpga::MCB_CONTROL_OFFSET).
    ///
    static const unsigned MCB_READ_CMD     = 1 <<  0;

    ///
    /// @brief          Set by the software to signal that a write operation should be carried out (see
    ///                 Fpga::MCB_CONTROL_OFFSET).
    ///
    static const unsigned MCB_WRITE_CMD    = 1 <<  1;

    ///
    /// @brief          Signals a read overflow (see Fpga::MCB_FIFO_STATUS_OFFSET).
    ///
    static const unsigned MCB_RD_OVERFLOW  = 1 <<  5;

    ///
    /// @brief          Signals a read error (see Fpga::MCB_FIFO_STATUS_OFFSET).
    ///
    static const unsigned MCB_RD_ERROR     = 1 <<  6;

    ///
    /// @brief          Signals that the read FIFO is full (see Fpga::MCB_FIFO_STATUS_OFFSET).
    ///
    static const unsigned MCB_RD_FULL      = 1 <<  7;

    ///
    /// @brief          Signals that the read FIFO is empty (see Fpga::MCB_FIFO_STATUS_OFFSET).
    ///
    static const unsigned MCB_RD_EMPTY     = 1 <<  8;

    ///
    /// @brief          Signals a write underrun (see Fpga::MCB_FIFO_STATUS_OFFSET).
    ///
    static const unsigned MCB_WR_UNDERRUN  = 1 << 21;

    ///
    /// @brief          Signals a write error (see Fpga::MCB_FIFO_STATUS_OFFSET).
    ///
    static const unsigned MCB_WR_ERROR     = 1 << 22;

    ///
    /// @brief          Signals that the write FIFO is full (see Fpga::MCB_FIFO_STATUS_OFFSET).
    ///
    static const unsigned MCB_WR_FULL      = 1 << 23;

    ///
    /// @brief          Signals that the write FIFO is empty (see Fpga::MCB_FIFO_STATUS_OFFSET).
    ///
    static const unsigned MCB_WR_EMPTY     = 1 << 24;

    //------------------------------------------------------------------------------------------------------------------
    //  Bit fields for the GSM status register
    //

    ///
    /// @brief          The number of values in the GSM result FIFO.
    ///
    static const unsigned GSM_FIFO_COUNT     = 0x03FF;

    ///
    /// @brief          The GSM result FIFO's empty flag.
    ///
    static const unsigned GSM_FIFO_EMPTY     = 0x0400;

    ///
    /// @brief          The GSM result FIFO's full flag.
    ///
    static const unsigned GSM_FIFO_FULL      = 0x0800;

    ///
    /// @brief          The GSM result FIFO's underflow flag.
    ///
    static const unsigned GSM_FIFO_UNDERFLOW = 0x1000;

    ///
    /// @brief          The GSM result FIFO's overflow flag.
    ///
    static const unsigned GSM_FIFO_OVERFLOW  = 0x2000;

    //------------------------------------------------------------------------------------------------------------------
    //  Details of the memory.
    //

    ///
    /// @brief          The offset of the start of the coefficient memory in the general control and status register
    ///                 address space.
    ///
    static const unsigned CORR_BASE_COEFF_OFFSET       = 0x10000;

    ///
    /// @brief          The maximum number of coefficients per slot.
    ///
    static const unsigned CORR_MAX_NUM_COEFFS_PER_SLOT = 4096;

    ///
    /// @brief          The number of coefficents in a 32-bit word.
    ///
    static const unsigned CORR_NUM_COEFFS_PER_WORD32   = 2;

    ///
    /// @brief          The base address in the DDR of the even result buffer.
    ///
    static const unsigned CORR_EVEN_RESULT_BASE_ADDRESS = 0x00800000;

    ///
    /// @brief          The base address in the DDR of the odd result buffer.
    ///
    static const unsigned CORR_ODD_RESULT_BASE_ADDRESS  = 0x00c00000;

    //------------------------------------------------------------------------------------------------------------------
    //  Other constants.
    //

    ///
    /// @brief          The scaling factor to be applied to the channel filter RSSI power to account for the 3G FPGA
    ///                 implementation.
    /// @details        The 3G FPGA sums the sample powers over the period of a frame and divides the result by 2^18
    ///                 whereas the true mean would have been obtained by dividing by the number of samples in a
    ///                 frame.
    ///
    static const double CHAN_RSSI_CORRECTION_3G = 262144.0/Design::samples_per_frame;

    ///
    /// @brief          The scaling factor to be applied to the ADC RSSI power to account for the 3G FPGA
    ///                 implementation.
    /// @details        The 3G FPGA sums the sample powers over the period of a frame and divides the result by 2^9
    ///                 whereas the true mean would have been obtained by dividing by the number of samples in a
    ///                 frame.
    ///
    static const double ADC_RSSI_CORRECTION_3G = 512.0/Design::adc_samples_per_frame;

    ///
    /// @brief          The scaling factor to be applied to the ADC RSSI power to account for the GSM FPGA
    ///                 implementation.
    /// @details        The GSM FPGA sums the sample powers over the period of four frames and divides the result by 2^?
    ///                 whereas the true mean would have been obtained by dividing by the number of samples in four
    ///                 frame.
    ///
    static const double ADC_RSSI_CORRECTION_GSM = 512.0/(4.0*Design::ADC_SAMPLES_PER_FRAME_GSM);

    ///
    /// @brief          The result timeout in milliseconds.
    /// @details        If the FPGA takes longer than this to produce a new result then an error has occurred.
    ///
    static const double RESULT_TIMEOUT_VAL_MS = 1000.0;

    ///
    /// @brief          The number of samples in the array that spans the peak.
    /// @details        When interpolating the position of the peak a quadratic fit is used and so 3 samples are needed.
    ///
    static const unsigned SAMPLES_IN_PEAK_ARRAY = 3;

public:
    ///
    /// @brief          Maximum number of channel filter taps as programmed to FPGA in each stage
    /// @details        16-bit numbers rather than 32.
    ///                 Actual number of taps in channel filter = 63 = (NUM_CHANNEL_FILTER_TAP_VALS_4G * 2) + 1
    ///
    static const unsigned NUM_CHANNEL_FILTER_TAP_VALS_4G = 32;

public:
    ///
    /// @brief          Maximum number of correlator taps
    ///
    static const unsigned NUM_CORRELATOR_TAPS_4G = Design::NUM_CORRELATOR_TAPS_4G;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The types.
    //
public:
    ///
    /// @brief          Mode control register bits.
    ///
    typedef enum
    {
        ///
        /// @brief          Operational mode field: correlator idle
        ///
        OP_MODE_CORR_IDLE               = 0<<0,

        ///
        /// @brief          Operational mode field: correlator running with a 256-chip filter using samples from a
        ///                 single antenna.
        ///
        OP_MODE_CAL                     = 1<<0,

        ///
        /// @brief          Operational mode field: correlator running with a 2048-chip filter using samples from a
        ///                 single antenna.
        ///
        OP_MODE_SEARCH                  = 2<<0,

        ///
        /// @brief          Operational mode field: correlator running with a 2048-chip filter using samples from
        ///                 both antennas.
        ///
        OP_MODE_TRACK                   = 3<<0,

        ///
        /// @brief          Antenna mode field: enable antenna swapping, starting with antenna 1.
        ///
        ANT_MODE_SWAP_ANTS_START1       = 0<<2,

        ///
        /// @brief          Antenna mode field: disable antenna swapping, use antenna 1.
        ///
        ANT_MODE_ANT1                   = 1<<2,

        ///
        /// @brief          Antenna mode field: disable antenna swapping, use antenna 2.
        ///
        ANT_MODE_ANT2                   = 2<<2,

        ///
        /// @brief          Antenna mode field: enable antenna swapping, starting with antenna 2.
        ///
        ANT_MODE_SWAP_ANTS_START2       = 3<<2,

        ///
        /// @brief          Start the correlator.
        ///
        GEN_MODE_CORR_START             = 0x080,

        ///
        /// @brief          Enable the channel filter.
        ///
        GEN_MODE_CHAN_FILT_ENABLE       = 0x100,

        ///
        /// @brief          Reset the correlator.
        ///
        GEN_MODE_CORR_RESET             = 0x200,

        ///
        /// @brief          The correlator should produce power samples.
        /// @details        The correlator can produce IQ samples (2 x 16 bits) or power samples (1 x 32 bits).
        ///                 Only the power samples can be used with the summing functionality and the IQ mode is
        ///                 only used for testing.
        ///
        GEN_MODE_CORR_PWR_RESULT        = 0x400,

        ///
        /// @brief          The number of GSM channel power samples per slot.
        /// @details        A value of 1 configures the GSM power meter to produce one result per slot.  A value of
        ///                 0 produces three results per slot.
        ///
        GSM_MODE_ONE_RESULT_PER_SLOT    = 0x800,

        ///
        /// @brief          Enables the DC offset correction.
        ///
        GSM_MODE_ENABLE_DC_OFFSET       = 0x1000
    } mode_t;

    ///
    /// @brief          The result returned by the result_available() function.
    ///
    typedef enum
    {
        ///
        /// @brief          A matched-filter result is available.
        ///
        RESULT_AVAILABLE,

        ///
        /// @brief          A matched-filter result is not yet ready.
        ///
        RESULT_PENDING,

        ///
        /// @brief          It has taken too long to produce a matched-filter result and there is probably something
        ///                 wrong with the FPGA.
        /// @details        This was introduced to handle the FPGA lockup condition that was fixed in build 23 of
        ///                 the FPGA image.
        ///
        RESULT_TIMEOUT
    } result_avail_t;

    ///
    /// @brief          The peak information, including the samples around the peak.
    ///
    typedef struct
    {
        ///
        /// @brief          The frame number of the first frame that contributed to the reesult.
        ///
        int64_t     frame;

        ///
        /// @brief          The offset into the result buffer of the peak.
        ///
        uint32_t    sample_offset;

        ///
        /// @brief          The magnitude of the matched-filter results around the peak.
        ///
        double      peak[SAMPLES_IN_PEAK_ARRAY];
    } peak_array_t;

    ///
    /// @brief          The raw peak information.
    ///
    typedef struct
    {
        ///
        /// @brief          true if the peak power exceeds the threshold.
        ///
        bool        exceeds_threshold;

        ///
        /// @brief          The offset into the result buffer of the peak.
        ///
        uint32_t    sample_offset;

        ///
        /// @brief          The power of the peak.
        ///
        double      sample_power;
    } raw_peak_t;


    //------------------------------------------------------------------------------------------------------------------
    // Commands and fields for controlling the 4g Correlator (see Fpga::CORRELATOR_CMD_QUEUE_4G_OFFSET)
    //
    typedef enum CORRELATION_COMMANDS_4G {
        CORRELATE_CONTINUOUSLY = 1,
        STOP_IMMEDIATELY = 2,
        CORRELATE_FROM_TO = 3
    } eCorrCmd_4g;

    typedef struct CORRELATION_CMD_4G {
        eCorrCmd_4g cmd;
        uint8_t id;
        uint64_t start;
        uint64_t stop;
    } CorrCmd_4g_t;

    //------------------------------------------------------------------------------------------------------------------
    // Commands and fields for determing the status of the 4g Correlator (see Fpga::CORRELATOR_CMD_STATUS_FIFO_4G_OFFSET)
    //
    typedef enum CORRELATION_COMMAND_STATUS_4G {
        UNUSED0 = 0,
        REJECTED = 1,
        FETCHED = 2,
        UNUSED3 = 3
    } eCorrCmdStatus_4g;
    // eCorrCmdStatus_4g is in lowest 2 bits of the register
    static const uint32_t CORR_CMD_STATUS_FIFO_STATUS_MASK_4G = 0x3;
    static const unsigned int CORR_CMD_STATUS_FIFO_STATUS_BITS_4G = 2U;

    typedef struct CORRELATION_CMD_STATUS_4G {
        eCorrCmdStatus_4g status;
        uint8_t id;
    } CorrCmdStatus_4g_t;

    //------------------------------------------------------------------------------------------------------------------
    // structure to return processed results derived from Corrleator Peaks Fifo (see Fpga::CORRELATION_RESULT_FIFO_BASE_4G_OFFSET)
    typedef struct CORRELATION_PEAK_4G
    {
        uint64_t time;
        float CorrPeakNum;
        float CorrPeakDenSqrd;
        float SumCorrInpAtPeak;
    } CorrPeak_4g_t;

private:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The values.
    //
private:
    //------------------------------------------------------------------------------------------------------------------
    //  Addresses of general control and status registers common for all FPGAs
    //

    ///
    /// @brief          The base address of the memory mapped control and status registers.
    ///
    volatile uint32_t* reg_base;

    ///
    /// @brief          The address of the firmware revision register (see Fpga::REG_FIRMWARE_REV_OFFSET).
    ///
    volatile uint32_t* reg_firmware_rev_ptr;

    ///
    /// @brief          The address of the test control register (see Fpga::REG_TEST_REGISTER_OFFSET).
    ///
    volatile uint32_t* reg_test_register_ptr;

    //------------------------------------------------------------------------------------------------------------------
    //  Addresses of the general control and status registers for GSM and 3G FPGAs
    //

    ///
    /// @brief          The address of the mode control register (see Fpga::REG_MODE_OFFSET).
    ///
    volatile uint32_t* reg_mode_ptr;

    ///
    /// @brief          The address of the averaging control register (see Fpga::REG_AVERAGE_CTRL_OFFSET).
    ///
    volatile uint32_t* reg_average_ctrl_ptr;

    ///
    /// @brief          The address of the correlation start-offset register (see
    ///                 Fpga::REG_CORRELATION_START_OFFSET).
    ///
    volatile uint32_t* reg_start_ptr;

    ///
    /// @brief          The address of the correlation length register (see Fpga::REG_CORRELATION_LENGTH_OFFSET).
    ///
    volatile uint32_t* reg_length_ptr;

    ///
    /// @brief          The address of the peak detector threshold register (see Fpga::REG_THRESHOLD_OFFSET).
    ///
    volatile uint32_t* reg_threshold_ptr;

    ///
    /// @brief          The address of the buffer control and status register (see
    ///                 Fpga::REG_SWING_BUFFER_CONTROL_OFFSET).
    ///
    volatile uint32_t* reg_swing_buffer_ctrl_ptr;

    ///
    /// @brief          The address of the antenna swap control register (see Fpga::REG_ANT_SW_CTRL_OFFSET).
    ///
    volatile uint32_t* reg_ant_sw_ctrl_ptr;

    ///
    /// @brief          The address of the even-result frame number register (see
    ///                 Fpga::REG_EVEN_FRAME_COUNT_OFFSET).
    ///
    volatile uint32_t* reg_even_frame_count_ptr;

    ///
    /// @brief          The address of the even-result peak address register (see
    ///                 Fpga::REG_EVEN_PEAK_ADDRESS_OFFSET).
    ///
    volatile uint32_t* reg_even_peak_address_ptr;

    ///
    /// @brief          The address of the even-result peak power register (see Fpga::REG_EVEN_PEAK_DATA_OFFSET).
    ///
    volatile uint32_t* reg_even_peak_data_ptr;

    ///
    /// @brief          The address of the even-result "number of samples over the threshold" register (see
    ///                 Fpga::REG_EVEN_NUM_PEAKS_OFFSET).
    ///
    volatile uint32_t* reg_even_num_peaks_ptr;

    ///
    /// @brief          The address of the channel filter RSSI register (see Fpga::REG_CHAN_RSSI_OFFSET).
    ///
    volatile uint32_t* reg_chan_rssi_ptr;

    ///
    /// @brief          The address of the state machine debug register 1 (see Fpga::REG_FSM_DEBUG1_OFFSET).
    ///
    volatile uint32_t* reg_fsm_debug1_ptr;

    ///
    /// @brief          The address of the 1 PPS event-time register (see Fpga::REG_PPS_COUNT_OFFSET).
    ///
    volatile uint32_t* reg_1pps_ptr;

    ///
    /// @brief          The address of the FFT gain control register (see Fpga::REG_FFT_GAIN_CTRL_OFFSET).
    ///
    volatile uint32_t* reg_fft_gain_control_ptr;

    ///
    /// @brief          The address of the state machine debug register 2 (see Fpga::REG_FSM_DEBUG2_OFFSET).
    ///
    volatile uint32_t* reg_fsm_debug2_ptr;

    ///
    /// @brief          The address of the odd-result frame number register (see Fpga::REG_ODD_FRAME_COUNT_OFFSET).
    ///
    volatile uint32_t* reg_odd_frame_count_ptr;

    ///
    /// @brief          The address of the odd-result peak address register (see Fpga::REG_ODD_PEAK_ADDRESS_OFFSET).
    ///
    volatile uint32_t* reg_odd_peak_address_ptr;

    ///
    /// @brief          The address of the odd-result peak power register (see Fpga::REG_ODD_PEAK_DATA_OFFSET).
    ///
    volatile uint32_t* reg_odd_peak_data_ptr;

    ///
    /// @brief          The address of the odd-result "number of samples over the threshold" register (see
    ///                 Fpga::REG_ODD_NUM_PEAKS_OFFSET).
    ///
    volatile uint32_t* reg_odd_num_peaks_ptr;

    ///
    /// @brief          The address of the ADC RSSI register (see Fpga::REG_ADC_RSSI_OFFSET).
    ///
    volatile uint32_t* reg_adc_rssi_ptr;

    ///
    /// @brief          The address of the GSM result FIFO register (see Fpga::REG_GSM_RESULT_FIFO_OFFSET).
    ///
    volatile uint32_t* reg_gsm_result_fifo_ptr;

    ///
    /// @brief          The address of the GSM result status register (see Fpga::REG_GSM_RESULT_STATUS_OFFSET).
    ///
    volatile uint32_t* reg_gsm_result_status_ptr;

    ///
    /// @brief          The address of the GSM DC offset register (see Fpga::REG_GSM_DC_OFFSET).
    ///
    volatile uint32_t* reg_gsm_dc_offset_ptr;

    //------------------------------------------------------------------------------------------------------------------
    //  Addresses of the general control and status registers for 4G FPGA
    //

    ///
    /// @brief          The address of the mode control register (see Fpga::REG_TEST_REGISTER_4G_OFFSET).
    ///
    volatile uint32_t* reg_test_register_4g_ptr;

    ///
    /// @brief          The address of the antenna selection register (see Fpga::REG_ANTENNA_SELECT_4G_OFFSET).
    ///
    volatile uint32_t* reg_antenna_select_4g_ptr;

    ///
    /// @brief          The address of the sample time control register (see Fpga::REG_SAMPLE_TIME_4G_OFFSET).
    ///
    volatile uint32_t* reg_sample_time_ctrl_4g_ptr;

    ///
    /// @brief          The address of the sample time least-significant word register (see Fpga::REG_SAMPLE_TIME_4G_OFFSET).
    ///
    volatile uint32_t* reg_sample_time_lsw_4g_ptr;

    ///
    /// @brief          The address of the sample time most-significant word register (see Fpga::REG_SAMPLE_TIME_4G_OFFSET).
    ///
    volatile uint32_t* reg_sample_time_msw_4g_ptr;

    ///
    /// @brief          The address of the channel filter stage 1 taps for the 4G FPGA (see Fpga::CHANNEL_FILTER_1_4G_OFFSET)
    ///
    volatile uint32_t* channel_filter_1_4g_ptr;

    ///
    /// @brief          The address of the channel filter stage 2 taps for the 4G FPGA (see Fpga::CHANNEL_FILTER_2_4G_OFFSET)
    ///
    volatile uint32_t* channel_filter_2_4g_ptr;

    ///
    /// @brief          The address of the Floating point format test register (see Fpga::REG_FLOAT_FORMAT_TEST_4G_OFFSET).
    ///
    volatile uint32_t* reg_float_format_test_4g_ptr;

    ///
    /// @brief          The address of the correlator power multiplier offset (see Fpga::CORRELATOR_PWR_MULTIPLICATION_4G_OFFSET).
    ///
    volatile uint32_t* reg_correlator_threshold_sqrd_4g_ptr;

    ///
    /// @brief          The address of the correlator command queue offset (see Fpga::CORRELATOR_CMD_QUEUE_4G_OFFSET).
    ///
    volatile uint32_t* reg_correlator_cmd_queue_4g_ptr;

    ///
    /// @brief          The address of the correlator command queue start time (see Fpga::CORRELATOR_CMD_START_TIME_4G_OFFSET).
    ///
    volatile uint64_t* reg_correlator_cmd_queue_start_4g_ptr;

    ///
    /// @brief          The address of the correlator command queue stop time (see Fpga::CORRELATOR_CMD_STOP_TIME_4G_OFFSET).
    ///
    volatile uint64_t* reg_correlator_cmd_queue_stop_4g_ptr;

    ///
    /// @brief          The address of the correlator command queue status register (see Fpga::CORRELATOR_CMD_QUEUE_STATUS_4G_OFFSET).
    ///
    volatile uint32_t *reg_correlator_cmd_queue_status_ptr;

    ///
    /// @brief          The address of information about the status of the correlator command queue fifo itself (see Fpga::CORRELATOR_CMD_STATUS_INFO_4G_OFFSET).
    ///
    volatile uint32_t *reg_correlator_cmd_status_info_ptr;

    ///
    /// @brief          The address of the correlator command queue status fifo (see Fpga::CORRELATOR_CMD_STATUS_FIFO_4G_OFFSET).
    ///
    volatile uint32_t *reg_correlator_cmd_status_fifo_ptr;

    ///
    /// @brief          The address of the correlator results fifo information (see Fpga::CORRELATION_RESULT_INFO_4G_OFFSET).
    ///
    volatile uint32_t *reg_correlation_result_info_ptr;

    ///
    /// @brief          The address of result components in the correlator results fifo (see Fpga::CORRELATION_RESULT_FIFO_BASE_4G_OFFSET).
    ///
    volatile uint32_t *reg_correlation_result_fifo_ls_time_ptr;
    volatile uint32_t *reg_correlation_result_fifo_ms_time_ptr;
    volatile uint32_t *reg_correlation_result_fifo_correl_num_ptr;
    volatile uint32_t *reg_correlation_result_fifo_correl_den_ptr;
    volatile uint32_t *reg_correlation_result_fifo_sum_pwr_ptr;

    ///
    /// @brief          The address of the correlator statistics block (see Fpga::CORRELATION_MEAN_STATISTICS_BASE_4G_OFFSET).
    ///
    volatile uint32_t* reg_correlation_statistics_control_ptr;

    ///
    /// @brief          The address of the number means in the correlator statistics block (see Fpga::CORRELATION_MEAN_STATISTICS_BASE_4G_OFFSET).
    ///
    volatile uint32_t* reg_correlation_statistics_num_means_ptr;

    ///
    /// @brief          The address of the peak power in the correlator statistics block (see Fpga::CORRELATION_MEAN_STATISTICS_BASE_4G_OFFSET).
    ///
    volatile uint32_t* reg_correlation_statistics_peak_power_ptr;

    ///
    /// @brief          The address of the sum of powers in the correlator statistics block (see Fpga::CORRELATION_MEAN_STATISTICS_BASE_4G_OFFSET).
    ///
    volatile uint32_t* reg_correlation_statistics_sum_power_ptr;

    ///
    /// @brief          The addresses of the correlator taps for the 4G FPGA
    /// @details        32-bit value containing 24-bits of data in format IEEE-754 with 8-bit mantissa
    ///
    volatile uint32_t* correlator_taps_4G[NUM_CORRELATOR_TAPS_4G];

    ///
    /// @brief          The address of the mode control register (see Fpga::REG_REGRESSION_TEST_CONTROL_4G_OFFSET).
    ///
    volatile uint32_t* reg_regression_test_ctrl_4g_ptr;

    ///
    /// @brief          The address of the Fpga regression test sample input register (see Fpga::REG_REGRESSION_TEST_VALUE_4G_OFFSET).
    ///
    volatile uint32_t* reg_regression_test_value_4g_ptr;

    //------------------------------------------------------------------------------------------------------------------
    //  Addresses of the MCB control and status registers.
    //

    ///
    /// @brief          The address of the read address register (see Fpga::MCB_READ_ADDRESS_OFFSET).
    ///
    volatile uint32_t* mcb_read_address_ptr;

    ///
    /// @brief          The address of the read FIFO register (see Fpga::MCB_READ_FIFO_OFFSET).
    ///
    volatile uint32_t* mcb_read_fifo_ptr;

    ///
    /// @brief          The address of the read length register (see Fpga::MCB_READ_LENGTH_OFFSET).
    ///
    volatile uint32_t* mcb_read_length_ptr;

    ///
    /// @brief          The address of the write address register (see Fpga::MCB_WRITE_ADDRESS_OFFSET).
    ///
    volatile uint32_t* mcb_write_address_ptr;

    ///
    /// @brief          The address of the write FIFO register (see Fpga::MCB_WRITE_FIFO_OFFSET).
    ///
    volatile uint32_t* mcb_write_fifo_ptr;

    ///
    /// @brief          The address of the write length register (see Fpga::MCB_WRITE_LENGTH_OFFSET).
    ///
    volatile uint32_t* mcb_write_length_ptr;

    ///
    /// @brief          The address of the MCB control register (see Fpga::MCB_CONTROL_OFFSET).
    ///
    volatile uint32_t* mcb_control_ptr;

    ///
    /// @brief          The address of the MCB status register (see Fpga::MCB_FIFO_STATUS_OFFSET).
    ///
    volatile uint32_t* mcb_fifo_status_ptr;

    //------------------------------------------------------------------------------------------------------------------
    //  General member variables.
    //

    ///
    /// @brief          The addresses of the coefficients for each slot.
    ///
    volatile uint32_t* coeff_base[Design::MAX_CORRELATIONS_PER_FRAME];

    ///
    /// @brief          Indicates type of the currently loaded image.
    ///
    eSystemMode cur_mode;

    ///
    /// @brief          The flag that indicates if the frame data is valid.
    /// @details        The frame data are the values that are read from the FPGA at the same time as the frame
    ///                 number and are cached by the software.  The cached values become invalid when the
    ///                 software releases the current result buffer back to the FPGA.  The variables covered by
    ///                 the flag are: Fpga::num_samples_over_threshold, Fpga::peak_sample_offset and
    ///                 Fpga::peak_data.
    ///
    bool frame_data_valid;

    ///
    /// @brief          The number of samples in the current result that are greater than the threshold.
    /// @details        This is cached by the software and only valid while Fpga::frame_data_valid is true.
    ///
    uint32_t num_samples_over_threshold;

    ///
    /// @brief          The offset in the current result buffer of the peak.
    /// @details        This is cached by the software and only valid while Fpga::frame_data_valid is true.
    ///
    uint32_t peak_sample_offset;

    ///
    /// @brief          The power of the peak in the current result buffer.
    /// @details        This is cached by the software and only valid while Fpga::frame_data_valid is true.
    ///
    uint32_t peak_pwr;

    ///
    /// @brief          Flag that indicates if the 64-bit frame number (Fpga::big_frame_count) has been initialised.
    ///
    bool big_frame_count_started;

    ///
    /// @brief          The 64-bit frame number maintained by the software (see update_big_frame_count()).
    ///
    int64_t big_frame_count;

    ///
    /// @brief          The last FPGA frame number that was read.
    /// @details        This is used to maintain the 64-bit frame number (Fpga::big_frame_count), by allowing the
    ///                 software to detect that the FPGA frame number has wrapped (see update_big_frame_count()).
    ///
    uint32_t last_fpga_frame;

    ///
    /// @brief          The time when the last FPGA frame number was read.
    /// @details        This is used to maintain the 64-bit frame number (Fpga::big_frame_count), by allowing the
    ///                 software to calculate how many times the FPGA frame number has wrapped (see
    ///                 update_big_frame_count()).
    ///
    struct timeval time_of_last_frame_update;

    ///
    /// @brief          The flag that indicates that the software is waiting for the even result to be completed.
    /// @details        If false, the software is waiting for the odd result to be completed.
    ///
    bool proc_waiting_for_even_buffer;

    ///
    /// @brief          The time when the current measure was last (re-)started by a call to setup_correlation().
    /// @details        Only used in printing the elapsed measurement time.
    ///
    struct timeval start_time;

    ///
    /// @brief          The time when the last result buffer was released to the FPGA by a call to pop_result().
    /// @details        Used to implement a timeout so as to detect an error in the FPGA (see
    ///                 Fpga::RESULT_TIMEOUT_VAL_MS and result_available()).
    ///
    struct timeval last_pop_time;

    ///
    /// @brief          The debug stream.
    /// @details        Debug messages are sent to this stream.  If the debugging level has been appropriately
    ///                 configured they appear on the standard output console.  See ::DebugStream.
    ///
    DebugStream debugStr;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The default constructor is declared but not defined in order to prevent a compiler generated
    ///                 version appearing.
    ///
private:
    Fpga(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Constructs an instance of the class.
    /// @details        Initialises that pointers that are used to access the memory mapped registers.  The hardware
    ///                 addresses are mapped by the mmap function into software addresses that are stored in the
    ///                  pointers.  Two groups of hardware addresses are used:
    ///                 -   The general control registers are mapped into the region enabled by chip select 0.
    ///                 -   The DDR memory interface registers are mapped into the region enabled by chip select 1.
    /// @param[in]      debug               Controls the debug messages that may be generated.
    ///
public:
    explicit Fpga(unsigned debug);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The destructor has nothing to do.
    ///
public:
    virtual ~Fpga();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Load the FPGA image.
    /// @details        Automatically detects whether the image uses big or little endian bytes in halfwords and
    ///                 bit- reversal in each byte based on the sync words that should be at the begining of the
    ///                 file.
    /// @param[in]      filename            The name of the ".bin" format image file.
    /// @param[in]      mode                GSM, 3G or 4G?
    ///
public:
    void load_image(std::string filename, eSystemMode mode);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Reset the FPGA image.
    ///
public:
    void reset_image(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Reads the current channel power result.
    /// @details        The channel power is measured at the output of the channl filter and is in units of channel
    ///                 filter LSB^2.
    /// @return                             The channel power.
    ///
public:
    double get_channel_power(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Reads the current ADC power result.
    /// @details        The ADC power is measured at the output of the ADC and is in units of ADC LSB^2.
    /// @return                             The ADC power.
    ///
public:
    double get_adc_power(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Writes the correlator coefficients to the FPGA.
    /// @details        Converts the 32-bit coefficient components into 8-bit values, packs them into a 32-bit words
    ///                 and writes them to the FPGA coefficient memory.
    /// @param[in]      data                Vector of complex integer values to load
    /// @param[in]      slot                Slot number to send coeffients to
    ///
public:
    void send_coefs(const std::vector<std::complex<int32_t> > &data, int slot);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Configure the correlator engine.
    /// @details        Reset the engine and then leave it idle while writing the setup to the control registers.
    ///                 Reset the pending buffer flag, start the engine, record the time and mark the frame data as
    ///                 being invalid.
    /// @param[in]      mode                Mode bits controlling size/type of correlation.
    /// @param[in]      fft_gains           The FFT gain schedule.
    /// @param[in]      start               Offset in samples from fpga clock frame/sample counter.
    /// @param[in]      slots               Number of slots per frame to correlate and add non-coherently.
    /// @param[in]      frames              Number of frames to correlate and add non-coherently.
    /// @param[in]      threshold           For correlation peak detector.
    ///
public:
    void setup_correlation(uint32_t mode,
                           uint32_t fft_gains,
                           uint32_t start,
                           uint32_t slots,
                           uint32_t frames,
                           uint32_t threshold);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Determines if a new correlator is available.
    /// @details        Calculate the time for which the FPGA has been working on this results so that a timeout
    ///                 can be declared if necessary.  Check the engine's result status to determine if the expected
    ///                 result is available and if not check if a timeout should be asserted.
    /// @return                             The status of the new result, available, pending or timeout.
    ///
public:
    result_avail_t result_available(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Releases the current result buffer back to the FPGA.
    /// @details        Determine which result buffer is currently being used by the software, release it back to
    ///                 the FPGA and record the fact that the software must now wait for it to be refilled before
    ///                 using it again.  Also store the current time so that the time taken to produce the next
    ///                 result can be checked and mark the frame data as being invalid
    ///
public:
    void pop_result(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Get the frame number for the current result.
    /// @details        If the cached value is valid it is returned, otherwise an exception is thrown.
    /// @return                             The frame number of the first frame that contributed to the current
    ///                                     result.
    ///
public:
    int64_t get_frame(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Gets the peak data in the current correlator result including the samples around it.
    /// @details        Gets the position of the peak from the software cache, then reads the three samples that
    ///                 span the true peak.  Fill out the result with the frame number of the first frame that
    ///                 contributed to the result.
    /// @param[out]     peak_array          The destination for the result.
    ///
public:
    void get_peak_array(peak_array_t &peak_array);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Gets the details of the peak in the current correlator result.
    /// @details        Reads the peak data for the current result from the software cache.  If the number of
    ///                 samples over the threshold is greater than zero the peak is valid and its details are added
    ///                 to the result.
    /// @param[out]     raw_peak            The raw peak data.
    /// @return                             The frame number of the first frame that contributed to the result.
    //
public:
    int64_t get_peak_data(raw_peak_t &raw_peak);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Read from the DDR memory.
    /// @details        The memory is read by requesting the data from the FPGA's Memory Controller Block (MCB) in
    ///                 batches of up to 64 words.  The MCB writes the words into a FIFO from where the software may
    ///                 read it.
    ///
    ///                 The function first checks that the MCB is ready to process a new read command and that no
    ///                 errors have occurred.  If there is a problem it asserts an error, otherwise it begins the
    ///                 read process.
    ///
    ///                 While more the 64 words are required the read loop requests 64 words and waits for the FIFO
    ///                 to be full before reading the new words.  If on the final iteration fewer than 64 words are
    ///                 needed then FIFO is read while it is not empty.
    ///
    /// @param[in]      address             The start address.
    /// @param[in]      length              The number of 32-bit words to read.
    /// @param[out]     data                The destination for the data.
    ///
public:
    void mem_raw_read(uint32_t address, uint32_t length, std::vector<uint32_t> &data);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Write to the DDR memory.
    /// @details        The memory is written to by requesting FPGA's Memory Controller Block (MCB) to write it in
    ///                 batches of up to 64 words.  The software writes the words into a FIFO from where the MCB
    ///                 reads it.
    ///
    ///                 The function first checks that the MCB is ready to process a new write command and that no
    ///                 errors have occurred.  If there is a problem it asserts an error, otherwise it begins the
    ///                 write process.
    ///
    ///                 The function writes the data into the FIFO until it is full orthere are no more words to
    ///                 write.  It then requests MCB to carry out the write operation and waits for the FIFO to be
    ///                 emptied.
    ///
    /// @param[in]      address             The start address.
    /// @param[in]      data                The data to be written.
    ///
public:
    void mem_raw_write(uint32_t address, const std::vector<uint32_t> &data);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Write to an FPGA register in chip select 0 address space.
    /// @param[in]      offset              The offset of the register from the base address.
    /// @param[in]      data                The destination for the data.
    ///
public:
    void debug_reg_write(uint32_t offset, uint32_t data);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Read from an FPGA register in chip select 0 address space.
    /// @param[in]      offset              The offset of the register from the base address.
    /// @return                             The register value.
    ///
public:
    uint32_t debug_reg_read(uint32_t offset);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Reads a block of IQ result samples.
    /// @details        Reads the samples from the result buffer as 32-bit words then separates the 16-bit
    ///                 components and constructs the result samples as complex doubles.
    /// @param[in]      start               Offset in samples from the start of the result buffer.
    /// @param[in]      len                 The number of samples to read.
    /// @param[in]      alt_ant             true if the samples for the alternate antenna should be read, false
    ///                                     otherwise.
    /// @param[out]     data                The destination for the samples.
    /// @return                             The frame number of the first frame that contributed to the result.
    ///
public:
    int64_t get_raw_iq_mode_corr_data(uint32_t                            start,
                                      uint32_t                            len,
                                      bool                                alt_ant,
                                      std::vector<std::complex<double> >& data);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Reads a block of power result samples.
    /// @details        Reads the samples from the result buffer as 32-bit words and then converts them to doubles.
    /// @param[in]      start               Offset in samples from the start of the result buffer.
    /// @param[in]      len                 The number of samples to read.
    /// @param[in]      alt_ant             true if the samples for the alternate antenna should be read, false
    ///                                     otherwise.
    /// @param[out]     data                The destination for the samples.
    /// @return                             The frame number of the first frame that contributed to the result.
    ///
public:
    int64_t get_raw_pwr_mode_corr_data(uint32_t             start,
                                       uint32_t             len,
                                       bool                 alt_ant,
                                       std::vector<double>& data);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Get sample counter value of the last 1pps event.
    /// @details        The time of the 1 pps event is latched by the hardware.  It is measured by a 25-bit counter
    ///                 clocked by the 26 MHz reference clock.  Thus the value wraps every 1.27 s.  As long as this
    ///                 function is called at least every 100 ms the interval between events can be unambiguously
    ///                 calculated.
    ///
    ///                 The FPGA interface reads the latch register as separate high and low half words, and so if
    ///                 an event occurs during the read a false value may be read.  However an a change in the
    ///                 latched value always indicates an event and a second read will produce the correct counter
    ///                 value.
    /// @param[out]     count               The number of 26 MHz clock cycles bteween the last two 1pps events.
    /// @return                             true if an event has occurred since the last time that this function
    ///                                     was called.
    ///
public:
    bool get_1pps_count(uint32_t &count);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Set a new threshold for the peak detection.
    /// @param[in]      threshold           The new threshold value.
    ///
public:
    void set_threshold(uint32_t threshold);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Get the FPGA image version number.
    /// @return                             The version number.
    ///
public:
    uint8_t get_version(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Get the sample of the peak in the current result.
    /// @details        If the cached value is valid it is returned, otherwise an exception is thrown.
    /// @return                             The sample offset of the peak.
    ///
public:
    uint32_t get_peak_sample_offset(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Set the test register word.
    /// @param[in]      val                 The new value.
    ///
public:
    void set_test_register(uint32_t val);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Enter low power mode.
    ///
public:
    void enter_low_power_mode(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Exit low power mode.
    ///
public:
    void exit_low_power_mode(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Configure the GSM power meter engine.
    /// @details        Reset the engine and then start it.
    /// @param[in]      one_result_per_slot True if one power results per slot is required, false if three are needed.
    /// @param[in]      antenna_mode        The antenna mode to be used.
    ///
public:
    void setup_gsm_pwr_meter(bool one_result_per_slot, unsigned antenna_mode);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Empty the GSM result FIFO.
    /// @details        The FIFO is read until it is empty and no errors are issued if an overflow or underflow is
    ///                 detected.
    /// @return                             The number of discarded samples.
    ///
public:
    unsigned empty_gsm_fifo(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Read from the GSM result FIFO.
    /// @details        The FIFO is read until it is empty or the number of required samples have been read.
    ///
    /// @param[in]      max_num_samples     The maximum number of 32-bit words to read.
    /// @param[out]     data                The destination for the data.
    ///
public:
    void read_gsm_fifo(uint32_t max_num_samples, std::vector<uint32_t>& data);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Gets the DC offset estimates.
    /// @param[out]     inph_dc             The DC offset of the in-phase component.
    /// @param[out]     quad_dc             The DC offset of the quadrature component.
    ///
public:
    void get_dc_offset(int16_t &inph_dc, int16_t &quad_dc);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Sets the DC offset estimates.
    /// @param[in]      inph_dc             The DC offset of the in-phase component.
    /// @param[in]      quad_dc             The DC offset of the quadrature component.
    ///
public:
    void set_dc_offset(int16_t inph_dc, int16_t quad_dc);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Set the debug verbosity level.
    /// @param[in]      level               The new verbosity level.
    ///
public:
    void debug_printing(unsigned level);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Writes the coefficients for the channel filters and correlator for the 4G fpga
    /// @details        If there is a fault with any of the supplied coefficients nothing is progrrammed to the fpga
    /// @param[in]      channel1    Array of int16_t with parameters for channel filter stage 1
    /// @param[in]      channel2    Array of int16_t with parameters for channel filter stage 1
    /// @param[in]      correl_taps Array of 320 correlator taps in IEEE 754 binary32 format
    /// @returns        false if there is a problem with any of the supplied parameters/coefficients
    ///
public:
    bool send_4g_coefs(const std::vector<int16_t>&channel1,
                       const std::vector<int16_t>&channel2,
                       const std::vector<float>&correl_taps);

    /// @brief          Writes the squared threshold to be used in the correlator peak detection
    /// @param[in]      threshold_sqrd   float squared threshold
    ///
    ///
    /// Note on formatting.  The FPGA expects IEEE-754 format
public:
    void set_4g_threshold_sqrd(float threshold_sqrd);

    /// @brief          Disable statistics engine.  Read results.  Reset statistics engine if requested
    /// @param[in]      restart  Restart engine afterwards?
    /// @param[out]     peak     Statistics engine peak values
    /// @param[out]     sum      Statistics engine sum of values
    /// @returns        Number of values contributing to the results
    ///
public:
    uint32_t get_4g_statistics(bool restart, float &peak, float &sum);

    /// @brief          Starts a sequence of correlation commands
    /// @param[in]      cmds  a list of CorrCmd_4g_t type
    /// @returns        unsigned, the number of commands that were written
    ///
public:
    uint32_t setup_4g_correlation(std::list<CorrCmd_4g_t> cmds);

    /// @brief          Reads the status of recent colleration commands
    /// @param[out]     cmd_status a list of CorrCmdStatus_4g_t type showing command ids and associated status
    /// @returns        bool, "false" if the FIFO's overflow flag is set
    ///
public:
    bool get_4g_correlation_command_status(std::list<CorrCmdStatus_4g_t> &cmd_status);

    /// @brief          Reads the status of recent colleration commands
    /// @param[in]      maxNumPeaks the maximum number of peaks to write into the destination array
    /// @param[out]     peaks array of peaks read from the peaks fifo
    /// @param[out]     numPeaks actual number of peaks placed into peaks array
    /// @returns        bool, "false" if the FIFO's overflow flag is set
    ///
public:
    bool get_4g_peak_data(size_t maxNumPeaks, CorrPeak_4g_t peaks[], uint32_t &numPeaks);


public:
    /// @brief          Reads the current FPGA time
    /// @returns        Current FPGA time in a uint64_t
    ///
    uint64_t get_4g_time(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Set the antenna input of the 4G FPGA
    /// @details        Uses the previously defined values of ANT_MODE_ANT1 and ANT_MODE_ANT2 from the gsm
    ///                 antenna mode selection.
    ///                 All other values other than ANT_MODE_ANT1 and ANT_MODE_ANT2 are invalid and will
    ///                 result in the system displaying an error message
    /// @param[in]     antenna_mode    ANT_MODE_ANT1 or ANT_MODE_ANT2 only
    ///
public:
    void set_4g_antenna(uint32_t antenna_mode);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief         Stop any ongoing correlation
    ///
public:
    void stop_correlation(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Report whether or not the FPGA has finished using the even buffer.
    /// @return         true if the FPGA has finished using the even buffer.
    ///
private:
    bool fpga_finished_with_even_buffer(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Report whether or not the FPGA has finished using the odd buffer.
    /// @return         true if the FPGA has finished using the odd buffer.
    ///
private:
    bool fpga_finished_with_odd_buffer(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Set the controls that indicate that the FPGA may use the even buffer.
    ///
private:
    void fpga_may_use_even_buffer(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Set the controls that indicate that the FPGA may use the odd buffer.
    ///
private:
    void fpga_may_use_odd_buffer(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Reverse the order of the bits in a byte.
    /// @param[in, out] b               The byte to be modified.
    ///
private:
    void swap_bits(uint8_t& b);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Load the FPGA image using the WEIM bus.
    /// @details        The WEIM bus is mapped into the chip-select 3 address space so the function first maps this
    ///                 onto a range of corresponding software addresses.  It then sets the FPGA image reset line so
    ///                 that the non-essential essential elements in the image are held in reset even after it has
    ///                 been loaded.
    ///
    ///                 The programming of the image is started when the function clears the PROGRAM_B GPIO pin,
    ///                 which resets the FPGA device.  It then forces the configuration lines to be inputs by
    ///                 clearing the RDWR_B GPIO pin, before bringing the FPGA device out of reset by setting the
    ///                 PROGRAM_B GPIO pin.
    ///
    ///                 The function waits for the INIT_B GPIO pin to be driven high, a signal the the FPGA is ready
    ///                 to be programmed.  It then writes the image, adjusting the bit and byte ordering as
    ///                 necessary and when it is finished it clears FPGA image reset line.
    //
    ///                 Finally the fuunction waits for the DONE GPIO pin to be driven high, a signal that the FPGA
    ///                 has been configured.  It then reads the status of the CRC and reports its value (the
    ///                 software used to abort if the CRC failed, but experience has shown that the signal is
    ///                 unreliable) and unmaps the WEIM bus.
    ///
    /// @param[in]      fin                 The file stream to get data from.
    /// @param[in]      swap                true if the bit-ordering in the bytes needs to be reversed, false
    ///                                     otherwise.
    /// @param[in]      bigend              true if the bytes in half words are in big-endian order, false
    ///                                     otherwise.
    /// @param[in]      mode                GSM, 3G or 4G?
    ///
private:
    void send_config_weim(std::ifstream& fin, bool swap, bool bigend, eSystemMode mode);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Update the 64-bit frame counter.
    /// @details        The FPGA's frame counter is a 16-bit value that wraps every 11 minutes or so.  This function
    ///                 maintains the software' 64-bit frame counter.
    ///
    ///                 It first checks the overrun and underrun bits that are mapped into the 32-bit frame word.
    ///                 If either of these is set it indicates an FPGA problem and the software issues an error
    ///                 message and clears them both.
    ///
    ///                 If the 64-bit counter is being reset the function simply sets 64-bit counter to the FPGA
    ///                 value, stores the FPGA value for use the next time that the function is called, records the
    ///                 time and clears the reset flag.
    ///
    ///                 Otherwise it attempts to determine the amount by which the 64-bit counter should be
    ///                 incremented.  It uses the difference between the current and last FPGA frame count and the
    ///                 time difference between the two function calls.  The time difference is used to estimate the
    ///                 number of counter wraps that might have occurred and this is added to the difference in the
    ///                 FPGA values to ensure a positive increment.
    ///
    ///                 (A bug in the FPGA used to result in the frame counter decrementing and this is detected as
    ///                 a negative increment after the elapsed time has been taken into account - the problem was
    ///                 fixed in build 23 of the FPGA image but the detection is left the code - it issues an error
    ///                 message and attempts to recover.)
    ///
    /// @param[in]      frame               The 16-bit FPGA frame count.
    ///
private:
    void update_big_frame_count(uint32_t frame);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Read the data associated with the even-numbered result.
    /// @details        Read the FPGA's frame counter and peak detector results for the even buffer and store them
    ///                 in the software cache.  Use the frame number to update the internal 64.bit counter.  Mark
    ///                 the frame data as being valid.
    ///
private:
    void read_even_frame_data(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Read the data associated with the odd-numbered result.
    /// @details        Read the FPGA's frame counter and peak detector results for the odd buffer and store them
    ///                 in the software cache.  Use the frame number to update the internal 64.bit counter.  Mark
    ///                 the frame data as being valid.
    ///
private:
    void read_odd_frame_data(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Get the number of samples in the current result that exceeded the threshold.
    /// @details        If the cached value is valid it is returned, otherwise an exception is thrown.
    /// @return                             The number of samples over the threshold.
    ///
private:
    uint32_t get_num_samples_over_threshold(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Get the power of the peak in the current result.
    /// @details        If the cached value is valid it is returned, otherwise an exception is thrown.
    /// @return                             The power of the peak.
    ///
private:
    uint32_t get_peak_pwr(void);

private:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Enum to allow easy conversion between IEEE754 float and FPGA private format
    typedef union {
        float ieee754;
        uint32_t hex;
    }IEEE754Hex;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          When writing a internal float(16,8) mask to set the unused 8-bits to zero
    ///
    static const uint32_t BINARY_16_8_SET_ZEROES_MASK = 0xffffff00;

#ifdef SWTEST_4G_FPGA
public:
    //------------------------------------------------------------------------------------------------------------------
    // Bit fields for the 4G regression test interface
    //
    ///
    /// @brief          Set when regression block is enabled  (see
    ///                 Fpga::REG_REGRESSION_TEST_CONTROL_4G_OFFSET).
    ///
    static const uint32_t REGRESSION_4G_EN = 1U << 0;

    ///
    /// @brief          Set to trigger playback of regression block FIFO  (see
    ///                 Fpga::REG_REGRESSION_TEST_CONTROL_4G_OFFSET).
    ///
    static const uint32_t REGRESSION_4G_TRIG = 1U << 1;

    ///
    /// @brief          Set when regression test FIFO is empty (see
    ///                 Fpga::REG_REGRESSION_TEST_CONTROL_4G_OFFSET).
    ///
    static const uint32_t REGRESSION_4G_EMPTY = 1U << 2;

    ///
    /// @brief          Set when regression test FIFO is full (see
    ///                 Fpga::REG_REGRESSION_TEST_CONTROL_4G_OFFSET).
    ///
    static const uint32_t REGRESSION_4G_FULL = 1U << 3;

    ///
    /// @brief          Length of 4G Regression test interface FIFO (see
    ///                 Fpga::REG_REGRESSION_TEST_VALUE_4G_OFFSET).
    ///
    static const unsigned REGRESSION_4G_MAX_SAMPLES = 8192U;

    ///
    /// @brief          Maximum absolute value of ADC sample for 4G Regression test interface FIFO (see
    ///                 Fpga::REG_REGRESSION_TEST_VALUE_4G_OFFSET).
    ///
    static const unsigned REGRESSION_4G_MAX_ADC_SAMPLE = 2047U;

    ///
    /// @brief          Status of ongoing regression 4g testing
    ///
    typedef enum eregression_4g_state {
        REGRESSION_4G_ST_NOENABLED,
        REGRESSION_4G_ST_RUNNING,
        REGRESSION_4G_ST_EMPTY,
        REGRESSION_4G_ST_RESULT,
        REGRESSION_4G_ST_NORESULT,
        REGRESSION_4G_ST_FINISHED,
        REGRESSION_4G_ST_WAITING,
    } REGRESSION_4G_STATE;

private:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///  @brief          Print out internal floats as hex to confirm binary representation
    ///  @details        Includes printing out the result of converting to FPGA 16-8 format
    void checkInternalFloatRepresentation(void);

public:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///  @brief          Uses FPGA Floating point test input/output to confirm correct floating point representation
    ///  @details        Check the 6 test values
    ///  @returns       EXIT_SUCCESS if all 6 values are returned as expected.  EXIT_FAILURE otherwise.
    int checkFpgaFloatRepresentation(void);

public:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///  @brief         Enable/disable the regression test interface
    ///  @returns       Value of regression test register EN flag
    bool regression_4g_block_enable(bool enable);

public:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///  @brief         Program regression test FIFO with samples from the provided file
    ///  @details       Enables the test interface if required
    ///  @param adc_samples List of 16-bit ADC sample pairs packed as 32-bit ints for the FPGA
    ///  @returns       EXIT_FAILURE if FIFO not currently EMPTY or if the FULL flag becomes full during programming
    int regression_4g_program_fifo(std::list <uint32_t> adc_samples);

public:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///  @brief         Set the regression control TRIG flag
    ///  @details       If block is set do not return until EMPTY flag is set.  Otherwise return immediately
    ///  @returns       EXIT_FAILURE if regression block is not enabled or the FIFO is empty
    int regression_4g_trigger(bool block);

public:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///  @brief         Returns value of the regression block control register
    ///  @returns       Contains the 4 control flags
    uint32_t regression_4g_status(void);

public:
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///  @brief         Check ongoing status of the 4g regression test
    ///  @returns       integer containing an FPGA enum value showing progress
    int32_t regression_4g_iterate(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///  @brief         The a file containing the regression test ADC samples
    ///  @param[in] filename file containing the ADC sample pairs
    ///  @param[out] adc_samples list of ADC sample pairs packed into uint32_t ready for FPGA interface
    ///  @returns       EXIT_SUCCESS if read was successful
    int read_regression_4g_samples(std::string filename, 
                                    std::list<uint32_t> &adc_samples);

#endif // ifdef SWTEST_4G_FPGA
};

#endif

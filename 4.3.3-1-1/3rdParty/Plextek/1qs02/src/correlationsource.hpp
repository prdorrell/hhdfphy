/***********************************************************************************************************************
 *  Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 *  --------------------------------------------------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/correlationsource.hpp $
 * $Revision: 5367 $
 * $Author: pdm $
 * $Date: 2011-04-14 15:21:39 +0100 (Thu, 14 Apr 2011) $
 *******************************************************************************************************************//**
 *  @file
 *  @brief          The declaration of ::CorrelationSource class and its derived classes that are used to generate the
 *                  reference sequences for the matched filter.
 **********************************************************************************************************************/
#ifndef __CORRELATIONSOURCE_HPP__
#define __CORRELATIONSOURCE_HPP__

#include <vector>
#include <complex>
#include <stdint.h>
#include "debug.hpp"

//----------------------------------------------------------------------------------------------------------------------
///
/// @brief          The base class that defines the common interface to the various types of correlation source.
/// @details        The class provides the debug printing capability for its derived classes and implements the
///                 get_block() function that provides clients with the reference sequences.
///
class CorrelationSource
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The constants.
    //
private:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The types.
    //
public:
    ///
    /// @brief          A block of reference chips.
    /// @details        A block of chips is a variable (depending on correlation source type and its parameters) number
    ///                 of chips at the begining of a frame.
    ///
    typedef std::vector< std::complex<double> > block_t;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The values.
    //
private:

protected:
    ///
    /// @brief          A vector of blocks - one block for each slot in a frame.
    ///
    std::vector< block_t > blocks;

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
    CorrelationSource(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Constructs an instance of the class.
    /// @details        The only action is to store the requested debugging level.
    /// @param[in]      debug               Controls the debug messages that may be generated.
    ///
public:
    explicit CorrelationSource(unsigned debug);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The destructor has nothing to do.
    ///
public:
    virtual ~CorrelationSource();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Retrieves a block of reference chips for the specified slot.
    /// @param[in]      slot_num            The number of the slot for which the reference chips are needed.
    /// @return                             The block of reference chips.
    ///
public:
    const block_t &get_block(unsigned slot_num);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Set the debug verbosity level.
    /// @param[in]      level               The new verbosity level.
    ///
public:
    void debug_printing(unsigned level)
    {
        debugStr.show_level(level);
    }
};

//----------------------------------------------------------------------------------------------------------------------
///
/// @brief          Generates the reference sequence for an uplink DPCCH.
/// @details        The reference sequence is generated from the pilot bits in each slot of a frame by the constructor,
///                 which stores the results in the base class' CorrelationSource::blocks member variable.
///
class UplinkDPCCHSource : public CorrelationSource
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The constants.
    //
private:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The types.
    //
private:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The values.
    //
private:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The default constructor is declared but not defined in order to prevent a compiler generated
    ///                 version appearing.
    ///
private:
    UplinkDPCCHSource(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Constructs an instance of the class.
    /// @details        Generates the uplink scrambling code for a frame, reads the pilot bits from a lookup table,
    ///                 spreads them (the channelisation code for the uplink DPCCH is a trivial sequence of 256 '+1's),
    ///                 maps them onto the imaginary part of a complex chip with zero real component and multiplies the
    ///                 resulting chips by the scrambling code chips.
    /// @param[in]      code_num            The scrambling code number.
    /// @param[in]      num_pilots          The number of pilot pits to include in the reference sequence for each slot.
    /// @param[in]      debug               Controls the debug messages that may be generated.
    ///
public:
    UplinkDPCCHSource(uint32_t code_num, uint32_t num_pilots, unsigned debug);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The destructor has nothing to do.
    ///
public:
    virtual ~UplinkDPCCHSource();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Generates the long uplink scrambling code for a frame.
    /// @details        The algorithm to generate the scrambling code is a shift-register implementation like that
    ///                 described in 25,213 ("3GPP TS 25.213 V9.1.0 (2009-12), Technical Specification, 3rd Generation
    ///                 Partnership Project; Technical Specification Group Radio Access Network; Spreading and
    ///                 modulation (FDD) (Release 9)".
    /// @param[in]      code_num            The scrambling code number.
    /// @param[out]     frame               The scrambling code.
    ///
private:
    void gen_scramble(uint32_t code_num, block_t &frame);
};

//----------------------------------------------------------------------------------------------------------------------
///
/// @brief          Generates the reference sequence for a downlink PCPICH.
/// @details        The reference sequence is generated from a number of symbols at the start of each slot in a frame by
///                 the constructor, which stores the results in the base class' CorrelationSource::blocks member
///                 variable.
///
class DownlinkPCPICHSource : public CorrelationSource
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The constants.
    //
private:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The types.
    //
private:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The values.
    //
private:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The default constructor is declared but not defined in order to prevent a compiler generated
    ///                 version appearing.
    ///
private:
    DownlinkPCPICHSource(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Constructs an instance of the class.
    /// @details        Generates the downlink scrambling code for a frame and generates the pilot symbols from the
    ///                 known patterns defined in 25.211 ("3GPP TS 25.211 V9.1.0 (2009-12), Technical Specification, 3rd
    ///                 Generation Partnership Project; Technical Specification Group Radio Access Network; Physical
    ///                 channels and mapping of transport channels onto physical channels (FDD) (Release 9)").  Spreads
    ///                 the pilot symbols (the channelisation code for the downlink PCPICH is a trivial sequence of 256
    ///                 '+1's) and multiplies the resulting chips by the scrambling code chips.  The resulting chips
    ///                 have values of +2, -2, +2j or -2j, so they are multiplied by (1+j)/2 so that they have values
    ///                 from the set { +/-1+/-j }, just like the other sources.
    /// @param[in]      code_num            The scrambling code number.
    /// @param[in]      antenna             The Node B antenna for which the reference should be generated.
    /// @param[in]      syms                The number of symbols to be used.
    /// @param[in]      debug               Controls the debug messages that may be generated.
    /// @param[in]      debug_files         true if the scrambling code should be written to a file.
    ///
public:
    DownlinkPCPICHSource(uint32_t code_num, uint32_t antenna, uint32_t syms, unsigned debug, bool debug_files);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The destructor has nothing to do.
    ///
public:
    virtual ~DownlinkPCPICHSource();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Generates the primary downlink scrambling code for a frame.
    /// @details        The algorithm to generate the scrambling code is a shift-register implementation like that
    ///                 described in 25,213 ("3GPP TS 25.213 V9.1.0 (2009-12), Technical Specification, 3rd Generation
    ///                 Partnership Project; Technical Specification Group Radio Access Network; Spreading and
    ///                 modulation (FDD) (Release 9)".
    /// @param[in]      code_num            The scrambling code number.
    /// @param[out]     frame               The scrambling code.
    ///
private:
    void gen_pri_scramble(uint32_t code_num, block_t &frame);
};

//----------------------------------------------------------------------------------------------------------------------
///
/// @brief          Generates the reference sequence for a downlink PSCH.
/// @details        The reference sequence is generated by the constructor, which stores the results the fixed PSCH
///                 chips in the base class' CorrelationSource::blocks member variable.
///
class DownlinkPSCHSource : public CorrelationSource
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The constants.
    //
private:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The types.
    //
private:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The values.
    //
private:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The default constructor is declared but not defined in order to prevent a compiler generated
    ///                 version appearing.
    ///
private:
    DownlinkPSCHSource(void);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Constructs an instance of the class.
    /// @details        The PSCH is a fixed sequence of 256 chips that appears at the start of each downlink slot.  The
    ///                 generation of the reference sequence simply involves copying the chips into the base class'
    ///                 buffer for each slot in a frame.
    /// @param[in]      debug               Controls the debug messages that may be generated.
    ///
public:
    DownlinkPSCHSource(unsigned debug);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The destructor has nothing to do.
    ///
public:
    virtual ~DownlinkPSCHSource();
};

#endif

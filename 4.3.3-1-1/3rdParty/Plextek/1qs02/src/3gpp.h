/******************************************************************************
 * Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 * All Rights Reserved
 ******************************************************************************
 * THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 * ----------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/3gpp.h $
 * $Revision: 5367 $
 * $Author: pdm $
 * $Date: 2011-04-14 15:21:39 +0100 (Thu, 14 Apr 2011) $
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file 3gpp.h
 * \brief Constants from the 3gpp specifications
 *
 *****************************************************************************/

#ifndef __3GPP_H__
#define __3GPP_H__

/// A collection of constants from the 3gpp specifications.
namespace ThreeGPP{

/// \name self explanatory constants
//@{
const unsigned CHIPS_PER_SEC                 = 3840000;

const unsigned FRAMES_PER_SEC                = 100;
const unsigned CHIPS_PER_FRAME               = CHIPS_PER_SEC/FRAMES_PER_SEC;

const unsigned SLOTS_PER_FRAME               = 15;
const unsigned CHIPS_PER_SLOT                = CHIPS_PER_FRAME/SLOTS_PER_FRAME;

const unsigned SCRAMBLING_CODE_LEN           = CHIPS_PER_FRAME;
const unsigned NUM_PRIMARY_SCRAMBLING_CODES  = 512;
const unsigned PRIMARY_SCRAMBLING_CODE_SEP   = 16;
const unsigned NUM_SCRAMBLING_CODE_GROUPS    = 64;
const unsigned NUM_PSC_PER_SCG               = NUM_PRIMARY_SCRAMBLING_CODES/NUM_SCRAMBLING_CODE_GROUPS;

const unsigned NUM_PSCH_CHIPS                = 256;

const unsigned NUM_SSCH_CHIPS                = 256;
const unsigned NUM_SSCH_CODES                = 16;
const unsigned NUM_SSCH_SEQ                  = NUM_SCRAMBLING_CODE_GROUPS;

const unsigned PCPICH_SF                     = 256;
const unsigned PCPICH_SYMS_PER_SLOT          = CHIPS_PER_SLOT/PCPICH_SF;
const unsigned PCPICH_SYMS_PER_FRAME         = PCPICH_SYMS_PER_SLOT*SLOTS_PER_FRAME;

const unsigned PCCPCH_SF                     = 256;
const unsigned PCCPCH_SYMS_PER_SLOT          = CHIPS_PER_SLOT/PCCPCH_SF;
const unsigned PCCPCH_SYMS_PER_FRAME         = PCCPCH_SYMS_PER_SLOT*SLOTS_PER_FRAME;
const unsigned PCCPCH_BITS_PER_SLOT          = 2*PCCPCH_SYMS_PER_SLOT-1;
const unsigned PCCPCH_BITS_PER_FRAME         = PCCPCH_BITS_PER_SLOT*SLOTS_PER_FRAME;

const unsigned DPCCH_SF                      = 256;
const unsigned DPCCH_SYMS_PER_SLOT           = CHIPS_PER_SLOT/DPCCH_SF;
const unsigned DPCCH_SYMS_PER_FRAME          = DPCCH_SYMS_PER_SLOT*SLOTS_PER_FRAME;

const unsigned BCH_BLOCKS_PER_SEC            = 50;
const unsigned BCH_BITS_PER_BLOCK            = 246;
//@}


/// Pilot bit sequences look up table, packed LSbit first.
/// Ref 25.211 tables 3 and 4.
/// First index is number of bits (3-8 are valid).
/// Second index is slot number.
/// Used by DownlinkPSCHSource class.
const unsigned pilotBits[9][SLOTS_PER_FRAME] =
{
    // # pilot bits = 0 row not used;
    { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },

    // # pilot bits = 1 row not used;
    { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },

    // # pilot bits = 2 row not used;
    { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 },

    // # pilot bits = 3
    { 0x7, 0x4, 0x6, 0x4, 0x5, 0x7, 0x7, 0x5, 0x6, 0x7, 0x6, 0x5, 0x5, 0x4, 0x4 },

    // # pilot bits = 4
    { 0xF, 0x9, 0xD, 0x9, 0xB, 0xF, 0xF, 0xB, 0xD, 0xF, 0xD, 0xB, 0xB, 0x9, 0x9 },

    // # pilot bits = 5
    { 0xF, 0xC, 0x16, 0x4, 0x15, 0xF, 0x7, 0x5, 0xE, 0x1F, 0x16, 0x1D, 0x5, 0x1C, 0x1C },

    // # pilot bits = 6
    { 0x1F, 0x19, 0x2D, 0x9, 0x2B, 0x1F, 0xF, 0xB, 0x1D, 0x3F, 0x2D, 0x3B, 0xB, 0x39, 0x39 },

    // # pilot bits = 7
    { 0x5F, 0x59, 0x6D, 0x49, 0x6B, 0x5F, 0x4F, 0x4B, 0x5D, 0x7F, 0x6D, 0x7B, 0x4B, 0x79, 0x79 },

    // # pilot bits = 8
    { 0x7F, 0x75, 0xDD, 0x55, 0xD7, 0x7F, 0x5F, 0x57, 0x7D, 0xFF, 0xDD, 0xF7, 0x57, 0xF5, 0xF5 }
};


}

#endif // __3GPP_H__

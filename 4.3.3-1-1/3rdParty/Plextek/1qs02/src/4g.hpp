/***********************************************************************************************************************
 *  Copyright (c) 2021 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  Filename:   4G.hpp
 *  Author(s):  pdm
 *******************************************************************************************************************//**
 * File Description
 * ================
 *
 *  @file
 *  @brief          Constants from the 4G specifications
 *
 **********************************************************************************************************************/

#ifndef __4G_HPP__
#define __4G_HPP__

/// A collection of constants from the 4G specifications.
namespace FourG{

const unsigned SUBCARRIER_SPACING_HZ = 15000;                   /// Section 5.6 in 36.211

const unsigned BASIC_SYMBOLS_PER_SEC = SUBCARRIER_SPACING_HZ;   /// No cyclic prefix

const unsigned BASIC_TIME_UNITS_PER_BASIC_SYMBOL = 2048;        /// Section 5.6 in 36.211

const unsigned BASIC_TIME_UNITS_PER_NORMAL_SLOT6_CYCLIC_PREFIX = 144;
                                                                /// Section 5.6 in 36.211

const unsigned BASIC_TIME_UNITS_PER_EXTENDED_CYCLIC_PREFIX = 512;
                                                                /// Section 5.6 in 36.211

const unsigned BASIC_TIME_UNITS_PER_SEC = BASIC_TIME_UNITS_PER_BASIC_SYMBOL * BASIC_SYMBOLS_PER_SEC;

const unsigned BASIC_TIME_UNITS_PER_SLOT = 15360;               /// Section 4 in 36.211

const unsigned SLOTS_PER_SUBFRAME = 2;                          /// Section 4 in 36.211

const unsigned SUBFRAMES_PER_FRAME = 10;                        /// Section 4 in 36.211

const unsigned BASIC_TIME_UNITS_PER_SUBFRAME = BASIC_TIME_UNITS_PER_SLOT * SLOTS_PER_SUBFRAME;

const unsigned BASIC_TIME_UNITS_PER_FRAME = BASIC_TIME_UNITS_PER_SUBFRAME * SUBFRAMES_PER_FRAME;

const unsigned SUBFRAMES_PER_SEC = BASIC_TIME_UNITS_PER_SEC / BASIC_TIME_UNITS_PER_SUBFRAME;

const unsigned FRAMES_PER_SEC = BASIC_TIME_UNITS_PER_SEC / BASIC_TIME_UNITS_PER_FRAME;

}

#endif // __4G_HPP__

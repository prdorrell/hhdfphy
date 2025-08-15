/******************************************************************************
 * Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
 * All Rights Reserved
 ******************************************************************************
 * THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 * ----------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/gsm.hpp $
 * $Revision: 6380 $
 * $Author: pdm $
 * $Date: 2011-07-20 09:32:59 +0100 (Wed, 20 Jul 2011) $
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file gsm.h
 * \brief Constants from the GSM specifications
 *
 *****************************************************************************/

#ifndef __GSM_HPP__
#define __GSM_HPP__

/// A collection of constants from the GSM specifications.
namespace Gsm
{

/// \name self explanatory constants
//@{
const unsigned BITS_PER_FRAME  = 1250;
const unsigned SLOTS_PER_FRAME = 8;

}

#endif // __GSM_HPP__

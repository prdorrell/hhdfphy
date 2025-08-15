/******************************************************************************
 * Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 * All Rights Reserved
 ******************************************************************************
 * THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 * ----------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/dac.hpp $
 * $Revision: 5327 $
 * $Author: pdm $
 * $Date: 2011-04-12 11:35:01 +0100 (Tue, 12 Apr 2011) $
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file dac.hpp
 * \brief Serial bit bash for VCTCXO DAC ( AD5320 )
 *
 *****************************************************************************/
#ifndef __DAC_HPP__
#define __DAC_HPP__

namespace DAC
{
    void set( unsigned int );
    unsigned int get( void );
    void debug_printing( unsigned );
}

#endif


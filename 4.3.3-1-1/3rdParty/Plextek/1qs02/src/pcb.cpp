/******************************************************************************
 * Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
 * All Rights Reserved
 ******************************************************************************
 * THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 * ----------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/pcb.cpp $
 * $Revision: 4982 $
 * $Author: pdm $
 * $Date: 2011-03-08 14:33:39 +0000 (Tue, 08 Mar 2011) $
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file pcb.cpp
 * \brief Definition of general functions associated with the PCB
 *
 *****************************************************************************/
#include "debug.hpp"
#include "gpio.hpp"
#include "pcb.hpp"

namespace Pcb
{
/// debug tracing output
DebugStream debugStr("Pcb......... ");

// /////////////////////////////////////////////////////////////////////////////
/// Get the PCB revision number
//
/// \return The PCB revision number
unsigned getPcbRev(void)
{
    //
    //  Only read the revision number the first time it is needed - cache it for
    //  later use.
    //
    static int rev = -1;

    if (rev == -1)
    {
        gpio pcb_rev0(gpio::PCB_REV0, gpio::in, debugStr.get_state());
        gpio pcb_rev1(gpio::PCB_REV1, gpio::in, debugStr.get_state());
        gpio pcb_rev2(gpio::PCB_REV2, gpio::in, debugStr.get_state());

        rev =   (pcb_rev2.get() ? 4 : 0)
              + (pcb_rev1.get() ? 2 : 0)
              + (pcb_rev0.get() ? 1 : 0);
    }

    return ((unsigned)rev);
}

}//end namespace radio

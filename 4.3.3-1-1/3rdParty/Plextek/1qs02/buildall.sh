#!/bin/bash
# ---------------------------------------------------------------------------------------------------------------------
# Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
# All Rights Reserved
# ---------------------------------------------------------------------------------------------------------------------
# THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
# ---------------------------------------------------------------------------------------------------------------------
# $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/buildall.sh $
# $Revision: 5327 $
# $Author: pdm $
# $Date: 2011-04-12 11:35:01 +0100 (Tue, 12 Apr 2011) $
# ---------------------------------------------------------------------------------------------------------------------
# File Description (User Field)
# ----------------------------
#
# Build all the targets and install them
#
# ---------------------------------------------------------------------------------------------------------------------
#
# Exit on cal-file, lockup or frame errors
#
make -f makefile.mak clean
EXITCOND=exit_all make -f makefile.mak
EXITCOND=exit_all make -f makefile.mak installbrg
EXITCOND=exit_all make -f makefile.mak install100

#
# Exit on lockup or frame errors
#
make -f makefile.mak clean
EXITCOND=exit_frame_lockup make -f makefile.mak
EXITCOND=exit_frame_lockup make -f makefile.mak installbrg
EXITCOND=exit_frame_lockup make -f makefile.mak install100

#
# Exit on lockup errors
#
make -f makefile.mak clean
EXITCOND=exit_lockup make -f makefile.mak
EXITCOND=exit_lockup make -f makefile.mak installbrg
EXITCOND=exit_lockup make -f makefile.mak install100

#
# Exit on frame errors
#
make -f makefile.mak clean
EXITCOND=exit_frame make -f makefile.mak
EXITCOND=exit_frame make -f makefile.mak installbrg
EXITCOND=exit_frame make -f makefile.mak install100

#
# Do not exit on errors (the release build)
#
make -f makefile.mak clean
EXITCOND=none make -f makefile.mak
EXITCOND=none make -f makefile.mak installbrg
EXITCOND=none make -f makefile.mak install100

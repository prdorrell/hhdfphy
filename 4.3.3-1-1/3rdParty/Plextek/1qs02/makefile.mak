# -----------------------------------------------------------------------------
#
#
# -----------------------------------------------------------------------------
# Filename:   makefile.mak
# Author(s):  pdm
# -----------------------------------------------------------------------------
# File Description (User Field)
# ----------------------------
#
# cross compile the main program
#
# -----------------------------------------------------------------------------

#
# Paths.
#
srcdir      = ./src
objdir      = ./xobj
gendir      = ./test/src
hdrdirs     = ../../kiss_fft130:../../boost_1_48_0:$(objdir)

#
# The executable, named to indicate exit conditions
#
# exit conditions are: exit_all, exit_frame_lockup, exit_frame, exit_lockup or none (the release build)
#
ifeq ($(EXITCOND),)
EXITCOND = none
endif

ifeq ($(EXITCOND),exit_all)
output       = ./xobj/hhdfphy_exit_all
EXITDEFFLAGS = -DEXIT_ON_CAL_FILE_ERROR -DEXIT_ON_FRAME_ERROR -DEXIT_ON_LOCKUP
else
ifeq ($(EXITCOND),exit_frame_lockup)
output       = ./xobj/hhdfphy_exit_frame_lockup
EXITDEFFLAGS = -DEXIT_ON_FRAME_ERROR -DEXIT_ON_LOCKUP
else
ifeq ($(EXITCOND),exit_frame)
output       = ./xobj/hhdfphy_exit_frame
EXITDEFFLAGS = -DEXIT_ON_FRAME_ERROR
else
ifeq ($(EXITCOND),exit_lockup)
output       = ./xobj/hhdfphy_exit_lockup
EXITDEFFLAGS = -DEXIT_ON_LOCKUP
else
output       = ./xobj/hhdfphy
EXITDEFFLAGS = 
endif
endif
endif
endif

#
# The basic files required by the linker (excluding main, which is handled
# separately).
#
FILES = 
FILES += adc
FILES += cmdint
FILES += correlationsource
FILES += correlator
FILES += correlator_4g
FILES += correlator_4g_fileio
FILES += correlator_4g_test
FILES += dac
FILES += fpga
FILES += fpga_4g_test
FILES += gpio
FILES += phy
FILES += radio
FILES += ranging
FILES += refcal
FILES += searchandtrack
FILES += searchandtrack_4g
FILES += searchandtrack_4g_test
FILES += pcb
FILES += configfile
FILES += pwrcal
FILES += gsmpwrmeter
FILES += timeconttrack

#
# Prepend intermediate directory 
#
OBJS_EX_MAIN = $(patsubst %, $(objdir)/%.o,$(FILES) )
OBJS         = $(OBJS_EX_MAIN) $(objdir)/main.o

#
# Create list of dependacy files
#
DEPS         = $(patsubst %.o,%.dep,$(OBJS) )

#
# Libraries
#
LIBS         = 
# stdc++

#
# Name of the composite dependency file
#
DEPEND = $(objdir)/depend.inc

#
# Compiler
#
ifeq ($(CROSS_COMPILE),)
# try this on path if env variable not set
GCC         = arm-none-linux-gnueabi-g++
else
# we expect the cross compiler prefix to be set
GCC         = $(CROSS_COMPILE)g++
endif
CFLAGS      = -O2
DEFFLAGS    =

#
# Flags
#
incflags = $(patsubst %,-I%,$(subst :, ,$(hdrdirs)) )

ALL_CPPFLAGS = $(CFLAGS) $(incflags) $(DEFFLAGS) \
               -DGIT_HASH='"$(shell svnversion)"' \
               -DCOMPILE_INFO='"$(shell $(GCC) -dumpmachine) $(shell $(GCC) -dumpversion)"' \
               -DHOST_MACHINE='"$(USER)@$(shell hostname)"' \
               -DUSE_FPGA_LOW_POWER_MODE \
               -DUSE_CDMA2000_UPLINK_BAND_EXTENSIONS \
               $(EXITDEFFLAGS) \
               -Wall

#
# Compiler #defines: - those wanted must be placed in DEFFLAGS variable above and not just in DEPFLAGS
#
#  USE_FPGA_LOW_POWER_MODE     - use the FPGA's low power mode.
#                            
#  USE_CHAN_RSSI_RANGING	   - the software uses the channel filter RSSI for ranging.
#  WAIT_FOR_MCB_AND_CLOCKS     - during initialisation wait for the MCB and PLLs in the FPGA to signal that they
#                                are operational
#  USE_SW_PEAK_DETECTION       - use software peak detection, simulating the FPGA's function
#  CHECK_FPGA_PEAK_DETECTION   - compare the FPGA peak offset with the software result and warn of diferences
#  LOCKUP_RECOVERY             - recover from an FPGA lockup
#  EXIT_ON_CAL_FILE_ERROR      - exits the program if an error in the calibration file format is detected
#  EXIT_ON_FRAME_ERROR         - exits the program if an FPGA frame number error is detected
#  EXIT_ON_LOCKUP              - exits the program if an FPGA lockup error is detected
#  SWTEST_4G_CORR              - test mode for the 4G correltor class
#  SWTEST_4G_FPGA              - test mode the the 4G Fpga class
#  SWTEST_4G_FPGA_TIMING       - test mode the the 4G FPGA timing
#  SWTEST_4G_SEARCH_TRACK      - test mode for the 4G search and track class
#                            
#  WIDE_TRACK_WINDOW_4G        - use the wide tracking window (had no noticeable effect and raising tracking threshold
#                                by 1 dB is recommended)
#  DISCARD_LOW_PWR_PEAKS_4G    - discard raw correlation peaks whose burst power is less than 1/10 of the maximum burst
#                                power (recommended for improved resistance to false positives)
#  DISCARD_HIGH_CORR_PEAKS_4G  - discard raw correlation peaks whose peak value is greater than 1.1 - the theoretical
#                                maximum is 1.0 and greater values only appear as a result of quantisation effects in
#                                the FPGA processing that seem to be more likely in noise (recommended for improved
#                                resistance to false positives)
#  RANGING_FORCES_SEARCH_4G    - forces a range change to result in a switch into search mode if currently tracking
#  ALWAYS_USE_LL_RESULT_CODE   - forces the results to be returned using the LL message code even when ranging is
#                                underway.
#

DEPFLAGS     = -MM -MG $(incflags) $(DEFFLAGS)

#
# Declare the phony targets, i.e. targets should never exist and therefore
# should always be out of date.  If, by accident, a file with one of the phony
# targets as its name should be created, the following line ensure that its
# existence is ignored.  e.g. "make clean" will always work, even if there is
# a file with name "clean".
#
.PHONY: all depend clean

#
# The default target
#
all: $(output)

depend: $(DEPEND)

clean:
	rm -f $(objdir)/*.*
	rm -f $(objdir)/*
	rm -f $(srcdir)/*.pyc
	rm -f $(output)
	rm -f ./dump/*.*
	rm -f ./test/*.pyc
	$(if $(wildcard $(objdir)), rmdir $(objdir) )
	$(if $(wildcard ./dump), rmdir ./dump )

#
# Explicit dependencies
#
# create the object directory
$(objdir) :
	mkdir $(objdir)

./dump :
	mkdir ./dump

# link the program
$(output) : $(OBJS) | ./dump
	@echo linking...
	$(GCC) $(DEBUG) $(OBJS) -o $(output) $(patsubst %, -l%,$(subst ;, ,$(LIBS)) )
	@echo
	@echo successfully built: $(output)

# compile the source
$(objdir)/%.o : $(srcdir)/%.cpp | $(objdir)
	$(GCC) $(ALL_CPPFLAGS) -c $< -o $@

# construct the dependencies
$(DEPEND) : $(DEPS) | $(objdir)
	cat $(DEPS) > $(DEPEND)

# special target to build table header
# (pipe means build objdir if not exist but ignore timestamp)
$(objdir)/tables.hpp : $(gendir)/gencodes.py | $(objdir)
	@echo
	python $< $@

# create the auto deps
# this depends on tables being built or it can't figure out where it is
$(objdir)/%.dep : $(srcdir)/%.cpp $(objdir)/tables.hpp | $(objdir)
	@echo generating dependency file $@
	@$(GCC) $< $(DEPFLAGS) -MT'$(objdir)/$*.o $(objdir)/$*.dep' -o $(objdir)/$*.dep

# make main depend on everything so date info etc gets updated
$(objdir)/main.o : $(OBJS_EX_MAIN)

# install rules
.PHONY: install2
install2:
	sudo cp gpio-export.sh /opt/freescale/ltib/rootfs/home
	sudo cp $(output)      /opt/freescale/ltib/rootfs/home

.PHONY: install
install:
	sudo cp gpio-export.sh /opt/testing/rootfs/home
	sudo cp $(output)      /opt/testing/rootfs/home

.PHONY: install93
install93:
	sudo cp gpio-export.sh /opt/testing93/rootfs/home
	sudo cp $(output)      /opt/testing93/rootfs/home

.PHONY: install95
install95:
	sudo cp gpio-export.sh /opt/testing95/rootfs/home
	sudo cp $(output)      /opt/testing95/rootfs/home

.PHONY: install97
install97:
	sudo cp gpio-export.sh /opt/testing/rootfs/home
	sudo cp $(output)      /opt/testing/rootfs/home

.PHONY: install100
install100:
	sudo cp gpio-export.sh /opt/testing100/rootfs/home
	sudo cp $(output)      /opt/testing100/rootfs/home	

.PHONY: install102
install102:
	sudo cp gpio-export.sh /opt/testing102/rootfs/home
	sudo cp $(output)      /opt/testing102/rootfs/home	

.PHONY: installbrg
installbrg:
	sudo cp gpio-export.sh /opt/brg_bluetooth/rootfs/home
	sudo cp $(output)      /opt/brg_bluetooth/rootfs/home	

#
# Include the auto dependencies
#
ifneq ($(MAKECMDGOALS),clean)
-include $(DEPEND)
endif

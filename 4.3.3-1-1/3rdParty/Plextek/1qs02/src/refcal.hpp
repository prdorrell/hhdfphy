/***********************************************************************************************************************
 *  Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 *  All Rights Reserved
 ***********************************************************************************************************************
 *  THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 *  --------------------------------------------------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/refcal.hpp $
 * $Revision: 6378 $
 * $Author: pdm $
 * $Date: 2011-07-20 09:24:28 +0100 (Wed, 20 Jul 2011) $
 *******************************************************************************************************************//**
 *  @file
 *  @brief          The declaration of classes that control the calibration of the reference VCTCXO.
 **********************************************************************************************************************/
#ifndef __REFCAL_HPP__
#define __REFCAL_HPP__

#include "correlator.hpp"
#include "debug.hpp"
#include "fpga.hpp"
#include "design.hpp"
#include "ranging.hpp"
#include <memory>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief          The pure virtual base class that defines the interface to the different reference calibration
///                 implemenations.
///
class RefCal
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The types.
    //
public:
    ///
    /// @brief          The result of the iterate() function call.
    ///
    typedef enum
    {
        ///
        /// @brief          The algorithm is running normally.
        ///
        REF_CAL_ACTIVE,

        ///
        /// @brief          The algorithm has been successfully completed.
        ///
        REF_CAL_DONE,

        ///
        /// @brief          The algorithm has timed out.
        ///
        REF_CAL_ABORT_TIMEOUT
    } iter_res_t;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The constructor has nothing to do.
    ///
public:
    RefCal()
    {
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The destructor has nothing to do.
    ///
public:
    virtual ~RefCal()
    {
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Processes an iteration of the control loop.
    /// @return         The state of the algorithm.
    ///
public:
    virtual iter_res_t iterate(void) = 0;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Set the debug verbosity level.
    /// @param[in]      level               The new verbosity level.
    ///
public:
    virtual void debug_printing(unsigned level) = 0;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief          The pure virtual base class that defines the interface to the different VCTCXO DAC controller
///                 implemenations.
///
class Controller
{

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The values.
    //
protected:
    ///
    /// @brief          The debug stream.
    /// @details        Debug messages are sent to this stream.  If the debugging level has been appropriately
    ///                 configured they appear on the standard output console.  See ::DebugStream.
    ///
    DebugStream                 debugStr;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The constructor just configures the debugging.
    /// @param[in]      debug               The debug verbosity setting.
    ///
public:
    Controller(unsigned debug) :
        debugStr( "Controller.. ", debug )
    {
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The destructor has nothing to do.
    ///
public:
    virtual ~Controller()
    {
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Performs one iteration of the control loop using the supplied offset frequency.
    /// @param[in]      off                 The current estimate of the frequency offset.
    /// @return                             true if the function needs to be called again (the DAC has been updated),
    ///                                     false if the loop has stabilised and the calibration is complete.
    ///
    virtual bool update(double off) = 0;

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief          The PID implementation of the VCTCXO DAC controller.
/// @details        The class implements a basic PID controller, with the refinement that the frequency offset estimates
///                 are filtered by a leaky integrator.
///
class RefCalPID : public Controller
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The constants.
    //
private:
    ///
    /// @brief          The number of past error terms that are used to calculate the mean squared error.
    ///
    static const int MSE_SIZE = 50;

    ///
    /// @brief          The maximum filtered relative frequency offset that is used by the PID controller.
    /// @details        Clipping the frequency offset estimate reduces the effect of extreme results.  The value of
    ///                 7 ppm is chosen because this is the limit of the VCTCXO control range.  Any value much greater
    ///                 that this is also a problem because the coherent integration is seriously affected by the
    ///                 frequency offset (the first null of the sinc function is at about 7.75 ppm).
    ///
    static const double MAX_REL_FREQ_OFFSET_PPM = 7;

    ///
    /// @brief          The target relative frequency offset.
    ///
    static const double TARGET_REL_FREQ_OFFSET_PPM = 0.02;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The values.
    //
private:
    ///
    /// @brief          The gain of the proportional term.
    ///
    double      KP;                     ///< Gain of the proportional term

    ///
    /// @brief          The gain of the integral term.
    ///
    double      KI;                     ///< Gain of the integral term

    ///
    /// @brief          The gain of the differential term.
    ///
    double      KD;

    ///
    /// @brief          The time constant of the leaky integrator filter.
    ///
    double      FILTER_TIME;

    ///
    /// @brief          Indicates that the leaky integrator has been initialised by the first result.
    ///
    bool        filter_initialised;

    ///
    /// @brief          The filtered frequency offset estimate.
    ///
    double      filtered_offset;

    ///
    /// @brief          The current integrator component.
    ///
    double      integrator;

    ///
    /// @brief          The current differentiator component.
    ///
    double      differentiator;

    ///
    /// @brief          The array of past error-squared values used for the MSE calculation.
    ///
    double      error_squared[MSE_SIZE];

    ///
    /// @brief          The number past error-squared values that have been stored in the array.
    ///
    int         n_err_vals;

    ///
    /// @brief          The index to the current error-squared value in the array.
    ///
    int         curr_err_val;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Constructs an instance of the class.
    /// @details        The correlator is constructed and the member variables are initialised.
    /// @param[in]      kp                  The gain of the proportional term.
    /// @param[in]      ki                  The gain of the integral term.
    /// @param[in]      kd                  The gain of the differential term.
    /// @param[in]      ft                  The time constant of the leaky integrator filter.
    /// @param[in]      debug               The debug verbosity setting.
    ///
public:
    RefCalPID(double kp, double ki, double kd, double ft, unsigned debug);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The destructor has nothing to do.
    ///
public:
    ~RefCalPID();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Performs one iteration of the control loop using the supplied offset frequency.
    /// @details        This function implements a PID controller to try and minimise the time taken to reach the
    ///                 correct setting. Typically this function will need to be called a number of times to reach a
    ///                 stable state.  The values pass through a simple filter before passing to the pid.
    /// @param[in]      off                 The current estimate of the frequency offset.
    /// @return                             true if the function needs to be called again (the DAC has been updated),
    ///                                     false if the loop has stabilised and the calibration is complete.
    ///
public:
    bool update(double off);
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief          The alternative implementation of the VCTCXO DAC controller.
/// @details        The class implements an alternative controller intended for use with the gps 1pps approach that
///                 should robustly get to an answer faster than the PID controller.
///
class RefCalAltB : public Controller
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The types.
    //
public:
    ///
    /// @brief          The states of the controller.
    ///
    typedef enum
    {
        ///
        /// @brief          The controller is measuring the first point in the slope estimation.
        ///
        INITIAL_MEASURE1,

        ///
        /// @brief          The controller is measuring the second point in the slope estimation.
        ///
        INITIAL_MEASURE2,

        ///
        /// @brief          The controller is looping to minimise the error.
        ///
        RUNNING
    } state_t;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The constants.
    //
private:
    ///
    /// @brief          The RMS error threshold used for declaring a success.
    ///
    static const double RMS_THRESH_PPM = 0.2;

    ///
    /// @brief          The absolute mean error threshold used for declaring a success.
    ///
    static const double MEAN_THRESH_PPM = 0.02;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The values.
    //
private:
    ///
    /// @brief          The current state of the controller.
    ///
    state_t controller_state;

    ///
    /// @brief          The averaging window size.
    ///
    double  n;

    ///
    /// @brief          The ratio to increase N by each iteration.
    ///
    double  N_grow;

    ///
    /// @brief          The simple integration of the error.
    ///
    double  integrator;

    ///
    /// @brief          The dac/freq transfer gradient estimate.
    ///
    double  m;

    ///
    /// @brief          The initial frequency error measurement.
    ///
    double  f0;

    ///
    /// @brief          The count of measurements in a block.
    ///
    int     measurement_count;

    ///
    /// @brief          The running total of results in block.
    ///
    double  measurement_sum;

    ///
    /// @brief          The running total of squared results.
    ///
    double  measurement_sum_sq;

    ///
    /// @brief          The first value for gradient estimation.
    ///
    int     dacval_init0;

    ///
    /// @brief          The second value for gradient estimation.
    ///
    int     dacval_init1;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Constructs an instance of the class.
    /// @details        The correlator is constructed and the member variables are initialised.
    /// @param[in]      n_init              The initial averaging window size.
    /// @param[in]      N_grow_init         The factor to grow window by each iteration.
    /// @param[in]      debug               The debug verbosity setting.
    ///
public:
    RefCalAltB(double n_init, double N_grow_init, unsigned debug);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The destructor has nothing to do.
    ///
public:
    ~RefCalAltB();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Performs one iteration of the control loop using the supplied offset frequency.
    /// @details        First take two measurements to estimate DAC value / frequency gradient.  If this was ideal
    ///                 (noise free/linear) we could imediately calculate the required DAC value to make the error 0.
    ///                 Iteratively make improvements, averaging over bigger windows each time.  Successful when the
    ///                 average error and RMS error within some defined values.
    /// @param[in]      off                 The current estimate of the frequency offset.
    /// @return                             true if the function needs to be called again (the DAC has been updated),
    ///                                     false if the loop has stabilised and the calibration is complete.
    ///
    bool update(double off);
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief          The main implementation for the GPS-based reference calibration.
/// @details        The .
///
class RefCalGPS : public RefCal
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The types.
    //
public:
    ///
    /// @brief          The mode/state of the algorithm.
    ///
    typedef enum
    {
        ///
        /// @brief          The algorithm has been started but the first pps pulse has not yet been detected.
        ///
        REF_CAL_GPS_IDLE,

        ///
        /// @brief          The algorithm has been started but the second pps pulse has not yet been detected.
        ///
        REF_CAL_GPS_INIT,

        ///
        /// @brief          The algorithm is active.
        ///
        REF_CAL_GPS_ACTIVE,

        ///
        /// @brief          The algorithm is complete.
        ///
        REF_CAL_GPS_DONE,

        ///
        /// @brief          The algorithm has timed out.
        ///
        REF_CAL_GPS_ABORT_TIMEOUT
    }  mode_t;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The constants.
    //
private:
    ///
    /// @brief          The maximum  relative offset that the algorithms can handle.
    /// @details        This is is an estimate (guess) of the maximum tuning range of VCTCXO.  The data sheet guarantees
    ///                 a minimum of +/-7 ppm and one has been measured with +/-10 ppm, so +/-20 ppm seems reasonable.
    ///
    static const double INVALID_OFFSET_PPM = 20;

    ///
    /// @brief          The maximum time between 1 pps measurements.
    ///
    static const unsigned MEAS_1PPS_TIMEOUT_VAL_MS = 2500;

    ///
    /// @brief          The maximum number of iterations for which the engine should remain in main calibration loop
    ///                 before declaring a timeout.
    ///
    static const unsigned LOOP_TIMEOUT_ITERS = 300;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The values.
    //
private:
    ///
    /// @brief          The FPGA interface instance to be used.
    ///
    Fpga                    &fpga;

    ///
    /// @brief          The VCTCXO DAC controller instance to be used.
    ///
    RefCalAltB              controller_impl;

    ///
    /// @brief          The mode/state of the algorithm.
    ///
    mode_t                  mode;

    ///
    /// @brief          The time when the current calibration was started.
    /// @details        Only used in printing the elapsed calibration time.
    ///
    struct timeval          start_time;

    ///
    /// @brief          The time when the last 1 pps measurement was taken.
    /// @details        Used to implement a timeout so as to detect an error in the FPGA.
    ///
    struct timeval          last_1pps_time;

    ///
    /// @brief          The timeout counter for the main calibration loop.
    ///
    unsigned                loop_timeout_cntr;

    ///
    /// @brief          The debug stream.
    /// @details        Debug messages are sent to this stream.  If the debugging level has been appropriately
    ///                 configured they appear on the standard output console.  See ::DebugStream.
    ///
    DebugStream             debugStr;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Constructs an instance of the class.
    /// @details        The members are initialised and the start time is logged.
    /// @param[in]      fpga                The interface to the FPGA.
    /// @param[in]      debug_files         true if the debug files should be generated.
    ///
public:
    RefCalGPS( Fpga &fpga, unsigned debug );

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The destructor has nothing to do.
    ///
public:
    ~RefCalGPS();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Processes an iteration of the control loop.
    /// @details        The function is called at regular intervals and it gets the current time, which is used in
    ///                 implementing a timeout for the 1 PPS signal.  If the 1 PPS signal has not yet been detected and
    ///                 the timeout expires the algorithm is aborted and the state is changed to
    ///                 REF_CAL_GPS_ABORT_TIMEOUT.
    ///
    ///                 If a 1 PPS signal has been detected the function controls the operation of the algorithm
    ///                 according to the state:
    ///
    ///                 -   REF_CAL_GPS_IDLE    This is first 1 PPS signal since the algorithm was started and so may be
    ///                                         a pending event of indeterminate age.  It is discarded and the function
    ///                                         changes the state to REF_CAL_GPS_INIT.
    ///                 -   REF_CAL_GPS_INIT    This is second 1 PPS signal since the algorithm was started and so may
    ///                                         be incomplete.  It is discarded and the function changes the state to
    ///                                         REF_CAL_GPS_ACTIVE.
    ///                 -   REF_CAL_GPS_ACTIVE  The function calculate the frequency offset relative to the sample
    ///                                         clock.  If this is excessively large a warning is printed and the value
    ///                                         is ignored.  Otherwise it passes the value to the VCTCXO DAC controller,
    ///                                         which determines if the DAC needs to be updated.  If no update is
    ///                                         necessary the calibration is complete and the function changes the state
    ///                                         to REF_CAL_GPS_DONE.
    /// @return         The state of the algorithm.
    ///
public:
    iter_res_t iterate(void);

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief          The main implementation for the correlator-based reference calibration.
/// @details        The derived classes define the reference source and the VCTCXO DAC controller implementation that
///                 should be used, and this class controls the correlatorA and provides the controller with the
///                 frequency offset estimates that it should minimise.  The frequency offset is estimated from the
///                 timing drift of the peak produced by the correlator.
///
class RefCalCorrelator : public RefCal
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The types.
    //
public:
    ///
    /// @brief          The mode/state of the algorithm.
    ///
    typedef enum
    {
        ///
        /// @brief          The algorithm has been started but the range is not yet stable.
        ///
        REF_CAL_CORR_IDLE,

        ///
        /// @brief          The algorithm is waiting for a new correlator result.
        ///
        REF_CAL_CORR_WAITING,

        ///
        /// @brief          The algorithm is waiting to perform the initial measurements.
        ///
        REF_CAL_CORR_INIT,

        ///
        /// @brief          The DAC has been updated so the FPGA pipeline is being flushed.
        ///
        REF_CAL_CORR_FLUSH1,

        ///
        /// @brief          The DAC has been updated so the FPGA pipeline is being flushed.
        ///
        REF_CAL_CORR_FLUSH2,

        ///
        /// @brief          The algorithm is starting a new measurement.
        ///
        REF_CAL_CORR_START_MEAS,

        ///
        /// @brief          The algorithm is completing a measurement and updating the DAC if necessary.
        ///
        REF_CAL_CORR_FINISH_MEAS,

        ///
        /// @brief          The algorithm is complete.
        ///
        REF_CAL_CORR_DONE,

        ///
        /// @brief          The algorithm has timed out.
        ///
        REF_CAL_CORR_ABORT_TIMEOUT
    }  mode_t;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The constants.
    //
private:
    ///
    /// @brief          The minimum number of frames used for each timing drift measurement.
    /// @details        This value should be as large as possible so that errors in estimating the timing of the
    ///                 reference signal have a small effect on the estimate of the timing drift.  However a large value
    ///                 requires a wider search window and the result is an increase in the probability that a false
    ///                 positive will produce a wildly inaccurate estimate.  A value of 100 frames produces an estimate
    ///                 every second, which is a reasonable compromise.
    ///
    static const int MIN_FRAMES_FOR_MEAS = 100;

    ///
    /// @brief          The maximum timing drift that can be measured.
    /// @details        This determines the width of the search window used in identifying the peak that determines the
    ///                 signal timing.  It is related to the MIN_FRAMES_FOR_MEAS value and the maximum frequency error
    ///                 that the algorithm can handle.  With MIN_FRAMES_FOR_MEAS set to 100 a 6 ppm error produces a
    ///                 drift of 46.1 samples, so a value of 50.0 seems appropriate.
    ///
    static const double MAX_OFFSET_DIFF_SAMPLES = 50.0;

    ///
    /// @brief          The maximum number of iterations for which the engine should remain in the REF_CAL_INIT state
    ///                 before declaring a timeout.
    ///
    static const unsigned INIT_STATE_TIMEOUT_ITERS = 5;

    ///
    /// @brief          The maximum number of iterations for which the engine should remain in main calibration loop
    ///                 before declaring a timeout.
    ///
    static const unsigned LOOP_TIMEOUT_ITERS = 300;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // The values.
    //
private:
    ///
    /// @brief          The correlator instance to be used.
    ///
    Correlator                  corr;

    ///
    /// @brief          The VCTCXO DAC controller instance to be used.
    ///
    std::auto_ptr<Controller>   controller_pimpl;

    ///
    /// @brief          The range controller instance to be used.
    ///
    Ranging                     &ranging;

    ///
    /// @brief          The mode/state that the algorithm is in.
    ///
    mode_t                      mode;

    ///
    /// @brief          The position of the correlation peak in the last iteration.
    ///
    Correlator::peak_pos_t      prev_pos;

    ///
    /// @brief          The adjustment in dB to the threshold used to identify significant correlation peaks..
    ///
    double                      threshold_factor_dB;

    ///
    /// @brief          The timeout counter for the REF_CAL_INIT state.
    ///
    unsigned init_state_timeout_cntr;

    ///
    /// @brief          The timeout counter for the main calibration loop.
    ///
    unsigned loop_timeout_cntr;

    ///
    /// @brief          The debug stream.
    /// @details        Debug messages are sent to this stream.  If the debugging level has been appropriately
    ///                 configured they appear on the standard output console.  See ::DebugStream.
    ///
    DebugStream                 debugStr;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Constructs an instance of the class using the supplied correlation reference source and VCTCXO
    ///                 DAC controller.
    /// @details        The correlator is constructed and the member variables are initialised.
    /// @param[in]      fpga                The interface to the FPGA.
    /// @param[in]      rangeCtrl           The range controller.
    /// @param[in]      src                 The correlation reference source.
    /// @param[in]      low_power           True if the low-power settings should be used, false if high-sensitivity is
    ///                                     required.
    /// @param[in]      cnt                 The VCTCXO DAC controller.
    /// @param[in]      th                  The correlation threshold factor.
    /// @param[in]      antenna_mode        The antenna mode to be used.
    /// @param[in]      str                 The name of the correlation reference source.
    /// @param[in]      debug               The debug verbosity setting.
    /// @param[in]      debug_files         true if the debug files should be generated.
    ///
protected:
    RefCalCorrelator(Fpga                             &fpga,
                     Ranging                          &rangeCtrl,
                     std::auto_ptr<CorrelationSource> src,
                     bool                             low_power,
                     std::auto_ptr<Controller>        cnt,
                     double                           th,
                     unsigned                         antenna_mode,
                     std::string                      str,
                     unsigned                         debug,
                     bool                             debug_files);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The destructor has nothing to do.
    ///
protected:
    ~RefCalCorrelator();

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Processes an iteration of the control loop.
    /// @details        The function is called at regular intervals and it checks to see if a new correlator result is
    ///                 available.
    ///
    ///                 If there has been a correlator timeout the processing depends on the LOCKUP_RECOVERY compiler
    ///                 switch, but even if it is set recovery is only attempted if the algorithm state is
    ///                 REF_CAL_CORR_IDLE.  Generally the function changes the algorithm state to REF_CAL_ABORT_TIMEOUT
    ///                 and returns.
    ///
    ///                 If a correlator result is available the function lets the ranging controller decide if the range
    ///                 needs to be adjusted.  If the latter adjusts the range the function instructs the correlator to
    ///                 release the current result buffer and returns.
    ///
    ///                 If a correlator result is available and the range is stable the function controls the operation
    ///                 of the algorithm according to the state:
    ///
    ///                 -   REF_CAL_CORR_IDLE           The function instructs the correlator to release the current
    ///                                                 result buffer, and changes the state to REF_CAL_CORR_INIT.
    ///                 -   REF_CAL_CORR_INIT           The function gets the details of the new correlator peak and
    ///                                                 instructs the correlator to release the current result buffer
    ///                                                 (in this state the search for the peak spans a whole frame) and
    ///                                                 if this is the first iteration in this state it simply stores
    ///                                                 the details.  Otherwise it compares the position of  the new
    ///                                                 peak with that of the peak in last iteration.  If the timing
    ///                                                 drift is less than MAX_OFFSET_DIFF_SAMPLES the function changes
    ///                                                 the state to REF_CAL_CORR_START_MEAS.  In any case it stores the
    ///                                                 details of the peak for use in the next iteration ofthe algorithm.
    ///                 -   REF_CAL_CORR_FLUSH1         The function instructs the correlator to release the current
    ///                                                 result buffer and changes the algorithm state to
    ///                                                 REF_CAL_CORR_FLUSH2.
    ///                 -   REF_CAL_CORR_FLUSH2         The function instructs the correlator to release the current
    ///                                                 result buffer and changes the algorithm state to
    ///                                                 REF_CAL_CORR_START_MEAS.
    ///                 -   REF_CAL_CORR_START_MEAS     The function gets the details of the new correlator peak and
    ///                                                 instructs the correlator to release the current result buffer
    ///                                                 (in this state the search for the peak spans a region
    ///                                                 +/-MAX_OFFSET_DIFF_SAMPLES samples either side of the position
    ///                                                 of the last peak).  The details of the peak are stored.
    ///                 -   REF_CAL_CORR_FINISH_MEAS    The function gets the frame number of the of the new correlation
    ///                                                 result and if fewer than MIN_FRAMES_FOR_MEAS frames have passed
    ///                                                 since the last result it simply instructs the correlator to
    ///                                                 release the current result buffer and returns.  Otherwise it
    ///                                                 gets the details of the new correlator peak and instructs the
    ///                                                 correlator to release the current result buffer (in this state
    ///                                                 the search for the peak spans a region
    ///                                                 +/-MAX_OFFSET_DIFF_SAMPLES samples either side of the position
    ///                                                 of the last peak).  The function calculates the timing drift of
    ///                                                 the peak and converts this to an equivalent sample frequency
    ///                                                 offset.  It passes this value to the VCTCXO DAC controller,
    ///                                                 which determines if the DAC needs to be updated.  If no update
    ///                                                 is necessary the calibration is complete and the function
    ///                                                 changes the state to REF_CAL_CORR_DONE.  Otherwise it changes
    ///                                                 the state to REF_CAL_CORR_FLUSH1.
    /// @return         The state of the algorithm.
    ///
public:
    iter_res_t iterate(void);

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief          Configures the correlator-based reference calibration to use the PCPICH reference source.
/// @details        This class only instantiates the constructor and destructor - the ::RefCalCorrelator class provides
///                 the interface and implementation for all other methods.
///
class RefCalCPICH : public RefCalCorrelator
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Constructs an instance of the class using the ::DownlinkPCPICHSource correlation reference
    ///                 source and the ::RefCalPID controller.
    /// @details        The base class constructor is called with an instance of the ::DownlinkPCPICHSource class as the
    ///                 correlation reference source and an instance of the ::RefCalPID class as the controller.
    /// @param[in]      fpga                The interface to the FPGA.
    /// @param[in]      rangeCtrl           The range controller.
    /// @param[in]      low_power           True if the low-power settings should be used, false if high-sensitivity is
    ///                                     required.
    /// @param[in]      code                The scrambling code to be used.
    /// @param[in]      div_ant             The diversity antenna pattern to be used.
    /// @param[in]      th                  The correlation threshold factor.
    /// @param[in]      antenna_mode        The antenna mode to be used.
    /// @param[in]      debug               The debug verbosity setting.
    /// @param[in]      debug_files         true if the debug files should be generated.
    ///
public:
    RefCalCPICH(Fpga     &fpga,
                Ranging  &rangeCtrl,
                bool     low_power,
                uint32_t code,
                uint32_t div_ant,
                double   th,
                unsigned antenna_mode,
                unsigned debug,
                bool     debug_files);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The destructor has nothing to do.
    ///
public:
    ~RefCalCPICH();
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief          Configures the correlator-based reference calibration to use the PSCH reference source.
/// @details        This class only instantiates the constructor and destructor - the ::RefCalCorrelator class provides
///                 the interface and implementation for all other methods.
///
class RefCalPSCH : public RefCalCorrelator
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          Constructs an instance of the class using the ::DownlinkPSCHSource correlation reference source
    ///                 and the ::RefCalPID controller.
    /// @details        The base class constructor is called with an instance of the ::DownlinkPSCHSource class as the
    ///                 correlation reference source and an instance of the ::RefCalPID class as the controller.
    /// @param[in]      fpga                The interface to the FPGA.
    /// @param[in]      rangeCtrl           The range controller.
    /// @param[in]      low_power           True if the low-power settings should be used, false if high-sensitivity is
    ///                                     required.
    /// @param[in]      th                  The correlation threshold factor.
    /// @param[in]      antenna_mode        The antenna mode to be used.
    /// @param[in]      debug               The debug verbosity setting.
    /// @param[in]      debug_files         true if the debug files should be generated.
    ///
public:
    RefCalPSCH(Fpga     &fpga,
               Ranging  &rangeCtrl,
               bool     low_power,
               double   th,
               unsigned antenna_mode,
               unsigned debug,
               bool     debug_files);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief          The destructor has nothing to do.
    ///
public:
    ~RefCalPSCH();
};



#endif


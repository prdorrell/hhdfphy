/******************************************************************************
 * Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 * All Rights Reserved
 ******************************************************************************
 * THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 * ----------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/refcal.cpp $
 * $Revision: 7469 $
 * $Author: pdm $
 * $Date: 2011-10-06 13:26:22 +0100 (Thu, 06 Oct 2011) $
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file refcal.cpp
 * \brief Definitions of controller classes to calibrate vctcxo.
 * Calibrators for cpich, psch, gps-1pps and couple of different base
 * controller implementations.
 *
 *****************************************************************************/
#include "refcal.hpp"
#include "design.hpp"
#include "dac.hpp"
#include <boost/format.hpp>
#include <cmath>
#include <sys/time.h>

///////////////////////////////////////////////////////////////////////////////
//
// PID controller stuff
//
///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class.
//  @details        The correlator is constructed and the member variables are initialised.
//  @param[in]      kp                  The gain of the proportional term.
//  @param[in]      ki                  The gain of the integral term.
//  @param[in]      kd                  The gain of the differential term.
//  @param[in]      ft                  The time constant of the leaky integrator filter.
//  @param[in]      debug               The debug verbosity setting.
//
RefCalPID::RefCalPID( double kp, double ki, double kd, double ft, unsigned debug ) :
    Controller(debug),
    KP(kp),
    KI(ki),
    KD(kd),
    FILTER_TIME(ft),
    filter_initialised(false),
    filtered_offset(0.0),
    integrator(0),
    differentiator(0.0),
    error_squared(),
    n_err_vals(0),
    curr_err_val(0)
{
    DAC::set(Design::MID_VCTCXO_RANGE_DAC_VAL);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          The destructor has nothing to do.
//
RefCalPID::~RefCalPID()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief          Performs one iteration of the control loop using the supplied offset frequency.
/// @details        This function implements a PID controller to try and minimise the time taken to reach the correct
///                 setting. Typically this function will need to be called a number of times to reach a stable state.
///                 The values pass through a simple filter before passing to the pid.
/// @param[in]      off                 The current estimate of the frequency offset.
/// @return                             true if the function needs to be called again (the DAC has been updated), false
///                                     if the loop has stabilised and the calibration is complete.
///
bool RefCalPID::update( double off )
{
    bool retVal = true;

    //
    //  Apply a simple filter to the offset estimates.
    //
    if (!filter_initialised)
    {
        filtered_offset = off;
        filter_initialised = true;
    }
    else
    {
        double k = 1.0/FILTER_TIME;

        filtered_offset = k*off + (1-k)*filtered_offset;
    }

    off = filtered_offset;

    //
    //  Clip input offset to avoid large disturbances to controller
    //
    if      (off >  (Design::samples_per_second*MAX_REL_FREQ_OFFSET_PPM*1e-6))
    {
        off =  (Design::samples_per_second*MAX_REL_FREQ_OFFSET_PPM*1e-6);
    }
    else if (off < -(Design::samples_per_second*MAX_REL_FREQ_OFFSET_PPM*1e-6))
    {
        off = -(Design::samples_per_second*MAX_REL_FREQ_OFFSET_PPM*1e-6);
    }

    //
    //  Calculate the error for the set-point of 0
    //
    double error = 0 - off;

    //
    //  Save error squared for use later on in the convergence test
    //
    error_squared[curr_err_val++] = error*error;

    //
    //  Wrap the index into the array of squared error values.
    //
    if (curr_err_val >= MSE_SIZE)
    {
        curr_err_val = 0;
    }

    //
    //  Saturate the count of squared error terms.
    //
    if (n_err_vals < MSE_SIZE)
    {
        n_err_vals++;
    }

    //
    //  Update integrator. Output is clamped to prevent windup
    //
    integrator += error;
    const double INT_LIMIT = Design::MAX_DAC_VAL/KI;
    if      (integrator >  INT_LIMIT)
    {
        integrator =  INT_LIMIT;
    }
    else if (integrator < -INT_LIMIT)
    {
        integrator = -INT_LIMIT;
    }

    //
    //  Calculate the controller output
    //
    double output =   static_cast<double>(Design::MID_VCTCXO_RANGE_DAC_VAL)
                    + (error * KP + integrator * KI - (error - differentiator) * KD + 0.5);

    //
    //  Don't let dac over/underflow
    //
    if (output > static_cast<double>(Design::MAX_VCTCXO_RANGE_DAC_VAL))
    {
        output = static_cast<double>(Design::MAX_VCTCXO_RANGE_DAC_VAL);
    }
    if (output < static_cast<double>(Design::MIN_VCTCXO_RANGE_DAC_VAL))
    {
        output = static_cast<double>(Design::MIN_VCTCXO_RANGE_DAC_VAL);
    }
    unsigned int dacval = static_cast<unsigned int>(output);

    // line end comes later
    debugStr(DebugStream::info4) << boost::format("e: %8.4f ppm, i: %8.2f, d: %6.2f, p: %8.2f i: %8.2f, d: %6.2f, dac: %4d, ")
                                    % (error/(Design::samples_per_second/1e6))
                                    % integrator
                                    % (error - differentiator)
                                    % (error*KP)
                                    % (integrator*KI)
                                    % ((error - differentiator) * KD)
                                    % dacval;

    //
    //  Update the differentiator
    //
    differentiator = error;

    //
    //  Write the new DAC value
    //
    DAC::set( dacval );

    //
    //  Check mean square error in frequency offset to determine if we are close enough
    //
    if (n_err_vals >= MSE_SIZE)
    {
        double mse = 0.0;
        for (int i=0; i<MSE_SIZE; i++)
        {
            mse += error_squared[i];
        }

        debugStr(DebugStream::info4) << boost::format("rms:%8.4f ppm\n")
                                        % (sqrt(mse/MSE_SIZE)/(Design::samples_per_second/1e6));

        if (  (mse/MSE_SIZE)
            < (  Design::samples_per_second*TARGET_REL_FREQ_OFFSET_PPM*1e-6
               * Design::samples_per_second*TARGET_REL_FREQ_OFFSET_PPM*1e-6))
        {
            debugStr(DebugStream::info4) << "cal success\n";
            retVal = false;
        }
    }
    else
    {
        debugStr(DebugStream::info4) << "\n";
    }

    return (retVal);
}

///////////////////////////////////////////////////////////////////////////////
//
// Alternative controller stuff
//
///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class.
//  @details        The correlator is constructed and the member variables are initialised.
//  @param[in]      n_init              The initial averaging window size.
//  @param[in]      N_grow_init         The factor to grow window by each iteration.
//  @param[in]      debug               The debug verbosity setting.
//
RefCalAltB::RefCalAltB( double n_init, double N_grow_init, unsigned debug ) :
    Controller( debug ),
    controller_state(INITIAL_MEASURE1),
    n( n_init ),
    N_grow( N_grow_init ),
    integrator(0),
    m(1),
    f0(),
    measurement_count(0),
    measurement_sum(0),
    measurement_sum_sq(0),
    dacval_init0(Design::MID_VCTCXO_RANGE_DAC_VAL),
    dacval_init1(Design::MID_VCTCXO_RANGE_DAC_VAL)
{
    DAC::set( dacval_init0 );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          The destructor has nothing to do.
//
RefCalAltB::~RefCalAltB()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief          Performs one iteration of the control loop using the supplied offset frequency.
/// @details        First take two measurements to estimate DAC value / frequency gradient.  If this was ideal
///                 (noise free/linear) we could imediately calculate the required DAC value to make the error 0.
///                 Iteratively make improvements, averaging over bigger windows each time.  Successful when the
///                 average error and RMS error within some defined values.
/// @param[in]      off                 The current estimate of the frequency offset.
/// @return                             true if the function needs to be called again (the DAC has been updated),
///                                     false if the loop has stabilised and the calibration is complete.
///
bool RefCalAltB::update( double off )
{
    bool call_again = false;
    using std::fabs;

    ++measurement_count;

    if( measurement_count == 1 )
    {
        //
        //  Throw away the first measurement after a change because it will be inbetween old and new running speed
        //  also reset sums at this point
        //
        measurement_sum = 0;
        measurement_sum_sq = 0;
        call_again = true;
    }
    else
    {
        debugStr(DebugStream::info4) << boost::format("off = %8.4f ppm (%6.1f)\n")
                                        % (off/(Design::samples_per_second/1e6))
                                        % (off/Design::samples_per_second*Design::gps_nominal_ticks);

        measurement_sum += off;
        measurement_sum_sq += off*off;
        if( measurement_count <= n )
        {
            call_again = true;
        }
        else
        {
            switch( controller_state )
            {
                case INITIAL_MEASURE1:
                {
                    //
                    //  Calculate the average offset and reset the measurement count for the next measurement.
                    //
                    f0 = measurement_sum / double(measurement_count-1);
                    debugStr(DebugStream::info4) << boost::format("f0 = %8.4f ppm\n") % (f0/(Design::samples_per_second/1e6));
                    measurement_count = 0;

                    //
                    //  Choose the DAC value for the second measurement based on the sign of the offset so that the
                    //  results should have the opposite sign.
                    //
                    if (f0 < 0)
                    {
                        dacval_init1 = (Design::MAX_VCTCXO_RANGE_DAC_VAL+Design::MID_VCTCXO_RANGE_DAC_VAL)/2;
                    }
                    else
                    {
                        dacval_init1 = (Design::MIN_VCTCXO_RANGE_DAC_VAL+Design::MID_VCTCXO_RANGE_DAC_VAL)/2;
                    }

                    //
                    //  Program the DAC value for the second measurement of the slope estimation.
                    //
                    DAC::set(dacval_init1);

                    //
                    //  Change the state so that the second measurement can be taken.
                    //
                    controller_state = INITIAL_MEASURE2;

                    call_again = true;
                    break;
                }
                case INITIAL_MEASURE2:
                {
                    //
                    //  Calculate the average offset and reset the measurement count for the next measurement.
                    //
                    double f1 = measurement_sum / double(measurement_count-1);
                    debugStr(DebugStream::info4) << boost::format("f1 = %8.4f ppm\n") % (f1/(Design::samples_per_second/1e6));
                    measurement_count = 0;

                    //
                    //  Calculate the slope and check that it is reasonable.
                    //
                    m = double(dacval_init1 - dacval_init0) / (f1 - f0);
                    if ((m < 0) || (m > 1e6))
                    {
                        debugStr(DebugStream::warn1) << boost::format("m not resonable! (f0 = %8.4f, f1 = %8.4f, m = %f\n")
                                                        % (f0/(Design::samples_per_second/1e6))
                                                        % (f1/(Design::samples_per_second/1e6))
                                                        % m;

                        //
                        //  Program the DAC value to repeat the first measurement of the slope estimation.
                        //
                        DAC::set(dacval_init0);

                        //
                        //  Change the state so that the first measurement can be taken.
                        //
                        controller_state = INITIAL_MEASURE1;

                        call_again = true;
                    }
                    else
                    {
                        //
                        //  Use the slope to make a better estimate of the ideal DAC value, store it in the integral term
                        //  and program it into the DAC.
                        //
                        integrator = dacval_init1 - m*f1;
                        if ( integrator > Design::MAX_VCTCXO_RANGE_DAC_VAL )
                        {
                            integrator = Design::MAX_VCTCXO_RANGE_DAC_VAL;
                        }
                        else
                        if ( integrator < Design::MIN_VCTCXO_RANGE_DAC_VAL )
                        {
                            integrator = Design::MIN_VCTCXO_RANGE_DAC_VAL;
                        }

                        DAC::set( static_cast<unsigned int>( integrator+0.5 ) );

                        //
                        //  Require the nest offset measurement to use more results.
                        //
                        n *= N_grow;

                        debugStr(DebugStream::info4) << boost::format("e: %8.4f ppm, s: %8.4f, i: %6.1f, n: %d\n")
                                                        % (-f1/(Design::samples_per_second/1e6))
                                                        % (m*(Design::samples_per_second/1e6))
                                                        % integrator
                                                        % n;

                        //
                        //  Change the state to enter the main control loop.
                        //
                        controller_state = RUNNING;

                        call_again = true;
                    }
                    break;
                }
                case RUNNING:
                {
                    //
                    //  Calculate the average error and mean-square error.
                    //
                    double error = - measurement_sum / double(measurement_count-1);
                    double ms = measurement_sum_sq / double(measurement_count-1);
                    measurement_count = 0;

                    //
                    //  Decide if the errors are small enough to declare success.
                    //
                    if (   (fabs(error) < (Design::samples_per_second*MEAN_THRESH_PPM*1e-6))
                        && (ms          < (  (Design::samples_per_second*RMS_THRESH_PPM*1e-6)
                                           * (Design::samples_per_second*RMS_THRESH_PPM*1e-6))))
                    {
                        debugStr(DebugStream::info1) << boost::format("mean error: %5.3f, mean sq error: %5.3f, int: %f\n")
                                                        % error
                                                        % ms
                                                        % integrator;
                        debugStr(DebugStream::info1) << boost::format("mean thr:   %5.3f, mean sq THR:   %5.3f. Lock success.\n")
                                                        % (Design::samples_per_second*MEAN_THRESH_PPM*1e-6)
                                                        % (  (Design::samples_per_second*RMS_THRESH_PPM*1e-6)
                                                           * (Design::samples_per_second*RMS_THRESH_PPM*1e-6));
                        call_again = false;
                    }
                    else
                    {
                        //
                        //  Update the integral term and program it into the DAC..
                        //
                        integrator += m * error;
                        if ( integrator > Design::MAX_VCTCXO_RANGE_DAC_VAL )
                        {
                            integrator = Design::MAX_VCTCXO_RANGE_DAC_VAL;
                        }
                        else
                        if ( integrator < Design::MIN_VCTCXO_RANGE_DAC_VAL )
                        {
                            integrator = Design::MIN_VCTCXO_RANGE_DAC_VAL;
                        }

                        DAC::set( static_cast<unsigned int>( integrator+0.5 ) );

                        //
                        //  Require the nest offset measurement to use more results.
                        //
                        n *= N_grow;

                        debugStr(DebugStream::info4) << boost::format("e: %8.4f ppm, rms: %8.4f, i: %6.1f, n: %d\n")
                                                        % (error/(Design::samples_per_second/1e6))
                                                        % (sqrt(ms)/(Design::samples_per_second/1e6))
                                                        % integrator
                                                        % n;

                        call_again = true;
                    }
                    break;
                }
            }
        }
    }

    return (call_again);
}


///////////////////////////////////////////////////////////////////////////////
//
// GPS cal
//
///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class.
//  @details        The members are initialised and the start time is logged.
//  @param[in]      fpga                The interface to the FPGA.
//  @param[in]      debug_files         true if the debug files should be generated.
//
RefCalGPS::RefCalGPS(Fpga &fpga, unsigned debug) :
    RefCal(),
    fpga(fpga),
    controller_impl(10, 1.5, debug),
    mode(REF_CAL_GPS_IDLE),
    start_time(),
    last_1pps_time(),
    loop_timeout_cntr(0),
    debugStr( "RefCalGPS... ", debug )
{
    gettimeofday(&start_time, NULL);
    last_1pps_time = start_time;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          The destructor has nothing to do.
//
RefCalGPS::~RefCalGPS()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Processes an iteration of the control loop.
//  @details        The function is called at regular intervals and it gets the current time, which is used in
//                  implementing a timeout for the 1 PPS signal.  If the 1 PPS signal has not yet been detected and
//                  the timeout expires the algorithm is aborted and the state is changed to REF_CAL_GPS_ABORT_TIMEOUT.
//
//                  If a 1 PPS signal has been detected the function controls the operation of the algorithm according
//                  to the state:
//
//                  -   REF_CAL_GPS_IDLE    This is first 1 PPS signal since the algorithm was started and so may be
//                                          a pending event of indeterminate age.  It is discarded and the function
//                                          changes the state to REF_CAL_GPS_INIT.
//                  -   REF_CAL_GPS_INIT    This is second 1 PPS signal since the algorithm was started and so may be
//                                          incomplete.  It is discarded and the function changes the state to
//                                          REF_CAL_GPS_ACTIVE.
//                  -   REF_CAL_GPS_ACTIVE  The function calculate the frequency offset relative to the sample clock.
//                                          If this is excessively large a warning is printed and the value is ignored.
//                                          Otherwise it passes the value to the VCTCXO DAC controller, which determines
//                                          if the DAC needs to be updated.  If no update is necessary the calibration
//                                          is complete and the function changes the state to REF_CAL_GPS_DONE.
//  @return         The state of the algorithm.
//
RefCal::iter_res_t RefCalGPS::iterate()
{
    RefCal::iter_res_t ret_mode = REF_CAL_ACTIVE;

    double new_offset;

    //
    //  Determine how long the calibration has been busy so that a timeout can be implemented.
    //
    struct timeval curr_time;
    gettimeofday(&curr_time, NULL);

    double milli_secs_since_start =   double(curr_time.tv_sec -start_time.tv_sec )*1e3
                                    + double(curr_time.tv_usec-start_time.tv_usec)/1e3;

    double milli_secs_since_last_1pps  =   double(curr_time.tv_sec -last_1pps_time.tv_sec )*1e3
                                         + double(curr_time.tv_usec-last_1pps_time.tv_usec)/1e3;

    //
    //  Get the frequency offset from fpga
    //
    uint32_t delta;

    if(!fpga.get_1pps_count(delta))
    {
        //
        //  Implement the GPS timeout.
        //
        if (milli_secs_since_last_1pps > MEAS_1PPS_TIMEOUT_VAL_MS)
        {
            debugStr(DebugStream::error) << boost::format(  "Aborted: time since test started = %10.3f, since milli_secs_since_last_1pps = %10.3f\n")
                                                          % milli_secs_since_start
                                                          % milli_secs_since_last_1pps;
            mode = REF_CAL_GPS_ABORT_TIMEOUT;
            ret_mode = REF_CAL_ABORT_TIMEOUT;
        }
    }
    else
    {
        debugStr(DebugStream::info1) << boost::format(  "Time since calibration started = %10.3f, since 1 pps last measured = %10.3f\n")
                                                      % milli_secs_since_start
                                                      % milli_secs_since_last_1pps;

        switch (mode)
        {
            case REF_CAL_GPS_IDLE :
            {
                //
                //  Ignore the first result from an incomplete second.
                //
                debugStr(DebugStream::info1) << "RefCalGPS   REF_CAL_GPS_IDLE         -> REF_CAL_GPS_INIT\n";
                mode = REF_CAL_GPS_INIT;
                break;
            }
            case REF_CAL_GPS_INIT :
            {
                //
                //  Ignore the second result from an incomplete second.
                //
                debugStr(DebugStream::info1) << "RefCalGPS   REF_CAL_GPS_INIT         -> REF_CAL_GPS_ACTIVE\n";
                mode = REF_CAL_GPS_ACTIVE;
                break;
            }
            case REF_CAL_GPS_ACTIVE :
            {
                ++loop_timeout_cntr;

                //
                //  Convert the tick count into a frequency offset relative to the sample clock.
                //
                new_offset = (int)delta - (int)Design::gps_nominal_ticks;
                new_offset /= Design::gps_nominal_ticks;
                new_offset *= Design::samples_per_second;

                //
                //  Detect unreasonable estimates and ignore them.
                //
                if(   (new_offset > +static_cast<double>(Design::samples_per_second)*INVALID_OFFSET_PPM*1e-6)
                   || (new_offset < -static_cast<double>(Design::samples_per_second)*INVALID_OFFSET_PPM*1e-6) )
                {
                    debugStr(DebugStream::warn1) <<"invalid offset "<<new_offset<<", "<<delta<<'\n';

                    if (loop_timeout_cntr >= LOOP_TIMEOUT_ITERS)
                    {
                        debugStr(DebugStream::info1) << "RefCalGPS   REF_CAL_GPS_ACTIVE       -> REF_CAL_GPS_ABORT_TIMEOUT\n";
                        mode = REF_CAL_GPS_ABORT_TIMEOUT;
                        ret_mode = REF_CAL_ABORT_TIMEOUT;
                    }
                }
                else
                {
                    if (!controller_impl.update(new_offset))
                    {
                        debugStr(DebugStream::info1) << "RefCalGPS   REF_CAL_GPS_ACTIVE       -> REF_CAL_GPS_DONE\n";
                        mode = REF_CAL_GPS_DONE;
                        ret_mode = REF_CAL_DONE;
                    }
                    else
                    if (loop_timeout_cntr >= LOOP_TIMEOUT_ITERS)
                    {
                        debugStr(DebugStream::info1) << "RefCalGPS   REF_CAL_GPS_ACTIVE       -> REF_CAL_GPS_ABORT_TIMEOUT\n";
                        mode = REF_CAL_GPS_ABORT_TIMEOUT;
                        ret_mode = REF_CAL_ABORT_TIMEOUT;
                    }

                }
                break;
            }
            default :
            {
                throw std::runtime_error( "invalid RefCalGPS mode" );
                break;
            }
        }

        //
        //  Store the time when the last 1 pps measurement was completed.
        //
        gettimeofday(&last_1pps_time, NULL);
    }

    return ret_mode;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class using the supplied correlation reference source and VCTCXO DAC
//                  controller.
//  @details        The correlator is constructed and the member variables are initialised.
//  @param[in]      fpga                The interface to the FPGA.
//  @param[in]      rangeCtrl           The range controller.
//  @param[in]      src                 The correlation reference source.
//  @param[in]      low_power           True if the low-power settings should be used, false if high-sensitivity is
//                                      required.
//  @param[in]      cnt                 The VCTCXO DAC controller.
//  @param[in]      th                  The correlation threshold factor.
//  @param[in]      antenna_mode        The antenna mode to be used.
//  @param[in]      str                 The name of the correlation reference source.
//  @param[in]      debug               The debug verbosity setting.
//  @param[in]      debug_files         true if the debug files should be generated.
//
RefCalCorrelator::RefCalCorrelator(Fpga                             &fpga,
                                   Ranging                          &rangeCtrl,
                                   std::auto_ptr<CorrelationSource> src,
                                   bool                             low_power,
                                   std::auto_ptr<Controller>        cnt,
                                   double                           th,
                                   unsigned                         antenna_mode,
                                   std::string                      str,
                                   unsigned                         debug,
                                   bool                             debug_files) :
    RefCal(),
    corr(fpga,
         src,
         low_power,
         (antenna_mode == 2) ? Correlator::CAL_ANT2 : Correlator::CAL_ANT1,
         0,
         debug,
         debug_files),
    controller_pimpl(cnt),
    ranging(rangeCtrl),
    mode(REF_CAL_CORR_IDLE),
    prev_pos(),
    threshold_factor_dB(th),
    init_state_timeout_cntr(0),
    loop_timeout_cntr(0),
    debugStr(str, debug)
{
    prev_pos.frame = -1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          The destructor has nothing to do.
//
RefCalCorrelator::~RefCalCorrelator()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Processes an iteration of the control loop.
//  @details        The function is called at regular intervals and it checks to see if a new correlator result is
//                  available.
//
//                  If there has been a correlator timeout the processing depends on the LOCKUP_RECOVERY compiler
//                  switch, but even if it is set recovery is only attempted if the algorithm state is
//                  REF_CAL_CORR_IDLE.  Generally the function changes the algorithm state to REF_CAL_ABORT_TIMEOUT and
//                  returns.
//
//                  If a correlator result is available the function lets the ranging controller decide if the range
//                  needs to be adjusted.  If the latter adjusts the range the function instructs the correlator to
//                  release the current result buffer and returns.
//
//                  If a correlator result is available and the range is stable the function controls the operation of
//                  the algorithm according to the state:
//
//                  -   REF_CAL_CORR_IDLE           The function instructs the correlator to release the current result
//                                                  buffer, and changes the state to REF_CAL_CORR_INIT.
//                  -   REF_CAL_CORR_INIT           The function gets the details of the new correlator peak and
//                                                  instructs the correlator to release the current result buffer (in
//                                                  this state the search for the peak spans a whole frame) and if this
//                                                  is the first iteration in this state it simply stores the details.
//                                                  Otherwise it compares the position of  the new peak with that of the
//                                                  peak in last iteration.  If the timing drift is less than
//                                                  MAX_OFFSET_DIFF_SAMPLES the function changes the state to
//                                                  REF_CAL_CORR_START_MEAS.  In any case it stores the details of the
//                                                  peak for use in the next iteration ofthe algorithm.
//                  -   REF_CAL_CORR_FLUSH1         The function instructs the correlator to release the current result
//                                                  buffer and changes the algorithm state to REF_CAL_CORR_FLUSH2.
//                  -   REF_CAL_CORR_FLUSH2         The function instructs the correlator to release the current result
//                                                  buffer and changes the algorithm state to REF_CAL_CORR_START_MEAS.
//                  -   REF_CAL_CORR_START_MEAS     The function gets the details of the new correlator peak and
//                                                  instructs the correlator to release the current result buffer (in
//                                                  this state the search for the peak spans a region
//                                                  +/-MAX_OFFSET_DIFF_SAMPLES samples either side of the position of
//                                                  the last peak).  The details of the peak are stored.
//                  -   REF_CAL_CORR_FINISH_MEAS    The function gets the frame number of the of the new correlation
//                                                  result and if fewer than MIN_FRAMES_FOR_MEAS frames have passed
//                                                  since the last result it simply instructs the correlator to release
//                                                  the current result buffer and returns.  Otherwise it gets the
//                                                  details of the new correlator peak and instructs the correlator to
//                                                  release the current result buffer (in this state the search for the
//                                                  peak spans a region +/-MAX_OFFSET_DIFF_SAMPLES samples either side
//                                                  of the position of the last peak).  The function calculates the
//                                                  timing drift of the peak and converts this to an equivalent sample
//                                                  frequency offset.  It passes this value to the VCTCXO DAC
//                                                  controller, which determines if the DAC needs to be updated.  If no
//                                                  update is necessary the calibration is complete and the function
//                                                  changes the state to REF_CAL_CORR_DONE.  Otherwise it changes the
//                                                  state to REF_CAL_CORR_FLUSH1.
//  @return         The mode/state of the algorithm.
//
RefCal::iter_res_t RefCalCorrelator::iterate()
{
    RefCal::iter_res_t ret_mode = REF_CAL_ACTIVE;

    Correlator::busy_result_t busy_result = corr.busy();
    if (busy_result == Correlator::RESULT_AVAILABLE)
    {
        double channel_power = corr.get_channel_power();
        corr.set_threshold(channel_power, threshold_factor_dB);

        Ranging::RangingUpdateResult rangingUpdateResult = Ranging::RANGE_STABLE;
#ifdef USE_CHAN_RSSI_RANGING
        rangingUpdateResult = ranging.update(channel_power);
#else
        rangingUpdateResult = ranging.update(corr.get_adc_power());
#endif

        if( rangingUpdateResult == Ranging::RANGE_UNSTABLE )
        {
            corr.pop_result();
        }
        else
        {
            switch ( mode )
            {
                case REF_CAL_CORR_IDLE:
                {
                    corr.pop_result();
                    debugStr(DebugStream::info1) << "RefCalCorrelator   REF_CAL_CORR_IDLE        -> REF_CAL_CORR_INIT\n";
                    mode = REF_CAL_CORR_INIT;
                    break;
                }
                case REF_CAL_CORR_INIT:
                {
                    using Design::samples_per_second;

                    Correlator::peak_pos_t p = corr.get_peak(0, Design::samples_per_frame);
                    corr.pop_result();

                    debugStr(DebugStream::info4) << boost::format("frame = %10u, offset = %6.1f, value = %f\n")
                                                    % p.frame
                                                    % p.offset
                                                    % p.pwr;

                    if (isnan(p.offset))
                    {
                        //
                        //  All the results lie below the threshold.
                        //
                        ++init_state_timeout_cntr;
                        if (init_state_timeout_cntr >= INIT_STATE_TIMEOUT_ITERS)
                        {
                            debugStr(DebugStream::info1) << "RefCalCorrelator   REF_CAL_CORR_INIT        -> REF_CAL_CORR_ABORT_TIMEOUT\n";
                            mode = REF_CAL_CORR_ABORT_TIMEOUT;
                            ret_mode = REF_CAL_ABORT_TIMEOUT;
                        }
                    }
                    else
                    if(prev_pos.frame == -1)
                    {
                        //
                        //  The first pass so there is no preveious peak with which to compare the new peak.
                        //
                        prev_pos = p;
                    }
                    else
                    {
                        //
                        //  Get the difference in the last two positions and unwrap it.
                        //
                        double pos_diff = p.offset-prev_pos.offset;
                        if (pos_diff > +static_cast<double>(Design::samples_per_frame)/2.0)
                        {
                            pos_diff -= static_cast<double>(Design::samples_per_frame);
                        }
                        else
                        if (pos_diff < -static_cast<double>(Design::samples_per_frame)/2.0)
                        {
                            pos_diff += static_cast<double>(Design::samples_per_frame);
                        }
                        prev_pos = p;

                        //
                        //  If the difference is small enough allow the calibration to start.
                        //
                        if (fabs(pos_diff) < MAX_OFFSET_DIFF_SAMPLES)
                        {
                            debugStr(DebugStream::info3) << "RefCalCorrelator   REF_CAL_CORR_INIT        -> REF_CAL_CORR_FINISH_MEAS\n";
                            mode = REF_CAL_CORR_FINISH_MEAS;
                        }
                        else
                        {
                            ++init_state_timeout_cntr;
                            if (init_state_timeout_cntr >= INIT_STATE_TIMEOUT_ITERS)
                            {
                                debugStr(DebugStream::info1) << "RefCalCorrelator   REF_CAL_CORR_INIT        -> ABORT_TIMEOUT\n";
                                mode = REF_CAL_CORR_ABORT_TIMEOUT;
                                ret_mode = REF_CAL_ABORT_TIMEOUT;
                            }
                        }
                    }
                    break;
                }
                case REF_CAL_CORR_FLUSH1:
                {
                    corr.pop_result();
                    debugStr(DebugStream::info1) << "RefCalCorrelator   REF_CAL_CORR_FLUSH1      -> REF_CAL_CORR_FLUSH2\n";
                    mode = REF_CAL_CORR_FLUSH2;
                    break;
                }
                case REF_CAL_CORR_FLUSH2:
                {
                    corr.pop_result();
                    debugStr(DebugStream::info1) << "RefCalCorrelator   REF_CAL_CORR_FLUSH2      -> REF_CAL_CORR_START_MEAS\n";
                    mode = REF_CAL_CORR_START_MEAS;
                    break;
                }
                case REF_CAL_CORR_START_MEAS:
                {
                    uint32_t search_start = static_cast<uint32_t>(prev_pos.offset);
                    if (search_start < static_cast<uint32_t>(MAX_OFFSET_DIFF_SAMPLES))
                    {
                        search_start =   (search_start+Design::samples_per_frame)
                                       - static_cast<uint32_t>(MAX_OFFSET_DIFF_SAMPLES);
                    }
                    else
                    {
                        search_start =   search_start
                                       - static_cast<uint32_t>(MAX_OFFSET_DIFF_SAMPLES);
                    }

                    uint32_t search_len   = 2*static_cast<uint32_t>(MAX_OFFSET_DIFF_SAMPLES)+1;

                    prev_pos = corr.get_peak(search_start, search_len);
                    corr.pop_result();

                    debugStr(DebugStream::info4) << boost::format("frame = %10u, offset = %6.1f, value = %f\n")
                                                    % prev_pos.frame
                                                    % prev_pos.offset
                                                    % prev_pos.pwr;

                    if (isnan(prev_pos.offset))
                    {
                        //
                        //  All the results lie below the threshold.
                        //
                        ++loop_timeout_cntr;
                        if (loop_timeout_cntr >= LOOP_TIMEOUT_ITERS)
                        {
                            debugStr(DebugStream::info1) << "RefCalCorrelator   REF_CAL_CORR_START_MEAS  -> REF_CAL_CORR_ABORT_TIMEOUT\n";
                            mode = REF_CAL_CORR_ABORT_TIMEOUT;
                            ret_mode = REF_CAL_ABORT_TIMEOUT;
                        }
                    }
                    else
                    {
                        debugStr(DebugStream::info1) << "RefCalCorrelator   REF_CAL_CORR_START_MEAS  -> REF_CAL_CORR_FINISH_MEAS\n";
                        mode = REF_CAL_CORR_FINISH_MEAS;
                    }
                    break;
                }
                case REF_CAL_CORR_FINISH_MEAS:
                {
                    int64_t delta_frame = corr.get_frame() - prev_pos.frame;
                    if( delta_frame >= MIN_FRAMES_FOR_MEAS )
                    {
                        using Design::samples_per_second;

                        //
                        //  Get the samples near the previous position ofthe peak.
                        //
                        uint32_t search_start = static_cast<uint32_t>(prev_pos.offset);
                        if (search_start < static_cast<uint32_t>(MAX_OFFSET_DIFF_SAMPLES))
                        {
                            search_start =   (search_start+Design::samples_per_frame)
                                           - static_cast<uint32_t>(MAX_OFFSET_DIFF_SAMPLES);
                        }
                        else
                        {
                            search_start =   search_start
                                           - static_cast<uint32_t>(MAX_OFFSET_DIFF_SAMPLES);
                        }

                        uint32_t search_len   = 2*static_cast<uint32_t>(MAX_OFFSET_DIFF_SAMPLES)+1;

                        Correlator::peak_pos_t p = corr.get_peak(search_start, search_len);
                        corr.pop_result();

                        debugStr(DebugStream::info4) << boost::format("frame = %10u, offset = %6.1f, value = %f\n")
                                                        % p.frame
                                                        % p.offset
                                                        % p.pwr;

                        if (isnan(p.offset))
                        {
                            //
                            //  All the results lie below the threshold.
                            //
                            ++loop_timeout_cntr;
                            if (loop_timeout_cntr >= LOOP_TIMEOUT_ITERS)
                            {
                                debugStr(DebugStream::info1) << "RefCalCorrelator   REF_CAL_CORR_FINISH_MEAS -> REF_CAL_CORR_ABORT_TIMEOUT\n";
                                mode = REF_CAL_CORR_ABORT_TIMEOUT;
                                ret_mode = REF_CAL_ABORT_TIMEOUT;
                            }
                        }
                        else
                        {
                            //
                            //  Calculate the change in the peak position.
                            //
                            double delta_offset = p.offset - prev_pos.offset;
                            if( delta_offset < -static_cast<double>(Design::samples_per_frame)/2.0 )
                            {
                                delta_offset += static_cast<double>(Design::samples_per_frame);
                            }
                            else if ( delta_offset > static_cast<double>(Design::samples_per_frame)/2.0 )
                            {
                                delta_offset -= static_cast<double>(Design::samples_per_frame);
                            }

                            //
                            //  Calculate the time separation of the two peaks.
                            //
                            double delta = delta_frame*static_cast<double>(Design::samples_per_frame) + delta_offset;

                            //
                            //  Calculate the number of frames separating the two peaks.
                            //
                            double actual_frames = floor(  (delta+static_cast<double>(Design::samples_per_frame)/2.0)
                                                         / static_cast<double>(Design::samples_per_frame));

                            //
                            //  Calculate the frequency offset that corresponds to the timing drift.
                            //
                            double freq_offset =   (  delta
                                                    - actual_frames*static_cast<double>(Design::samples_per_frame))
                                                 / (actual_frames*static_cast<double>(Design::samples_per_frame))
                                                 * samples_per_second;

                            debugStr(DebugStream::info1) << boost::format("frequency offset = %f\n")
                                                            % freq_offset;

                            //
                            //  Update the loop controller.
                            //
                            if (!controller_pimpl->update( freq_offset ))
                            {
                                debugStr(DebugStream::info1) << "RefCalCorrelator   REF_CAL_CORR_FINISH_MEAS -> REF_CAL_CORR_DONE\n";
                                mode = REF_CAL_CORR_DONE;
                                ret_mode = REF_CAL_DONE;
                            }
                            else
                            {
                                ++loop_timeout_cntr;
                                if (loop_timeout_cntr >= LOOP_TIMEOUT_ITERS)
                                {
                                    debugStr(DebugStream::info1) << "RefCalCorrelator   REF_CAL_CORR_FINISH_MEAS -> REF_CAL_CORR_ABORT_TIMEOUT\n";
                                    mode = REF_CAL_CORR_ABORT_TIMEOUT;
                                    ret_mode = REF_CAL_ABORT_TIMEOUT;
                                }
                                else
                                {
                                    debugStr(DebugStream::info1) << "RefCalCorrelator   REF_CAL_CORR_FINISH_MEAS -> REF_CAL_CORR_FLUSH1\n";
                                    mode = REF_CAL_CORR_FLUSH1;
                                }
                            }
                        }
                    }
                    else
                    {
                        corr.pop_result();
                    }
                    break;
                }
                default :
                {
                    throw std::runtime_error( "invalid RefCalCorrelator mode" );
                    break;
                }
            }
        }
    }
    else
    if (busy_result == Correlator::RESULT_TIMEOUT)
    {
#ifdef LOCKUP_RECOVERY
        switch ( mode )
        {
            case REF_CAL_CORR_IDLE:
            {
                corr.restart_correlation();
                break;
            }
            case REF_CAL_CORR_INIT:
            {
                debugStr(DebugStream::info1) << "RefCalCorrelator   REF_CAL_CORR_INIT        -> REF_CAL_CORR_ABORT_TIMEOUT\n";
                mode = REF_CAL_CORR_ABORT_TIMEOUT;
                ret_mode = REF_CAL_ABORT_TIMEOUT;
                break;
            }
            case REF_CAL_CORR_FLUSH1:
            {
                debugStr(DebugStream::info1) << "RefCalCorrelator   REF_CAL_CORR_FLUSH1      -> REF_CAL_CORR_ABORT_TIMEOUT\n";
                mode = REF_CAL_CORR_ABORT_TIMEOUT;
                ret_mode = REF_CAL_ABORT_TIMEOUT;
                break;
            }
            case REF_CAL_CORR_FLUSH2:
            {
                debugStr(DebugStream::info1) << "RefCalCorrelator   REF_CAL_CORR_FLUSH2      -> REF_CAL_CORR_ABORT_TIMEOUT\n";
                mode = REF_CAL_CORR_ABORT_TIMEOUT;
                ret_mode = REF_CAL_ABORT_TIMEOUT;
                break;
            }
            case REF_CAL_CORR_START_MEAS:
            {
                debugStr(DebugStream::info1) << "RefCalCorrelator   REF_CAL_CORR_START_MEAS  -> REF_CAL_CORR_ABORT_TIMEOUT\n";
                mode = REF_CAL_CORR_ABORT_TIMEOUT;
                ret_mode = REF_CAL_ABORT_TIMEOUT;
                break;
            }
            case REF_CAL_CORR_FINISH_MEAS:
            {
                debugStr(DebugStream::info1) << "RefCalCorrelator   REF_CAL_CORR_FINISH_MEAS -> REF_CAL_CORR_ABORT_TIMEOUT\n";
                mode = REF_CAL_CORR_ABORT_TIMEOUT;
                ret_mode = REF_CAL_ABORT_TIMEOUT;
                break;
            }
            default :
            {
                throw std::runtime_error( "invalid RefCalCorrelator mode" );
                break;
            }
        }
#else
        debugStr(DebugStream::info1) << "RefCalCorrelator   Some State               -> REF_CAL_CORR_ABORT_TIMEOUT\n";
        mode = REF_CAL_CORR_ABORT_TIMEOUT;
        ret_mode = REF_CAL_ABORT_TIMEOUT;
#endif
    }

    return ret_mode;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class using the ::DownlinkPCPICHSource correlation reference
//                  source and the ::RefCalPID controller.
//  @details        The base class constructor is called with an instance of the ::DownlinkPCPICHSource class as the
//                  correlation reference source and an instance of the ::RefCalPID class as the controller.
//  @param[in]      fpga                The interface to the FPGA.
//  @param[in]      rangeCtrl           The range controller.
//  @param[in]      low_power           True if the low-power settings should be used, false if high-sensitivity is
//                                      required.
//  @param[in]      code                The scrambling code to be used.
//  @param[in]      div_ant             The diversity antenna pattern to be used.
//  @param[in]      th                  The correlation threshold factor.
//  @param[in]      antenna_mode        The antenna mode to be used.
//  @param[in]      debug               The debug verbosity setting.
//  @param[in]      debug_files         true if the debug files should be generated.
//
RefCalCPICH::RefCalCPICH(Fpga     &fpga,
                         Ranging  &rangeCtrl,
                         bool     low_power,
                         uint32_t code,
                         uint32_t div_ant,
                         double   th,
                         unsigned antenna_mode,
                         unsigned debug,
                         bool     debug_files) :
    RefCalCorrelator(fpga,
                     rangeCtrl,
                     std::auto_ptr<CorrelationSource>(new DownlinkPCPICHSource(code, div_ant, 1, debug, debug_files)),
                     low_power,
                     std::auto_ptr<Controller>(new RefCalPID(5,2,0,10, debug)),
                     th,
                     antenna_mode,
                     "RefCalCPICH. ",
                     debug,
                     debug_files)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          The destructor has nothing to do.
//
RefCalCPICH::~RefCalCPICH()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          Constructs an instance of the class using the ::DownlinkPSCHSource correlation reference source and
//                  the ::RefCalPID controller.
//  @details        The base class constructor is called with an instance of the ::DownlinkPSCHSource class as the
//                  correlation reference source and an instance of the ::RefCalPID class as the controller.
//  @param[in]      fpga                The interface to the FPGA.
//  @param[in]      rangeCtrl           The range controller.
//  @param[in]      low_power           True if the low-power settings should be used, false if high-sensitivity is
//                                      required.
//  @param[in]      th                  The correlation threshold factor.
//  @param[in]      antenna_mode        The antenna mode to be used.
//  @param[in]      debug               The debug verbosity setting.
//  @param[in]      debug_files         true if the debug files should be generated.
//
RefCalPSCH::RefCalPSCH(Fpga     &fpga,
                       Ranging  &rangeCtrl,
                       bool     low_power,
                       double   th,
                       unsigned antenna_mode,
                       unsigned debug,
                       bool     debug_files) :
    RefCalCorrelator(fpga,
                     rangeCtrl,
                     std::auto_ptr<CorrelationSource>(new DownlinkPSCHSource(debug)),
                     low_power,
                     std::auto_ptr<Controller>(new RefCalPID(5,2,0,10, debug)),
                     th,
                     antenna_mode,
                     "RefCalPSCH.. ",
                     debug,
                     debug_files)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  @brief          The destructor has nothing to do.
//
RefCalPSCH::~RefCalPSCH()
{
}



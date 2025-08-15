/******************************************************************************
 * Copyright (c) 2010 Plextek Limited, Great Chesterford, England.
 * All Rights Reserved
 ******************************************************************************
 * THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
 * ----------------------------------------------------------------------------
 * $HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/src/searchandtrack.hpp $
 * $Revision: 6276 $
 * $Author: pdm $
 * $Date: 2011-07-12 15:11:56 +0100 (Tue, 12 Jul 2011) $
 ******************************************************************************
 * File Description (User Field)
 * ================
 */
/** \file searchandtrack.hpp
 * \brief Declaration of SearchAndTrack class that class that implements algorithm to
 * find and track the correlation paths.
 *
 *****************************************************************************/
#ifndef __SEARCHANDTRACK_HPP__
#define __SEARCHANDTRACK_HPP__

#include "correlator.hpp"
#include "debug.hpp"
#include "pwrcal.hpp"
#include "ranging.hpp"

#include <list>
#include <memory>

/// \brief Search for a correlation peak.
/// Reduce correlation window to a smaller size.
/// Track peaks (paths) in that window as they drift
/// (due to reference timing errors).
class SearchAndTrack{
public:

    /// The constants controlling the life of a path in terms of the valid_count values.
    static const int32_t VALID_COUNT_INIT_VAL       = -4; ///< The value used when a path is first identified.
    static const int32_t VALID_COUNT_CONFIRMED      = +0; ///< The threshold used to confirm the path as valid
    static const int32_t VALID_COUNT_MAX_VAL        = +0; ///< The maximum value taken by the valid_count
    static const int32_t VALID_COUNT_IS_INVALID     = -6; ///< The value at which the path is declared invalid and is removed from the list
    static const int32_t VALID_COUNT_PATH_FOUND     = +1;  ///< The increment used when a peak matches
    static const int32_t VALID_COUNT_PATH_NOT_FOUND = -2;  ///< The increment used when a peak matches

    /// The threshold relative to the strongest peak in the tracking window that defines significant multipath
    /// components.
    static const double REL_THRESHOLD_FACTOR = 0.1;

    /// Structure to group useful correlation path information.
    struct Path_t
    {
        uint32_t        id;             ///< GUID (increments from 1)
        int32_t         valid_count;    ///< For temporal filtering + old path destruction
        uint64_t        pos;            ///< Absolute position of path in samples, note big enough for thousands of years
        double          power;          ///< Path power (corr peak value divided by radio power gain, linear)
        double          power_alt;      ///< power for alternate antenna
    };

    /// List of paths found
    typedef std::list< Path_t >   PathList_t;

    /// Mode/state algorithm is in.
    enum mode_t{ IDLE,
                 START_SEARCH_FLUSH1, START_SEARCH_FLUSH2,
                 SEARCHING_PART1, SEARCHING_PART2,
                 START_TRACK_FLUSH1, START_TRACK_FLUSH2,
                 TRACKING, TRACKING_PAUSE };

    /// Result of the iterate() function.
    enum iter_res_t{ WAITING, START_SEARCH, START_TRACK, NEW_RESULT, ABORT_TIMEOUT };

    SearchAndTrack( Ranging &rangeCtrl,
                    PwrCal &pwrCal,
                    std::auto_ptr< Correlator >,
                    unsigned antenna_mode,
                    double search_th,
                    double track_th,
                    unsigned searching_pause,
                    unsigned tracking_pause,
                    unsigned debug
                  );

    iter_res_t          iterate( void );
    PathList_t const&   get_paths( void );
    double              get_power( void );
    void                restart( void );

    void                debug_printing( unsigned b );

private:
    int                 search      ( void );
    int                 track       ( void );
    void                annihilate  ( int, std::vector<double>& );

    /// Algorithm predicate
    static bool         is_invalid  ( Path_t const & a ) { return a.valid_count <= VALID_COUNT_IS_INVALID; }
    /// Algorithm predicate
    static bool         descending_power( Path_t const & a, Path_t const & b )
                                        { return a.power > b.power; }

    PathList_t                  path_list;                  ///< list of paths found
    std::auto_ptr< Correlator > the_correlator;             ///< correlator object to use
    unsigned                    antenna_mode;               ///< diversity antenna mode
    Ranging                     &ranging;                   ///< auto ranging object
    PwrCal                      &power_cal;                 ///< power calibration object
    mode_t                      mode;                       ///< mode/state this algorithm is in
    int                         corr_start;                 ///< offset in samples from fpga frame counter to start corr
    double                      search_threshold_factor_dB; ///< added to the built-in threshold for search mode
    double                      track_threshold_factor_dB;  ///< added to the built-in threshold for track mode
    uint32_t                    path_id_counter;            ///< GUID for paths
    double                      input_channel_power;        ///< correlator value divided by radio power gain
    unsigned const              searching_pause;            ///< number of frame counts to wait between searching iterations
    unsigned const              tracking_pause;             ///< number of frame counts to wait between tracking iterations

    Correlator::peak_t          search_peak1;               ///< the peak found in the first half of the search
    Correlator::peak_t          search_peak2;               ///< the peak found in the second half of the search

    int const           peak_annihilation_span;             ///< peak width in samples
    int const           peak_search_span;                   ///< number of samples to look for moved peak
    int const           tracking_window_size;               ///< in number of samples

    static const unsigned   TRACKING_TIMEOUT           = 500;
    static const unsigned   TRACKING_PATH_FOUND_DEC    = 5;
    static const unsigned   TRACKING_NO_PATH_FOUND_INC = 1;
    unsigned                no_path_timer;

    DebugStream debugStr;   ///< debug tracing output stream
};

#endif

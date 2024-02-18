/*
 *  Copyright (c) 2000-2022 Inria
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *  * Neither the name of the ALICE Project-Team nor the names of its
 *  contributors may be used to endorse or promote products derived from this
 *  software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Contact: Bruno Levy
 *
 *     https://www.inria.fr/fr/bruno-levy
 *
 *     Inria,
 *     Domaine de Voluceau,
 *     78150 Le Chesnay - Rocquencourt
 *     FRANCE
 *
 */

#ifndef GEOGRAM_BASIC_STOPWATCH
#define GEOGRAM_BASIC_STOPWATCH

#include <geogram/basic/common.h>
#include <geogram/basic/numeric.h>
#include <geogram/basic/logger.h>

#include <chrono>

/**
 * \file geogram/basic/stopwatch.h
 * \brief Classes for measuring time
 */

namespace GEO {

    /**
     * \brief Scope restricted stopwatch
     * \details Stopwatch prints the elapsed time
     * since its construction when it goes out of scope.
     * It uses SystemStopwatch to measure time.
     *
     * \code
     * {
     *     Stopwatch W("compute my stuff") ;
     *     ... do something ...
     * } // <- W prints the elapsed time here.
     * \endcode
     */
    class GEOGRAM_API Stopwatch {
    public:
        /**
         * \brief Stopwatch constructor
         * \param[in] task_name name of the job to measure. This name is
         * used as a Logger feature when displaying the elapsed time.
	 * \param[in] verbose if true, then elapsed time is displayed
	 *  when this Stopwatch is destroyed, else nothing is displayed.
         */
        Stopwatch(const std::string& task_name, bool verbose=true);


        /**
         * \brief Stopwatch constructor
         * \details Constructs a silent (verbose=false) Stopwatch
         */
        Stopwatch();
        
        /**
         * \brief Get the user elapsed time
         * \details Returns the user time elapsed since the StopWatch
         * construction (in seconds)
         */
        double elapsed_time() const;

        /**
         * \brief Stopwatch destructor
         * \details This prints the time elapsed since the Stopwatch
         * construction
         */
        ~Stopwatch();

        /**
         * \details Gets the current time since epoch (in seconds).
         */
        static double now();
        
    private:
        std::chrono::time_point<std::chrono::system_clock> start_;
        std::string task_name_;
	bool verbose_;
    };
}

#endif


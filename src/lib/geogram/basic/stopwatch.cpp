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

#include <geogram/basic/stopwatch.h>
#include <geogram/basic/command_line.h>
#include <iostream>

namespace GEO {

    double Stopwatch::process_start_time_ = 0.0;
    bool Stopwatch::global_stats_ = false;

    void Stopwatch::initialize() {
        process_start_time_ = now();
        global_stats_ =
            CmdLine::arg_is_declared("sys:stats") &&
            CmdLine::get_arg_bool("sys:stats") ;

    }

    void Stopwatch::show_stats() {
        Logger::out("Process") << "Total elapsed time: "
                               << process_elapsed_time()
                               << "s" << std::endl;
        // TODO: specific stats.
    }

    Stopwatch::Stopwatch(const std::string& task_name, bool verbose) :
        start_(std::chrono::system_clock::now()),
        task_name_(task_name),
        verbose_(verbose)
    {
    }

    Stopwatch::Stopwatch() :
        start_(std::chrono::system_clock::now()),
        verbose_(false)
    {
    }

    double Stopwatch::elapsed_time() const {
	// OMG, such nonsense ...
	// ... but well, lets me get time with reasonable resolution
	// in a portable way.
        auto now(std::chrono::system_clock::now());
        auto elapsed = now-start_;
        auto elapsed_milliseconds =
	    std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
        return 0.001 * double(elapsed_milliseconds.count());
    }

    double Stopwatch::now() {
        auto now(std::chrono::system_clock::now());
        auto elapsed = now.time_since_epoch();
        auto elapsed_milliseconds =
	    std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
        return 0.001 * double(elapsed_milliseconds.count());
    }

    Stopwatch::~Stopwatch() {
        if(verbose_) {
	    print_elapsed_time();
        }
    }

    void Stopwatch::print_elapsed_time() {
	Logger::out(task_name_)
	    << "Elapsed: " << String::format_time(elapsed_time())
	    << std::endl;
    }

}

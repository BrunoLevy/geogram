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

#include <geogram/numerics/PCK.h>
#include <geogram/basic/string.h>
#include <vector>
#include <algorithm>

namespace {
    using namespace GEO;

#ifdef PCK_STATS
    inline double percent(Numeric::int64 a, Numeric::int64 b) {
        return 100.0 * double(a) / double(b);
    }
#endif
    
}

namespace GEO {
    namespace PCK {

#ifdef PCK_STATS
        
        PredicateStats* PredicateStats::first_ = nullptr;
        
        PredicateStats::PredicateStats(
            const char* name
        ) : next_(first_),
            name_(name),
            invoke_count_(0),
            exact_count_(0),
            SOS_count_(0) {
            first_ = this;
        }

        void PredicateStats::show_all_stats() {
            std::vector<PredicateStats*> all_stats;
            for(
                PredicateStats* stats = first_;
                stats != nullptr; stats = stats->next_) {
                all_stats.push_back(stats);
            }
            std::sort(
                all_stats.begin(), all_stats.end(),
                [](const PredicateStats* a, const PredicateStats* b)->bool {
                    return (a->invoke_count_ > b->invoke_count_);
                }
            );
            for(PredicateStats* stats : all_stats) {
                stats->show_stats();
            }
        }

        void PredicateStats::show_stats() {

            if(invoke_count_ == 0) {
                return;
            }
            
            Logger::out("PCK stats") << "Predicate stats for: "
                                     << name_ << std::endl;
            
            Logger::out("PCK stats") 
                << String::format("   invocations : %12ld",
                                  Numeric::int64(invoke_count_))
                << std::endl;

            Numeric::int64 filter_hit_count = invoke_count_ - exact_count_;
            
            Logger::out("PCK stats")
                << String::format("    filter hit : %12ld (%3.2f %%)",
                                  Numeric::int64(filter_hit_count),
                                  percent(filter_hit_count, invoke_count_)
                                 )
                << std::endl;

            Logger::out("PCK stats")
                << String::format("         exact : %12ld (%3.2f %%)",
                                  Numeric::int64(exact_count_),
                                  percent(exact_count_, invoke_count_)
                                 )
                << std::endl;
            
            if(SOS_count_ != 0 || strstr(name_, "SOS") != nullptr) {
                Logger::out("PCK stats")                
                << String::format("           SOS : %12ld (%3.2f %%)",
                                  Numeric::int64(SOS_count_),
                                  percent(SOS_count_, invoke_count_)
                                 )
                << std::endl;
            }
            Logger::out("PCK stats") << std::endl;               
        }
#endif
        
    }
}

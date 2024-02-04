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

#ifndef GEOGRAM_BASIC_THREAD_SYNC
#define GEOGRAM_BASIC_THREAD_SYNC

#include <geogram/basic/common.h>
#include <geogram/basic/numeric.h>
#include <geogram/basic/assert.h>
#include <geogram/basic/argused.h>
#include <vector>
#include <atomic>


/**
 * \file geogram/basic/thread_sync.h
 * \brief Functions and classes for process manipulation
 */


// On MacOS, I get many warnings with atomic_flag initialization,
// such as std::atomic_flag f = ATOMIC_FLAG_INIT
#if defined(__clang__)
#pragma GCC diagnostic ignored "-Wbraced-scalar-init"
#endif

#ifdef GEO_OS_WINDOWS

// For splinlocks, I'd prefer to use std::atomic_flag, unfortunately;
// atomic_flag's constructor is not implemented in MSCV's stl,
// so we reimplement them using atomic compare-exchange functions...

#include <windows.h>
#include <intrin.h>
#pragma intrinsic(_InterlockedCompareExchange8)
#pragma intrinsic(_InterlockedCompareExchange16)
#pragma intrinsic(_InterlockedCompareExchange)
#pragma intrinsic(_interlockedbittestandset)
#pragma intrinsic(_interlockedbittestandreset)
#pragma intrinsic(_ReadBarrier)
#pragma intrinsic(_WriteBarrier)
#pragma intrinsic(_ReadWriteBarrier)

namespace GEO {
    namespace Process {
        /** A lightweight synchronization structure. */
        typedef short spinlock;

        /** The initialization value of a spin lock. */
#       define GEOGRAM_SPINLOCK_INIT 0
        inline void acquire_spinlock(volatile spinlock& x) {
            while(_InterlockedCompareExchange16(&x, 1, 0) == 1) {
                // Intel recommends to have a PAUSE asm instruction
                // in the spinlock loop. Under MSVC/Windows,
                // YieldProcessor() is a macro that calls the
                // (undocumented) _mm_pause() intrinsic function
                // that generates a PAUSE opcode.
                YieldProcessor();
            }
            // We do not need _ReadBarrier() here since
            // _InterlockedCompareExchange16
            // "acts as a full barrier in VC2005" according to the doc
        }

        inline void release_spinlock(volatile spinlock& x) {
            _WriteBarrier();   // prevents compiler reordering
            x = 0;
        }

        /**
         * \brief An array of light-weight synchronisation
         *  primitives (spinlocks).
         *
         * \details In this implementation, storage is optimized so that
         * a single bit per spinlock is used.
         *
         * \see acquire_spinlock(), release_spinlock()
         */
        class CompactSpinLockArray {
        public:
            /**
             * \brief Internal representation of SpinLockArray elements.
             * \details Each word_t represents 32 spinlocks.
             * \internal
             * LONG is 32 bits under MSVC
             * and is what interlockedbittestand(re)set uses.
             */
            typedef LONG word_t;

            /**
             * \brief Constructs a new SpinLockArray of size 0.
             */
            CompactSpinLockArray() : size_(0) {
            }

            /**
             * \brief Constructs a new SpinLockArray of size \p size_in.
             * \param[in] size_in number of spinlocks in the array.
             */
            CompactSpinLockArray(index_t size_in) : size_(0) {
                resize(size_in);
            }

            /**
             * \brief Resizes a SpinLockArray.
             * \details All the spinlocks are reset to 0.
             * \param[in] size_in The desired new size.
             */
            void resize(index_t size_in) {
                if(size_ != size_in) {
                    size_ = size_in;
                    index_t nb_words = (size_ >> 5) + 1;
                    spinlocks_.assign(nb_words, 0);
                }
            }

            /**
             * \brief Gets the number of spinlocks in this array.
             */
            index_t size() const {
                return size_;
            }

            /**
             * \brief Resets size to 0 and clears all the memory.
             */
            void clear() {
                spinlocks_.clear();
            }

            /**
             * \brief Acquires a spinlock at a given index
             * \details Loops until spinlock at index \p i is available then
             * reserve it.
             * \param[in] i index of the spinlock
             */
            void acquire_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                index_t w = i >> 5;
                index_t b = i & 31;
                while(_interlockedbittestandset((long *)(&spinlocks_[w]), long(b))) {
                    // Intel recommends to have a PAUSE asm instruction
                    // in the spinlock loop. Under MSVC/Windows,
                    // YieldProcessor() is a macro that calls the
                    // (undocumented) _mm_pause() intrinsic function
                    // that generates a PAUSE opcode.
                    YieldProcessor();
                }
                // We do not need here _ReadBarrier() since
                // _interlockedbittestandset
                // "acts as a full barrier in VC2005" according to the doc
            }

            /**
             * \brief Releases a spinlock at a given index
             * \details Makes spinlock at index \p i available to other threads.
             * \param[in] i index of the spinlock
             */
            void release_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                index_t w = i >> 5;
                index_t b = i & 31;
                // Note1: we need here to use a synchronized bit reset
                // since |= is not atomic.
                // Note2: We do not need here _WriteBarrier() since
                // _interlockedbittestandreset
                // "acts as a full barrier in VC2005" according to the doc
                _interlockedbittestandreset((long*)(&spinlocks_[w]), long(b));
            }

        private:
            std::vector<word_t> spinlocks_;
            index_t size_;
        };
        #define GEO_HAS_COMPACT_SPINLOCK_ARRAY
    }
}

#else

namespace GEO {
    namespace Process {

        /** The initialization value of a spinlock. */
#define GEOGRAM_SPINLOCK_INIT ATOMIC_FLAG_INIT 

        /** 
         * \brief A lightweight synchronization structure. 
         * \details See
         *  - https://rigtorp.se/spinlock/
         *  - https://www.sobyte.net/post/2022-06/cpp-memory-order/
         */
        typedef std::atomic_flag spinlock;

        /**
         * \brief Loops until \p x is available then reserves it.
         * \param[in] x a spinlock 
         */
        inline void acquire_spinlock(volatile spinlock& x) {
            for (;;) {
                if (!x.test_and_set(std::memory_order_acquire)) {
                    break;
                }
#if __cplusplus >= 202002L                
                while (x.test(std::memory_order_relaxed)) {
#endif
                    
#ifdef GEO_PROCESSOR_X86
#    ifdef __ICC
#      define geo_pause _mm_pause
#    else
#      define geo_pause __builtin_ia32_pause
#    endif
#endif

#if __cplusplus >= 202002L                
                }
#endif
            }            
        }

        /**
         * \brief Makes \p x available to other threads.
         * \param[in] x a spinlock 
         */
        inline void release_spinlock(volatile spinlock& x) {
            x.clear(std::memory_order_release); 
        }
    }
}
#endif


namespace GEO {

    namespace Process {
    
        // TODO: compact spinlock array.
        
        /**
         * \brief An array of light-weight synchronisation
         *  primitives (spinlocks).
         *
         * \details This is the reference implementation, that uses
         * an array of spinlock. There are more efficient
         * implementations for Linux and Windows.
         *
         * \see acquire_spinlock(), release_spinlock()
         */
        class BasicSpinLockArray {
        public:
            /**
             * \brief Constructs a new BasicSpinLockArray of size 0.
             */
            BasicSpinLockArray() : spinlocks_(nullptr), size_(0) {
            }

            /**
             * \brief Constructs a new BasicSpinLockArray of size \p size_in.
             * \param[in] size_in number of spinlocks in the array.
             */
            BasicSpinLockArray(index_t size_in) : spinlocks_(nullptr), size_(0) {
                resize(size_in);
            }

            /**
             * \brief Resizes a BasicSpinLockArray.
             * \details All the spinlocks are reset to 0.
             * \param[in] size_in The desired new size.
             */
            void resize(index_t size_in) {
                delete[] spinlocks_;
                spinlocks_ = new spinlock[size_in];
                size_ = size_in;
                // Need to initialize the spinlocks to false (dirty !)
                // (maybe use placement new on each item..., to be tested)
                for(index_t i=0; i<size_; ++i) {
                    Process::release_spinlock(spinlocks_[i]);
                }
            }

            /**
             * \brief Resets size to 0 and clears all the memory.
             */
            void clear() {
                delete[] spinlocks_;
                spinlocks_ = nullptr;
            }

            /**
             * \brief Gets the number of spinlocks in this array.
             */
            index_t size() const {
                return size_;
            }

            /**
             * \brief Acquires a spinlock at a given index
             * \details Loops until spinlock at index \p i is available then
             * reserve it.
             * \param[in] i index of the spinlock
             */
            void acquire_spinlock(index_t i) {
                geo_debug_assert(i < size());
                GEO::Process::acquire_spinlock(spinlocks_[i]);
            }

            /**
             * \brief Releases a spinlock at a given index
             * \details Makes spinlock at index \p i available to other threads.
             * \param[in] i index of the spinlock
             */
            void release_spinlock(index_t i) {
                geo_debug_assert(i < size());
                GEO::Process::release_spinlock(spinlocks_[i]);
            }

        private:
            spinlock* spinlocks_;
            index_t size_;
        };


        // TODO: compact spinlock array with atomic bit manip.
        
        typedef BasicSpinLockArray SpinLockArray;
    }
}

#ifdef GEO_OS_WINDOWS

namespace GEO {

    // Emulation of pthread mutexes using Windows API

    typedef CRITICAL_SECTION pthread_mutex_t;
    typedef unsigned int pthread_mutexattr_t;
    
    inline int pthread_mutex_lock(pthread_mutex_t *m) {
        EnterCriticalSection(m);
        return 0;
    }

    inline int pthread_mutex_unlock(pthread_mutex_t *m) {
        LeaveCriticalSection(m);
        return 0;
    }
        
    inline int pthread_mutex_trylock(pthread_mutex_t *m) {
        return TryEnterCriticalSection(m) ? 0 : EBUSY; 
    }

    inline int pthread_mutex_init(pthread_mutex_t *m, pthread_mutexattr_t *a) {
        geo_argused(a);
        InitializeCriticalSection(m);
        return 0;
    }

    inline int pthread_mutex_destroy(pthread_mutex_t *m) {
        DeleteCriticalSection(m);
        return 0;
    }


    // Emulation of pthread condition variables using Windows API

    typedef CONDITION_VARIABLE pthread_cond_t;
    typedef unsigned int pthread_condattr_t;

    inline int pthread_cond_init(pthread_cond_t *c, pthread_condattr_t *a) {
        geo_argused(a);
        InitializeConditionVariable(c);
        return 0;
    }

    inline int pthread_cond_destroy(pthread_cond_t *c) {
        geo_argused(c);
        return 0;
    }

    inline int pthread_cond_broadcast(pthread_cond_t *c) {
        WakeAllConditionVariable(c);
        return 0;
    }

    inline int pthread_cond_wait(pthread_cond_t *c, pthread_mutex_t *m) {
        SleepConditionVariableCS(c, m, INFINITE);
        return 0;
    }
}
#endif    


namespace GEO {
    namespace Process {
#ifdef GEO_HAS_COMPACT_SPINLOCK_ARRAY
        typedef CompactSpinLockArray SpinLockArray;
#else
        typedef BasicSpinLockArray SpinLockArray;
#endif        
    }
}




#endif


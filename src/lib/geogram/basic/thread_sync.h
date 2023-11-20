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
#include <geogram/basic/atomics.h>
#include <geogram/basic/numeric.h>
#include <geogram/basic/assert.h>
#include <geogram/basic/argused.h>
#include <vector>
#include <atomic>
#include <iostream>

#ifdef GEO_OS_APPLE
# define GEO_USE_DEFAULT_SPINLOCK_ARRAY
# include <AvailabilityMacros.h>
# if defined(MAC_OS_X_VERSION_10_12) && MAC_OS_X_VERSION_MIN_REQUIRED >= MAC_OS_X_VERSION_10_12
#   define GEO_APPLE_HAS_UNFAIR_LOCK 1
#   include <os/lock.h>
# endif
# if defined(__IPHONE_OS_VERSION_MIN_REQUIRED) && __IPHONE_OS_VERSION_MIN_REQUIRED >= __IPHONE_10_0
#   define GEO_APPLE_HAS_UNFAIR_LOCK 1
#   include <os/lock.h>
# endif
#endif

#ifdef geo_debug_assert
#define geo_thread_sync_assert(x) geo_debug_assert(x)
#else
#define geo_thread_sync_assert(x) 
#endif

/**
 * \file geogram/basic/thread_sync.h
 * \brief Functions and classes for process manipulation
 */

namespace GEO {

    namespace Process {

    // Non copyable spinlock class so we can put it in an array and resize.
    // Resizing the SpinLockArray will cause the spinlocks to lose their
    // atomicity though
    class SpinLock{
        std::atomic_flag locked;
    public:
        SpinLock() : locked(ATOMIC_FLAG_INIT)
        {

        }

        SpinLock(int state) : locked((bool)state)
        {

        }

        SpinLock(__attribute__((unused))const SpinLock &other)
        {
            // Nothing, this operator is just to make std::vector happy
        }
        SpinLock &operator=(__attribute__((unused))const SpinLock &other)
        {
            // Nothing, this operator is just to make std::vector happy
            return *this;
        }

        void lock(){
            while(locked.test_and_set(std::memory_order_acquire)) { ; }
        }
        void unlock(){
            locked.clear(std::memory_order_release);
        }
    };

        typedef SpinLock spinlock;
        /** The initialization value of a spin lock. */
#       define GEOGRAM_SPINLOCK_INIT 0
        inline void acquire_spinlock(spinlock& x) {
            x.lock();
        }

        inline void release_spinlock(spinlock& x) {
            x.unlock();
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
        class SpinLockArray {
        private:
            std::vector<SpinLock> spinlocks_;
            std::vector<std::atomic_flag>::size_type size_;
        public:

            /**
             * \brief Constructs a new SpinLockArray of size 0.
             */
            SpinLockArray() {
            }

            /**
             * \brief Constructs a new SpinLockArray of size \p size_in.
             * \param[in] size_in number of spinlocks in the array.
             */
            SpinLockArray(std::vector<std::atomic_flag>::size_type size_in) {
                resize(size_in);
                size_ = size_in;
            }

            /**
             * \brief Resizes a SpinLockArray.
             * \details All the spinlocks are reset to 0.
             * \param[in] size_in The desired new size.
             */
            void resize(std::vector<std::atomic_flag>::size_type size_in) {
                spinlocks_.resize(size_in);
                size_ = size_in;
            }
            
            /**
             * \brief Gets the number of spinlocks in this array.
             */
            std::vector<std::atomic_flag>::size_type size() const {
                return size_;
            }

            /**
             * \brief Resets size to 0 and clears all the memory.
             */
            void clear() {
                spinlocks_.clear();
                size_ = 0;
            }

            /**
             * \brief Acquires a spinlock at a given index
             * \details Loops until spinlock at index \p i is available then
             * reserve it.
             * \param[in] i index of the spinlock
             */
            void acquire_spinlock(std::vector<std::atomic_flag>::size_type i) {
                geo_thread_sync_assert(i < size());

                spinlocks_[i].lock();
            }

            /**
             * \brief Releases a spinlock at a given index
             * \details Makes spinlock at index \p i available to other threads.
             * \param[in] i index of the spinlock
             */
            void release_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                spinlocks_[i].unlock();
            }
        };

    }

#ifdef GEO_OS_WINDOWS

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
   
#endif    
    
}

#endif


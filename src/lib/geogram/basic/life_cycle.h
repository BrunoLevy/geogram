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

#ifndef GEOGRAM_BASIC_LIFE_CYCLE
#define GEOGRAM_BASIC_LIFE_CYCLE

#include <geogram/basic/common.h>
#include <geogram/basic/memory.h>
#include <geogram/basic/counted.h>

/**
 * \file geogram/basic/life_cycle.h
 * \brief Implementation of generic object lifecycle service
 */

#ifdef GEO_COMPILER_CLANG
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
#endif

namespace GEO {

    /**
     * \brief Manages the life cycle of an object.
     * \details Provides ways to call the constructor, destructor, copy
     *  constructor and assignation operator on objects known only from
     *  their addresses as raw untyped pointers. This is used by the
     *  Attribute mechanism of Geogram and by the Any class in Graphite/GOM.
     */
    class GEOGRAM_API LifeCycle : public Counted {
      public:

	/**
	 * \brief LifeCycle constructor.
	 * \param[in] object_size the size of an object in bytes.
	 */
        LifeCycle(size_t object_size, bool is_pod=false) :
	   object_size_(object_size), is_pod_(is_pod) {
	}

	/**
	 * \brief LifeCycle destructor.
	 */
	~LifeCycle() override;

	/**
	 * \brief Gets the size of an object.
	 * \return the size of an object in bytes.
	 */
	size_t object_size() const {
	    return object_size_;
	}

	/**
	 * \brief Tests whether object is pod (plain ordinary
	 *  datatype).
	 * \details Plain ordinary datatypes can be copied with
	 *  memcpy().
	 * \retval true if object is pod.
	 * \retval false otherwise.
	 */
	bool is_pod() const {
	    return is_pod_;
	}

	/**
	 * \brief Calls the constructor of an object.
	 * \param[in] address the address of the object
	 *  to be constructed.
	 * \details No memory allocation is done.
	 */
	virtual void construct(Memory::pointer address) = 0;

	/**
	 * \brief Copy-Constructs an object at a given address.
	 * \param[in] lhs the address where the object should
	 *  be constructed.
	 * \param[in] rhs the address of the right-hand-side object
	 *  to be copied.
	 * \details No memory allocation is done.
	 */
	virtual void copy_construct(
	    Memory::pointer lhs, Memory::const_pointer rhs
	) = 0;

	/**
	 * \brief Calls the destructor of an object.
	 * \param[in] address the address of the object to be destructed.
	 * \details No memory deallocation is done.
	 */
	virtual void destroy(Memory::pointer address) = 0;

	/**
	 * \brief Calls the assignment operator.
	 * \param[in] lhs the address of the left hand side.
	 * \param[in] rhs the address of the right hand side.
	 */
	virtual void assign(Memory::pointer lhs, Memory::const_pointer rhs) = 0;

	/**
	 * \brief Resets an object to its default value.
	 * \details Creates a new object with the default constructor and assigns
	 *  it to the object at \p address.
	 * \param[in] address the address of an already initialized object.
	 */
	virtual void reset(Memory::pointer address) = 0;


	/**
	 * \brief Swaps the objects at two addresses
	 * \param[in] a , b pointers to the two objects to be swapped
	 */
	virtual void swap(Memory::pointer a, Memory::pointer b) = 0;

	/**
	 * \brief Calls the constructor of objects in an array.
	 * \param[in] address the address of the object array
	 *  to be constructed.
	 * \param[in] nb number of objects.
	 * \details No memory allocation is done.
	 */
	virtual void construct_array(Memory::pointer address, index_t nb) = 0;

	/**
	 * \brief Copy-Constructs an array of object at a given address.
	 * \param[in] lhs the address of the objects to be constructed.
	 * \param[in] rhs the address of the right-hand-side objects
	 *  to be copied.
	 * \param[in] nb number of objects
	 * \details No memory allocation is done.
	 */
	virtual void copy_construct_array(
	    Memory::pointer lhs, Memory::const_pointer rhs, index_t nb
	) = 0;

	/**
	 * \brief Destroys an array of object at a given address.
	 * \param[in] address the address of the objects to be destroyed.
	 * \param[in] nb number of objects
	 * \details No memory deallocation is done.
	 */
	virtual void destroy_array(Memory::pointer address, index_t nb) = 0;

	/**
	 * \brief Copies an array of object at a given address.
	 * \param[in] lhs the address of the left-hand-side objects
	 * \param[in] rhs the address of the right-hand-side objects
	 *  to be copied.
	 * \param[in] nb number of objects
	 * \details No memory allocation is done.
	 */
	virtual void assign_array(
	    Memory::pointer lhs, Memory::const_pointer rhs, index_t nb
	) = 0;


	/**
	 * \brief Resets all objects in an array to their default value.
	 * \details creates an object with the default value and assigns
	 *  it to the element for each element in the array.
	 * \param[in] address the address of the object array
	 *  to be reset.
	 * \param[in] nb number of objects.
	 */
	virtual void reset_array(Memory::pointer address, index_t nb) = 0;

	/**
	 * \brief Dynamically allocates a new object.
	 * \return the address of the new object.
	 */
	virtual Memory::pointer new_object() = 0;

	/**
	 * \brief Dynamically allocates a new object with copy constructor.
	 * \param[in] rhs the address of the right-hand-side objects
	 *  to be copied.
	 * \return the address of the new object.
	 */
	virtual Memory::pointer new_object(Memory::const_pointer rhs) = 0;

	/**
	 * \brief Deletes an object.
	 * \param[in] address the address of the object.
	 */
	virtual void delete_object(Memory::pointer address) = 0;

	/**
	 * \brief Dynamically allocates an array of objects.
	 * \param[in] nb number of objects.
	 */
	virtual Memory::pointer new_array(index_t nb) = 0;

	/**
	 * \brief Deletes an array of objects.
	 * \param[in] address the address of the object array to
	 *  be deleted.
	 */
	virtual void delete_array(Memory::pointer address) = 0;


      private:
	size_t object_size_;
	bool is_pod_;
    };

    /**
     * \brief A reference_counted pointer to a LifeCycle.
     */
    typedef SmartPointer<LifeCycle> LifeCycle_var;

    /*************************************************************************/

    /**
     * \brief Concrete implementation of LifeCycle.
     */
    template <class T> class GenericLifeCycle : public LifeCycle {
      public:

	/**
	 * \brief GenericLifeCycle constructor.
	 */
        GenericLifeCycle() : LifeCycle(sizeof(T), std::is_pod<T>::value) {
	}

	/**
	 * \copydoc LifeCycle::construct()
	 */
	void construct(Memory::pointer address) override {
	    new(address)T;
	}

	/**
	 * \copydoc LifeCycle::copy_construct()
	 */
	void copy_construct(
	    Memory::pointer lhs, Memory::const_pointer rhs
	) override {
	    new(lhs)T(*(const T*)rhs);
	}

	/**
	 * \copydoc LifeCycle::destroy()
	 */
	void destroy(Memory::pointer address) override {
	    geo_argused(address); // Silences a MSVC warning.
	    ((T*)address)->~T();
	}

	/**
	 * \copydoc LifeCycle::assign()
	 */
	void assign(Memory::pointer lhs, Memory::const_pointer rhs) override {
	    *(T*)lhs=*(const T*)rhs;
	}

	/**
	 * \copydoc LifeCycle::reset()
	 */
	void reset(Memory::pointer address) override {
	    *(T*)address = T();
	}
	/**
	 * \copydoc LifeCycle::swap()
	 */
	void swap(Memory::pointer a, Memory::pointer b) override {
	    std::swap(*(T*)a, *(T*)b);
	}

	/**
	 * \copydoc LifeCycle::construct_array()
	 */
	void construct_array(Memory::pointer address, index_t nb) override {
	    T* array = (T*)address;
	    for(index_t i=0; i<nb; ++i) {
		new(&array[i])T;
	    }
	}

	/**
	 * \copydoc LifeCycle::copy_construct_array()
	 */
	void copy_construct_array(
	    Memory::pointer lhs, Memory::const_pointer rhs, index_t nb
	) override {
	    T* lhs_array = (T*)lhs;
	    const T* rhs_array = (const T*)rhs;
	    for(index_t i=0; i<nb; ++i) {
		new(&lhs_array[i])T(rhs_array[i]);
	    }
	}

	/**
	 * \copydoc LifeCycle::destroy_array()
	 */
	void destroy_array(Memory::pointer address, index_t nb) override {
	    T* array = (T*)address;
	    geo_argused(array); // Silences a MSVC warning.
	    for(index_t i=0; i<nb; ++i) {
		array[i].~T();
	    }
	}

	/**
	 * \copydoc LifeCycle::assign_array()
	 */
	void assign_array(
	    Memory::pointer lhs, Memory::const_pointer rhs, index_t nb
	) override {
	    T* lhs_array = (T*)lhs;
	    const T* rhs_array = (const T*)rhs;
	    for(index_t i=0; i<nb; ++i) {
		lhs_array[i] = rhs_array[i];
	    }
	}

	/**
	 * \copydoc LifeCycle::reset_array()
	 */
	void reset_array(Memory::pointer address, index_t nb) override {
	    T* array = (T*)address;
	    geo_argused(array); // Silences a MSVC warning.
	    for(index_t i=0; i<nb; ++i) {
		array[i] = T();
	    }
	}

	/**
	 * \copydoc LifeCycle::new_object()
	 */
	Memory::pointer new_object() override {
	    return Memory::pointer(new T);
	}

	/**
	 * \copydoc LifeCycle::new_object()
	 */
	Memory::pointer new_object(Memory::const_pointer rhs) override {
	    return Memory::pointer(new T(*(const T*)rhs));
	}

	/**
	 * \copydoc LifeCycle::delete_object()
	 */
	void delete_object(Memory::pointer address) override {
	    delete((T*)address);
	}

	/**
	 * \copydoc LifeCycle::new_array()
	 */
	Memory::pointer new_array(index_t nb) override {
	    return Memory::pointer(new T[nb]);
	}

	/**
	 * \copydoc LifeCycle::delete_array()
	 */
	void delete_array(Memory::pointer address) override {
	    delete[]((T*)address);
	}

    };

}

#ifdef GEO_COMPILER_CLANG
#pragma GCC diagnostic pop
#endif

#endif

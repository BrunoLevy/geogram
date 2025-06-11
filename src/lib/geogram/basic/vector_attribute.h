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


#ifndef GEOGRAM_BASIC_VECTOR_ATTRIBUTE
#define GEOGRAM_BASIC_VECTOR_ATTRIBUTE

#include <geogram/basic/common.h>
#include <geogram/basic/attributes.h>
#include <geogram/basic/geometry.h>

namespace GEO {

    /**
     * \brief An attribute class that can access a vector attribute
     *  as a scalar attribute with vec2,vec3 or vec4 element types
     */
    template <index_t DIM> class Attribute< vecng<DIM,double> > {
    public:
	typedef vecng<DIM,double> vec_type;
	static constexpr index_t vec_dim = DIM;

	Attribute() {
	    manager_ = nullptr;
	    store_ = nullptr;
	    stored_as_vec_ = false;
	}

	~Attribute() {
	    if(is_bound()) {
		unbind();
	    }
	}

	Attribute(AttributesManager& manager, const std::string& name) {
	    manager_ = nullptr;
	    store_ = nullptr;
	    bind(manager, name);
	}

	static bool is_defined(
	    AttributesManager& manager, const std::string& name
	) {
	    AttributeStore* store = manager.find_attribute_store(name);
	    if(store == nullptr) {
		return false;
	    }
	    if(store->elements_type_matches(typeid(double).name())) {
		return (store->dimension() == vec_dim);
	    }
	    if(store->elements_type_matches(typeid(vec_type).name())) {
		return (store->dimension() == 1);
	    }
	    return false;
	}

	void bind(AttributesManager& manager, const std::string& name) {
	    geo_assert(!is_bound());
	    manager_ = &manager;
	    store_ = manager.find_attribute_store(name);
	    if(store_ == nullptr) {
		store_ = new TypedAttributeStore<double>(vec_dim);
		manager_->bind_attribute_store(name,store_);
	    }
	    geo_assert(is_defined(manager, name));
	    stored_as_vec_ =
		store_->elements_type_matches(typeid(vec_type).name());
	    store_observer_.register_me(store_);
	}

	void unbind() {
	    geo_assert(is_bound());
            // If the AttributesManager was destroyed before, do not
            // do anything. This can occur in Lua scripting when using
            // Attribute wrapper objects.
            if(!store_observer_.disconnected()) {
                store_observer_.unregister_me(store_);
            }
            manager_ = nullptr;
            store_ = nullptr;
	    stored_as_vec_ = false;
	}

	void bind_if_is_defined(
	    AttributesManager& manager, const std::string& name
	) {
	    geo_assert(!is_bound());
	    if(is_defined(manager, name)) {
		bind(manager, name);
	    }
	}

	bool is_bound() const {
	    return (store_ != nullptr && !store_observer_.disconnected());
	}

	index_t size() const {
	    return store_observer_.size();
	}

	index_t dimension() const {
	    return 1;
	}

	AttributesManager* manager() {
	    return manager_;
	}

#ifdef GEO_COMPILER_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wcast-align"
#endif
	vec_type& operator[](index_t i) {
	    geo_debug_assert(i < size());
	    return ((vec_type*)store_observer_.base_addr())[i];
	}

	const vec_type& operator[](index_t i) const {
	    geo_debug_assert(i < size());
	    return ((vec_type*)store_observer_.base_addr())[i];
	}

#ifdef GEO_COMPILER_CLANG
#pragma clang diagnostic pop
#endif

    private:
	AttributeStoreObserver store_observer_;
	AttributesManager* manager_;
	AttributeStore* store_;
	bool stored_as_vec_;
    };

}

#endif

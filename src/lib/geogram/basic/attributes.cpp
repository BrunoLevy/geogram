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

#include <geogram/basic/attributes.h>
#include <geogram/basic/permutation.h>
#include <geogram/basic/string.h>
#include <geogram/basic/geometry.h>
#include <algorithm>

namespace GEO {

    void AttributeStoreObserver::register_me(AttributeStore* store) {
        store->register_observer(this);
    }

    void AttributeStoreObserver::unregister_me(AttributeStore* store) {
        store->unregister_observer(this);
    }

    /******************************************************************/

    AttributeStoreCreator::~AttributeStoreCreator() {
    }

    /******************************************************************/

    std::map<std::string, AttributeStoreCreator_var>
    AttributeStore::type_name_to_creator_;

    std::map<std::string, std::string>
    AttributeStore::typeid_name_to_type_name_;

    std::map<std::string, std::string>
    AttributeStore::type_name_to_typeid_name_;

    AttributeStore::AttributeStore(
        index_t elemsize,
        index_t dim
    ) :
        element_size_(elemsize),
        dimension_(dim),
        cached_base_addr_(nullptr),
        cached_size_(0),
        cached_capacity_(0),
        lock_(GEOGRAM_SPINLOCK_INIT)
    {
    }

    void AttributeStore::notify(
        Memory::pointer base_addr, index_t size, index_t dim
    ) {
        Process::acquire_spinlock(lock_);
        if(
            size != cached_size_ ||
            base_addr != cached_base_addr_ ||
            dim != dimension_
        ) {
            cached_base_addr_ = base_addr;
            cached_size_ = size;
            dimension_ = dim;
            for(auto cur : observers_) {
                cur->notify(cached_base_addr_, cached_size_, dim);
            }
        }
        Process::release_spinlock(lock_);
    }

    AttributeStore::~AttributeStore() {
        // Disconnect all the attributes, for the special case where
        // the AttributeStore is destroyed before the Attributes, can
        // occur for instance when using Lua scripting with Attribute wrapper
        // objects.
        for(auto cur : observers_) {
            cur->disconnect();
        }
    }

    void AttributeStore::register_observer(AttributeStoreObserver* observer) {
        Process::acquire_spinlock(lock_);
        geo_assert(observers_.find(observer) == observers_.end());
        observers_.insert(observer);
        observer->notify(cached_base_addr_, cached_size_, dimension_);
        Process::release_spinlock(lock_);
    }

    void AttributeStore::unregister_observer(AttributeStoreObserver* observer) {
        Process::acquire_spinlock(lock_);
        auto it = observers_.find(observer);
        geo_assert(it != observers_.end());
        observers_.erase(it);
        Process::release_spinlock(lock_);
    }

    void AttributeStore::apply_permutation(
        const vector<index_t>& permutation
    ) {
        geo_debug_assert(permutation.size() <= cached_size_);
        Permutation::apply(
            cached_base_addr_, permutation, element_size_ * dimension_
        );
    }

    void AttributeStore::compress(
        const vector<index_t>& old2new
    ) {
        geo_debug_assert(old2new.size() <= cached_size_);
        index_t item_size = element_size_ * dimension_;
        for(index_t i=0; i<old2new.size(); ++i) {
            index_t j = old2new[i];
            if(j == index_t(-1) || j == i) {
                continue;
            }
            geo_debug_assert(j <= i);
            Memory::copy(
                cached_base_addr_+j*item_size,
                cached_base_addr_+i*item_size,
                item_size
            );
        }
    }

    void AttributeStore::zero() {
        Memory::clear(
            cached_base_addr_, element_size_ * dimension_ * cached_size_
        );
    }

    void AttributeStore::swap_items(index_t i, index_t j) {
        geo_debug_assert(i < cached_size_);
        geo_debug_assert(j < cached_size_);
        index_t item_size = element_size_ * dimension_;
        void* temp = alloca(item_size);
        Memory::copy(
            temp,
            cached_base_addr_+i*item_size,
            item_size
        );
        Memory::copy(
            cached_base_addr_+i*item_size,
            cached_base_addr_+j*item_size,
            item_size
        );
        Memory::copy(
            cached_base_addr_+j*item_size,
            temp,
            item_size
        );
    }

    void AttributeStore::scale_item(index_t to, double s) {
        geo_argused(to);
        geo_argused(s);
    }

    void AttributeStore::madd_item(index_t to, double s, index_t from) {
        geo_argused(to);
        geo_argused(s);
        geo_argused(from);
    }

    /*************************************************************************/

    AttributesManager::AttributesManager() : size_(0), capacity_(0) {
    }

    AttributesManager::~AttributesManager() {
        clear(false,false);
    }

    void AttributesManager::resize(index_t new_size) {
        if(new_size == size_) {
            return;
        }
        for(auto& cur : attributes_) {
            cur.second->resize(new_size);
        }
        size_ = new_size;
    }

    void AttributesManager::reserve(index_t new_capacity) {
        if(new_capacity <= capacity_) {
            return;
        }
        for(auto& cur : attributes_) {
            cur.second->reserve(new_capacity);
        }
        capacity_ = new_capacity;
    }

    void AttributesManager::apply_permutation(
        const vector<index_t>& permutation
    ) {
        for(auto& cur : attributes_) {
            cur.second->apply_permutation(permutation);
        }
    }

    void AttributesManager::compress(
        const vector<index_t>& old2new
    ) {
        for(auto& cur : attributes_) {
            cur.second->compress(old2new);
        }
    }


    void AttributesManager::bind_attribute_store(
        const std::string& name, AttributeStore* as
    ) {
        geo_assert(find_attribute_store(name) == nullptr);
        attributes_[name] = as;
        as->reserve(capacity_);
        as->resize(size_);
    }

    void AttributesManager::list_attribute_names(
        vector<std::string>& names
    ) const {
        names.clear();
        for(auto& cur : attributes_) {
            names.push_back(cur.first);
        }
    }

    AttributeStore* AttributesManager::find_attribute_store(
        const std::string& name
    ) {
        auto it = attributes_.find(name);
        if(it == attributes_.end()) {
            return nullptr;
        }
        return it->second;
    }

    const AttributeStore* AttributesManager::find_attribute_store(
        const std::string& name
    ) const {
        auto it = attributes_.find(name);
        if(it == attributes_.end()) {
            return nullptr;
        }
        return it->second;
    }


    void AttributesManager::delete_attribute_store(const std::string& name) {
        auto it = attributes_.find(name);
        geo_assert(it != attributes_.end());
        geo_assert(!it->second->has_observers());
        delete it->second;
        attributes_.erase(it);
    }

    void AttributesManager::delete_attribute_store(AttributeStore* as) {
        for(auto it=attributes_.begin(); it != attributes_.end(); ++it) {
            if(it->second == as) {
                delete as;
                attributes_.erase(it);
                return;
            }
        }
        geo_assert_not_reached;
    }


    void AttributesManager::clear(bool keep_attributes, bool keep_memory) {
        if(keep_attributes) {
            for(auto& cur : attributes_) {
                cur.second->clear(keep_memory);
            }
        } else {
            for(auto& cur : attributes_) {
                delete cur.second;
            }
            attributes_.clear();
        }
        size_ = 0;
    }

    void AttributesManager::zero() {
        for(auto& cur : attributes_) {
            cur.second->zero();
        }
    }

    void AttributesManager::copy(const AttributesManager& rhs) {
        clear(false, false);
        reserve(rhs.capacity());
        resize(rhs.size());
        for(auto& cur : rhs.attributes_) {
            bind_attribute_store(cur.first, cur.second->clone());
        }
    }

    void AttributesManager::copy_item(index_t to, index_t from) {
        for(auto& cur : attributes_) {
            cur.second->copy_item(to,from);
        }
    }

    void AttributesManager::swap_items(index_t i, index_t j) {
        for(auto& cur : attributes_) {
            cur.second->swap_items(i,j);
        }
    }

    void AttributesManager::zero_item(index_t i) {
        for(auto& cur : attributes_) {
            cur.second->zero_item(i);
        }
    }

    void AttributesManager::scale_item(index_t i, double s) {
        for(auto& cur : attributes_) {
            cur.second->scale_item(i,s);
        }
    }

    void AttributesManager::madd_item(index_t i, double s, index_t j) {
        for(auto& cur : attributes_) {
            cur.second->madd_item(i,s,j);
        }
    }

    bool AttributesManager::copy_attribute(
        const std::string& name, const std::string& new_name
    ) {
        const auto old_itr = attributes_.find(name);
        if( old_itr == attributes_.end() ) {
            return false;
        }
        const AttributeStore* store = old_itr->second;

        const auto new_itr = attributes_.find(new_name);
        if( new_itr != attributes_.end() ) {
            AttributeStore* new_store = new_itr->second;
            if( !store->elements_type_matches(
                    new_store->element_typeid_name())
              ) {
                return false;
            }
            if(
                (store->size() != new_store->size()) &&
                (store->dimension() != new_store->dimension()) &&
                (store->element_size() != new_store->element_size())
            ) {
                return false;
            }
            memcpy(
                new_store->data(), store->data(),
                store->size() * store->dimension() * store->element_size()
            );
        } else {
            AttributeStore* new_store = store->clone();
            attributes_[new_name] = new_store;
        }

        return true;
    }

    bool AttributesManager::rename_attribute(
        const std::string& old_name, const std::string& new_name
    ) {
        const auto old_itr = attributes_.find(old_name);
        if( old_itr == attributes_.end() ) {
            return false;
        }
        const auto new_itr = attributes_.find(new_name);
        if( new_itr != attributes_.end() ) {
            return false;
        }
        attributes_[new_name] = old_itr->second;
        attributes_.erase(old_itr);
        return true;
    }


    /************************************************************************/

    index_t ScalarAttributeAdapterBase::nb_scalar_elements_per_item(
        const AttributeStore* store
    ) {
        ElementType et = element_type(store);
        if(et == ET_NONE) {
            return 0;
        }
        index_t result = store->dimension();
        if(et == ET_VEC2) {
            result *= 2;
        } else if(et == ET_VEC3) {
            result *= 3;
        }
        return result;
    }

    std::string ScalarAttributeAdapterBase::attribute_base_name(
        const std::string& name
    ) {
        size_t pos = name.find('[');
        if(pos == std::string::npos) {
            return name;
        }
        return name.substr(0,pos);
    }

    index_t ScalarAttributeAdapterBase::attribute_element_index(
        const std::string& name
    ) {
        index_t result = 0;
        size_t pos = name.find('[');
        if(pos != std::string::npos) {
            try {
                if(pos+2 > name.length()) {
                    result = index_t(-1);
                } else {
                    result = String::to_uint(
                        name.substr(pos+1, name.length()-pos-2)
                    );
                }
            } catch(...) {
                result = index_t(-1);
            }
        }
        return result;
    }

    ScalarAttributeAdapterBase::ElementType
    ScalarAttributeAdapterBase::element_type(const AttributeStore* store) {
        if(store->element_typeid_name() == typeid(Numeric::uint8).name()) {
            return ET_UINT8;
        }

        if(
            store->element_typeid_name() == typeid(char).name() ||
            store->element_typeid_name() == typeid(Numeric::int8).name()
        ) {
            return ET_INT8;
        }

        if(
            store->element_typeid_name() == typeid(Numeric::uint32).name() ||
            store->element_typeid_name() == typeid(index_t).name() ||
            store->element_typeid_name() == typeid(unsigned int).name()
        ) {
            return ET_UINT32;
        }

        if(
            store->element_typeid_name() == typeid(Numeric::int32).name() ||
            store->element_typeid_name() == typeid(int).name()
        ) {
            return ET_INT32;
        }

        if(
            store->element_typeid_name() == typeid(Numeric::float32).name() ||
            store->element_typeid_name() == typeid(float).name()
        ) {
            return ET_FLOAT32;
        }

        if(
            store->element_typeid_name() == typeid(Numeric::float64).name() ||
            store->element_typeid_name() == typeid(double).name()
        ) {
            return ET_FLOAT64;
        }

        if(store->element_typeid_name() == typeid(vec2).name()) {
            return ET_VEC2;
        }

        if(store->element_typeid_name() == typeid(vec3).name()) {
            return ET_VEC3;
        }

        return ET_NONE;
    }

    void ScalarAttributeAdapterBase::bind_if_is_defined(
        const AttributesManager& manager, const std::string& name
    ) {
        geo_assert(!is_bound());
        manager_ = &manager;
        element_index_ = attribute_element_index(name);
        store_ = manager_->find_attribute_store(attribute_base_name(name));

        if(store_ == nullptr || element_index_ == index_t(-1)) {
            store_ = nullptr;
            element_index_ = index_t(-1);
            return;
        }

        element_type_ = element_type(store_);

        if(element_type_ == ET_NONE) {
            store_ = nullptr;
            element_index_ = index_t(-1);
            return;
        }

        // Test element_index_ validity: should be smaller than
        // store's dimension (or 2*store dimension if a vec2,
        // or 3*store's dimension if a vec3)
        if(element_index_ >= nb_scalar_elements_per_item(store_)) {
            store_ = nullptr;
            element_index_ = index_t(-1);
            element_type_ = ET_NONE;
            return;
        }

        register_me(const_cast<AttributeStore*>(store_));
    }

    bool ScalarAttributeAdapterBase::is_defined(
        const AttributesManager& manager, const std::string& name
    ) {
        std::string attribute_name = attribute_base_name(name);
        const AttributeStore* store = manager.find_attribute_store(
            attribute_name
        );

        if(store == nullptr) {
            return false;
        }

        index_t element_index = attribute_element_index(name);
        if(element_index == index_t(-1)) {
            return false;
        }

        if(element_index >= nb_scalar_elements_per_item(store)) {
            return false;
        }

        if(element_type(store) == ET_NONE) {
            return false;
        }

        return true;
    }

    /************************************************************************/

}

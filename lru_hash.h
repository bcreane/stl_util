/**
 *  Author: Brendan Creane
 *
 *  Hash (unordered_map) with LRU eviction when hash size exceeds capacity.
 */

#ifndef LRU_HASH_LRU_HASH_H
#define LRU_HASH_LRU_HASH_H

#include <cstdlib>
#include <unordered_map>
#include <list>
#include "compiler.h"

/**
 * @file        lru_hash.h
 * @brief       Provide a key/value hash with least recently used eviction when hash size exceeds capacity.
 *
 * Example:     LruHash<int, std::string> fun(10);
 *              fun.insert(2, std::string("fun"));
 *
 *              std::string value;
 *              fun.lookup(2, &value);
 *              fun.erase(2);
 */

template <class Key_t, class Value_t>
class LruHash
{
public:
    /**
     * @name        LruHash
     * @param[in]   capacity Number of values the hash will hold before evicting least recently used key/values
     */
    LruHash(std::size_t capacity);

    /**
     * @name        get_capacity
     * @brief       retrieve the hash capacity
     * @retval      the number of values the hash will hold
     */
    std::size_t get_capacity() const {return capacity_;}

    /**
     * @name        set_capacity
     * @brief       set the hash capacity
     * @param[in]   capacity Number of values the hash will hold before evicting least recently used key/values
     *
     * Set_capacity will potentially evict enough key/value pairs to bring hash size below the new capacity.
     */
    void set_capacity(std::size_t capacity);

    /**
     * @name        lookup
     * @brief       Retrieve the value associated with given key. If the key isn't present in the hash,
     *              return false.
     * @retval      True if value_ptr holds the value associated with the key, false otherwise
     * @param[in]   key key to lookup
     * @param[out]  value_ptr pointer to Value_t. Value_ptr will point to a copy of the value associated
     *              with key.
     *
     * If lookup finds a key/value pair, that pair is moved to the "most recently used" position.
     */
    bool lookup(const Key_t& key, Value_t* value_ptr);

    /**
     * @name        insert
     * @brief       If key doesn't exist already, create hash entry for key/value. Otherwise, return false
     *              and leave
     *              original key/value pair.
     * @retval      Return true if a new key/value pair is inserted in hash, false if key already exists.
     * @param[in]   key key to associate with value
     * @param[in]   value value to associate with key
     */
    bool insert(const Key_t& key, const Value_t& value);

    /**
     * @name        erase
     * @brief       Erase key/value from hash.
     * @retval      Returns true if key exists, false otherwise.
     * @param[in]   key key to erase
     */
    bool erase(const Key_t& key);

    /**
     * @name        size
     * @brief       retrieve number of key/value pairs in hash
     * @retval      the number of key/value pairs in hash
     */
    std::size_t size() const {return hash_map_.size();}

private:
    std::size_t capacity_;

    typedef std::unordered_map<Key_t, std::pair<typename std::list<Key_t>::iterator,
                                                Value_t>> hash_t;

    hash_t hash_map_;
    std::list<Key_t> lru_dqueue_;

    /**
     * @name        move_to_front
     * @brief       mark key/value pair as most recently used
     * @param[in]   key key to mark as most recently used
     * @param[in]   it iterator pointing to key/value pair in hash
     */
    void move_to_front(const Key_t& key, const typename hash_t::iterator& it);
};

template <class Key_t, class Value_t>
bool LruHash<Key_t, Value_t>::erase(const Key_t& key)
{
    auto it = hash_map_.find(key);
    if (it == hash_map_.end()) {
        return false;
    }

    lru_dqueue_.erase(it->second.first);
    hash_map_.erase(it);

    return true;
}

template <class Key_t, class Value_t>
void LruHash<Key_t, Value_t>::move_to_front(const Key_t& key,
                                            const typename hash_t::iterator& it)
{
    UNREFERENCED_PARAMETER(key);
    // Move iterator to beginning of list
    lru_dqueue_.splice(lru_dqueue_.begin(), lru_dqueue_, it->second.first);
    it->second.first = lru_dqueue_.begin();
}

template <class Key_t, class Value_t>
LruHash<Key_t, Value_t>::LruHash(std::size_t capacity)
        : capacity_(capacity)
{
}

template <class Key_t, class Value_t>
void LruHash<Key_t, Value_t>::set_capacity(std::size_t capacity)
{
    capacity_ = capacity;
    while (hash_map_.size() > capacity_) {
        hash_map_.erase(lru_dqueue_.back());
        lru_dqueue_.pop_back();
    }
}

template <class Key_t, class Value_t>
bool LruHash<Key_t, Value_t>::insert(const Key_t& key, const Value_t& value)
{
    auto it = hash_map_.find(key);
    if (it != hash_map_.end()) {
        move_to_front(key, it);
        return false;
    } else {
        if (hash_map_.size() >= capacity_) {
            hash_map_.erase(lru_dqueue_.back());
            lru_dqueue_.pop_back();
        }

        lru_dqueue_.emplace_front(key);
        hash_map_[key] = {lru_dqueue_.begin(), value};
    }

    return true;
}

template <class Key_t, class Value_t>
bool LruHash<Key_t, Value_t>::lookup(const Key_t& key, Value_t* value_ptr)
{
    auto it = hash_map_.find(key);
    if (it == hash_map_.end()) {
        return false;
    }

    *value_ptr = it->second.second;
    move_to_front(key, it);

    return true;
}

#endif // LRU_HASH_LRU_HASH_H


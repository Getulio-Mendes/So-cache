#include "Cache.h"

bool Cache::find(uint64_t inst) {
    return data.find(inst) != data.end();
}


int Cache::access(uint64_t inst, int clock) {
    if (strategy == CacheStrategy::NoCache) {
        return -1;  
    }

    auto it = data.find(inst);
    if (it != data.end()) {  
        if (strategy == CacheStrategy::LRU) {
            lastAccess[inst] = clock;  // Update access time for LRU
        }
        return it->second;  
    } else {  
        return -1;  
    }
}

void Cache::write(uint64_t inst, int value, int clock) {
    if (strategy == CacheStrategy::NoCache) return;

    // Evict entry if cache is full
    if (data.size() >= CACHE_SIZE) {
        uint32_t evictKey = 0;
        int minAccess = std::numeric_limits<int>::max();

        // Find entry with oldest access timestamp, for FIFO it will be the oldest
        // for LRU will be the last to be acessed
        for (const auto& entry : data) {
            const uint32_t key = entry.first;
            const int accessTime = lastAccess[key];
            
            if (accessTime < minAccess) {
                minAccess = accessTime;
                evictKey = key;
            }
        }

        // save back to ram
        this->ram->WriteMem(evictKey,data[evictKey]);
        data.erase(evictKey);
        lastAccess.erase(evictKey);
    }

    data[inst] = value;  
    lastAccess[inst] = clock;  
}

#ifndef CACHE_H
#define CACHE_H

#include <unordered_map>
#include "../memory/MAINMEMORY.h"
#include <limits> // For std::numeric_limits

#define CACHE_SIZE 10

enum class CacheStrategy {
    NoCache = 0,
    LRU = 1,
    FIFO = 2
};

struct Cache {
    std::unordered_map<uint64_t, int> data;
    std::unordered_map<uint64_t, int> lastAccess;
    CacheStrategy strategy;
    MainMemory *ram;

    bool find(uint64_t inst);
    int access(uint64_t inst, int clock); 
    void write(uint64_t inst,int value, int clock);

    
    Cache(int strategy, MainMemory *ram)
        : strategy(static_cast<CacheStrategy>(strategy)), 
          ram(ram)
    {}
  
    Cache()
        : data(), lastAccess(),
        strategy(CacheStrategy::LRU),   
        ram(nullptr)
    {}  
};

#endif

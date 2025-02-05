#ifndef PCB_H
#define PCB_H

#include "./cpu/REGISTER_BANK.h"
#include "./memory/MAINMEMORY.h"
#include "./cpu/Cache.h"
#include <mutex>
#include <memory>
#include <type_traits>
#include <chrono>

 
enum class State {
    Ready,
    Blocked,
    Executing,
    Finished
};

enum class Schedulling{
    FCFS = 1,
    Priority = 2,
    Random = 3,
    SJF = 4
};

struct PCB{
  int quantum;
  int timestamp;
  chrono::steady_clock::time_point prevTime;
  chrono::milliseconds timestampMS;
  int size;
  REGISTER_BANK regBank;
  State state;
  int baseAddr;
  int finalAddr;
  int id;
  int priority;
};

struct ioRequest {
    string msg;
    PCB* process;
};


struct scheduleInfo {
    MainMemory* ram;
    vector<unique_ptr<PCB>>* processes;
    mutex* queueLock;
    vector<unique_ptr<ioRequest>>* ioRequests;
    bool shutdown;
    bool printLock;
    Schedulling schedulling;
    Cache cache;
};

#endif


#include "./cpu/CONTROL_UNIT.h"
#include "./memory/MAINMEMORY.h"
#include "./loader/loader.h"
#include "./assembler/assembler.h"
#include "./PCB.h"
#include "./cpu/Cache.h"

#include <unistd.h>
#include <cstdlib>
#include <chrono>
#include <pthread.h>
#include <filesystem>
#include <memory>
#include <mutex>
#include <vector>
#include <algorithm>
#include <random>

using namespace std;

#define NUM_CORES 2
#define QUANTUM 20 
// 1 instrução = 1 quantum

int MMU::getRealAddr(PCB& p,int addr){

  if(addr < p.finalAddr && addr > p.baseAddr){
    return p.baseAddr + addr;
  }
  else{
    return -1;
  }
  
}

string getFileName(char* file) {
    string filePath(file);
    filesystem::path path(filePath);
    string fileName = path.filename().string();
    size_t dotPos = fileName.find_last_of('.');
    if (dotPos != string::npos) {
        fileName = fileName.substr(0, dotPos);
    }
    return fileName;
}

void* coreManage(void* arg) {
    scheduleInfo* info = static_cast<scheduleInfo*>(arg);

    while (!info->shutdown) {
        PCB* currentProcess = nullptr;

        {
            lock_guard<mutex> lock(*info->queueLock);
             // Find a ready process
            for (auto& pcb : *info->processes) {
                if (pcb->state == State::Ready && pcb->state != State::Blocked) {
                    currentProcess = pcb.get();
                    currentProcess->state = State::Executing;
                    break;
                }
            }        
        }


        if(currentProcess){
            
            currentProcess->prevTime = chrono::steady_clock::now();
            
            Core(*info->ram,info->mmu, *currentProcess, info->ioRequests,info->printLock, info->cache);
            currentProcess->timestampMS += chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - currentProcess->prevTime);
            
            lock_guard<mutex> lock(*info->queueLock);
            if (currentProcess->state == State::Executing) {
                currentProcess->state = State::Ready;
            }
        }
    }

    return nullptr;
}

void* scheduler(void* arg) {
    scheduleInfo* info = static_cast<scheduleInfo*>(arg);
    random_device rd;
    mt19937 g(rd());

    while (!info->shutdown) {
        lock_guard<mutex> lock(*info->queueLock);

        bool allDone = true;
        
        for (const auto& pcb : *info->processes) {
            if (pcb->state != State::Finished) {
                allDone = false;
                break;
            }
        }

        if(!info->ioRequests->empty()){
            allDone = false;
        }
                
        if (allDone) {
            //cout << "Shutdown" << endl;
            info->shutdown = true;
        }


        // Re organize the list based on the schedulling type

        if(info->schedulling == Schedulling::Priority){
            sort(info->processes->begin(), info->processes->end(), [](const auto& a, const auto& b) {
                return a->priority < b->priority;
            });
        }
        else if(info->schedulling == Schedulling::FCFS){
            
            sort(info->processes->begin(), info->processes->end(), [](const auto& a, const auto& b) {
                // Put "Executing" state at the end
                if (a->state == State::Executing && b->state != State::Executing) {
                    return false; // a is executing, so it should be placed after b
                }
                if (a->state != State::Executing && b->state == State::Executing) {
                    return true; // b is executing, so it should be placed after a
                }
                return false;
            });
        }
        else if(info->schedulling == Schedulling::Random){
            shuffle(info->processes->begin(),info->processes->end() ,g);
        }
        else if(info->schedulling == Schedulling::SJF){
            sort(info->processes->begin(), info->processes->end(), [](const auto& a, const auto& b) {
                return a->size < b->size;
            });
        }
   
    }

    return nullptr;
}


void * resourceManager(void * arg){
    scheduleInfo* info = static_cast<scheduleInfo*>(arg);
    
   
    while (!info->shutdown) {

        if(!info->ioRequests->empty()){
            
            auto req = move(info->ioRequests->front());
            info->printLock = true;

            info->ioRequests->erase(info->ioRequests->begin());

            // Simulate a 0.05 ms delay
            usleep(50000);

            cout << "Program " << req->process->id << ": " << req->msg << std::endl;          
             
            req->process->state = State::Ready;
            info->printLock = false;
        }
    }

    return nullptr;
}

int main(int argc, char* argv[]) {
    if (argc < 4) {
        cerr << "Usage: " << argv[0] << " <input_files> <schedule type>" << endl;
        cerr << "<Schedule type>:\nFCFS = 1\nPriority = 2\nRandom = 3\nShortest Job first = 4" << endl;
        cerr << "<Cache strategy>:\nNo cache=0 \nLRU = 1\nFIFO = 2" << endl;
        return 1;
    }

    // Trim filepaths to get file names
    vector<string> files;
    for (int i = 0; i < argc; i++) {
        files.push_back(getFileName(argv[i]));
    }

    // Compile prograns
    assembleFiles(argc-2, files, argv);

    MainMemory ram = MainMemory(2048, 2048);
    int initProgram[argc-2];
    int estimatives[argc-2];
    for(int i =0; i < argc-2;i++){
        estimatives[i] = 0;
    }
    initProgram[0] = 0;
    
    for (int i = 1; i < argc-2; i++) {
        initProgram[i] = loadProgram(files[i], ram, initProgram[i - 1], estimatives[i]);
    }
    
    auto scheduleInfo = make_unique<struct scheduleInfo>();
    scheduleInfo->ram = &ram;
    MMU mmu;
    scheduleInfo->mmu = mmu;
    scheduleInfo->processes = new vector<unique_ptr<PCB>>();
    scheduleInfo->ioRequests = new vector<unique_ptr<ioRequest>>();
    scheduleInfo->queueLock = new mutex();
    scheduleInfo->shutdown = false;
    scheduleInfo->printLock = false;
    scheduleInfo->schedulling = Schedulling(stoi(argv[argc-2]));
    scheduleInfo->cache = Cache(stoi(argv[argc-1]),&ram);

    for (int i = 1; i < argc-2; i++) {
        auto pcb = make_unique<PCB>();
        pcb->baseAddr = initProgram[i - 1];
        pcb->finalAddr = initProgram[i];
        pcb->size = estimatives[i];
        pcb->id = i;
        pcb->priority = i;
        pcb->quantum = QUANTUM + (pcb->priority - argc-2 - i)*2;
        pcb->state = State::Ready;
        pcb->regBank.pc.value = initProgram[i - 1];
        scheduleInfo->processes->push_back(move(pcb));
    }

    
    pthread_t threads[NUM_CORES + 2];
    
    if (pthread_create(&threads[NUM_CORES], nullptr, scheduler, scheduleInfo.get()) != 0) {
        cerr << "Error creating scheduler thread" << endl;
        return 1;
    }
    if (pthread_create(&threads[NUM_CORES +1], nullptr, resourceManager, scheduleInfo.get()) != 0) {
        cerr << "Error creating I/O manage thread" << endl;
        return 1;
    }

    for (int i = 0; i < NUM_CORES; ++i) {
        if (pthread_create(&threads[i], nullptr, coreManage, scheduleInfo.get()) != 0) {
            cerr << "Error creating core thread " << i << endl;
            return 1;
        }
    }

    // Join threads
    for (int i = 0; i < NUM_CORES + 2; i++) {
        pthread_join(threads[i], nullptr);
    }

    for(int i=0; i < scheduleInfo->processes->size(); i++){
        auto& process = scheduleInfo->processes->at(i);
        cout << "Process " << process->id << ": Timestamp (Clock): " << process->timestamp <<"  Timestamp (MS): " <<  process->timestampMS.count() << endl;
    }

    return 0;
}

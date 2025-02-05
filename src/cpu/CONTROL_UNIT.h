#ifndef CONTROL_UNIT_H
#define CONTROL_UNIT_H

#include "ALU.h"
#include "Cache.h"
#include "REGISTER_BANK.h"
#include"HashRegister.h"
#include"unordered_map"
#include"../memory/MAINMEMORY.h"
#include"../PCB.h"
#include <string>
#include <vector>
#include <cstdint>
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <mutex>

void Core(MainMemory &ram, PCB &process, vector<unique_ptr<ioRequest>>* ioRequests, bool &printLock, Cache &cache);

struct Instruction_Data{
    string source_register;
    string target_register;
    string destination_register;
    string op;
    string addressRAMResult;
    int rawInstrucion;

};


struct ControlContext {
    REGISTER_BANK &registers;
    MainMemory &ram;
    vector<unique_ptr<ioRequest>> &ioRequests;
    bool &printLock;
    PCB &process;
    int &counter;
    int &counterForEnd;
    bool &endProgram;
    bool &endExecution;
    Cache& cache;
    int &clock;
};

struct Control_Unit{

    operation op;

    vector<Instruction_Data> data;

    Map map;
   
    unordered_map<string, string> instructionMap = {
        {"add", "000000"},
        {"and", "000001"},
        {"div", "000010"},
        {"mult","000011"},
        {"sub", "000100"},
        {"beq", "000101"},
        {"bne", "000110"},
        {"bgt", "000111"},
        {"bgti","001000"},
        {"blt", "001001"},
        {"blti","001010"},
        {"j", "001011"},
        {"lw", "001100"},
        {"sw", "001101"},
        {"li", "001110"},
        {"la", "001111"},
        {"print", "010000"},
        {"end", "111111"}
    };

    string Get_immediate(const uint32_t instruction);
    string Pick_Code_Register_Load(const uint32_t instruction);
    string Get_destination_Register(const uint32_t instruction);
    string Get_target_Register(const uint32_t instruction);
    string Get_source_Register(const uint32_t instruction);
    uint64_t encodeALUInst(int op,int x,int y);

    string Identificacao_instrucao(uint32_t instruction, REGISTER_BANK &registers);
    void Fetch(ControlContext &context);
    void Decode(REGISTER_BANK &registers, Instruction_Data &data, ControlContext &context,bool& endProgram);
    void Execute_Aritmetic_Operation(REGISTER_BANK &registers,Instruction_Data &data, Cache &cache,int clock);
    void Execute_Operation(Instruction_Data &data,ControlContext &context);
    void Execute_Loop_Operation(REGISTER_BANK &registers,Instruction_Data &data, int &counter, int &counterForEnd, bool& endProgram, MainMemory& ram); 
    void Execute(Instruction_Data &data, ControlContext &context);
    void Memory_Acess(Instruction_Data &data, ControlContext &context);
    void Write_Back(Instruction_Data &data, ControlContext &context);
};

#endif

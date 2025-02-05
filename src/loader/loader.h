#ifndef LOADER_H
#define LOADER_H

#include"../memory/MAINMEMORY.h"
#include <string>

int loadProgram(std::string inputFile, MainMemory & ram, int initialAddress, int & estimative);

#endif

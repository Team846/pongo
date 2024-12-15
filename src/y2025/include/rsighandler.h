#pragma once

#ifndef _WIN32
#include <dirent.h>
#include <execinfo.h>
#include <signal.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <iostream>
#include <locale>
#include <map>

void handler(int sig) {
  std::map<int, std::string> sigErrors;

  sigErrors[SIGFPE] = "FATAL ERROR >> Arithmetic Error, SIGFPE";
  sigErrors[SIGILL] = "FATAL ERROR >> Illegal Instruction, SIGILL";
  sigErrors[SIGSEGV] = "FATAL ERROR >> Illegal Memory Access, SIGSEGV";
  sigErrors[SIGBUS] = "FATAL ERROR >> Bus Error, SIGBUS";
  sigErrors[SIGABRT] = "FATAL ERROR >> Abort, SIGABRT";
  sigErrors[SIGSYS] = "FATAL ERROR >> Invalid system call, SIGSYS";
  sigErrors[SIGTRAP] = "FATAL ERROR >> Exception Occured, SIGTRAP";

  void* callstack[24];

  int frames = backtrace(callstack, 24);

  std::cerr << "Backtrace:\n";
  char** symbols = backtrace_symbols(callstack, frames);

  if (symbols == nullptr) {
    std::cerr << "Error obtaining backtrace symbols\n" << std::endl;
    return;
  }

  for (int i = 0; i < frames; ++i) {
    std::cerr << symbols[i] << std::endl;
  }

  if (sigErrors.contains(sig)) {
    std::cerr << sigErrors[sig] << std::endl;
  } else {
    std::cerr << "? Unknown Exception Occured" << std::endl;
  }
  // exit(1);
}
#endif

#ifndef _WIN32
void configureSignalHandlers() {
  for (int i = 1; i < NSIG; ++i) {
    signal(i, handler);
  }
}
#else
void configureSignalHandlers() {}
#endif
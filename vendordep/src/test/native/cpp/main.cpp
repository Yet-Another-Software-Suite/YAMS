// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include <cxxabi.h>
#include <execinfo.h>
#include <hal/HAL.h>

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "gtest/gtest.h"
#include "helpers/ExceptionTracer.h"
#include "yams/motorcontrollers/SmartMotorControllerCommandRegistry.hpp"

static void SigsegvHandler(int /*sig*/) {
  void* frames[64];
  int count = backtrace(frames, 64);
  char** symbols = backtrace_symbols(frames, count);

  std::fprintf(stderr, "\n[YAMS] Caught SIGSEGV after tests — stack trace:\n");

  for (int i = 0; i < count; ++i) {
    // backtrace_symbols format: "module(mangled+offset) [address]"
    // Extract the mangled name between '(' and '+'/').
    char* sym = symbols ? symbols[i] : nullptr;
    char* begin = sym ? std::strchr(sym, '(') : nullptr;
    char* end = begin ? std::strpbrk(begin + 1, "+)") : nullptr;

    if (begin && end && end > begin + 1) {
      *end = '\0';
      int status = 0;
      char* demangled = abi::__cxa_demangle(begin + 1, nullptr, nullptr, &status);
      *end = '+';  // restore
      if (status == 0 && demangled) {
        // Replace the mangled portion with the demangled name for display.
        std::fprintf(stderr, "  #%-2d %.*s%s%s\n", i, static_cast<int>(begin + 1 - sym), sym,
                     demangled, end);
        std::free(demangled);
        continue;
      }
    }
    // Fallback: print the raw symbol string.
    std::fprintf(stderr, "  #%-2d %s\n", i, sym ? sym : "(unknown)");
  }

  std::free(symbols);
  std::fflush(stderr);

  // Re-raise with default handler so the exit code stays 139.
  std::signal(SIGSEGV, SIG_DFL);
  std::raise(SIGSEGV);
}

int main(int argc, char** argv) {
  std::signal(SIGSEGV, SigsegvHandler);
  ::testing::InitGoogleTest(&argc, argv);
  // Print demangled stack traces whenever a test fails due to an exception.
  ::testing::UnitTest::GetInstance()->listeners().Append(new yams::test::ExceptionTracerListener{});
  int result = RUN_ALL_TESTS();
  // Destroy CommandPtrs while SendableRegistry is still alive; avoids a
  // SIGSEGV from static-destructor ordering (s_commands outlives the registry mutex).
  yams::motorcontrollers::SmartMotorControllerCommandRegistry::Clear();
  HAL_Shutdown();
  return result;
}

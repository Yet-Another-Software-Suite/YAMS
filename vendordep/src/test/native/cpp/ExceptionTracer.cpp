// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "helpers/ExceptionTracer.h"

#include <cxxabi.h>
#include <dlfcn.h>
#include <execinfo.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <utility>

namespace {

std::mutex s_mutex;
std::string s_lastTrace;

// Re-entrancy guard: if the trace-capture code itself throws (e.g. bad_alloc
// inside std::string), we skip the inner capture to avoid infinite recursion.
thread_local bool s_capturing = false;

// Demangle one symbol line from backtrace_symbols.
// Input format:  "path(mangled_name+offset) [address]"
// Output format: "path(demangled_name+offset) [address]"
std::string Demangle(const char* sym) {
  const char* lparen = std::strchr(sym, '(');
  const char* plus = lparen ? std::strchr(lparen, '+') : nullptr;
  if (!lparen || !plus) return sym;

  std::string mangled(lparen + 1, static_cast<std::size_t>(plus - lparen - 1));
  int status = 0;
  char* dm = abi::__cxa_demangle(mangled.c_str(), nullptr, nullptr, &status);
  if (status != 0 || !dm) return sym;

  std::string out;
  out.reserve(std::strlen(sym) + 64);
  out.append(sym, static_cast<std::size_t>(lparen - sym));
  out += '(';
  out += dm;
  out += plus;  // keeps "+offset) [address]"
  std::free(dm);
  return out;
}

}  // namespace

namespace yams::test {

std::string GetLastExceptionStackTrace() {
  std::lock_guard<std::mutex> lk{s_mutex};
  return s_lastTrace;
}

void ClearLastExceptionStackTrace() {
  std::lock_guard<std::mutex> lk{s_mutex};
  s_lastTrace.clear();
}

}  // namespace yams::test

// ---------------------------------------------------------------------------
// __cxa_throw interposition
//
// Every C++ throw statement calls __cxa_throw *before* stack unwinding, so
// the complete call stack is still intact here.  We capture it, store it
// globally, then forward to the real implementation in libstdc++.
// ---------------------------------------------------------------------------
extern "C" void __cxa_throw(void* obj, void* tinfo, void (*dest)(void*)) {
  if (!s_capturing) {
    s_capturing = true;

    void* frames[64];
    int n = ::backtrace(frames, 64);
    // frames[0] is this function; start the user-visible trace from frames[1].
    char** syms = (n > 1) ? ::backtrace_symbols(frames + 1, n - 1) : nullptr;
    if (syms) {
      std::string trace;
      trace.reserve(2048);
      char idx[16];
      for (int i = 0; i < n - 1; ++i) {
        std::snprintf(idx, sizeof(idx), "  [%2d] ", i);
        trace += idx;
        trace += Demangle(syms[i]);
        trace += '\n';
      }
      std::free(syms);

      std::lock_guard<std::mutex> lk{s_mutex};
      s_lastTrace = std::move(trace);
    }

    s_capturing = false;
  }

  using Fn = void (*)(void*, void*, void (*)(void*));
  static Fn real = reinterpret_cast<Fn>(dlsym(RTLD_NEXT, "__cxa_throw"));
  real(obj, tinfo, dest);
  // Never reached — the real __cxa_throw unwinds the stack.
  // std::terminate() gives the compiler a visible noreturn path so it does not
  // emit a "control reaches end of function" diagnostic.
  std::terminate();
}

// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <gtest/gtest.h>

#include <cstdio>
#include <string>

namespace yams::test {

// Returns the stack trace captured at the most recent C++ throw site, or an
// empty string if no throw has occurred since the last ClearLast... call.
std::string GetLastExceptionStackTrace();

// Clears the stored stack trace (called at the start of each test).
void ClearLastExceptionStackTrace();

// Appends to the gtest listener chain.  On any fatal failure that looks like
// an uncaught exception it prints the stack trace that was captured at the
// throw site, right after the default failure banner.
class ExceptionTracerListener : public ::testing::EmptyTestEventListener {
 public:
  void OnTestStart(const ::testing::TestInfo& /*info*/) override { ClearLastExceptionStackTrace(); }

  void OnTestPartResult(const ::testing::TestPartResult& result) override {
    if (result.type() != ::testing::TestPartResult::kFatalFailure) return;
    std::string trace = GetLastExceptionStackTrace();
    if (trace.empty()) return;
    std::printf("\nStack trace at throw site:\n%s\n", trace.c_str());
    std::fflush(stdout);
  }
};

}  // namespace yams::test

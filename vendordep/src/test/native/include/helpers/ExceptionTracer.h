// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <catch2/reporters/catch_reporter_event_listener.hpp>
#include <catch2/reporters/catch_reporter_registrars.hpp>

#include <cstdio>
#include <string>

namespace yams::test {

// Returns the stack trace captured at the most recent C++ throw site, or an
// empty string if no throw has occurred since the last ClearLast... call.
std::string GetLastExceptionStackTrace();

// Clears the stored stack trace (called at the start of each test case).
void ClearLastExceptionStackTrace();

// Registered with Catch2 via CATCH_REGISTER_LISTENER below. On any failed
// assertion it prints the stack trace that was captured at the throw site
// (if one was captured), right after Catch2's own failure banner.
class ExceptionTracerListener : public Catch::EventListenerBase {
 public:
  using Catch::EventListenerBase::EventListenerBase;

  void testCaseStarting(Catch::TestCaseInfo const& /*info*/) override {
    ClearLastExceptionStackTrace();
  }

  void assertionEnded(Catch::AssertionStats const& stats) override {
    if (stats.assertionResult.succeeded()) return;
    std::string trace = GetLastExceptionStackTrace();
    if (trace.empty()) return;
    std::printf("\nStack trace at throw site:\n%s\n", trace.c_str());
    std::fflush(stdout);
  }
};

}  // namespace yams::test

CATCH_REGISTER_LISTENER(yams::test::ExceptionTracerListener)

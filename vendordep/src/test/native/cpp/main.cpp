// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include <hal/HAL.h>

#include "gtest/gtest.h"
#include "helpers/ExceptionTracer.h"

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  // Print demangled stack traces whenever a test fails due to an exception.
  ::testing::UnitTest::GetInstance()->listeners().Append(new yams::test::ExceptionTracerListener{});
  int result = RUN_ALL_TESTS();
  HAL_Shutdown();
  return result;
}

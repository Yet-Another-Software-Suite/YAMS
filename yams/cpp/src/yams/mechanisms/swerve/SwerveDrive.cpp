// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// SwerveDrive is a template class; all implementation lives in SwerveDrive.hpp.
// This file provides explicit instantiations for the most common module counts to
// allow link-time de-duplication and reduce per-TU compile overhead.

#include "yams/mechanisms/swerve/SwerveDrive.hpp"

namespace yams::mechanisms::swerve {

template class SwerveDrive<3>;
template class SwerveDrive<4>;
template class SwerveDrive<6>;

}  // namespace yams::mechanisms::swerve

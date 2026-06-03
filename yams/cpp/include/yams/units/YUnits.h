// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/time.h>
#include <units/velocity.h>

/**
 * @namespace yams::units
 * Additional unit type aliases supplementing the WPILib units library.
 *
 * Provides imperial distance units (hand, yard, cubit, fathom, furlong, mile, league),
 * extended time units (hour, day, week, fortnight, year), and derived angular rate types.
 */
namespace yams::units {

// === Distance Units (imperial) ===
using HandUnit = ::units::unit<std::ratio<10160, 10000>, ::units::meters>;
using YardUnit = ::units::unit<std::ratio<9144, 10000>, ::units::meters>;
using CubitUnit = ::units::unit<std::ratio<4572, 10000>, ::units::meters>;
using FathomUnit = ::units::unit<std::ratio<18288, 10000>, ::units::meters>;
using FurlongUnit = ::units::unit<std::ratio<201168, 1000>, ::units::meters>;
using MileUnit = ::units::unit<std::ratio<1609344, 1000>, ::units::meters>;
using LeagueUnit = ::units::unit<std::ratio<4828032, 1000>, ::units::meters>;

using hand_t = ::units::unit_t<HandUnit>;
using yard_t = ::units::unit_t<YardUnit>;
using cubit_t = ::units::unit_t<CubitUnit>;
using fathom_t = ::units::unit_t<FathomUnit>;
using furlong_t = ::units::unit_t<FurlongUnit>;
using mile_t = ::units::unit_t<MileUnit>;
using league_t = ::units::unit_t<LeagueUnit>;

// === Time Units ===
using HourUnit = ::units::unit<std::ratio<3600>, ::units::seconds>;
using DayUnit = ::units::unit<std::ratio<86400>, ::units::seconds>;
using WeekUnit = ::units::unit<std::ratio<604800>, ::units::seconds>;
using FortnightUnit = ::units::unit<std::ratio<1209600>, ::units::seconds>;
using YearUnit = ::units::unit<std::ratio<31536000>, ::units::seconds>;

using hour_t = ::units::unit_t<HourUnit>;
using day_t = ::units::unit_t<DayUnit>;
using week_t = ::units::unit_t<WeekUnit>;
using fortnight_t = ::units::unit_t<FortnightUnit>;
using year_t = ::units::unit_t<YearUnit>;

// === Angular Velocity ===
using rotations_per_year_t =
    ::units::unit_t<::units::compound_unit<::units::turns, ::units::inverse<YearUnit>>>;

// === Angular Acceleration ===
using rpm_per_second_t =
    ::units::unit_t<::units::compound_unit<::units::angular_velocity::revolutions_per_minute,
                                           ::units::inverse<::units::seconds>>>;

}  // namespace yams::units

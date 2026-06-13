// Copyright (c) 2026 Yet Another Software Suite
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
using HandUnit = ::units::unit<std::ratio<10160, 10000>, ::units::meters>;  ///< 1 hand = 0.1016 m.
using YardUnit = ::units::unit<std::ratio<9144, 10000>, ::units::meters>;   ///< 1 yard = 0.9144 m.
using CubitUnit = ::units::unit<std::ratio<4572, 10000>, ::units::meters>;  ///< 1 cubit = 0.4572 m.
using FathomUnit =
    ::units::unit<std::ratio<18288, 10000>, ::units::meters>;  ///< 1 fathom = 1.8288 m.
using FurlongUnit =
    ::units::unit<std::ratio<201168, 1000>, ::units::meters>;  ///< 1 furlong = 201.168 m.
using MileUnit =
    ::units::unit<std::ratio<1609344, 1000>, ::units::meters>;  ///< 1 mile = 1609.344 m.
using LeagueUnit =
    ::units::unit<std::ratio<4828032, 1000>, ::units::meters>;  ///< 1 league = 4828.032 m.

using hand_t = ::units::unit_t<HandUnit>;        ///< Quantity in hands.
using yard_t = ::units::unit_t<YardUnit>;        ///< Quantity in yards.
using cubit_t = ::units::unit_t<CubitUnit>;      ///< Quantity in cubits.
using fathom_t = ::units::unit_t<FathomUnit>;    ///< Quantity in fathoms.
using furlong_t = ::units::unit_t<FurlongUnit>;  ///< Quantity in furlongs.
using mile_t = ::units::unit_t<MileUnit>;        ///< Quantity in miles.
using league_t = ::units::unit_t<LeagueUnit>;    ///< Quantity in leagues.

// === Time Units ===
using HourUnit = ::units::unit<std::ratio<3600>, ::units::seconds>;    ///< 1 hour = 3600 s.
using DayUnit = ::units::unit<std::ratio<86400>, ::units::seconds>;    ///< 1 day = 86400 s.
using WeekUnit = ::units::unit<std::ratio<604800>, ::units::seconds>;  ///< 1 week = 604800 s.
using FortnightUnit =
    ::units::unit<std::ratio<1209600>, ::units::seconds>;  ///< 1 fortnight = 1209600 s.
using YearUnit =
    ::units::unit<std::ratio<31536000>, ::units::seconds>;  ///< 1 year = 31536000 s (365 days).

using hour_t = ::units::unit_t<HourUnit>;            ///< Quantity in hours.
using day_t = ::units::unit_t<DayUnit>;              ///< Quantity in days.
using week_t = ::units::unit_t<WeekUnit>;            ///< Quantity in weeks.
using fortnight_t = ::units::unit_t<FortnightUnit>;  ///< Quantity in fortnights.
using year_t = ::units::unit_t<YearUnit>;            ///< Quantity in years.

// === Angular Velocity ===
/** Rotations per year — useful for very slow mechanisms. */
using rotations_per_year_t =
    ::units::unit_t<::units::compound_unit<::units::turns, ::units::inverse<YearUnit>>>;

// === Angular Acceleration ===
/** RPM per second — angular acceleration expressed as RPM/s. */
using rpm_per_second_t =
    ::units::unit_t<::units::compound_unit<::units::angular_velocity::revolutions_per_minute,
                                           ::units::inverse<::units::seconds>>>;

}  // namespace yams::units

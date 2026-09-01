// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/units/angle.hpp>
#include <wpi/units/angular_acceleration.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>
#include <wpi/units/moment_of_inertia.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/velocity.hpp>

/**
 * @namespace yams::units
 * Additional unit type aliases supplementing the WPILib units library.
 *
 * Provides imperial distance units (hand, yard, cubit, fathom, furlong, mile, league),
 * extended time units (hour, day, week, fortnight, year), and derived angular rate types.
 */
namespace yams::units {

// === Distance Units (imperial) ===
using HandUnit = ::wpi::units::unit<std::ratio<10160, 10000>, ::wpi::units::meters>;  ///< 1 hand = 0.1016 m.
using YardUnit = ::wpi::units::unit<std::ratio<9144, 10000>, ::wpi::units::meters>;   ///< 1 yard = 0.9144 m.
using CubitUnit = ::wpi::units::unit<std::ratio<4572, 10000>, ::wpi::units::meters>;  ///< 1 cubit = 0.4572 m.
using FathomUnit =
    ::wpi::units::unit<std::ratio<18288, 10000>, ::wpi::units::meters>;  ///< 1 fathom = 1.8288 m.
using FurlongUnit =
    ::wpi::units::unit<std::ratio<201168, 1000>, ::wpi::units::meters>;  ///< 1 furlong = 201.168 m.
using MileUnit =
    ::wpi::units::unit<std::ratio<1609344, 1000>, ::wpi::units::meters>;  ///< 1 mile = 1609.344 m.
using LeagueUnit =
    ::wpi::units::unit<std::ratio<4828032, 1000>, ::wpi::units::meters>;  ///< 1 league = 4828.032 m.

using hand_t = ::wpi::units::unit_t<HandUnit>;        ///< Quantity in hands.
using yard_t = ::wpi::units::unit_t<YardUnit>;        ///< Quantity in yards.
using cubit_t = ::wpi::units::unit_t<CubitUnit>;      ///< Quantity in cubits.
using fathom_t = ::wpi::units::unit_t<FathomUnit>;    ///< Quantity in fathoms.
using furlong_t = ::wpi::units::unit_t<FurlongUnit>;  ///< Quantity in furlongs.
using mile_t = ::wpi::units::unit_t<MileUnit>;        ///< Quantity in miles.
using league_t = ::wpi::units::unit_t<LeagueUnit>;    ///< Quantity in leagues.

// === Time Units ===
using HourUnit = ::wpi::units::unit<std::ratio<3600>, ::wpi::units::seconds>;    ///< 1 hour = 3600 s.
using DayUnit = ::wpi::units::unit<std::ratio<86400>, ::wpi::units::seconds>;    ///< 1 day = 86400 s.
using WeekUnit = ::wpi::units::unit<std::ratio<604800>, ::wpi::units::seconds>;  ///< 1 week = 604800 s.
using FortnightUnit =
    ::wpi::units::unit<std::ratio<1209600>, ::wpi::units::seconds>;  ///< 1 fortnight = 1209600 s.
using YearUnit =
    ::wpi::units::unit<std::ratio<31536000>, ::wpi::units::seconds>;  ///< 1 year = 31536000 s (365 days).

using hour_t = ::wpi::units::unit_t<HourUnit>;            ///< Quantity in hours.
using day_t = ::wpi::units::unit_t<DayUnit>;              ///< Quantity in days.
using week_t = ::wpi::units::unit_t<WeekUnit>;            ///< Quantity in weeks.
using fortnight_t = ::wpi::units::unit_t<FortnightUnit>;  ///< Quantity in fortnights.
using year_t = ::wpi::units::unit_t<YearUnit>;            ///< Quantity in years.

// === Angular Velocity ===
/** Rotations per year — useful for very slow mechanisms. */
using rotations_per_year_t =
    ::wpi::units::unit_t<::wpi::units::compound_unit<::wpi::units::turns, wpi::units::inverse<YearUnit>>>;

// === Angular Acceleration ===
/** RPM per second — angular acceleration expressed as RPM/s. */
using rpm_per_second_t =
    ::wpi::units::unit_t<::wpi::units::compound_unit<::wpi::units::angular_velocity::revolutions_per_minute,
                                           wpi::units::inverse<::wpi::units::seconds>>>;

}  // namespace yams::units

// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/Rivet.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <iterator>
#include <string>
#include <string_view>
#include <wpi/driverstation/DriverStationDisplay.hpp>

// The build defines YAMS_VERSION, unquoted, as the release version. Builds from source get "dev".
#define YAMS_STRINGIFY_IMPL(x) #x
#define YAMS_STRINGIFY(x) YAMS_STRINGIFY_IMPL(x)
#ifdef YAMS_VERSION
#define YAMS_VERSION_STRING YAMS_STRINGIFY(YAMS_VERSION)
#else
#define YAMS_VERSION_STRING "dev"
#endif

namespace yams {

namespace {

/**
 * Rivet, traced from the official pixel art with two characters per pixel. Each glyph stands for
 * one color: # outline and pupils, @ eye whites, % body, 8 feet shading, + belly, O and o
 * monocle, | monocle chain, ~ and = mustache, v mouth.
 */
constexpr std::string_view kArt[] = {
    "            ##########                  ########",
    "          ##%%%%%%%%####              ##%%%%%%%%####",
    "        ##%%%%%%%%%%%%################%%OOOOOOOO%%%%##",
    "        ##%%@@@@@@@@%%%%%%%%%%%%%%%%%%OOooooooooOO%%##",
    "      ##%%@@@@####@@@@%%%%%%%%%%%%%%OOoo@@@@@@@@ooOO%%##",
    "      ##%%@@########@@%%%%%%%%%%%%OOoo@@@@####@@@@ooOO##",
    "      ##%%@@########@@%%%%%%%%%%%%OOoo@@########@@ooOO##",
    "    ##%%%%@@@@####@@@@%%%%%%%%%%%%OOoo@@########@@ooOO##",
    "  ##%%%%%%%%@@@@@@@@%%%%%%%%%%%%%%OOoo@@@@####@@@@ooOO##",
    "  ##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%OOoo@@@@@@@@ooOO%%%%##",
    "  ##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%OOooooooooOO||%%%%##",
    "##%%%%%%%%%%%%%%%%~~~~~~%%~~~~~~~~%%%%%%OOOOOOOO%%%%||%%##",
    "##%%%%~~%%%%%%%%~~======~~======~~~~%%%%%%%%%%~~%%%%||%%##",
    "##%%%%vv~~%%%%~~========~~==========~~~~%%%%~~vv%%%%||%%##",
    "  ##%%%%%%~~~~========~~~~~~vv==========~~vv%%%%%%||%%%%##",
    "  ##%%%%%%vv========~~vv%%vvvv==========vv%%%%%%%%%%%%####",
    "  ##%%%%%%%%vvvvvvvvvv%%%%%%vvvvvvvvvvvv%%%%%%%%%%%%####",
    "    ####%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##",
    "      ##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##",
    "        ##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%####",
    "        ##%%%%%%%%++++++++++++++++%%%%%%%%%%%%%%%%%%%%##",
    "        ##%%%%%%++++++++++++++++++++%%%%%%%%%%%%%%%%%%##",
    "        ##%%%%%%++++++++++++++++++++%%%%%%%%%%%%%%%%%%##",
    "      ##%%%%%%%%++++++++++++++++++++%%%%%%%%%%88%%%%%%##",
    "      ##%%%%88%%%%++++++++++++++++%%%%%%%%%%%%88%%%%%%##",
    "    ##%%%%8888%%%%%%++++++++++++88%%%%%%%%%%%%88%%%%%%##",
    "    ##888888##%%%%%%############88%%%%%%%%%%##88888888##",
    "      ################          ######################",
};

/** Caption under the art. */
constexpr std::string_view kCaption = "                    Ribbit! - Rivet";

/** Row of kArt the YAMS version is printed beside, level with the mustache. */
constexpr size_t kVersionRow = 13;

/** Spaces between the widest row of kArt and the text beside it. */
constexpr size_t kVersionGap = 4;

/** Printed under the YAMS version. */
constexpr std::string_view kWebsite = "yassrobotics.com";

/** Width of the widest row of kArt. */
constexpr size_t ArtWidth() {
  size_t width = 0;
  for (std::string_view line : kArt) {
    width = std::max(width, line.size());
  }
  return width;
}

/** The version as shown beside Rivet, with a v in front of release numbers. */
std::string VersionLabel() {
  std::string_view version = Rivet::GetVersion();
  std::string label;
  if (!version.empty() && version.front() >= '0' && version.front() <= '9') {
    label += 'v';
  }
  label += version;
  return label;
}

std::atomic<bool> shown{false};

/** ANSI color for a glyph, close to the color it stands for in the pixel art. */
std::string_view Color(char glyph) {
  switch (glyph) {
    case '#':
      return "\033[90m";
    case '@':
      return "\033[97m";
    case '%':
    case '+':
      return "\033[92m";
    case '8':
      return "\033[32m";
    case 'O':
      return "\033[93m";
    case 'o':
    case '|':
      return "\033[33m";
    case '~':
    case '=':
    case 'v':
      return "\033[31m";
    default:
      return "\033[0m";
  }
}

/** Color a line of the art, switching the ANSI color only where the glyph changes. */
std::string Colorize(std::string_view line) {
  std::string colored;
  char previous = 0;
  for (char glyph : line) {
    if (glyph != previous && glyph != ' ') {
      colored += Color(glyph);
      previous = glyph;
    }
    colored += glyph;
  }
  return colored;
}

}  // namespace

std::string_view Rivet::GetVersion() {
  return YAMS_VERSION_STRING;
}

void Rivet::Show() {
  if (shown.exchange(true)) {
    return;
  }
  const std::string side[] = {"\033[97mYAMS " + VersionLabel(),
                              "\033[92m" + std::string{kWebsite}};
  for (size_t row = 0; row < std::size(kArt); ++row) {
    std::string line = Colorize(kArt[row]);
    if (row >= kVersionRow && row - kVersionRow < std::size(side)) {
      line.append(ArtWidth() - kArt[row].size() + kVersionGap, ' ');
      line += side[row - kVersionRow];
    }
    wpi::DriverStationDisplay::AddLine(line);
  }
  wpi::DriverStationDisplay::AddLine(kCaption);
  wpi::DriverStationDisplay::UpdateLines();
}

}  // namespace yams

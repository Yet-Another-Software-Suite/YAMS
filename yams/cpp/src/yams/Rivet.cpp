// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/Rivet.hpp"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <iterator>
#include <random>
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

/**
 * What Rivet might say under the art; one is picked at random each startup. The first thirteen are
 * Rivet's speech bubbles from yassrobotics.com/rivet.
 */
constexpr std::string_view kPhrases[] = {
    "I lick batteries for fun.",
    "I programmed an arm today :)",
    "This mustache is 100% real.",
    "Ask me about YAGSL sometime.",
    "I once inspected 40 robots in one day.",
    "Simulation first, always.",
    "My pronouns change when I feel like it.",
    "That tickles.",
    "Free software, free frog.",
    "I'm basically a CSA now.",
    "Pi Day is my Super Bowl.",
    "Rivet was here.",
    "I may have eaten a battery once.",
    "Ribbit!",
    "Is this battery charged? Only one way to find out.",
    "Have you checked your CAN wiring?",
    "Clipboard in hand, tape measure ready.",
    "Whistle on, penalty stick ready.",
    "Hop in, we're going for a joyride.",
    "Got a sticker? That's my face!",
    "Spare bolts make great stocking stuffers.",
    "Gear ratios are my love language.",
    "Did you tune your PIDs today?",
    "Part leprechaun, all frog.",
    "Bumpers on? Let's play!",
};

/** Credit after the phrase. */
constexpr std::string_view kSignature = " - Rivet";

/** A random phrase from kPhrases, signed and centered under the art. */
std::string Caption(size_t artWidth) {
  std::random_device seed;
  std::uniform_int_distribution<size_t> pick{0, std::size(kPhrases) - 1};
  std::string caption{kPhrases[pick(seed)]};
  caption += kSignature;
  if (caption.size() < artWidth) {
    caption.insert(0, (artWidth - caption.size()) / 2, ' ');
  }
  return caption;
}

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
  wpi::DriverStationDisplay::AddLine(Caption(ArtWidth()));
  wpi::DriverStationDisplay::UpdateLines();
}

}  // namespace yams

// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/Rivet.hpp"

#include <atomic>
#include <cstddef>
#include <iterator>
#include <random>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
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
 * Rivet, drawn from the official pixel art at 12 by 7 characters. Each glyph stands for one color:
 * @ eye whites, # pupils, % body, + belly, 8 feet, O monocle, | monocle chain, ~ and = mustache,
 * v mouth.
 */
constexpr std::string_view kArt[] = {
    " %@@%  OOOO",
    "%@##@%O@##@O",
    "%%@@%%%OOOO|",
    "%~~====~~%%|",
    "%%vvvvvv%%%%",
    " %%+++++%%%",
    "888%    %888",
};

/** Columns Rivet takes up, plus one space before the text beside Rivet. */
constexpr size_t kArtColumns = 13;

/** Columns left beside Rivet for the version and phrase. */
constexpr size_t kSideColumns = 12;

/** Full-width rows under Rivet. */
constexpr std::string_view kFooter[] = {
    "yassrobotics.com",
};

/**
 * What Rivet might say beside the art; one is picked at random each startup. The first thirteen are
 * Rivet's speech bubbles from yassrobotics.com/rivet. Each must word-wrap, with kSignature, into the
 * five rows under the version.
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
    "Battery charged? *lick*",
    "Need a CSA? I brought zip ties.",
    "Clipboard in hand, tape measure ready.",
    "Whistle on, penalty stick ready.",
    "Hop in, we're going for a joyride.",
    "Got a sticker? That's my face!",
    "Spare bolts make great stocking stuffers.",
    "Gear ratios are my love language.",
    "kP, kI, kD, kRibbit.",
    "Part leprechaun, all frog.",
    "Bumpers on? Let's play!",
};

/** Credit after the phrase. */
constexpr std::string_view kSignature = "- Rivet";

/** ANSI white, for the text around Rivet. */
constexpr std::string_view kWhite = "\033[97m";

std::atomic<bool> shown{false};

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

/** Word-wrap a phrase to the columns beside Rivet and sign it, on the last line when it fits. */
std::vector<std::string> Wrap(std::string_view phrase) {
  std::vector<std::string> lines;
  std::string line;
  while (!phrase.empty()) {
    size_t space = phrase.find(' ');
    std::string_view word = phrase.substr(0, space);
    phrase = space == std::string_view::npos ? std::string_view{} : phrase.substr(space + 1);
    if (line.empty()) {
      line = word;
    } else if (line.size() + 1 + word.size() <= kSideColumns) {
      line += ' ';
      line += word;
    } else {
      lines.push_back(line);
      line = word;
    }
  }
  if (line.size() + 1 + kSignature.size() <= kSideColumns) {
    line += ' ';
    line += kSignature;
    lines.push_back(line);
  } else {
    lines.push_back(line);
    lines.emplace_back(kSignature);
  }
  return lines;
}

/** ANSI color for a glyph, close to the color it stands for in the pixel art. */
std::string_view Color(char glyph) {
  switch (glyph) {
    case '#':
      return "\033[90m";
    case '@':
      return kWhite;
    case '%':
    case '+':
      return "\033[92m";
    case '8':
      return "\033[32m";
    case 'O':
      return "\033[93m";
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
  std::random_device seed;
  std::uniform_int_distribution<size_t> pick{0, std::size(kPhrases) - 1};
  std::vector<std::string> side{"YAMS", VersionLabel()};
  for (std::string& line : Wrap(kPhrases[pick(seed)])) {
    side.push_back(std::move(line));
  }
  for (size_t row = 0; row < std::size(kArt); ++row) {
    std::string line = Colorize(kArt[row]);
    if (row < side.size()) {
      line.append(kArtColumns - kArt[row].size(), ' ');
      line += kWhite;
      line += side[row];
    }
    wpi::DriverStationDisplay::AddLine(line);
  }
  for (std::string_view footer : kFooter) {
    std::string line{kWhite};
    line += footer;
    wpi::DriverStationDisplay::AddLine(line);
  }
  wpi::DriverStationDisplay::UpdateLines();
}

}  // namespace yams

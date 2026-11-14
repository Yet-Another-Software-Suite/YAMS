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

/** Rivet, monocle and mustache included, at 12 by 6 characters. */
constexpr std::string_view kArt[] = {
    R"( .-.    .-.)",
    R"(( o )--( O ))",
    R"((  ,~^^~, |))",
    R"( \  `--' ;/)",
    R"( /`-....-'\)",
    R"((_/      \_))",
};

/**
 * Color of each character in kArt: g green body, l light green belly, w white eye, y gold monocle,
 * r red mustache and smile.
 */
constexpr std::string_view kArtColors[] = {
    " ggg    yyy",
    "g w gggy y y",
    "g  rrrrrr yg",
    " g  rrrr yg",
    " gggllllggg",
    "ggg      ggg",
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
 * What Rivet might say beside the art; one is picked at random each startup. Keep each to three or
 * four words. The first thirteen are Rivet's speech bubbles from yassrobotics.com/rivet, trimmed.
 */
constexpr std::string_view kPhrases[] = {
    "I lick batteries.",
    "I programmed an arm.",
    "Mustache 100% real.",
    "Ask me about YAGSL.",
    "Inspected 40 robots today.",
    "Simulation first, always.",
    "My pronouns change often.",
    "Hey, that tickles!",
    "Free software, free frog.",
    "Basically a CSA now.",
    "Pi Day rules.",
    "Rivet was here.",
    "Ate a battery once.",
    "Ribbit ribbit ribbit!",
    "Battery charged? *lick*",
    "Need a CSA?",
    "Clipboard in hand.",
    "Whistle on, penalty ready.",
    "Hop in, joyride time!",
    "That sticker's my face!",
    "Bolts in my stocking.",
    "I love gear ratios.",
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

/** ANSI color for a code from kArtColors, close to Rivet's colors. */
std::string_view Color(char code) {
  switch (code) {
    case 'g':
      return "\033[32m";
    case 'l':
      return "\033[92m";
    case 'w':
      return kWhite;
    case 'y':
      return "\033[93m";
    case 'r':
      return "\033[31m";
    default:
      return "\033[0m";
  }
}

/** Color a line of the art from its line of kArtColors, switching only where the color changes. */
std::string Colorize(std::string_view line, std::string_view colors) {
  std::string colored;
  char previous = 0;
  for (size_t i = 0; i < line.size(); ++i) {
    char code = i < colors.size() ? colors[i] : ' ';
    if (code != previous && code != ' ') {
      colored += Color(code);
      previous = code;
    }
    colored += line[i];
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
  // The phrase can run past Rivet's feet; it stays in the column beside Rivet.
  for (size_t row = 0; row < std::max(std::size(kArt), side.size()); ++row) {
    std::string_view art = row < std::size(kArt) ? kArt[row] : std::string_view{};
    std::string line = row < std::size(kArt) ? Colorize(kArt[row], kArtColors[row]) : "";
    if (row < side.size()) {
      line.append(kArtColumns - art.size(), ' ');
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

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

/** Ends just left of the docked panel's pop-out button, which sits over columns 21 to 25. */
constexpr std::string_view kClickForMore = "Click for more ---->";

/** Shown under the YAMS version. */
constexpr std::string_view kWebsite = "yassrobotics.com";

/** Shown under the website, after a blank row. */
constexpr std::string_view kTuner[] = {"Don't forget to try", "tuning.yassrobotics.com"};

/**
 * Rivet at full resolution, two characters per pixel, traced from the official Rivet pixel art
 * with the outline left off. Each glyph stands for one color: # pupils, @ eye whites, % body,
 * + belly, 8 feet shading, O and o monocle, | monocle chain, ~ and = handlebar mustache, v mouth.
 */
constexpr std::string_view kFull[] = {
    "          %%%%%%%%                    %%%%%%%%",
    "        %%%%%%%%%%%%                %%OOOOOOOO%%%%",
    "        %%@@@@@@@@%%%%%%%%%%%%%%%%%%OOooooooooOO%%",
    "      %%@@@@####@@@@%%%%%%%%%%%%%%OOoo@@@@@@@@ooOO%%",
    "      %%@@########@@%%%%%%%%%%%%OOoo@@@@####@@@@ooOO",
    "      %%@@########@@%%%%%%%%%%%%OOoo@@########@@ooOO",
    "    %%%%@@@@####@@@@%%%%%%%%%%%%OOoo@@########@@ooOO",
    "  %%%%%%%%@@@@@@@@%%%%%%%%%%%%%%OOoo@@@@####@@@@ooOO",
    "  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%OOoo@@@@@@@@ooOO%%%%",
    "  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%OOooooooooOO||%%%%",
    "%%%%%%%%%%%%%%%%~~~~~~%%~~~~~~~~%%%%%%OOOOOOOO%%%%||%%",
    "%%%%~~%%%%%%%%~~======~~======~~~~%%%%%%%%%%~~%%%%||%%",
    "%%%%vv~~%%%%~~========~~==========~~~~%%%%~~vv%%%%||%%",
    "  %%%%%%~~~~========~~~~~~vv==========~~vv%%%%%%||%%%%",
    "  %%%%%%vv========~~vv%%vvvv==========vv%%%%%%%%%%%%",
    "  %%%%%%%%vvvvvvvvvv%%%%%%vvvvvvvvvvvv%%%%%%%%%%%%",
    "      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%",
    "      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%",
    "        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%",
    "        %%%%%%%%++++++++++++++++%%%%%%%%%%%%%%%%%%%%",
    "        %%%%%%++++++++++++++++++++%%%%%%%%%%%%%%%%%%",
    "        %%%%%%++++++++++++++++++++%%%%%%%%%%%%%%%%%%",
    "      %%%%%%%%++++++++++++++++++++%%%%%%%%%%88%%%%%%",
    "      %%%%88%%%%++++++++++++++++%%%%%%%%%%%%88%%%%%%",
    "    %%%%8888%%%%%%++++++++++++88%%%%%%%%%%%%88%%%%%%",
    "    888888  %%%%%%            88%%%%%%%%%%  88888888",
};

/** Column kFull starts at, past the docked panel. */
constexpr size_t kFullColumn = 30;

/**
 * What Rivet might say under kFull; one is picked at random each startup. Keep each to three or
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
constexpr std::string_view kSignature = " - Rivet";

/** ANSI white, for text. */
constexpr std::string_view kWhite = "\033[97m";

/** ANSI bright green, for the welcome. */
constexpr std::string_view kGreen = "\033[92m";

std::atomic<bool> shown{false};

/** The version as shown in the welcome, with a v in front of release numbers. */
std::string VersionLabel() {
  std::string_view version = Rivet::GetVersion();
  std::string label;
  if (!version.empty() && version.front() >= '0' && version.front() <= '9') {
    label += 'v';
  }
  label += version;
  return label;
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
      return kGreen;
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

/** Color a line of kFull, switching the ANSI color only where the glyph changes. */
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
  const std::string welcome[] = {"Welcome to YASS!",
                                 std::string{kClickForMore},
                                 "",
                                 "YAMS " + VersionLabel(),
                                 std::string{kWebsite},
                                 "",
                                 std::string{kTuner[0]},
                                 std::string{kTuner[1]}};
  std::random_device seed;
  std::uniform_int_distribution<size_t> pick{0, std::size(kPhrases) - 1};
  std::string phrase{kPhrases[pick(seed)]};
  phrase += kSignature;
  size_t fullWidth = 0;
  for (std::string_view line : kFull) {
    fullWidth = std::max(fullWidth, line.size());
  }
  size_t rows = std::max(std::size(welcome), std::size(kFull) + 2);
  for (size_t row = 0; row < rows; ++row) {
    std::string_view text = row < std::size(welcome) ? std::string_view{welcome[row]} : "";
    std::string line;
    if (!text.empty()) {
      line = row == 0 ? kGreen : kWhite;
      line += text;
    }
    std::string right;
    if (row < std::size(kFull)) {
      right = Colorize(kFull[row]);
    } else if (row == std::size(kFull) + 1) {
      right = kWhite;
      if (phrase.size() < fullWidth) {
        right.append((fullWidth - phrase.size()) / 2, ' ');
      }
      right += phrase;
    }
    if (!right.empty()) {
      line.append(kFullColumn - text.size(), ' ');
      line += right;
    }
    wpi::DriverStationDisplay::AddLine(line);
  }
  wpi::DriverStationDisplay::UpdateLines();
}

}  // namespace yams

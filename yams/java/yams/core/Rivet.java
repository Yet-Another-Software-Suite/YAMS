// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ThreadLocalRandom;
import java.util.concurrent.atomic.AtomicBoolean;
import org.wpilib.driverstation.DriverStationDisplay;

/**
 * Rivet, the YASS mascot, says hi on the Driver Station display when YAMS starts up.
 *
 * <p>The splash is written once, the first time a YAMS motor controller config or swerve drive is
 * built, and fits the display's default 25 columns by 10 rows: Rivet on the left, the YAMS version
 * and a random phrase from Rivet beside, and yassrobotics.com along the bottom. It uses {@link
 * DriverStationDisplay} line mode, so robot code that writes its own lines afterwards simply
 * replaces it.
 */
public final class Rivet {

  /** Rivet, monocle and mustache included, at 12 by 6 characters. */
  private static final String[] ART = {
      " .-.    .-.",
      "( o )--( O )",
      "(  ,~^^~, |)",
      " \\  `--' ;/",
      " /`-....-'\\",
      "(_/      \\_)",
  };

  /**
   * Color of each character in {@link #ART}: {@code g} green body, {@code l} light green belly,
   * {@code w} white eye, {@code y} gold monocle, {@code r} red mustache and smile.
   */
  private static final String[] ART_COLORS = {
      " ggg    yyy",
      "g w gggy y y",
      "g  rrrrrr yg",
      " g  rrrr yg",
      " gggllllggg",
      "ggg      ggg",
  };

  /** Columns Rivet takes up, plus one space before the text beside Rivet. */
  private static final int ART_COLUMNS = 13;

  /** Columns left beside Rivet for the version and phrase. */
  private static final int SIDE_COLUMNS = 12;

  /** Full-width rows under Rivet. */
  private static final String[] FOOTER = {
      "yassrobotics.com",
  };

  /**
   * What Rivet might say beside the art; one is picked at random each startup. Keep each to three or
   * four words. The first thirteen are Rivet's speech bubbles from yassrobotics.com/rivet, trimmed.
   */
  private static final String[] PHRASES = {
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
  private static final String SIGNATURE = "- Rivet";

  /** ANSI white, for the text around Rivet. */
  private static final String WHITE = "\033[97m";

  /** Resource, next to this class, that the build writes the YAMS version into. */
  private static final String VERSION_RESOURCE = "yams-version.txt";

  /** Version shown when YAMS was built from source rather than a release. */
  private static final String DEV_VERSION = "dev";

  /** Whether Rivet has already been shown. */
  private static final AtomicBoolean shown = new AtomicBoolean(false);

  private Rivet() {}

  /**
   * Show Rivet on the Driver Station display, with the YAMS version and a random phrase beside
   * Rivet and yassrobotics.com underneath. Only the first call does anything.
   */
  public static void show() {
    if (!shown.compareAndSet(false, true)) {
      return;
    }
    try {
      List<String> side = new ArrayList<>();
      side.add("YAMS");
      side.add(versionLabel());
      side.addAll(wrap(PHRASES[ThreadLocalRandom.current().nextInt(PHRASES.length)]));
      // The phrase can run past Rivet's feet; it stays in the column beside Rivet.
      for (int row = 0; row < Math.max(ART.length, side.size()); row++) {
        String art = row < ART.length ? ART[row] : "";
        String line = row < ART.length ? colorize(ART[row], ART_COLORS[row]) : "";
        if (row < side.size()) {
          line += " ".repeat(ART_COLUMNS - art.length()) + WHITE + side.get(row);
        }
        DriverStationDisplay.addLine(line);
      }
      for (String line : FOOTER) {
        DriverStationDisplay.addLine(WHITE + line);
      }
      DriverStationDisplay.updateLines();
    } catch (RuntimeException | LinkageError e) {
      // The splash is a nicety; never stop a robot because the display is unavailable.
    }
  }

  /**
   * Get the YAMS version this library was built as.
   *
   * @return The release version, such as {@code 2026.11.14}, or {@code dev} when YAMS was built
   *     from source.
   */
  public static String getVersion() {
    try (InputStream in = Rivet.class.getResourceAsStream(VERSION_RESOURCE)) {
      if (in != null) {
        String version = new String(in.readAllBytes(), StandardCharsets.UTF_8).trim();
        if (!version.isEmpty()) {
          return version;
        }
      }
    } catch (IOException e) {
      // Fall through to the source build version.
    }
    return DEV_VERSION;
  }

  /**
   * The version as shown beside Rivet, with a {@code v} in front of release numbers.
   *
   * @return Version label, such as {@code v2026.11.14} or {@code dev}.
   */
  private static String versionLabel() {
    String version = getVersion();
    return Character.isDigit(version.charAt(0)) ? "v" + version : version;
  }

  /**
   * Word-wrap a phrase to the columns beside Rivet and sign it, on the last line when it fits.
   *
   * @param phrase Phrase from {@link #PHRASES}.
   * @return Lines of at most {@link #SIDE_COLUMNS} characters.
   */
  private static List<String> wrap(String phrase) {
    List<String> lines = new ArrayList<>();
    String line = "";
    for (String word : phrase.split(" ")) {
      if (line.isEmpty()) {
        line = word;
      } else if (line.length() + 1 + word.length() <= SIDE_COLUMNS) {
        line += " " + word;
      } else {
        lines.add(line);
        line = word;
      }
    }
    if (line.length() + 1 + SIGNATURE.length() <= SIDE_COLUMNS) {
      lines.add(line + " " + SIGNATURE);
    } else {
      lines.add(line);
      lines.add(SIGNATURE);
    }
    return lines;
  }

  /**
   * Color a line of the art, switching the ANSI color only where the color changes.
   *
   * @param line Line of {@link #ART}.
   * @param colors Matching line of {@link #ART_COLORS}.
   * @return The line with ANSI colors.
   */
  private static String colorize(String line, String colors) {
    StringBuilder colored = new StringBuilder();
    char previous = 0;
    for (int i = 0; i < line.length(); i++) {
      char code = i < colors.length() ? colors.charAt(i) : ' ';
      if (code != previous && code != ' ') {
        colored.append(color(code));
        previous = code;
      }
      colored.append(line.charAt(i));
    }
    return colored.toString();
  }

  /**
   * ANSI color for a code from {@link #ART_COLORS}, close to Rivet's colors.
   *
   * @param code Color code.
   * @return ANSI color escape.
   */
  private static String color(char code) {
    return switch (code) {
      case 'g' -> "\033[32m";
      case 'l' -> "\033[92m";
      case 'w' -> WHITE;
      case 'y' -> "\033[93m";
      case 'r' -> "\033[31m";
      default -> "\033[0m";
    };
  }
}

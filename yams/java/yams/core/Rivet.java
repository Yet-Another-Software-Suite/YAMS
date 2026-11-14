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

  /**
   * Rivet, drawn from the official pixel art at 12 by 7 characters. Each glyph stands for one
   * color: {@code @} eye whites, {@code #} pupils, {@code %} body, {@code +} belly, {@code 8} feet,
   * {@code O} monocle, {@code |} monocle chain, {@code ~} and {@code =} mustache, {@code v} mouth.
   */
  private static final String[] ART = {
      " %@@%  OOOO",
      "%@##@%O@##@O",
      "%%@@%%%OOOO|",
      "%~~====~~%%|",
      "%%vvvvvv%%%%",
      " %%+++++%%%",
      "888%    %888",
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
   * What Rivet might say beside the art; one is picked at random each startup. The first thirteen
   * are Rivet's speech bubbles from yassrobotics.com/rivet. Each must word-wrap, with {@link
   * #SIGNATURE}, into the five rows under the version.
   */
  private static final String[] PHRASES = {
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
      for (int row = 0; row < ART.length; row++) {
        String line = colorize(ART[row]);
        if (row < side.size()) {
          line += " ".repeat(ART_COLUMNS - ART[row].length()) + WHITE + side.get(row);
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
   * Color a line of the art, switching the ANSI color only where the glyph changes.
   *
   * @param line Plain ASCII line.
   * @return The line with ANSI colors.
   */
  private static String colorize(String line) {
    StringBuilder colored = new StringBuilder();
    char previous = 0;
    for (char glyph : line.toCharArray()) {
      if (glyph != previous && glyph != ' ') {
        colored.append(color(glyph));
        previous = glyph;
      }
      colored.append(glyph);
    }
    return colored.toString();
  }

  /**
   * ANSI color for a glyph, close to the color it stands for in the pixel art.
   *
   * @param glyph Glyph from {@link #ART}.
   * @return ANSI color escape.
   */
  private static String color(char glyph) {
    return switch (glyph) {
      case '#' -> "\033[90m";
      case '@' -> WHITE;
      case '%', '+' -> "\033[92m";
      case '8' -> "\033[32m";
      case 'O' -> "\033[93m";
      case '|' -> "\033[33m";
      case '~', '=', 'v' -> "\033[31m";
      default -> "\033[0m";
    };
  }
}

// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.concurrent.ThreadLocalRandom;
import java.util.concurrent.atomic.AtomicBoolean;
import org.wpilib.driverstation.DriverStationDisplay;

/**
 * Rivet, the YASS mascot, says hi on the Driver Station display when YAMS starts up.
 *
 * <p>The splash is written once, the first time a YAMS motor controller config or swerve drive is
 * built. The Display panel docked in the Driver Station shows about 26 columns by 9 rows, and Rivet
 * fits in it, with the YAMS version and yassrobotics.com underneath. Expanding the display (or
 * popping it out) shows a full resolution Rivet to the right with a random phrase from Rivet under
 * it. It uses {@link DriverStationDisplay} line mode, so
 * robot code that writes its own lines afterwards simply replaces it.
 *
 * <p>Both pictures are traced from the official Rivet pixel art, outline left off. Each glyph
 * stands for one color: {@code #} pupils, {@code @} eye whites, {@code %} body, {@code +} belly,
 * {@code 8} feet shading, {@code O} and {@code o} monocle, {@code |} monocle chain, {@code ~} and
 * {@code =} mustache, {@code v} mouth.
 */
public final class Rivet {

  /** Rivet scaled to fit the docked Display panel, 25 columns by 9 rows. */
  private static final String[] SMALL = {
      "    %%%%%        %OOO%%",
      "   %@###@%%%%%%Oo@###@oO",
      " %%%@@#@@%%%%%%Oo@###@oO",
      "%%%%%%%%~~%%~~~%%OOOOO%|%",
      "%%%~~~====~~v===~~%%%%%|%",
      " %%%%vvvv%%%vvvvv%%%%%%",
      "   %%%%%+++++++%%%%%%%%%",
      "   %%%++++++++++%%%%%%%%",
      "  8888%%++++++8%%%%%8888",
  };

  /** Full resolution Rivet, two characters per pixel, to the right of {@link #SMALL}. */
  private static final String[] FULL = {
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

  /** Columns before {@link #FULL}: the default view's width plus a gap. */
  private static final int FULL_COLUMN = 30;

  /** Shown under {@link #SMALL} after a blank row, below the YAMS version. */
  private static final String WEBSITE = "yassrobotics.com";

  /**
   * What Rivet might say under {@link #FULL}; one is picked at random each startup. Keep each to
   * three or four words. The first thirteen are Rivet's speech bubbles from yassrobotics.com/rivet,
   * trimmed.
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
  private static final String SIGNATURE = " - Rivet";

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
   * Show Rivet on the Driver Station display. Only the first call does anything.
   */
  public static void show() {
    if (!shown.compareAndSet(false, true)) {
      return;
    }
    try {
      String[] under = {"", "YAMS " + versionLabel(), WEBSITE};
      String phrase = PHRASES[ThreadLocalRandom.current().nextInt(PHRASES.length)] + SIGNATURE;
      int fullWidth = Arrays.stream(FULL).mapToInt(String::length).max().orElse(0);
      int rows = Math.max(SMALL.length + under.length, FULL.length + 2);
      for (int row = 0; row < rows; row++) {
        String left = "";
        int leftWidth = 0;
        if (row < SMALL.length) {
          left = colorize(SMALL[row]);
          leftWidth = SMALL[row].length();
        } else if (row - SMALL.length < under.length) {
          left = WHITE + under[row - SMALL.length];
          leftWidth = under[row - SMALL.length].length();
        }
        String right = "";
        if (row < FULL.length) {
          right = colorize(FULL[row]);
        } else if (row == FULL.length + 1) {
          right = WHITE + " ".repeat(Math.max(0, (fullWidth - phrase.length()) / 2)) + phrase;
        }
        DriverStationDisplay.addLine(
            right.isEmpty() ? left : left + " ".repeat(FULL_COLUMN - leftWidth) + right);
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
   * The version as shown under Rivet, with a {@code v} in front of release numbers.
   *
   * @return Version label, such as {@code v2026.11.14} or {@code dev}.
   */
  private static String versionLabel() {
    String version = getVersion();
    return Character.isDigit(version.charAt(0)) ? "v" + version : version;
  }

  /**
   * Color a line of art, switching the ANSI color only where the glyph changes.
   *
   * @param line Line of {@link #SMALL} or {@link #FULL}.
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
   * @param glyph Glyph from the art.
   * @return ANSI color escape.
   */
  private static String color(char glyph) {
    return switch (glyph) {
      case '#' -> "\033[90m";
      case '@' -> WHITE;
      case '%', '+' -> "\033[92m";
      case '8' -> "\033[32m";
      case 'O' -> "\033[93m";
      case 'o', '|' -> "\033[33m";
      case '~', '=', 'v' -> "\033[31m";
      default -> "\033[0m";
    };
  }
}

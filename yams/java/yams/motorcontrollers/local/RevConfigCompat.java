// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.motorcontrollers.local;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;

/**
 * REVLib 2027.0.0-alpha-7 removed the typed setters for encoder conversion factors and
 * closed-loop position-wrapping input range from {@link EncoderConfig}, {@link
 * AbsoluteEncoderConfig}, and {@link ClosedLoopConfig}; but the underlying raw CAN parameters
 * (unchanged since 2026 REVLib's {@code SparkParameters} table) still work. These package-private
 * subclasses restore them by writing those parameter IDs directly through the {@code protected}
 * {@code putParameter} REVLib exposes to config subclasses, then get merged onto the real config
 * object via its public {@code apply(...)}.
 */
final class RevEncoderConversionFactors extends EncoderConfig {
  private static final int POSITION_CONVERSION_FACTOR = 112;
  private static final int VELOCITY_CONVERSION_FACTOR = 113;

  RevEncoderConversionFactors withPositionConversionFactor(double factor) {
    putParameter(POSITION_CONVERSION_FACTOR, (float) factor);
    return this;
  }

  RevEncoderConversionFactors withVelocityConversionFactor(double factor) {
    putParameter(VELOCITY_CONVERSION_FACTOR, (float) factor);
    return this;
  }
}


final class RevAbsoluteEncoderConversionFactors extends AbsoluteEncoderConfig {
  private static final int DUTY_CYCLE_POSITION_FACTOR = 139;
  private static final int DUTY_CYCLE_VELOCITY_FACTOR = 140;

  RevAbsoluteEncoderConversionFactors() {
    setSparkMaxDataPortConfig();
  }

  RevAbsoluteEncoderConversionFactors withPositionConversionFactor(double factor) {
    putParameter(DUTY_CYCLE_POSITION_FACTOR, (float) factor);
    return this;
  }

  RevAbsoluteEncoderConversionFactors withVelocityConversionFactor(double factor) {
    putParameter(DUTY_CYCLE_VELOCITY_FACTOR, (float) factor);
    return this;
  }
}


final class RevClosedLoopPositionWrapping extends ClosedLoopConfig {
  private static final int POSITION_PID_MIN_INPUT = 150;
  private static final int POSITION_PID_MAX_INPUT = 151;

  RevClosedLoopPositionWrapping withMinInput(double minInput) {
    putParameter(POSITION_PID_MIN_INPUT, (float) minInput);
    return this;
  }

  RevClosedLoopPositionWrapping withMaxInput(double maxInput) {
    putParameter(POSITION_PID_MAX_INPUT, (float) maxInput);
    return this;
  }
}

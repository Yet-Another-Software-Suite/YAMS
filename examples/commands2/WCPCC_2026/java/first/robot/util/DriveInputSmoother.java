// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.util;

import java.util.function.DoubleSupplier;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N2;

public class DriveInputSmoother {
    private static final double kJoystickDeadband = 0.15;
    private static final double kCurveExponent = 1.5;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;
    private final DoubleSupplier rotationInput;

    public DriveInputSmoother(DoubleSupplier forwardInput, DoubleSupplier leftInput, DoubleSupplier rotationInput) {
        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
        this.rotationInput = rotationInput;
    }

    public DriveInputSmoother(DoubleSupplier forwardInput, DoubleSupplier leftInput) {
        this(forwardInput, leftInput, () -> 0);
    }

    public ManualDriveInput getSmoothedInput() { 
        final Vector<N2> rawTranslationInput = VecBuilder.fill(forwardInput.getAsDouble(), leftInput.getAsDouble());
        final Vector<N2> deadbandedTranslationInput = MathUtil.applyDeadband(rawTranslationInput, kJoystickDeadband);
        final Vector<N2> curvedTranslationInput = MathUtil.copyDirectionPow(deadbandedTranslationInput, kCurveExponent);

        final double rawRotationInput = rotationInput.getAsDouble();
        final double deadbandedRotationInput = MathUtil.applyDeadband(rawRotationInput, kJoystickDeadband);
        final double curvedRotationInput = MathUtil.copyDirectionPow(deadbandedRotationInput, kCurveExponent);

        return new ManualDriveInput(
            curvedTranslationInput.get(0), 
            curvedTranslationInput.get(1), 
            curvedRotationInput
        );
    }
}

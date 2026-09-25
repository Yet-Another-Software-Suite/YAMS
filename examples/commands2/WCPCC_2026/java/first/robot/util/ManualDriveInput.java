// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.util;

public class ManualDriveInput {
    public final double forward;
    public final double left;
    public final double rotation;

    public ManualDriveInput(double forward, double left, double rotation) {
        this.forward = forward;
        this.left = left;
        this.rotation = rotation;
    }

    public ManualDriveInput() {
        this(0, 0, 0);
    }

    public boolean hasTranslation() {
        return Math.hypot(forward, left) > 0;
    }

    public boolean hasRotation() {
        return Math.abs(rotation) > 0;
    }
}

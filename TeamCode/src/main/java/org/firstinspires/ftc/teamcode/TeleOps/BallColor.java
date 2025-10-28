package org.firstinspires.ftc.teamcode.TeleOps;

import java.util.HashMap;
import java.util.Map;


public enum BallColor {
    PURPLE,
    GREEN,
    UNKNOWN;

    public static BallColor fromHue(float hue) {
        if (hue >= 170 && hue <= 230) return PURPLE;
        if (hue >= 135 && hue <= 160) return GREEN;
        return UNKNOWN;
    }

    public boolean isKnown() {
        return this != UNKNOWN;
    }
}

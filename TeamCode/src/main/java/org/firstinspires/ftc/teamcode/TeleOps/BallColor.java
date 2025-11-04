package org.firstinspires.ftc.teamcode.TeleOps;

import java.util.HashMap;
import java.util.Map;


public enum BallColor {
    PURPLE,
    GREEN,
    UNKNOWN;

    /**
     * Determines the BallColor based on HSV and proximity values.
     * @param hue The hue value from the color sensor (typically 0-360).
     * @param value The value (brightness) from the color sensor (0-1.0).
     * @return The detected BallColor (PURPLE, GREEN, or UNKNOWN).
     */

    public static BallColor fromHue(float hue, float value) {
        if (value > 0.1){
            if (hue >= 170 && hue <= 230) return PURPLE;
            if (hue >= 135 && hue <= 160) return GREEN;
        }
        return UNKNOWN;
    }

    public boolean isKnown() {
        return this != UNKNOWN;
    }
}

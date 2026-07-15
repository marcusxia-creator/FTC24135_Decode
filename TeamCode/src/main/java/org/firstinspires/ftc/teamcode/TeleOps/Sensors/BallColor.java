package org.firstinspires.ftc.teamcode.TeleOps.Sensors;


import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.greenRangeHigh;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.purpleRangeHigh;

public enum BallColor {
    PURPLE,
    GREEN,
    UNKNOWN;

    /**
     * Determines the BallColor based on HSV and proximity values.
     * @param hue The hue value from the color sensor (typically 0-360).
     *
     * @return The detected BallColor (PURPLE, GREEN, or UNKNOWN).
     */

    public static BallColor fromHue(float hue) {

        if (hue >= purpleRangeHigh[0] && hue <= purpleRangeHigh[1]) return PURPLE;
        if (hue >= greenRangeHigh[0] && hue <= greenRangeHigh[1]) return GREEN;
        return UNKNOWN;
    }

    public boolean isKnown() {
        return this != UNKNOWN;
    }
}
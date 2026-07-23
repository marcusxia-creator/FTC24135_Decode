package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

public enum AutoBallColors {
    GREEN,
    PURPLE,
    UNKNOWN;

    public static AutoBallColors fromHue (float hue) {
        if (greenRangeLow[0] < hue && hue < greenRangeHigh[1]) return GREEN;
        if (purpleRangeLow[0] < hue && hue < purpleRangeHigh[1]) return PURPLE;
        return UNKNOWN;
    }

    public boolean isKnown() {return this != UNKNOWN;}

}